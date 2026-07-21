"""
tools/diagnose_placement.py

Phase11 ターゲット1診断: 「2コンテナ×優先コンテナ指定」時に placement_score が 100 未満に
なる原因を切り分ける。

placement_score の減点要因は tools/scorer.py の定義より2種類:
  (a) 優先手荷物が非優先手荷物の下敷きになっている(上方向からの鉛直接触)
  (b) 優先コンテナが存在するのに、優先手荷物が非優先コンテナに入っている

本ツールはエピソードを1回実走させ、
  - 各ステップの action について「優先荷物 → 非優先コンテナ」に置いた瞬間を記録する
    (= (b) の発生タイミングと、その時 planner がどの経路(優先強制/フォールバック)を
      通ったか)
  - 最終状態で (a)/(b) の内訳と該当 item index を出す
を行う。src/ と agents/mysolver/ は変更しない(planner の関数を読み取り専用で呼ぶだけ)。

実行例:
    PYTHONPATH=. .venv/bin/python tools/diagnose_placement.py \
        --config-path configs/gen/suite_D03_A_2c_60_prioheavy_cont.json --optimize-budget 30
"""
import argparse
import json
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from src.ground_handling.env import GroundHandlingEnv
from tools.scorer import Scorer
from agents.mysolver import ordering
from agents.mysolver import planner as msplanner


def parse_args():
    parser = argparse.ArgumentParser()
    parser.add_argument('--config-path', required=True)
    parser.add_argument('--task-id', default=None)
    parser.add_argument('--optimize-budget', type=float, default=30.0)
    parser.add_argument('--policy-budget', type=float, default=5.5)
    return parser.parse_args()


def run_scene(task_config: dict, optimize_budget: float, policy_budget: float):
    env = GroundHandlingEnv(config=task_config, verbose=False, render_mode=None)
    try:
        env.reset_settings()
        init_states = env.get_init_states()
        container_prio = {c['index']: c.get('is_prioritized', False)
                          for c in init_states['container_list']}
        print(f'  containers: {container_prio}')

        if env.optimize:
            full_item_list = env.get_info_for_optimization()
            n_prio_stream = sum(1 for it in full_item_list if it.get('is_prioritized'))
            print(f'  stream items: {len(full_item_list)} (prioritized={n_prio_stream})')
            order = ordering.build_order(full_item_list, init_states['container_list'],
                                         init_states['lookahead_k'], time_budget=optimize_budget)
            env.set_item_order(order)

        env.reset_item_stream()
        obs, info = env.reset(seed=42)

        terminated = False
        truncated = False
        step = 0
        misroutes = []
        while not terminated and not truncated:
            container_list = obs['container_list']
            pool_list = obs['pool_list']
            if not pool_list:
                break
            action = msplanner.plan(container_list, pool_list, time_budget=policy_budget)
            if action is None:
                print(f'  step {step}: plan()=None (stall)')
                break
            item = pool_list[action['item_idx']]
            cidx = action['container_idx']
            if item.get('is_prioritized', False) and not container_prio.get(cidx, False):
                # 優先荷物を非優先コンテナに置いた瞬間。優先コンテナ限定で置けたかを再確認する。
                prio_containers = [c for c in container_list if c.get('is_prioritized', False)]
                retry_stats = {}
                retry = msplanner.plan(prio_containers, [item], time_budget=policy_budget,
                                       max_pool_items=None, stats=retry_stats)
                occ = []
                for c in container_list:
                    v = sum(it['length'] * it['width'] * it['height'] for it in c['packed_items'])
                    n_prio_in = sum(1 for it in c['packed_items'] if it.get('is_prioritized'))
                    occ.append(f'c{c["index"]}(prio={c.get("is_prioritized")}): '
                               f'{len(c["packed_items"])}個 vol={v:.2f}/{c["volume"]:.2f} '
                               f'({100 * v / c["volume"]:.1f}%) うち優先{n_prio_in}')
                misroutes.append({
                    'step': step,
                    'item_index': item['index'],
                    'container_idx': cidx,
                    'pool_size': len(pool_list),
                    'prio_container_had_legal_move': retry is not None,
                    'retry_stats': retry_stats,
                })
                print(f'  step {step}: MISROUTE item {item["index"]} (prio, '
                      f'{item["length"]:.2f}x{item["width"]:.2f}x{item["height"]:.2f}) -> container {cidx} '
                      f'(non-prio). 優先コンテナ単独で合法手あり? {retry is not None} stats={retry_stats}')
                for line in occ:
                    print(f'      {line}')
            obs, reward, terminated, truncated, step_info = env.step(action)
            step += 1

        scorer = Scorer(client=env.client, config=task_config)
        containers = env.container_manager.containers

        # --- placement 減点の内訳を、stability(破壊的)より前に取る ---
        prio_items = [it for c in containers for it in c.packed_items if it.is_prioritized]
        has_prio_c = any(c.is_prioritized for c in containers)
        crushed = set()
        for bottom, top in scorer._find_stacking_pairs(containers):
            if bottom.is_prioritized and not top.is_prioritized:
                crushed.add(bottom.index)
        container_of = {it.index: c for c in containers for it in c.packed_items}
        wrong_container = [it.index for it in prio_items
                           if has_prio_c and not container_of[it.index].is_prioritized]
        crushed_list = [it.index for it in prio_items if it.index in crushed]
        print(f'\n  [placement 減点内訳] 優先荷物 {len(prio_items)} 個中: '
              f'非優先コンテナ配置={len(wrong_container)} {wrong_container}, '
              f'下敷き={len(crushed_list)} {crushed_list}')

        metrics = scorer.evaluate(containers, env.num_total_items)
        print(f'\n  metrics: fill={metrics["fill_score"]:.2f} place={metrics["placement_score"]:.2f} '
              f'placed={metrics["num_placed_items_abs"]}/{metrics["total_items"]}')

        # 最終状態の内訳(注: stability計算後なので位置は揺れ後。内訳の再現は evaluate 前に取る)
        print(f'  misroute(優先荷物→非優先コンテナ) 件数: {len(misroutes)}')
        for m in misroutes:
            print(f'    {m}')
        return metrics, misroutes
    finally:
        try:
            env.close()
        except Exception:
            pass


def main():
    args = parse_args()
    with open(args.config_path) as f:
        config = json.load(f)
    task_id = args.task_id or next(iter(config.keys()))
    print(f'=== {args.config_path}::{task_id} (optimize_budget={args.optimize_budget}s) ===')
    run_scene(config[task_id], args.optimize_budget, args.policy_budget)


if __name__ == '__main__':
    main()
