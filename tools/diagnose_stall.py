"""
tools/diagnose_stall.py

Phase5診断用ツール: 「1コンテナあたり約20個で頭打ちになる」現象について、
episodeが停止する直前の状態を再現し、以下を定量化する。

  1. 実際に停止の引き金になった1手(その時点のpool_list, lookahead=1なら1個)について、
     どのorientationも合法にならなかった理由の内訳
     (候補位置が1つも生成されない/内包判定で全滅/天井制約で全滅/搬入経路のY掃引で全滅/
      搬入経路のX掃引で全滅)
  2. その時点で「まだ配置されていない全荷物」を対象に、同じ内訳を集計し、
     - 実は置ける荷物が残っている(=順序探索の視野不足)のか
     - どの荷物を持ってきても置けない(=真の空間的行き詰まり)のか
     を切り分ける。
  3. 停止時点の体積充填率・上下方向の利用率(最大到達高さ/コンテナ高さ)を出し、
     「上部空間が余っているか」を確認する。

src/, agents/mysolver/ は変更しない(planner.py の stats フックのみ利用)。

実行例:
    PYTHONPATH=. .venv/bin/python tools/diagnose_stall.py \
        --config-path configs/gen/gen_manyitems_patternA.json --task-id 000 \
        --optimize-budget 15
"""
import argparse
import json
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from src.ground_handling.env import GroundHandlingEnv
from agents.mysolver import geometry as geo
from agents.mysolver import ordering
from agents.mysolver import planner as msplanner

REASON_LABELS = {
    'no_xy': '候補位置が1つも生成されない',
    'fail_support': '着地面の支持条件(乗る面が足りない)で全滅',
    'fail_inclusion': '内包判定(壁食い込み)で全滅',
    'fail_ceiling': '天井制約で全滅',
    'fail_inclusion_and_ceiling': '内包+天井の複合で全滅',
    'fail_transport_y': '搬入経路Y掃引で全滅',
    'fail_transport_x': '搬入経路X掃引で全滅',
    'success': '合法候補あり',
}


def parse_args():
    parser = argparse.ArgumentParser()
    parser.add_argument('--config-path', required=True)
    parser.add_argument('--task-id', default=None, help='省略時はconfig内の最初のtask')
    parser.add_argument('--optimize-budget', type=float, default=15.0)
    parser.add_argument('--policy-budget', type=float, default=5.5)
    parser.add_argument('--per-item-budget', type=float, default=3.0,
                         help='残り荷物1個あたりの診断用planner呼び出し予算[s]')
    return parser.parse_args()


def summarize(stats: dict, total_attempts: int) -> str:
    lines = []
    for key, label in REASON_LABELS.items():
        n = stats.get(key, 0)
        if n:
            pct = 100.0 * n / max(total_attempts, 1)
            lines.append(f'    {label:24s}: {n:4d} 件 ({pct:5.1f}%)')
    return '\n'.join(lines) if lines else '    (試行なし)'


def run_scene(task_config: dict, optimize_budget: float, policy_budget: float, per_item_budget: float):
    env = GroundHandlingEnv(config=task_config, verbose=False, render_mode=None)
    try:
        env.reset_settings()
        init_states = env.get_init_states()

        full_item_list = None
        if env.optimize:
            full_item_list = env.get_info_for_optimization()
            order = ordering.build_order(full_item_list, init_states['container_list'],
                                          init_states['lookahead_k'], time_budget=optimize_budget)
            env.set_item_order(order)
        else:
            full_item_list = env.get_info_for_optimization()

        env.reset_item_stream()
        obs, info = env.reset(seed=42)

        terminated = False
        truncated = False
        stall_container_list = None
        stall_pool_list = None
        n_placed = 0

        while not terminated and not truncated:
            container_list = obs['container_list']
            pool_list = obs['pool_list']
            if not pool_list:
                break
            action = msplanner.plan(container_list, pool_list, time_budget=policy_budget)
            if action is None:
                stall_container_list = container_list
                stall_pool_list = pool_list
                break
            stall_container_list_candidate = container_list
            stall_pool_list_candidate = pool_list
            obs, reward, terminated, truncated, step_info = env.step(action)
            status = step_info.get('status', {})
            if terminated and not env.stream_manager.is_empty():
                # 直前(この action を決めた時点)の状態が「行き詰まり」の状態
                stall_container_list = stall_container_list_candidate
                stall_pool_list = stall_pool_list_candidate
            else:
                n_placed = sum(len(c['packed_items']) for c in obs['container_list'])

        if env.stream_manager.is_empty():
            print('  -> 全荷物を配置完了(頭打ちは発生しなかった)')
            return

        if stall_container_list is None:
            print('  -> 停止状態を捕捉できず(異常系)')
            return

        n_placed = sum(len(c['packed_items']) for c in stall_container_list)
        n_containers = len(stall_container_list)
        print(f'  -> 行き詰まり検出: 配置済み {n_placed} 個 / {n_containers} コンテナ '
              f'(1コンテナあたり平均 {n_placed / max(n_containers, 1):.1f} 個)')

        # --- 体積・高さの利用率 ---
        for c in stall_container_list:
            vol = c['volume']
            placed_vol = sum(it['length'] * it['width'] * it['height'] for it in c['packed_items'])
            max_top = 0.0
            floor_z = c['center'][2] - c['height'] / 2.0
            for it in c['packed_items']:
                if it.get('pos') is None or it.get('orn') is None:
                    continue
                center, half = geo.item_world_aabb(it)
                top_rel = (center[2] + half[2]) - floor_z
                max_top = max(max_top, top_rel)
            print(f'     container[{c["index"]}]: volume利用率 {100 * placed_vol / max(vol, 1e-9):5.1f}%, '
                  f'到達高さ/コンテナ高さ = {100 * max_top / max(c["height"], 1e-9):5.1f}% '
                  f'({len(c["packed_items"])} 個)')

        # --- (1) 停止の引き金になった1手の内訳 ---
        trigger_item = stall_pool_list[0]
        trigger_stats = {}
        msplanner.plan(stall_container_list, [trigger_item], time_budget=per_item_budget,
                        max_pool_items=None, stats=trigger_stats)
        n_orn = len(msplanner._unique_orientations((trigger_item['length'], trigger_item['width'], trigger_item['height'])))
        total_attempts = n_orn * n_containers
        print(f'\n  [引き金となった item {trigger_item["index"]} '
              f'(L{trigger_item["length"]:.2f} x W{trigger_item["width"]:.2f} x H{trigger_item["height"]:.2f}, '
              f'prio={trigger_item.get("is_prioritized")}, soft={trigger_item.get("is_soft")}) の内訳]')
        print(summarize(trigger_stats, total_attempts))

        # --- (2) 残り全荷物についての内訳(まだ置ける荷物が残っているか) ---
        placed_indices = {it['index'] for c in stall_container_list for it in c['packed_items']}
        remaining_items = [it for it in full_item_list if it['index'] not in placed_indices]
        print(f'\n  [残り未配置 {len(remaining_items)} 個 全数を、行き詰まり時点の状態に対して個別診断]')

        would_fit = 0
        blocked = 0
        agg_stats = {}
        for it in remaining_items:
            per_stats = {}
            act = msplanner.plan(stall_container_list, [it], time_budget=per_item_budget,
                                  max_pool_items=None, stats=per_stats)
            for k, v in per_stats.items():
                agg_stats[k] = agg_stats.get(k, 0) + v
            if act is not None:
                would_fit += 1
            else:
                blocked += 1
        total_item_attempts = sum(agg_stats.values())
        print(f'    置ける荷物が残っている(順序を変えれば置けた): {would_fit} 個')
        print(f'    どの向きでも置けない(真の空間的行き詰まり)  : {blocked} 個')
        print('    全試行(荷物×orientation×コンテナ)の内訳:')
        print(summarize(agg_stats, total_item_attempts))

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
    run_scene(config[task_id], args.optimize_budget, args.policy_budget, args.per_item_budget)


if __name__ == '__main__':
    main()
