"""placement_score / soft_item_score の乖離調査(Phase44/45)用の再実行可能な診断ツール。

`tools/local_eval.py` の `run_one_scene()` と同じ手順(env初期化 -> optimize -> policy
ループ)を踏襲しつつ、評価後に `containers` を保持したまま以下を追加集計する:

  - 配置された is_prioritized / is_soft 荷物の個数
  - `Scorer._find_stacking_pairs()` が検出した積み重なりペアの総数
  - うち優先手荷物が下敷き/ソフト貨物が下敷きのペア数
  - 各荷物の pos/orn/寸法(幾何クロスチェック用。`tools/diagnose_stacking_geocheck.py` 参照)
  - env が報告する status(`Stopped in the middle. Did not satisfy [...]` 等、
    sudden death 検出用)

読み取り専用(`src/`・`tools/scorer.py`・`configs/` は一切変更しない)。

実行方法(リポジトリルートで):
    # ローカル標準条件(壁時計非拘束、Phase41-44と同条件)
    MYSOLVER_HARD_WALL_LIMIT=3000 PYTHONPATH=. python tools/diagnose_stacking.py \\
        --out results/phase45_stacking_diag_off.json

    # 本番既定の壁時計拘束下(Phase45 ステップ2)
    MYSOLVER_HARD_WALL_LIMIT=165 PYTHONPATH=. python tools/diagnose_stacking.py \\
        --out results/phase45_stacking_diag_wallbound.json

    # シーンを絞る場合
    PYTHONPATH=. python tools/diagnose_stacking.py --config-path 'configs/gen/suite_B04*.json'

経緯: Phase41〜44で同種のスクリプトをセッションのスクラッチパッドに3回書いており、
そのたびに再実行できる形で残っていなかった。本ツールはそれを解消するためのもの
(results/phase44_report.md §2-1、results/phase45_report.md 参照)。
"""
import argparse
import glob
import json
import os
import sys
import time
import traceback

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from src.ground_handling.agent_factory import AgentFactory
from src.ground_handling.env import GroundHandlingEnv
from src.ground_handling.runner import TimedAgentRunner
from tools.scorer import Scorer


def parse_args():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('--config-path', default='configs/gen/suite_*.json',
                    help='評価するconfigファイルのglobパターン(既定: 26シーン全件)')
    p.add_argument('--module-path', default='agents/mysolver/',
                    help='エージェントモジュールのパス(末尾/込み)')
    p.add_argument('--out', default='/tmp/diagnose_stacking.json',
                    help='結果JSONの出力先')
    p.add_argument('--no-geo', action='store_true',
                    help='幾何ダンプ(pos/orn/寸法)を省いて出力を軽くする')
    return p.parse_args()


def run_one_scene_diag(task_config: dict, module_path: str, agent_module: str,
                        include_geo: bool) -> dict:
    agent_factory = AgentFactory(module_name=agent_module, class_name='Agent', module_path=module_path)
    env = GroundHandlingEnv(config=task_config, verbose=False, render_mode=None)
    runner = None
    try:
        env.reset_settings()
        init_states = env.get_init_states()
        allowed_methods = task_config['agent']['allowed_methods']
        max_mem = task_config['agent'].get('max_mem', 4)
        runner = TimedAgentRunner(agent_factory=agent_factory, allowed_methods=allowed_methods,
                                   max_mem=max_mem, verbose=False)
        runner.call('get_init_states', time_out_sec=task_config['agent']['init_timeout'],
                    fallback=None, init_states=init_states)

        if env.optimize:
            item_list = env.get_info_for_optimization()
            optimized_order, _ = runner.call(
                'optimize', time_out_sec=task_config['agent']['optimization_timeout'],
                fallback=list(env.stream_manager.all_indices), item_list=item_list)
            if not env.set_item_order(optimized_order):
                return {'status': 'optimize_failed'}

        env.reset_item_stream()
        obs, info = env.reset(seed=42)
        terminated = False
        truncated = False
        place_states: dict = {}
        while not terminated and not truncated:
            action, _ = runner.call('policy', time_out_sec=task_config['agent']['policy_timeout'],
                                     fallback=env.action_space.sample(), observation=obs)
            obs, reward, terminated, truncated, info = env.step(action)
            place_states = {k: v for k, v in info['status'].items()}

        # tools/local_eval.py の run_one_scene() と同じ status 判定
        # (sudden death かどうかの判定に使う)。
        if env.stream_manager.is_empty():
            episode_status = 'The packing has been completed successfully.'
        else:
            failed = [k for k, v in place_states.items() if v is False]
            episode_status = f'Stopped in the middle. Did not satisfy {failed}' if failed \
                else 'Stopped in the middle.'

        containers = env.container_manager.containers
        scorer = Scorer(client=env.client, config=task_config)
        metrics = scorer.evaluate(containers, env.num_total_items)

        pairs = scorer._find_stacking_pairs(containers)
        all_items = [item for c in containers for item in c.packed_items]
        n_prioritized = sum(1 for it in all_items if it.is_prioritized)
        n_soft = sum(1 for it in all_items if it.is_soft)
        n_prio_crushed = sum(1 for b, t in pairs if b.is_prioritized and not t.is_prioritized)
        n_soft_crushed = sum(1 for b, t in pairs if b.is_soft and not t.is_soft)

        result = {
            'status': 'ok',
            'episode_status': episode_status,
            'total_items': env.num_total_items,
            'n_placed': len(all_items),
            'n_prioritized_placed': n_prioritized,
            'n_soft_placed': n_soft,
            'n_stacking_pairs': len(pairs),
            'n_prio_crushed_pairs': n_prio_crushed,
            'n_soft_crushed_pairs': n_soft_crushed,
            'placement_score': metrics['placement_score'],
            'soft_item_score': metrics['soft_item_score'],
            'fill_score': metrics['fill_score'],
        }
        if include_geo:
            geo = []
            for c in containers:
                for it in c.packed_items:
                    pos, orn = it.get_pose(env.client)
                    if pos is None:
                        continue
                    geo.append({
                        'index': it.index, 'pos': list(pos), 'orn': list(orn) if orn else None,
                        'length': it.length, 'width': it.width, 'height': it.height,
                        'is_prioritized': it.is_prioritized, 'is_soft': it.is_soft,
                        'container_index': c.index,
                    })
            result['geo'] = geo
        return result
    except Exception:
        return {'status': f'error: {traceback.format_exc().splitlines()[-1]}'}
    finally:
        try:
            env.close()
        except Exception:
            pass
        if runner is not None:
            try:
                runner.close()
            except Exception:
                pass


def main():
    args = parse_args()
    module_path = args.module_path
    agent_module = '.'.join(module_path.split('/')) + 'agent'

    scenes = sorted(glob.glob(args.config_path))
    results = {}
    for cp in scenes:
        with open(cp) as f:
            config = json.load(f)
        for task_id, task_config in config.items():
            label = f'{os.path.basename(cp)}::{task_id}'
            t0 = time.time()
            r = run_one_scene_diag(task_config, module_path, agent_module, not args.no_geo)
            dt = time.time() - t0
            r['elapsed_s'] = dt
            results[label] = r
            print(f'[{label}] status={r["status"]} episode_status={r.get("episode_status")} '
                  f'n_placed={r.get("n_placed")}/{r.get("total_items")} '
                  f'prio_placed={r.get("n_prioritized_placed")} soft_placed={r.get("n_soft_placed")} '
                  f'pairs={r.get("n_stacking_pairs")} prio_crushed={r.get("n_prio_crushed_pairs")} '
                  f'soft_crushed={r.get("n_soft_crushed_pairs")} '
                  f'placement={r.get("placement_score")} soft_item={r.get("soft_item_score")} '
                  f'({dt:.1f}s)', flush=True)

    with open(args.out, 'w') as f:
        json.dump(results, f, indent=1)
    print(f'\n書き出し: {args.out}')

    n_stopped = sum(1 for r in results.values()
                     if r.get('episode_status', '').startswith('Stopped'))
    n_violation = sum(1 for r in results.values()
                       if (r.get('placement_score') is not None and r['placement_score'] < 100.0)
                       or (r.get('soft_item_score') is not None and r['soft_item_score'] < 100.0))
    print(f'途中終了(sudden death)シーン数: {n_stopped}/{len(results)}')
    print(f'placement_score/soft_item_scoreが100未満のシーン数: {n_violation}/{len(results)}')


if __name__ == '__main__':
    main()
