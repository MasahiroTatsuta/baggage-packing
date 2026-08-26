"""Phase67: 支持閾値スイープ用の統合計測ツール。

`tools/diagnose_stacking.py`(fill/placement_A/soft_A/cog + 積み重なりペア)と
`tools/diagnose_stability.py`(stability_score、`MYSOLVER_DIAG_SHAKE_AMPLITUDE`でenv化済み)を
1シーン1ロールアウトに統合し、さらに Phase59 定義B(分母=プール全体、未配置も違反)の
placement_B/soft_Bをその場で計算する。5成分・num_placed_items・確定採点式の合成スコア
(定義A: placement_A/soft_A、定義B: placement_B/soft_B)を1回の実行でまとめて出力する。

`agents/mysolver/planner.py`のPhase67 env化(MYSOLVER_MIN_UNION_SUPPORT_RATIO /
MYSOLVER_MIN_SUPPORT_SPAN_RATIO / MYSOLVER_MAX_SUPPORT_CENTROID_OFFSET /
MYSOLVER_STRICT_SUPPORT_DISABLE)、`tools/diagnose_stability.py`のPhase61 env化
(MYSOLVER_DIAG_SHAKE_AMPLITUDE)はいずれも呼び出し先が読むので、本ツール自体は
env変数を解釈しない(プロセス起動時にexportしておけば自動的に反映される)。

`tools/scorer.py`・`configs/`・既存26シーンのconfigはいずれも変更しない(読み取りのみ)。

実行方法(リポジトリルートで):
    MYSOLVER_DIAG_SHAKE_AMPLITUDE=19.6 PYTHONPATH=. .venv/bin/python tools/phase67_suite_metrics.py \\
        --config-path 'configs/gen/suite_*.json' --out results/phase67_baseline_2G.json

    # sample_config(2タスク)を測る場合
    MYSOLVER_DIAG_SHAKE_AMPLITUDE=19.6 PYTHONPATH=. .venv/bin/python tools/phase67_suite_metrics.py \\
        --config-path 'configs/sample_config.json' --out results/phase67_baseline_2G_sample.json
"""
import argparse
import glob
import json
import math
import os
import sys
import time
import traceback

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from src.ground_handling.agent_factory import AgentFactory
from src.ground_handling.env import GroundHandlingEnv
from src.ground_handling.runner import TimedAgentRunner
from tools.scorer import Scorer
from tools.diagnose_stability import stability_with_item_detail

COMPOSITE_WEIGHTS = {'fill': 2.0, 'cog_score': 1.5, 'stability_score': 1.5,
                     'placement_score': 1.0, 'soft_item_score': 1.0}
COMPOSITE_DENOM = sum(COMPOSITE_WEIGHTS.values())  # 7.0


def composite_score(fill, cog, stability, placement, soft):
    return (COMPOSITE_WEIGHTS['fill'] * fill + COMPOSITE_WEIGHTS['cog_score'] * cog
            + COMPOSITE_WEIGHTS['stability_score'] * stability
            + COMPOSITE_WEIGHTS['placement_score'] * placement
            + COMPOSITE_WEIGHTS['soft_item_score'] * soft) / COMPOSITE_DENOM


def _score(violated, denom):
    if denom <= 0:
        return 100.0
    return min(max(100.0 * (1.0 - violated / denom), 0.0), 100.0)


def parse_args():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('--config-path', default='configs/gen/suite_*.json',
                    help='評価するconfigファイルのglobパターン(既定: 26シーン全件)')
    p.add_argument('--module-path', default='agents/mysolver/')
    p.add_argument('--out', default='/tmp/phase67_suite_metrics.json')
    return p.parse_args()


def run_one_scene(task_config: dict, module_path: str, agent_module: str) -> dict:
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

        if env.stream_manager.is_empty():
            status = 'The packing has been completed successfully.'
        else:
            failed = [k for k, v in place_states.items() if v is False]
            status = f'Stopped in the middle. Did not satisfy {failed}' if failed else 'Stopped in the middle.'

        containers = env.container_manager.containers
        scorer = Scorer(client=env.client, config=task_config)

        # evaluate()と同じ呼び出し順(fill->placement->soft->cog->stability、Phase47規約)
        fill_score, _ = scorer.calculate_fill_score(containers)
        placement_A = scorer.calculate_placement_score(containers)
        soft_A = scorer.calculate_soft_item_score(containers)
        pairs = scorer._find_stacking_pairs(containers)
        cog_score = scorer.calculate_cog_score(containers)
        stab_detail = stability_with_item_detail(scorer, containers)  # 破壊的:必ず最後
        stability_score = stab_detail['stability_score']

        all_items = [item for c in containers for item in c.packed_items]
        num_placed_items = len(all_items)
        n_prio_placed = sum(1 for it in all_items if it.is_prioritized)
        n_soft_placed = sum(1 for it in all_items if it.is_soft)
        n_prio_crushed = sum(1 for b, t in pairs if b.is_prioritized and not t.is_prioritized)
        n_soft_crushed = sum(1 for b, t in pairs if b.is_soft and not t.is_soft)

        # 定義B(Phase59): 分母=プール全体、違反=下敷き or 誤コンテナ or 未配置
        pool_items = task_config['item_stream']['item_list']
        n_prio_pool = sum(1 for it in pool_items if it.get('is_prioritized'))
        n_soft_pool = sum(1 for it in pool_items if it.get('is_soft'))
        container_is_prio = {c['index']: c.get('is_prioritized', False)
                              for c in task_config['containers']['container_list']}
        has_prio_container = any(container_is_prio.values())
        n_wrong = 0
        if has_prio_container:
            for it in all_items:
                if it.is_prioritized and not container_is_prio.get(it.belongs_to, False):
                    n_wrong += 1
        violated_A_p = n_prio_crushed + n_wrong
        placement_B = _score(violated_A_p + (n_prio_pool - n_prio_placed), n_prio_pool)
        soft_B = _score(n_soft_crushed + (n_soft_pool - n_soft_placed), n_soft_pool)

        composite_A = composite_score(fill_score, cog_score, stability_score, placement_A, soft_A)
        composite_B = composite_score(fill_score, cog_score, stability_score, placement_B, soft_B)

        return {
            'status': status,
            'fill_score': fill_score,
            'placement_A': placement_A,
            'soft_A': soft_A,
            'placement_B': placement_B,
            'soft_B': soft_B,
            'cog_score': cog_score,
            'stability_score': stability_score,
            'composite_A': composite_A,
            'composite_B': composite_B,
            'num_placed_items': num_placed_items,
            'total_items': env.num_total_items,
            'n_prio_pool': n_prio_pool, 'n_prio_placed': n_prio_placed,
            'n_prio_crushed': n_prio_crushed, 'n_prio_wrong_container': n_wrong,
            'n_soft_pool': n_soft_pool, 'n_soft_placed': n_soft_placed,
            'n_soft_crushed': n_soft_crushed,
            'mean_disp': stab_detail.get('mean_disp'),
            'mean_energy': stab_detail.get('mean_energy'),
        }
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
    agent_module = '.'.join(args.module_path.split('/')) + 'agent'
    paths = sorted(glob.glob(args.config_path))
    if not paths:
        paths = [args.config_path]
    results = {}
    t_start = time.time()
    for cp in paths:
        tasks = json.load(open(cp))
        for tk, task in tasks.items():
            label = f'{os.path.basename(cp)}::{tk}' if len(tasks) > 1 else os.path.basename(cp)
            t0 = time.perf_counter()
            r = run_one_scene(task, args.module_path, agent_module)
            r['elapsed_sec'] = time.perf_counter() - t0
            results[label] = r
            print(f"[{label}] status={r.get('status')} fill={r.get('fill_score')} "
                  f"stability={r.get('stability_score')} composite_A={r.get('composite_A')} "
                  f"composite_B={r.get('composite_B')} placed={r.get('num_placed_items')}/"
                  f"{r.get('total_items')} ({r['elapsed_sec']:.1f}s)")

    meta = {
        'shake_amplitude': os.environ.get('MYSOLVER_DIAG_SHAKE_AMPLITUDE', '6.0'),
        'min_union_support_ratio': os.environ.get('MYSOLVER_MIN_UNION_SUPPORT_RATIO', '0.55'),
        'min_support_span_ratio': os.environ.get('MYSOLVER_MIN_SUPPORT_SPAN_RATIO', '0.6'),
        'max_support_centroid_offset': os.environ.get('MYSOLVER_MAX_SUPPORT_CENTROID_OFFSET', '0.15'),
        'strict_support_disable': os.environ.get('MYSOLVER_STRICT_SUPPORT_DISABLE', '0'),
    }
    out = {'meta': meta, 'results': results, 'elapsed_sec': time.time() - t_start}
    with open(args.out, 'w') as f:
        json.dump(out, f, ensure_ascii=False, indent=2)
    print(f'\nwrote {args.out} (meta={meta}, 総所要 {out["elapsed_sec"]:.0f}s)')


if __name__ == '__main__':
    main()
