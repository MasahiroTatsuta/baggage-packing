"""
tools/measure_regime.py

Phase12 ターゲット0: fill計上レジーム(inclusion_margin)の同定。

同一エピソード(同一の配置結果)に対して、fill_score だけを2つの inclusion_margin
(厳: -0.005 / 緩: +0.01) で再計算し、どちらの margin が実際の提出結果(public score)
と整合するかを判定するための計測ツール。

重要: config の inclusion_margin は変更しない(config を変えるとエピソード中の
validator の合否判定=物理的に何が置けるか自体が変わってしまい、比較が汚染される)。
代わりに、エピソード終了後の最終コンテナ状態に対して、
`src/ground_handling/evaluator.py Evaluator.calculate_fill_rate` (非破壊・幾何判定のみ、
stepSimulationを呼ばない)を margin違いで2回呼ぶだけで済む。これにより1エピソード
=1回のシミュレーションで両レジームのfillが同時に得られる(2倍計測にならない)。

他4指標(cog/placement/soft/stability)は通常どおり tools/scorer.Scorer で1回だけ算出する
(stability は破壊的なので必ず最後)。

src/ と agents/ は一切変更しない。

実行例:
    PYTHONPATH=. .venv/bin/python tools/measure_regime.py \
        --config-path configs/gen/suite_*.json --module-path _hist/p9/ --repeats 1 \
        --out results/phase12_regime_p9.json --label p9
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
from src.ground_handling.evaluator import Evaluator
from src.ground_handling.runner import TimedAgentRunner
from tools.scorer import Scorer

METRIC_KEYS = ['fill_strict', 'fill_loose', 'cog_score', 'stability_score', 'placement_score', 'soft_item_score']
STRICT_MARGIN = -0.005
LOOSE_MARGIN = 0.01


def parse_args():
    parser = argparse.ArgumentParser()
    parser.add_argument('--config-path', nargs='+', required=True)
    parser.add_argument('--module-path', default='agents/mysolver/')
    parser.add_argument('--repeats', type=int, default=1)
    parser.add_argument('--optimize-budget', type=float, default=None)
    parser.add_argument('--out', required=True)
    parser.add_argument('--label', default=None)
    return parser.parse_args()


def _mean_std(values):
    n = len(values)
    if n == 0:
        return (float('nan'), float('nan'))
    mean = sum(values) / n
    if n == 1:
        return (mean, 0.0)
    var = sum((v - mean) ** 2 for v in values) / (n - 1)
    return (mean, math.sqrt(var))


def run_one_scene_with_regimes(task_config: dict, module_path: str, agent_module: str) -> dict:
    agent_factory = AgentFactory(module_name=agent_module, class_name='Agent', module_path=module_path)
    env = GroundHandlingEnv(config=task_config, verbose=False, render_mode=None)
    runner = None
    optimize_time = 0.0
    policy_time = 0.0
    place_states: dict = {}

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
            optimized_order, opt_elapsed = runner.call(
                'optimize', time_out_sec=task_config['agent']['optimization_timeout'],
                fallback=list(env.stream_manager.all_indices), item_list=item_list)
            optimize_time = max(optimize_time, opt_elapsed)
            if not env.set_item_order(optimized_order):
                return {'status': 'Invalid index or data type for optimization', 'metrics': None,
                        'optimize_time': optimize_time, 'policy_time': policy_time}

        env.reset_item_stream()
        obs, info = env.reset(seed=42)

        terminated = False
        truncated = False
        while not terminated and not truncated:
            action, policy_elapsed = runner.call('policy', time_out_sec=task_config['agent']['policy_timeout'],
                                                  fallback=env.action_space.sample(), observation=obs)
            policy_time = max(policy_time, policy_elapsed)
            obs, reward, terminated, truncated, info = env.step(action)
            place_states = {k: v for k, v in info['status'].items()}

        if env.stream_manager.is_empty():
            status = 'The packing has been completed successfully.'
        else:
            failed = [k for k, v in place_states.items() if v is False]
            status = f'Stopped in the middle. Did not satisfy {failed}' if failed else 'Stopped in the middle.'

        containers = env.container_manager.containers

        # 非破壊: 同一最終状態に対し margin違いでfillを2回計算(stabilityより必ず前)
        strict_eval = Evaluator(client=env.client, config={'inclusion_margin': STRICT_MARGIN})
        loose_eval = Evaluator(client=env.client, config={'inclusion_margin': LOOSE_MARGIN})
        fill_strict, out_strict = strict_eval.calculate_fill_rate(containers)
        fill_loose, out_loose = loose_eval.calculate_fill_rate(containers)

        scorer = Scorer(client=env.client, config=task_config)
        num_packed_items = sum(len(c.packed_items) for c in containers)
        placement_score = scorer.calculate_placement_score(containers)
        soft_item_score = scorer.calculate_soft_item_score(containers)
        cog_score = scorer.calculate_cog_score(containers)
        stability_score = scorer.calculate_stability_score(containers)  # 破壊的: 必ず最後

        metrics = {
            'fill_strict': fill_strict,
            'fill_loose': fill_loose,
            'cog_score': cog_score,
            'stability_score': stability_score,
            'placement_score': placement_score,
            'soft_item_score': soft_item_score,
            'num_placed_items_abs': num_packed_items,
            'total_items': env.num_total_items,
            'fill_counted_ratio_strict': ((num_packed_items - len(out_strict)) / num_packed_items
                                           if num_packed_items else 1.0),
            'fill_counted_ratio_loose': ((num_packed_items - len(out_loose)) / num_packed_items
                                          if num_packed_items else 1.0),
        }
        return {'status': status, 'metrics': metrics, 'optimize_time': optimize_time, 'policy_time': policy_time}

    except Exception:
        return {'status': f'error: {traceback.format_exc().splitlines()[-1]}', 'metrics': None,
                'optimize_time': optimize_time, 'policy_time': policy_time}
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
    if args.optimize_budget is not None:
        os.environ['MYSOLVER_OPTIMIZE_BUDGET'] = str(args.optimize_budget)
        print(f'[measure_regime] MYSOLVER_OPTIMIZE_BUDGET={args.optimize_budget}s')

    module_path = args.module_path
    agent_module = '.'.join(module_path.split('/')) + 'agent'

    config_paths = []
    for pattern in args.config_path:
        matched = sorted(glob.glob(pattern))
        config_paths.extend(matched if matched else [pattern])

    scene_specs = []
    for config_path in config_paths:
        with open(config_path) as f:
            config = json.load(f)
        for task_id in config.keys():
            scene_specs.append((config_path, task_id, config[task_id]))

    scene_labels = [f'{os.path.basename(cp)}::{tid}' for cp, tid, _ in scene_specs]
    samples = {lab: {k: [] for k in METRIC_KEYS} for lab in scene_labels}
    samples_status = {lab: [] for lab in scene_labels}
    # Phase25a: optimize()/policy()の実測壁時計時間をシーン別に記録する(安全弁発火の有無・
    # 決定性を事後確認するため)。metrics算出とは独立にrun_one_scene_with_regimesの戻り値
    # から直接拾う(既に計算済みで捨てられていた値を保存するだけで、計測経路には影響しない)。
    samples_time = {lab: {'optimize_time': [], 'policy_time': []} for lab in scene_labels}
    per_run_suite_avg = []

    t_start = time.time()
    for r in range(args.repeats):
        print(f'\n===== repeat {r + 1}/{args.repeats} =====')
        run_metric_lists = {k: [] for k in METRIC_KEYS}
        for (config_path, task_id, task_config), label in zip(scene_specs, scene_labels):
            t0 = time.time()
            result = run_one_scene_with_regimes(task_config, module_path, agent_module)
            m = result['metrics']
            samples_status[label].append(result['status'])
            samples_time[label]['optimize_time'].append(result['optimize_time'])
            samples_time[label]['policy_time'].append(result['policy_time'])
            if m is None:
                print(f'  [{label}] ERROR/None metrics: {result["status"]}')
                continue
            for k in METRIC_KEYS:
                if k in m:
                    samples[label][k].append(m[k])
                    run_metric_lists[k].append(m[k])
            print(f'  [{label}] fill_strict={m["fill_strict"]:.2f} fill_loose={m["fill_loose"]:.2f} '
                  f'placed={m.get("num_placed_items_abs", 0)}/{m.get("total_items", 0)} '
                  f'({time.time() - t0:.1f}s)')
        suite_avg = {k: (sum(vals) / len(vals) if vals else float('nan'))
                     for k, vals in run_metric_lists.items()}
        per_run_suite_avg.append(suite_avg)
        print(f'  -- suite avg: fill_strict={suite_avg["fill_strict"]:.2f} fill_loose={suite_avg["fill_loose"]:.2f}')

    per_scene = {}
    for label in scene_labels:
        per_scene[label] = {}
        for k in METRIC_KEYS:
            mean, std = _mean_std(samples[label][k])
            per_scene[label][k] = {'mean': mean, 'std': std, 'n': len(samples[label][k])}
        per_scene[label]['statuses'] = samples_status[label]
        for tk in ('optimize_time', 'policy_time'):
            vals = samples_time[label][tk]
            mean, std = _mean_std(vals)
            per_scene[label][tk] = {'mean': mean, 'std': std, 'max': max(vals) if vals else float('nan')}

    suite_stats = {}
    for k in METRIC_KEYS:
        vals = [s[k] for s in per_run_suite_avg if not math.isnan(s[k])]
        mean, std = _mean_std(vals)
        suite_stats[k] = {'mean': mean, 'std': std, 'n': len(vals)}

    # 安全弁(HARD_WALL_LIMIT=165s / policy 8s)の発火有無を事後確認するための全シーン最大値。
    all_optimize_times = [v for lab in scene_labels for v in samples_time[lab]['optimize_time']]
    all_policy_times = [v for lab in scene_labels for v in samples_time[lab]['policy_time']]
    time_stats = {
        'optimize_time_max': max(all_optimize_times) if all_optimize_times else float('nan'),
        'optimize_time_mean': sum(all_optimize_times) / len(all_optimize_times) if all_optimize_times else float('nan'),
        'policy_time_max': max(all_policy_times) if all_policy_times else float('nan'),
        'policy_time_mean': sum(all_policy_times) / len(all_policy_times) if all_policy_times else float('nan'),
    }

    out = {
        'label': args.label,
        'module_path': module_path,
        'repeats': args.repeats,
        'strict_margin': STRICT_MARGIN,
        'loose_margin': LOOSE_MARGIN,
        'scene_labels': scene_labels,
        'per_scene': per_scene,
        'per_run_suite_avg': per_run_suite_avg,
        'suite_stats': suite_stats,
        'time_stats': time_stats,
        'elapsed_sec': time.time() - t_start,
    }
    with open(args.out, 'w') as f:
        json.dump(out, f, ensure_ascii=False, indent=2)

    print('\n========== SUMMARY ==========')
    print(f'fill_strict (margin={STRICT_MARGIN}): {suite_stats["fill_strict"]["mean"]:.2f} '
          f'(±{suite_stats["fill_strict"]["std"]:.2f})')
    print(f'fill_loose  (margin={LOOSE_MARGIN}): {suite_stats["fill_loose"]["mean"]:.2f} '
          f'(±{suite_stats["fill_loose"]["std"]:.2f})')
    print(f'optimize_time: mean={time_stats["optimize_time_mean"]:.2f}s max={time_stats["optimize_time_max"]:.2f}s '
          f'(HARD_WALL_LIMIT=165s)')
    print(f'policy_time  : mean={time_stats["policy_time_mean"]:.3f}s max={time_stats["policy_time_max"]:.2f}s '
          f'(timeout=8s)')
    print(f'\n出力: {args.out} (総所要 {out["elapsed_sec"]:.0f}s)')


if __name__ == '__main__':
    main()
