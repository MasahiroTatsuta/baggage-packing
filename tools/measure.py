"""
tools/measure.py

Phase10: 測定ノイズの定量化と、任意シーン集合の反復計測の共通土台。

local_eval.run_one_scene を再利用して、指定した (config_path, task_id) の各シーンを
N回ずつ実行し、5指標 + num_placed_items(絶対数/率) + fill_counted_ratio + 処理時間の
サンプルを収集する。出力はシーン別 mean±std に加えて、「シーン集合平均」を1回の実行
(=全シーンを1周)ごとに算出したサンプル列も出す。後者から「シーン集合平均の実行間ばらつき
(=何ptがノイズで、何ptから有意な改善と言えるか)」を直接見積もる。

src/ と agents/mysolver/ は一切変更しない(読み取り専用の計測ツール)。

実行例:
    PYTHONPATH=. .venv/bin/python tools/measure.py \
        --config-path configs/sample_config.json configs/gen/gen_manyitems_patternA.json \
        --module-path agents/mysolver/ --repeats 5 \
        --out /tmp/measure_out.json
"""
import argparse
import json
import math
import os
import sys
import time

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from tools.local_eval import run_one_scene

METRIC_KEYS = ['fill_score', 'cog_score', 'stability_score', 'placement_score', 'soft_item_score']
EXTRA_KEYS = ['num_placed_items', 'num_placed_items_abs', 'total_items', 'fill_counted_ratio']


def parse_args():
    parser = argparse.ArgumentParser()
    parser.add_argument('--config-path', nargs='+', required=True)
    parser.add_argument('--module-path', default='agents/mysolver/')
    parser.add_argument('--repeats', type=int, default=5)
    parser.add_argument('--optimize-budget', type=float, default=None,
                         help='開発用にoptimize予算を短縮(未指定なら本番相当の30s)')
    parser.add_argument('--out', required=True, help='結果JSONの出力先')
    parser.add_argument('--label', default=None, help='この計測セットの識別名(任意)')
    return parser.parse_args()


def _mean_std(values: list[float]) -> tuple[float, float]:
    n = len(values)
    if n == 0:
        return (float('nan'), float('nan'))
    mean = sum(values) / n
    if n == 1:
        return (mean, 0.0)
    var = sum((v - mean) ** 2 for v in values) / (n - 1)   # 標本標準偏差(不偏)
    return (mean, math.sqrt(var))


def main():
    args = parse_args()
    if args.optimize_budget is not None:
        os.environ['MYSOLVER_OPTIMIZE_BUDGET'] = str(args.optimize_budget)
        print(f'[measure] MYSOLVER_OPTIMIZE_BUDGET={args.optimize_budget}s')

    module_path = args.module_path
    agent_module = '.'.join(module_path.split('/')) + 'agent'

    # (config_path, task_id) を展開
    scene_specs = []
    for config_path in args.config_path:
        with open(config_path) as f:
            config = json.load(f)
        for task_id in config.keys():
            scene_specs.append((config_path, task_id, config[task_id]))

    scene_labels = [f'{os.path.basename(cp)}::{tid}' for cp, tid, _ in scene_specs]
    # samples[scene_label][metric] = [値, ...]  (repeats個)
    samples: dict[str, dict[str, list]] = {lab: {k: [] for k in METRIC_KEYS + EXTRA_KEYS}
                                            for lab in scene_labels}
    samples_status: dict[str, list] = {lab: [] for lab in scene_labels}
    # 各repeatごとの「シーン集合平均(5指標)」を記録
    per_run_suite_avg: list[dict[str, float]] = []

    t_start = time.time()
    for r in range(args.repeats):
        print(f'\n===== repeat {r + 1}/{args.repeats} =====')
        run_metric_lists: dict[str, list] = {k: [] for k in METRIC_KEYS}
        for (config_path, task_id, task_config), label in zip(scene_specs, scene_labels):
            t0 = time.time()
            result = run_one_scene(task_config, module_path, agent_module, None, False)
            m = result['metrics']
            status = result['status']
            samples_status[label].append(status)
            if m is None:
                print(f'  [{label}] ERROR/None metrics: {status}')
                continue
            for k in METRIC_KEYS + EXTRA_KEYS:
                if k in m:
                    samples[label][k].append(m[k])
            for k in METRIC_KEYS:
                run_metric_lists[k].append(m[k])
            print(f'  [{label}] fill={m["fill_score"]:.2f} cog={m["cog_score"]:.2f} '
                  f'stab={m["stability_score"]:.2f} place={m["placement_score"]:.2f} '
                  f'soft={m["soft_item_score"]:.2f} placed={m.get("num_placed_items_abs",0)}/{m.get("total_items",0)} '
                  f'({time.time() - t0:.1f}s)')
        # このrepeatのシーン集合平均
        suite_avg = {k: (sum(vals) / len(vals) if vals else float('nan'))
                     for k, vals in run_metric_lists.items()}
        per_run_suite_avg.append(suite_avg)
        print(f'  -- suite avg: ' + ' '.join(f'{k.split("_")[0]}={suite_avg[k]:.2f}' for k in METRIC_KEYS))

    # 集計
    per_scene = {}
    for label in scene_labels:
        per_scene[label] = {}
        for k in METRIC_KEYS + EXTRA_KEYS:
            mean, std = _mean_std(samples[label][k])
            per_scene[label][k] = {'mean': mean, 'std': std, 'n': len(samples[label][k])}
        per_scene[label]['statuses'] = samples_status[label]

    # シーン集合平均の実行間ばらつき(=ノイズ床)
    suite_stats = {}
    for k in METRIC_KEYS:
        vals = [s[k] for s in per_run_suite_avg if not math.isnan(s[k])]
        mean, std = _mean_std(vals)
        # 「有意」の目安: 2*std(実行間の標準偏差)。suite平均1回計測どうしを比べるときの最小有意差。
        suite_stats[k] = {'mean': mean, 'std': std, 'n': len(vals),
                          'min_significant_diff_2std': 2 * std}

    out = {
        'label': args.label,
        'module_path': module_path,
        'repeats': args.repeats,
        'optimize_budget': args.optimize_budget,
        'scene_labels': scene_labels,
        'per_scene': per_scene,
        'per_run_suite_avg': per_run_suite_avg,
        'suite_stats': suite_stats,
        'elapsed_sec': time.time() - t_start,
    }
    with open(args.out, 'w') as f:
        json.dump(out, f, ensure_ascii=False, indent=2)

    # コンソール要約
    print('\n\n========== SUMMARY (mean ± std, n=repeats) ==========')
    hdr = f'{"scene":32s} | ' + ' | '.join(f'{k.split("_")[0]:>15s}' for k in METRIC_KEYS)
    print(hdr)
    print('-' * len(hdr))
    for label in scene_labels:
        cells = []
        for k in METRIC_KEYS:
            st = per_scene[label][k]
            cells.append(f'{st["mean"]:6.2f}±{st["std"]:4.2f}')
        print(f'{label:32s} | ' + ' | '.join(f'{c:>15s}' for c in cells))
    print('-' * len(hdr))
    cells = []
    for k in METRIC_KEYS:
        st = suite_stats[k]
        cells.append(f'{st["mean"]:6.2f}±{st["std"]:4.2f}')
    print(f'{"SUITE AVG (across scenes)":32s} | ' + ' | '.join(f'{c:>15s}' for c in cells))
    print('\n最小有意差(シーン集合平均, 2×実行間std):')
    for k in METRIC_KEYS:
        print(f'  {k:18s}: ±{suite_stats[k]["min_significant_diff_2std"]:.2f} pt')
    print(f'\n出力: {args.out}  (総所要 {out["elapsed_sec"]:.0f}s)')


if __name__ == '__main__':
    main()
