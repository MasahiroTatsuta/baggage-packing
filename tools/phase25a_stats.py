"""
tools/phase25a_stats.py

Phase25a: 「26シーン平均の改善」だけでなく、シーン別効果のばらつきを考慮した採否判定を
行うための統計ツール。

before/after (tools/measure_regime.py の出力JSON、同一26シーン・同一順序) を突き合わせ、
シーンごとのペア差分 d_i = after_i - before_i を作り、以下を算出する:

  - 26シーン平均の差 (mean of d_i)
  - シーン別効果の標準偏差 σ = std(d_i) (n-1で割る不偏分散)
  - 標準誤差 SE = σ / sqrt(n)
  - t値 = mean(d_i) / SE  (対応のあるt検定、自由度 n-1)

「ノイズ床±0.90」(tools/compare_suite.py の NOISE_FLOOR)は同一シーンの再実行ばらつき
(repeats間のstd)であり、別シーン集合への一般化誤差ではない。本ツールが出すσ/SE/tは
シーン間のばらつきを直接見るため、両者は別物として併記する。

実行:
    PYTHONPATH=. .venv/bin/python tools/phase25a_stats.py \
        --before results/xxx_before.json --after results/xxx_after.json \
        --metric composite_strict --title "UNITS_PER_SEC 1.05e7 -> 1.55e7"

Phase37(ステップ1-2): 主KPIを合成スコア(composite_strict、tools/measure_regime.pyの
COMPOSITE_WEIGHTS参照)に変更した。--metric fill_strict で従来どおりの判定にも戻せる。
"""
import argparse
import json
import math


def _mean(vals):
    return sum(vals) / len(vals)


def _std(vals):
    n = len(vals)
    if n < 2:
        return 0.0
    m = _mean(vals)
    var = sum((v - m) ** 2 for v in vals) / (n - 1)
    return math.sqrt(var)


def _scene_metric(data, metric):
    return {label: st[metric]['mean'] for label, st in data['per_scene'].items()}


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--before', required=True)
    ap.add_argument('--after', required=True)
    ap.add_argument('--metric', default='composite_strict')
    ap.add_argument('--title', default='')
    ap.add_argument('--adopt-t', type=float, default=2.0, help='採用基準のt値の目安(既定2.0)')
    args = ap.parse_args()

    with open(args.before) as f:
        before = json.load(f)
    with open(args.after) as f:
        after = json.load(f)

    b = _scene_metric(before, args.metric)
    a = _scene_metric(after, args.metric)
    labels = [l for l in before['scene_labels'] if l in a and l in b]
    n = len(labels)

    diffs = [a[l] - b[l] for l in labels]
    mean_d = _mean(diffs)
    sigma = _std(diffs)
    se = sigma / math.sqrt(n) if n > 0 else float('nan')
    t = mean_d / se if se > 0 else float('inf') if mean_d != 0 else 0.0

    mean_before = _mean([b[l] for l in labels])
    mean_after = _mean([a[l] for l in labels])

    print(f'===== {args.title or "before vs after"} (metric={args.metric}, n={n}) =====')
    print(f'before: {before.get("label")}  mean={mean_before:.3f}')
    print(f'after : {after.get("label")}  mean={mean_after:.3f}')
    print()
    print(f'26シーン平均差分        : {mean_d:+.3f}')
    print(f'シーン別効果の標準偏差σ  : {sigma:.3f}')
    print(f'標準誤差 SE=σ/√{n:<2d}     : {se:.3f}')
    print(f't値 (mean/SE)          : {t:.3f}')
    verdict = '採用相当 (t > {:.1f})'.format(args.adopt_t) if abs(t) > args.adopt_t else '不採用相当 (|t| <= {:.1f})'.format(args.adopt_t)
    print(f'判定                    : {verdict}')
    print()
    print('--- シーン別差分(降順) ---')
    for l, d in sorted(zip(labels, diffs), key=lambda x: -x[1]):
        name = l.replace('suite_', '').replace('.json::000', '')
        print(f'  {name:34s} {b[l]:7.2f} -> {a[l]:7.2f}  ({d:+6.2f})')

    print()
    print('(注) この σ/SE/t はシーン間(=別シーン集合への一般化)のばらつきを見ている。')
    print('     tools/compare_suite.py のノイズ床±0.90は同一シーンの再実行ばらつきで別物。')


if __name__ == '__main__':
    main()
