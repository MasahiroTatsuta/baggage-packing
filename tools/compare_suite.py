"""
tools/compare_suite.py

Phase11: tools/measure.py の出力JSON 2本(before/after)を突き合わせ、
シーン別・層別(tools/suite_manifest.json のタグ)の before/after と差分を表示する。

判定基準は Phase10 タスク1 のノイズ床(シーン集合平均で fill ±0.90 / cog ±0.67 /
stab ±0.09 / place ±0.00 pt)。差がこれ未満なら "noise" と表示する。

実行:
    PYTHONPATH=. .venv/bin/python tools/compare_suite.py \
        --before results/phase11_suite_before.json --after results/phase11_suite_after.json \
        --manifest tools/suite_manifest.json
"""
import argparse
import json
import math

METRIC_KEYS = ['fill_score', 'cog_score', 'stability_score', 'placement_score', 'soft_item_score']
SHORT = {'fill_score': 'fill', 'cog_score': 'cog', 'stability_score': 'stab',
         'placement_score': 'place', 'soft_item_score': 'soft'}
NOISE_FLOOR = {'fill_score': 0.90, 'cog_score': 0.67, 'stability_score': 0.09,
               'placement_score': 0.0, 'soft_item_score': 0.0}

AXES = [
    ('initial_state', ['prepacked', 'empty']),
    ('pattern', ['A', 'B', 'C']),
    ('n_containers', [1, 2]),
    ('shelf', [False, True]),
    ('prio_container', [False, True]),
    ('dist', None),
]


def _mean(vals):
    vals = [v for v in vals if v is not None and not (isinstance(v, float) and math.isnan(v))]
    return sum(vals) / len(vals) if vals else float('nan')


def _scene_means(data):
    out = {}
    for label, st in data['per_scene'].items():
        out[label] = {k: st[k]['mean'] for k in METRIC_KEYS}
        for extra in ('num_placed_items_abs', 'total_items', 'fill_counted_ratio'):
            if extra in st:
                out[label][extra] = st[extra]['mean']
    return out


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--before', required=True)
    ap.add_argument('--after', required=True)
    ap.add_argument('--manifest', default=None)
    ap.add_argument('--title', default='')
    args = ap.parse_args()

    with open(args.before) as f:
        before = json.load(f)
    with open(args.after) as f:
        after = json.load(f)
    manifest = None
    if args.manifest:
        with open(args.manifest) as f:
            manifest = json.load(f)

    b = _scene_means(before)
    a = _scene_means(after)
    labels = [l for l in before['scene_labels'] if l in a]

    print(f'===== {args.title or "before vs after"} =====')
    print(f'before: {before.get("label")} ({before.get("module_path")}) repeats={before.get("repeats")}')
    print(f'after : {after.get("label")} ({after.get("module_path")}) repeats={after.get("repeats")}')

    print('\n=== シーン別 (fill / place, before -> after) ===')
    hdr = f'{"scene":36s} | {"fill(before->after, diff)":30s} | {"place(before->after)":24s} | {"placed個数":14s}'
    print(hdr)
    print('-' * len(hdr))
    for l in labels:
        name = l.replace('suite_', '').replace('.json::000', '')
        fb, fa = b[l]['fill_score'], a[l]['fill_score']
        pb, pa = b[l]['placement_score'], a[l]['placement_score']
        nb = b[l].get('num_placed_items_abs', float('nan'))
        na = a[l].get('num_placed_items_abs', float('nan'))
        print(f'{name:36s} | {fb:7.2f} -> {fa:7.2f}  ({fa - fb:+6.2f}) | '
              f'{pb:6.2f} -> {pa:6.2f} ({pa - pb:+6.2f}) | {nb:5.1f} -> {na:5.1f}')

    print('\n=== 補助指標 平均 (before -> after) ===')
    for key, label in (('num_placed_items_abs', '配置個数'), ('fill_counted_ratio', 'fill集計率')):
        vb = [b[l][key] for l in labels if key in b[l]]
        va = [a[l][key] for l in labels if key in a[l]]
        if vb and va:
            print(f'  {label:12s}: {_mean(vb):8.3f} -> {_mean(va):8.3f} ({_mean(va) - _mean(vb):+.3f})')

    print('\n=== 全体平均 ===')
    print(f'{"metric":18s} | {"before":>8s} | {"after":>8s} | {"diff":>8s} | 判定(ノイズ床)')
    for k in METRIC_KEYS:
        mb = _mean([b[l][k] for l in labels])
        ma = _mean([a[l][k] for l in labels])
        d = ma - mb
        verdict = 'noise' if abs(d) < NOISE_FLOOR[k] else ('改善' if d > 0 else '悪化')
        print(f'{SHORT[k]:18s} | {mb:8.2f} | {ma:8.2f} | {d:+8.2f} | {verdict} (±{NOISE_FLOOR[k]:.2f})')

    if manifest:
        print('\n=== 層別平均 (before -> after) ===')
        for axis, order in AXES:
            groups = {}
            for l in labels:
                if l not in manifest:
                    continue
                g = manifest[l][axis]
                groups.setdefault(g, []).append(l)
            keys = [k for k in (order or sorted(groups, key=str)) if k in groups]
            print(f'\n-- {axis} --')
            print(f'{"group":14s} {"n":>3s} | {"fill":>22s} | {"place":>20s} | {"cog":>20s} | {"stab":>18s}')
            for g in keys:
                ls = groups[g]
                cells = []
                for k in ('fill_score', 'placement_score', 'cog_score', 'stability_score'):
                    mb = _mean([b[l][k] for l in ls])
                    ma = _mean([a[l][k] for l in ls])
                    cells.append(f'{mb:6.2f}->{ma:6.2f}({ma - mb:+5.2f})')
                print(f'{str(g):14s} {len(ls):3d} | ' + ' | '.join(cells))


if __name__ == '__main__':
    main()
