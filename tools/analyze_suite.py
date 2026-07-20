"""
tools/analyze_suite.py

Phase10 タスク3: スイート計測結果(tools/measure.py の出力JSON)を tools/suite_manifest.json の
条件タグで層別集計し、「どの条件で弱いか」をワースト順に特定する。

出力:
  - シーン別 fill/5指標 (mean±std)
  - 条件軸別(pattern / initial_state / n_containers / shelf / prio_container / dist)の平均fill・5指標
  - fillワースト順のシーン一覧
  - 「初期状態=積付済み vs 空」など、盲点仮説に直結する対比

src/ agents/ は読まない(集計のみ)。
実行: PYTHONPATH=. .venv/bin/python tools/analyze_suite.py --measure <out.json> --manifest tools/suite_manifest.json
"""
import argparse
import json
import math

METRIC_KEYS = ['fill_score', 'cog_score', 'stability_score', 'placement_score', 'soft_item_score']
SHORT = {'fill_score': 'fill', 'cog_score': 'cog', 'stability_score': 'stab',
         'placement_score': 'place', 'soft_item_score': 'soft'}


def _agg(vals):
    vals = [v for v in vals if v is not None and not (isinstance(v, float) and math.isnan(v))]
    if not vals:
        return (float('nan'), float('nan'), 0)
    mean = sum(vals) / len(vals)
    if len(vals) == 1:
        return (mean, 0.0, 1)
    var = sum((v - mean) ** 2 for v in vals) / (len(vals) - 1)
    return (mean, math.sqrt(var), len(vals))


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--measure', required=True)
    ap.add_argument('--manifest', default='tools/suite_manifest.json')
    args = ap.parse_args()

    with open(args.measure) as f:
        meas = json.load(f)
    with open(args.manifest) as f:
        manifest = json.load(f)

    per_scene = meas['per_scene']
    # scene -> {metric: mean}, tags
    rows = []
    for label, tags in manifest.items():
        if label not in per_scene:
            continue
        ps = per_scene[label]
        row = {'label': label, 'tags': tags}
        for k in METRIC_KEYS:
            row[k] = ps[k]['mean']
            row[k + '_std'] = ps[k]['std']
        row['placed_abs'] = ps.get('num_placed_items_abs', {}).get('mean', float('nan'))
        row['total'] = tags.get('total_items', 0)
        rows.append(row)

    # --- シーン別テーブル(fill昇順=悪い順) ---
    print('=== シーン別(fillワースト順, mean±std) ===')
    hdr = f'{"scene":34s} {"pat":3s} {"init":9s} {"c":1s} | ' + ' '.join(f'{SHORT[k]:>13s}' for k in METRIC_KEYS)
    print(hdr)
    print('-' * len(hdr))
    for row in sorted(rows, key=lambda r: r['fill_score']):
        t = row['tags']
        cells = ' '.join(f'{row[k]:6.2f}±{row[k+"_std"]:4.2f}' for k in METRIC_KEYS)
        name = row['label'].replace('suite_', '').replace('.json::000', '')
        print(f'{name:34s} {t["pattern"]:3s} {t["initial_state"]:9s} {t["n_containers"]:1d} | {cells}')

    # --- 条件軸別の集計 ---
    def stratify(axis_fn, title):
        groups = {}
        for row in rows:
            key = axis_fn(row['tags'])
            groups.setdefault(key, []).append(row)
        print(f'\n=== {title} 別 平均 ===')
        print(f'{"group":22s} {"n":>3s} | ' + ' '.join(f'{SHORT[k]:>8s}' for k in METRIC_KEYS))
        for key in sorted(groups, key=lambda g: _agg([r['fill_score'] for r in groups[g]])[0]):
            grp = groups[key]
            cells = []
            for k in METRIC_KEYS:
                m, s, _ = _agg([r[k] for r in grp])
                cells.append(f'{m:8.2f}')
            print(f'{str(key):22s} {len(grp):3d} | ' + ' '.join(cells))

    stratify(lambda t: t['pattern'], 'パターン')
    stratify(lambda t: t['initial_state'], '初期状態(盲点仮説)')
    stratify(lambda t: f'{t["n_containers"]}container', 'コンテナ台数')
    stratify(lambda t: 'shelf' if t['shelf'] else 'no_shelf', '棚')
    stratify(lambda t: 'prio_cont' if t['prio_container'] else 'no_prio', '優先コンテナ')
    stratify(lambda t: t['dist'], '分布')

    # --- 全体平均 ---
    print('\n=== スイート全体平均(全シーン単純平均) ===')
    for k in METRIC_KEYS:
        m, s, n = _agg([r[k] for r in rows])
        print(f'  {k:18s}: {m:6.2f}  (シーン間std {s:.2f}, n={n})')


if __name__ == '__main__':
    main()
