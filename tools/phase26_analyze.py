"""
tools/phase26_analyze.py

Phase26(壁積み)のA/B集計。tools/measure_regime.py の出力JSON 2本(同一シーン集合)を
突き合わせ、以下を1度に出す:

  - 指標ごとの26シーン平均(before / after / 差)
  - fill_strict / fill_loose のシーン別差分と、σ・SE・t値(採否基準 t>2)
  - パターンB(optimize無効=決定的シーン)とパターンA/C(optimize有効)の層別集計
    ——Phase26 では両者で効果の符号が逆転したため、この層別が判定の核心になる。
  - 制約(placement>=?, stability>=97, optimize<170s, policy<7s)の確認

σ/SE/t の定義は tools/phase25a_stats.py と同一(対応のあるt検定)。本ツールはそれを
層別・多指標に拡張したもので、数式は変えていない。

実行:
    PYTHONPATH=. .venv/bin/python tools/phase26_analyze.py \
        --before results/phase25a_suite_ups_1.55e7.json \
        --after  results/phase26_suite_wall_q90.json
"""
import argparse
import json
import math

METRICS = ['fill_strict', 'fill_loose', 'cog_score', 'stability_score',
           'placement_score', 'soft_item_score']
# optimize が無効(=決定的)なシーン。planner は strict_support=True で呼ばれ、
# offline の順序事前検証が一切無い。
DETERMINISTIC = ('suite_B01', 'suite_B02', 'suite_B03', 'suite_B04', 'suite_P04')


def _get(d, label, key):
    v = d['per_scene'][label][key]
    return v['mean'] if isinstance(v, dict) else v


def _mean(vals):
    return sum(vals) / len(vals)


def _std(vals):
    n = len(vals)
    if n < 2:
        return 0.0
    m = _mean(vals)
    return math.sqrt(sum((v - m) ** 2 for v in vals) / (n - 1))


def _stats(diffs):
    n = len(diffs)
    m = _mean(diffs)
    s = _std(diffs)
    se = s / math.sqrt(n) if n else 0.0
    t = m / se if se > 0 else 0.0
    return m, s, se, t


def main():
    p = argparse.ArgumentParser()
    p.add_argument('--before', required=True)
    p.add_argument('--after', required=True)
    p.add_argument('--metrics', nargs='+', default=['fill_strict', 'fill_loose'])
    args = p.parse_args()

    a = json.load(open(args.before))
    b = json.load(open(args.after))
    labels = [l for l in b['scene_labels'] if l in a['per_scene']]

    print(f'before: {args.before}  ({a.get("label")})')
    print(f'after : {args.after}  ({b.get("label")})')
    print(f'共通シーン数: {len(labels)}')

    print('\n===== 全指標の平均 =====')
    print(f'{"metric":18s} {"before":>9s} {"after":>9s} {"diff":>9s}')
    for k in METRICS:
        xs = [_get(a, l, k) for l in labels]
        ys = [_get(b, l, k) for l in labels]
        print(f'{k:18s} {_mean(xs):9.3f} {_mean(ys):9.3f} {_mean(ys) - _mean(xs):+9.3f}')

    for k in args.metrics:
        print(f'\n===== {k}: シーン別差分と統計 =====')
        rows = [(l, _get(a, l, k), _get(b, l, k)) for l in labels]
        rows.sort(key=lambda r: r[2] - r[1], reverse=True)
        print(f'{"scene":30s} {"before":>8s} {"after":>8s} {"diff":>8s}')
        for l, x, y in rows:
            mark = ' [B]' if l.startswith(DETERMINISTIC) else ''
            print(f'{l[6:34]:30s} {x:8.2f} {y:8.2f} {y - x:+8.2f}{mark}')

        for name, sel in (('全シーン', lambda l: True),
                          ('パターンB(optimize無効/決定的)', lambda l: l.startswith(DETERMINISTIC)),
                          ('パターンA/C(optimize有効)', lambda l: not l.startswith(DETERMINISTIC))):
            d = [_get(b, l, k) - _get(a, l, k) for l in labels if sel(l)]
            if not d:
                continue
            m, s, se, t = _stats(d)
            verdict = '採用相当 (|t| > 2)' if abs(t) > 2.0 else '不採用相当 (|t| <= 2)'
            print(f'  [{name}] n={len(d):2d}  mean={m:+7.3f}  σ={s:6.3f}  SE={se:6.3f}  '
                  f't={t:+6.3f}  -> {verdict}')

    print('\n===== 制約確認 =====')
    for name, d in (('before', a), ('after', b)):
        stab = [(l, _get(d, l, 'stability_score')) for l in labels]
        place = [(l, _get(d, l, 'placement_score')) for l in labels]
        soft = [(l, _get(d, l, 'soft_item_score')) for l in labels]
        opt = [(l, d['per_scene'][l]['optimize_time']['max']
                if isinstance(d['per_scene'][l].get('optimize_time'), dict) else 0.0) for l in labels]
        pol = [(l, d['per_scene'][l]['policy_time']['max']
                if isinstance(d['per_scene'][l].get('policy_time'), dict) else 0.0) for l in labels]
        worst_stab = min(stab, key=lambda r: r[1])
        worst_place = min(place, key=lambda r: r[1])
        worst_soft = min(soft, key=lambda r: r[1])
        worst_opt = max(opt, key=lambda r: r[1])
        worst_pol = max(pol, key=lambda r: r[1])
        print(f'[{name}] stability最小 {worst_stab[1]:.2f} ({worst_stab[0][6:]}) / '
              f'placement最小 {worst_place[1]:.2f} ({worst_place[0][6:]}) / '
              f'soft最小 {worst_soft[1]:.2f}')
        print(f'         optimize最大 {worst_opt[1]:.2f}s ({worst_opt[0][6:]}) / '
              f'policy最大 {worst_pol[1]:.2f}s ({worst_pol[0][6:]})')


if __name__ == '__main__':
    main()
