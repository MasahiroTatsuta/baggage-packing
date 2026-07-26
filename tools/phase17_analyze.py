"""
tools/phase17_analyze.py

Phase17 の検証集計:
  - budget 掃引結果(30/60/120/165)から、シーン単位の fill_strict スプレッド(max-min)と
    単調性(隣接予算間で悪化した回数)を before(phase16) / after(phase17) で比較する。
  - 26シーン計測の before/after 差分を両レジームで出す。

実行例:
    PYTHONPATH=. .venv/bin/python tools/phase17_analyze.py sweep \
        --before results/phase16_budget_b{30,60,120,165}.json \
        --after  results/phase17_budget_b{30,60,120,165}.json
"""
import argparse
import json


BUDGETS = [30, 60, 120, 165]


def load(paths):
    out = []
    for p in paths:
        with open(p) as f:
            out.append(json.load(f))
    return out


def sweep_table(runs, key='fill_strict'):
    labels = sorted(runs[0]['per_scene'])
    rows = {}
    for lab in labels:
        vals = []
        for r in runs:
            ps = r['per_scene'].get(lab)
            vals.append(ps[key]['mean'] if ps else float('nan'))
        rows[lab] = vals
    return rows


def summarize(rows, tag):
    spreads = {}
    n_drops = 0
    n_pairs = 0
    for lab, v in rows.items():
        spreads[lab] = max(v) - min(v)
        for a, b in zip(v, v[1:]):
            n_pairs += 1
            if b < a - 1e-9:
                n_drops += 1
    order = sorted(spreads, key=lambda k: -spreads[k])
    print(f'--- {tag} ---')
    print(f'  平均スプレッド {sum(spreads.values())/len(spreads):.2f} / '
          f'最大 {spreads[order[0]]:.2f} ({order[0]}) / '
          f'中央 {sorted(spreads.values())[len(spreads)//2]:.2f}')
    print(f'  隣接予算間で悪化した回数: {n_drops}/{n_pairs}')
    return spreads


def cmd_sweep(args):
    before = load(args.before)
    after = load(args.after)
    for key in ('fill_strict', 'fill_loose'):
        print(f'\n===== {key} =====')
        rb = sweep_table(before, key)
        ra = sweep_table(after, key)
        sb = summarize(rb, 'before (phase16)')
        sa = summarize(ra, 'after  (phase17)')
        print(f'  {"scene":<34}' + ''.join(f'{b:>9}' for b in BUDGETS) + '   spread(B->A)')
        for lab in sorted(rb):
            short = lab.replace('suite_', '').replace('.json::000', '')
            vb = rb[lab]
            va = ra.get(lab, [float("nan")] * 4)
            print(f'  {short:<34}' + ''.join(f'{x:9.2f}' for x in va)
                  + f'   {sb[lab]:5.2f} -> {sa.get(lab, float("nan")):5.2f}')
        print('  集計平均: ' + ' '.join(
            f'b{b}={sum(v[i] for v in ra.values())/len(ra):.2f}' for i, b in enumerate(BUDGETS)))
        print('  (before)  ' + ' '.join(
            f'b{b}={sum(v[i] for v in rb.values())/len(rb):.2f}' for i, b in enumerate(BUDGETS)))


def cmd_suite(args):
    b = json.load(open(args.before))
    a = json.load(open(args.after))
    labels = sorted(set(b['per_scene']) & set(a['per_scene']))
    print(f'{"scene":<34}{"strict B":>10}{"strict A":>10}{"d":>8}'
          f'{"loose B":>10}{"loose A":>10}{"d":>8}')
    for lab in labels:
        sb = b['per_scene'][lab]
        sa = a['per_scene'][lab]
        short = lab.replace('suite_', '').replace('.json::000', '')
        print(f'{short:<34}{sb["fill_strict"]["mean"]:10.2f}{sa["fill_strict"]["mean"]:10.2f}'
              f'{sa["fill_strict"]["mean"]-sb["fill_strict"]["mean"]:+8.2f}'
              f'{sb["fill_loose"]["mean"]:10.2f}{sa["fill_loose"]["mean"]:10.2f}'
              f'{sa["fill_loose"]["mean"]-sb["fill_loose"]["mean"]:+8.2f}')
    for k in ('fill_strict', 'fill_loose', 'cog_score', 'stability_score', 'placement_score',
              'soft_item_score'):
        mb = sum(b['per_scene'][l][k]['mean'] for l in labels) / len(labels)
        ma = sum(a['per_scene'][l][k]['mean'] for l in labels) / len(labels)
        print(f'{k:<34}{mb:10.2f}{ma:10.2f}{ma-mb:+8.2f}')


def main():
    ap = argparse.ArgumentParser()
    sub = ap.add_subparsers(dest='cmd', required=True)
    s = sub.add_parser('sweep')
    s.add_argument('--before', nargs='+', required=True)
    s.add_argument('--after', nargs='+', required=True)
    s.set_defaults(fn=cmd_sweep)
    t = sub.add_parser('suite')
    t.add_argument('--before', required=True)
    t.add_argument('--after', required=True)
    t.set_defaults(fn=cmd_suite)
    args = ap.parse_args()
    args.fn(args)


if __name__ == '__main__':
    main()
