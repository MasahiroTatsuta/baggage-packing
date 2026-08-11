"""
tools/phase29_cmp.py

Phase29: 計測JSON2本を、共通するシーンだけで突き合わせる小道具。
tools/phase26_analyze.py は26シーン揃っている前提なので、部分集合の比較にはこちらを使う。
(判定式は同じ: 指標ごとの差、fill_strict/fill_loose のシーン別差分)
"""
import argparse
import json

METRICS = ['fill_strict', 'fill_loose', 'cog_score', 'stability_score',
           'placement_score', 'soft_item_score']


def get(d, label, key):
    v = d['per_scene'][label][key]
    return v['mean'] if isinstance(v, dict) else v


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--before', required=True)
    ap.add_argument('--after', required=True)
    args = ap.parse_args()
    b = json.load(open(args.before))
    a = json.load(open(args.after))
    labels = [l for l in a['scene_labels'] if l in b['per_scene']]
    print(f'共通シーン {len(labels)}件  before={args.before}  after={args.after}\n')
    print(f"{'scene':34s} " + ' '.join(f'{m:>14s}' for m in ('fill_strict', 'fill_loose')))
    n_same = 0
    for l in labels:
        ds = get(a, l, 'fill_strict') - get(b, l, 'fill_strict')
        dl = get(a, l, 'fill_loose') - get(b, l, 'fill_loose')
        if abs(ds) < 1e-9 and abs(dl) < 1e-9:
            n_same += 1
        print(f'{l:34s} {get(b, l, "fill_strict"):6.2f}->{get(a, l, "fill_strict"):6.2f} '
              f'({ds:+5.2f}) {get(b, l, "fill_loose"):6.2f}->{get(a, l, "fill_loose"):6.2f} ({dl:+5.2f})')
    print(f'\n  完全一致(ビット単位)のシーン: {n_same}/{len(labels)}')
    for m in METRICS:
        db = sum(get(b, l, m) for l in labels) / len(labels)
        da = sum(get(a, l, m) for l in labels) / len(labels)
        print(f'  {m:16s} {db:7.3f} -> {da:7.3f} ({da - db:+.3f})')


if __name__ == '__main__':
    main()
