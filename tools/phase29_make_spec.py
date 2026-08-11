"""
tools/phase29_make_spec.py

Phase29: `tools/phase29_order_eval.py` に食わせる spec(明示順序のリスト)を作る。

修正可能な衝突があったシーンについて
  base            : いま build_order が採用している順序
  advance_before  : X を全ブロッカーより前へ(本フェーズの主役の多手移動)
  delay_blockers  : ブロッカー群を X の直後へ
  advance_to_front: X を先頭へ
を並べる。これを本物の env で評価して、**代理(影シミュレータ)の判断が採点指標と
一致しているか**を直接確かめる。
"""
import argparse
import json
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from agents.mysolver import geometry as geo
from agents.mysolver import ordering as O
from agents.mysolver import reach as R
from tools.phase29_blockers import winner_order
from tools.phase29_repair_probe import setup, run_order, _advance_to_front


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--cand', nargs='+', required=True)
    ap.add_argument('--labels', nargs='+', required=True)
    ap.add_argument('--voxel', type=float, default=0.05)
    ap.add_argument('--out', required=True)
    args = ap.parse_args()

    cands = []
    for f in args.cand:
        cands.extend(json.load(open(f)))
    by_label = {c['label']: c for c in cands}

    spec = []
    for label in args.labels:
        c = by_label[label]
        task, cl, la, items = setup(label)
        ibx = {it['index']: it for it in items}
        prepacked = geo.initial_prepacked_ids(cl)
        tv = sum(cc.get('volume', 0.0) for cc in cl)
        order = winner_order(c, tv)['order']
        spec.append({'label': label, 'name': 'base', 'order': order})
        _, stall = run_order(cl, ibx, order, la, prepacked)
        if not stall.get('stalled'):
            continue
        sb = R.stall_blockers(stall['containers'], stall['pool'], voxel=args.voxel)
        cand = sb['candidate']
        if cand is None:
            continue
        x = cand['item_index']
        bl = cand['result']['blockers']
        for name, fn in (('advance_before', O._advance_before),
                         ('delay_blockers', O._delay_blockers),
                         ('advance_to_front', _advance_to_front)):
            o2 = fn(order, x, bl)
            if o2 is not None and o2 != order:
                spec.append({'label': label, 'name': name, 'order': o2,
                             'x': x, 'blockers': bl})
    json.dump(spec, open(args.out, 'w'), indent=1)
    print(f'{len(spec)} 件を {args.out} へ')
    for s in spec:
        print(f"  {s['label']:28s} {s['name']}")


if __name__ == '__main__':
    main()
