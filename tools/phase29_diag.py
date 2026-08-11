"""
tools/phase29_diag.py

Phase29: 「順序修正で開ける衝突が無い」と出たシーンについて、それが本当なのか
**voxel の粗さ(既定 0.10m)による見落とし**なのかを切り分ける。

行き詰まり時点の X(置けなかった荷物)について、位置の内訳を出す:
  fit          : 箱がそのまま入る位置(空き)
  supported    : そのうち支持がある位置
  open         : そのうち搬入経路が空いている位置(=幾何近似では置けるはず)
  shelf_blocked: 棚が塞いでいる位置(順序では動かせない)
  item_blocked : 既配置荷物だけが塞いでいる位置(= 順序修正の対象)

open>0 なのに planner が置けなかったのなら、原因は搬入経路ではなく planner のより厳しい
合法性判定(支持率・span・重心オフセット等)であり、順序修正の射程外である。
"""
import argparse
import io
import json
import os
import sys
from contextlib import redirect_stdout

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

import numpy as np

from src.ground_handling.env import GroundHandlingEnv
from agents.mysolver import geometry as geo
from agents.mysolver import planner
from agents.mysolver import reach as R
from agents.mysolver import simulate as simulate_mod
from tools.phase29_blockers import winner_order


def diag_item(containers, item, voxel):
    tot = dict(fit=0, supported=0, open=0, shelf_blocked=0, item_blocked=0)
    for cdict in containers:
        masks = R.build_masks(cdict, voxel=voxel)
        if not masks['empty'].any():
            continue
        packed = [it for it in cdict.get('packed_items', [])
                  if it.get('pos') is not None and it.get('orn') is not None]
        obs = [(geo.item_world_aabb(it), int(it['index'])) for it in packed]
        static = list(geo.static_obstacles(cdict))
        for key, ok_sup in R._fit_positions(masks, cdict, item, voxel):
            tot['supported'] += int(ok_sup.sum())
            got = R._sweep_boxes(masks, cdict, key, ok_sup, voxel)
            if got is None:
                continue
            pi, pj, pk, geom = got
            n = pi.shape[0]
            shelf = np.zeros(n, dtype=bool)
            for (oc, oh) in static:
                shelf |= R._hits(geom, oc, oh)
            nobs = np.zeros(n, dtype=np.int32)
            for (oc, oh), _ in obs:
                nobs += R._hits(geom, oc, oh)
            tot['open'] += int(((~shelf) & (nobs == 0)).sum())
            tot['shelf_blocked'] += int(shelf.sum())
            tot['item_blocked'] += int(((~shelf) & (nobs >= 1)).sum())
    return tot


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--cand', nargs='+', required=True)
    ap.add_argument('--labels', nargs='+', default=None)
    ap.add_argument('--voxels', nargs='+', type=float, default=[0.10, 0.05])
    args = ap.parse_args()

    cands = []
    for f in args.cand:
        cands.extend(json.load(open(f)))
    if args.labels:
        cands = [c for c in cands if c['label'] in args.labels]

    for c in cands:
        task = list(json.load(open(f"configs/gen/suite_{c['label']}.json")).values())[0]
        env = GroundHandlingEnv(config=task, verbose=False, render_mode=None)
        try:
            env.reset_settings()
            init = env.get_init_states()
            container_list = init['container_list']
            lookahead = init['lookahead_k']
            items = env.get_info_for_optimization()
        finally:
            try:
                env.close()
            except Exception:
                pass
        order = winner_order(c, sum(cc.get('volume', 0.0) for cc in container_list))['order']
        stall = {}
        with redirect_stdout(io.StringIO()):
            simulate_mod.simulate_order(
                container_list, {it['index']: it for it in items}, order,
                max(1, int(lookahead or 1)), planner.SearchBudget.from_seconds(600.0),
                prepacked_ids=geo.initial_prepacked_ids(container_list), stall_info=stall)
        if not stall.get('stalled'):
            print(f"{c['label']:28s} 行き詰まらず(全件配置)")
            continue
        for v in args.voxels:
            parts = []
            for it in stall['pool']:
                d = diag_item(stall['containers'], it, v)
                parts.append(f"item{it['index']}: sup={d['supported']} open={d['open']} "
                             f"shelf={d['shelf_blocked']} items={d['item_blocked']}")
            print(f"{c['label']:28s} voxel={v:.3f}  " + ' | '.join(parts), flush=True)


if __name__ == '__main__':
    main()
