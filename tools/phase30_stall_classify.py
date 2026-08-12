"""
tools/phase30_stall_classify.py

Phase30 計測1・計測2: 「支持付きで収まる位置がゼロ」の内訳を
 (A) 幾何的にそもそも入らない  vs  (B) 幾何的には入るが支持が無い
に切り分け(計測1)、さらに行き詰まり原因を相互排他な5分類
 (i) 幾何で入らない / (ii) 収まるが支持が無い / (iii) 支持もあるが既配置荷物が搬入経路を塞ぐ
 (iv) 支持もあるが棚が塞ぐ / (v) 置ける(=行き詰まっていない/別の理由)
に数え直す(計測2)。

agents/mysolver/ は一切変更しない。既存の `agents.mysolver.reach` の判定式
(`_pad3` / `_fit_positions` / `_sweep_boxes` / `_hits`)をそのまま呼び、支持フィルタを
外した「生の幾何fit」だけを追加で数える(_fit_positions の `ok_sup` 手前の `ok` を
別途複製して数えるだけで、判定式自体には一切手を入れていない)。

実行:
    MYSOLVER_UNITS_PER_SEC=1.55e7 PYTHONPATH=. .venv/bin/python tools/phase30_stall_classify.py \
        --cand results/phase29_cand_g1.json results/phase29_cand_g2.json \
        --voxels 0.05 0.025 --out results/phase30_stall_classify.json
"""
import argparse
import io
import json
import math
import os
import sys
from contextlib import redirect_stdout

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

import numpy as np

from src.ground_handling.env import GroundHandlingEnv
from agents.mysolver import geometry as geo
from agents.mysolver import ordering as ordering_mod
from agents.mysolver import planner
from agents.mysolver import reach as R
from agents.mysolver import simulate as simulate_mod
from tools.phase29_blockers import winner_order, scene_path


def raw_fit_count(masks, item, voxel):
    """支持フィルタを外した「幾何的に収まる位置」の数(orientation 重複除去)。

    `reach._fit_positions` の `ok`(支持フィルタ適用前)と同一の判定式。
    """
    empty = masks['empty']
    nx, ny, nz = empty.shape
    A = R._pad3(~empty)
    lwh = (item['length'], item['width'], item['height'])
    seen = set()
    total = 0
    for oi in range(6):
        half = geo.half_extent(lwh, oi)
        key = tuple(max(1, int(math.ceil(2 * h / voxel - 1e-9))) for h in half)
        if key in seen:
            continue
        seen.add(key)
        di, dj, dk = key
        if di > nx or dj > ny or dk > nz:
            continue
        I, J, K = nx - di + 1, ny - dj + 1, nz - dk + 1
        tot = (A[di:di + I, dj:dj + J, dk:dk + K]
               - A[0:I, dj:dj + J, dk:dk + K]
               - A[di:di + I, 0:J, dk:dk + K]
               - A[di:di + I, dj:dj + J, 0:K]
               + A[0:I, 0:J, dk:dk + K]
               + A[0:I, dj:dj + J, 0:K]
               + A[di:di + I, 0:J, 0:K]
               - A[0:I, 0:J, 0:K])
        ok = tot == 0
        total += int(ok.sum())
    return total


def classify_item(containers, item, voxel):
    """1荷物について、fit_nosupport / fit_support(=supported) / open / shelf_blocked / item_blocked
    を全コンテナ合算で返す(`tools/phase29_diag.py::diag_item` と同じ骨格 + raw_fit_count を追加)。
    """
    tot = dict(fit_nosupport=0, fit_support=0, open=0, shelf_blocked=0, item_blocked=0)
    for cdict in containers:
        masks = R.build_masks(cdict, voxel=voxel)
        if not masks['empty'].any():
            continue
        tot['fit_nosupport'] += raw_fit_count(masks, item, voxel)
        packed = [it for it in cdict.get('packed_items', [])
                  if it.get('pos') is not None and it.get('orn') is not None]
        obs = [(geo.item_world_aabb(it), int(it['index'])) for it in packed]
        static = list(geo.static_obstacles(cdict))
        for key, ok_sup in R._fit_positions(masks, cdict, item, voxel):
            tot['fit_support'] += int(ok_sup.sum())
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


def item_category(d):
    """5分類の1文字コード ('i'/'ii'/'iii'/'iv'/'v') を返す(単一荷物基準)。"""
    if d['fit_nosupport'] == 0:
        return 'i'
    if d['fit_support'] == 0:
        return 'ii'
    if d['open'] > 0:
        return 'v'          # 幾何・支持・経路とも問題ないのに置けなかった(別の理由)
    if d['item_blocked'] > 0:
        return 'iii'
    if d['shelf_blocked'] > 0:
        return 'iv'
    return 'unknown'         # 理論上到達しない(fit_support>0 なら open/item/shelf のどれかが>0)


SCENE_PRECEDENCE = ['iii', 'iv', 'v', 'ii', 'i']  # 修正可能性が高い側を優先してシーンを代表させる


def replay_scene(label, cand):
    task = list(json.load(open(scene_path(label))).values())[0]
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
    items_by_index = {it['index']: it for it in items}
    total_vol = float(sum(it['length'] * it['width'] * it['height'] for it in items))
    order = winner_order(cand, sum(c.get('volume', 0.0) for c in container_list))['order']
    prepacked_ids = geo.initial_prepacked_ids(container_list)
    budget = planner.SearchBudget.from_seconds(600.0)
    stall = {}
    buf = io.StringIO()
    with redirect_stdout(buf):
        out = simulate_mod.simulate_order(
            container_list, items_by_index, order, max(1, int(lookahead or 1)), budget,
            prepacked_ids=prepacked_ids,
            stability_weight=ordering_mod.STABILITY_PENALTY_WEIGHT, stall_info=stall)
    placed_ids = out[0]
    placed_vol = sum(items_by_index[i]['length'] * items_by_index[i]['width']
                      * items_by_index[i]['height'] for i in placed_ids)
    residual_vol = total_vol - placed_vol
    return stall, items_by_index, lookahead, total_vol, residual_vol, len(order), len(placed_ids)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--cand', nargs='+', required=True)
    ap.add_argument('--voxels', nargs='+', type=float, default=[0.05, 0.025])
    ap.add_argument('--out', default=None)
    args = ap.parse_args()

    cands = []
    for f in args.cand:
        cands.extend(json.load(open(f)))
    cands.sort(key=lambda c: c['label'])   # シーンID昇順(機械的な規則)

    rows = []
    for c in cands:
        label = c['label']
        stall, items_by_index, lookahead, total_vol, residual_vol, n_items, n_placed = \
            replay_scene(label, c)
        row = {'label': label, 'lookahead_k': int(lookahead or 1), 'n_items': n_items,
               'n_placed': n_placed, 'total_vol': total_vol, 'residual_vol': residual_vol,
               'stalled': bool(stall.get('stalled'))}
        if not stall.get('stalled'):
            row['scene_category'] = 'v'
            row['reason'] = 'not_stalled'
            rows.append(row)
            print(f"{label:28s} not stalled -> v  residual_vol={residual_vol:.3f}", flush=True)
            if args.out:
                json.dump(rows, open(args.out, 'w'), indent=1, ensure_ascii=False)
            continue

        pool = stall['pool']
        containers = stall['containers']
        row['pool_items'] = [int(it['index']) for it in pool]
        by_voxel = {}
        for v in args.voxels:
            per_item = []
            for it in pool:
                d = classify_item(containers, it, v)
                cat = item_category(d)
                per_item.append({'item': int(it['index']),
                                  'volume': float(it['length'] * it['width'] * it['height']),
                                  **d, 'category': cat})
            # シーン代表カテゴリ: プール内のどれかが該当すれば、その中で最も「望みがある」分類を採用
            present = {r['category'] for r in per_item}
            scene_cat = next((cat for cat in SCENE_PRECEDENCE if cat in present), 'unknown')
            rep = next(r for r in per_item if r['category'] == scene_cat)
            by_voxel[v] = {'per_item': per_item, 'scene_category': scene_cat,
                            'rep_item': rep['item'], 'rep_volume': rep['volume']}
        row['by_voxel'] = {str(v): by_voxel[v] for v in args.voxels}
        # 計測2のメイン分類は最も細かい voxel(見落とし側に倒さない)を使う
        finest = min(args.voxels)
        row['scene_category'] = by_voxel[finest]['scene_category']
        row['rep_item'] = by_voxel[finest]['rep_item']
        row['rep_volume'] = by_voxel[finest]['rep_volume']
        rows.append(row)
        vox_str = ' | '.join(f"vox={v:.3f}:{by_voxel[v]['scene_category']}" for v in args.voxels)
        print(f"{label:28s} stalled pool={row['pool_items']} {vox_str} "
              f"residual_vol={residual_vol:.3f}", flush=True)
        if args.out:
            json.dump(rows, open(args.out, 'w'), indent=1, ensure_ascii=False)

    print('\n===== 集計(最細 voxel 基準) =====')
    dist = {}
    vol_by_cat = {}
    for r in rows:
        cat = r['scene_category']
        dist[cat] = dist.get(cat, 0) + 1
        vol_by_cat.setdefault(cat, []).append(r['residual_vol'])
    for cat in SCENE_PRECEDENCE:
        n = dist.get(cat, 0)
        vols = vol_by_cat.get(cat, [])
        if vols:
            print(f"  {cat}: {n}シーン  残体積 mean={np.mean(vols):.2f} "
                  f"σ={np.std(vols):.2f} vals={[f'{x:.2f}' for x in vols]}")
        else:
            print(f"  {cat}: {n}シーン")


if __name__ == '__main__':
    main()
