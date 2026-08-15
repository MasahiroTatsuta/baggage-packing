"""Phase28: 行き詰まり時点の「到達可能性」を測る(オフライン順序探索の目的関数用)。

Phase24 が特定した corridor_penalty の限界(時間方向の myopia)を外すための評価軸である。

  corridor_penalty は **候補1手ごとに、その時点の障害物に対して** 罰則を課すため、
  「これから積み上がる荷物が作る通路」を守れない(results/phase24_report.md §2.3)。
  封鎖の主因である中間高さ帯・上部の通路は **その後の積み上げで初めて生まれる** ので、
  不感帯の精度をいくら上げてもこの分は取れない。
  一方、影シミュレータは順序を最後まで流し込むので、**封鎖の結果を事後に観測できる**。

本モジュールは `tools/phase24_corridor_audit.py` の判定ロジックを **そのまま移植** した
もので、新規に書き起こしたものではない(判定式を変えると Phase24 の 29.993 m³ という
測定値との接続が切れるため)。移植にあたって落としたのは Phase24 の**報告用の分類**
(帯域別 band_*、障害物種別 blk_*、経路上の障害物個数 obs* 、楽観近似 reach_optimistic)だけで、
`covered` / `supported` / `reach_strict` の判定式には一切手を入れていない。
同一性は `tools/phase28_verify_reach.py` が監査ツールとの数値一致で検証する。

コスト: 探索のホットパスから呼ばれるため voxel を粗くできるようにしてある(既定 0.10m)。
実測(B01 の行き詰まり状態、荷物18個)では 0.025m=3.375s / 0.05m=0.533s / 0.10m=0.106s。
Phase25b で探索予算は飽和しており増やすと悪化するため、**予算を増やして相殺してはならない**
(results/phase28_report.md §3)。
"""
import math
import os

import numpy as np

from . import geometry as geo

# 既定 voxel。Phase24 の監査は 0.025m だが、それは1シーン1回のオフライン監査だから許される
# コストであり、順序探索の内側からは重すぎる(§実測)。順序の**順位**さえ保てればよいので
# (Phase20 の教訓: build_order は同一シーン内の順位しか使わない)粗い格子を既定にする。
VOXEL = float(os.environ.get('MYSOLVER_REACH_VOXEL', '0.10'))
SUPPORT_RATIO = 0.55   # planner.MIN_UNION_SUPPORT_RATIO と同水準(監査ツールと同一)


def _axes(cdict, voxel):
    ox = cdict['center'][0]
    L, W, H, th = cdict['length'], cdict['width'], cdict['height'], cdict['thickness']
    xs = np.arange(ox - L / 2 + th + voxel / 2, ox + L / 2 - th, voxel)
    ys = np.arange(-W / 2 + th + voxel / 2, W / 2 - th, voxel)
    zs = np.arange(th + voxel / 2, H - th, voxel)
    return xs, ys, zs


def build_masks(cdict, voxel=VOXEL):
    """in_container / occupied(荷物) / struct(棚) を返す(監査ツール build_masks の移植)。

    Phase24 版との差分は labels(経路上の障害物個数を数えるためのラベル画像)を作らない点だけ。
    """
    xs, ys, zs = _axes(cdict, voxel)
    X, Y, Z = np.meshgrid(xs, ys, zs, indexing='ij')
    pts = np.stack([X.ravel(), Y.ravel(), Z.ravel()], axis=1)

    n_vecs = np.array(cdict['n_vecs']); points = np.array(cdict['points'])
    inside = np.ones(pts.shape[0], dtype=bool)
    for n, p in zip(n_vecs, points):
        inside &= (pts - p) @ n <= 0.0
    inside = inside.reshape(X.shape)

    struct = np.zeros(X.shape, dtype=bool)
    for center, half in geo.static_obstacles(cdict):
        struct |= ((np.abs(X - center[0]) <= half[0]) &
                   (np.abs(Y - center[1]) <= half[1]) &
                   (np.abs(Z - center[2]) <= half[2]))

    occ = np.zeros(X.shape, dtype=bool)
    for it in cdict.get('packed_items', []):
        if it.get('pos') is None or it.get('orn') is None:
            continue
        c, h = geo.item_world_aabb(it)
        occ |= ((np.abs(X - c[0]) <= h[0]) & (np.abs(Y - c[1]) <= h[1]) & (np.abs(Z - c[2]) <= h[2]))

    in_c = inside & ~struct
    return {'xs': xs, 'ys': ys, 'zs': zs, 'inside': inside, 'in_container': in_c,
            'occupied': occ & in_c, 'struct': struct, 'empty': in_c & ~occ}


def _window_any(a, size, axis):
    """a を axis 方向に「幅 size の窓のどこかが True か」へ変換する(分離可能な膨張)。"""
    if size <= 1:
        return a
    c = np.cumsum(a.astype(np.int32), axis=axis)
    lo = np.take(c, np.arange(a.shape[axis]), axis=axis)
    idx = np.arange(a.shape[axis]) - size
    prev = np.where(idx >= 0, idx, 0)
    sub = np.take(c, prev, axis=axis)
    shape = [1] * a.ndim
    shape[axis] = a.shape[axis]
    mask = (idx >= 0).reshape(shape)
    return (lo - np.where(mask, sub, 0)) > 0


def _positions_to_covered(pos, dims, shape):
    di, dj, dk = dims
    full = np.zeros(shape, dtype=bool)
    full[:pos.shape[0], :pos.shape[1], :pos.shape[2]] = pos
    out = _window_any(full, di, 0)
    out = _window_any(out, dj, 1)
    out = _window_any(out, dk, 2)
    return out


def _pad3(mask):
    return np.pad(mask.astype(np.int32).cumsum(0).cumsum(1).cumsum(2),
                  ((1, 0), (1, 0), (1, 0)), mode='constant')


def _corridor_geometry(cdict, dk, K, voxel):
    """各 k(荷物底面の voxel 行)について、掃引箱の z 範囲 [z0, z1] を返す。

    validator.check_transport_path と同式(監査ツール _corridor_geometry の移植。
    band の算出だけ落としてある)。
    """
    th = cdict['thickness']; H = cdict['height']; buf = cdict.get('buffer', 0.0)
    h = dk * voxel
    k = np.arange(K)
    bottom = th + k * voxel + geo.REST_CLEARANCE
    top = bottom + h
    world_z = bottom + h / 2.0

    rest_floor = th
    rest_shelf = H / 2.0 + th + buf
    d0 = bottom - rest_floor
    d1 = bottom - rest_shelf
    resting = ((d0 >= 0.0) & (d0 <= 0.05)) | ((d1 >= 0.0) & (d1 <= 0.05))

    eff = np.where(resting, 0.0, geo.START_Z)
    handled = resting.copy()
    for c_z in (H / 2.0 + buf, H + buf - th):
        clearance = c_z - top
        trig = (~handled) & (clearance >= 0.0) & (clearance < (eff + geo.CEILING_MARGIN))
        eff = np.where(trig, np.maximum(0.0, clearance - geo.CEILING_MARGIN - 0.0005), eff)
        handled = handled | trig

    ceiling_sweep = H + buf - th - h / 2.0 - geo.START_MARGIN
    sweep_z = np.minimum(ceiling_sweep, world_z + eff)
    return sweep_z - h / 2.0, sweep_z + h / 2.0


def fit_and_reach(masks, remaining, cdict, voxel=VOXEL):
    """残荷物×全 orientation を全位置で列挙し、covered / supported / reach_strict を返す。

    監査ツール fit_and_reach の移植。分類マスク(blk_*/band_*/obs*/reach_optimistic)を
    作らない点だけが差分で、判定式は同一である。
    戻り値には、コストのユニット換算に使う `n_shapes`(実際に評価したユニーク形状数)も含む。
    """
    empty = masks['empty']
    nx, ny, nz = empty.shape
    xs = masks['xs']; ys = masks['ys']
    y_entry = -cdict['width'] / 2.0

    A = _pad3(~empty)
    solid_below = masks['occupied'] | ~masks['in_container']
    Ab = np.pad(solid_below.astype(np.int32).cumsum(0).cumsum(1), ((1, 0), (1, 0), (0, 0)),
                mode='constant')
    # 掃引の障害物 = 既配置荷物 + 棚。validator._move_item はコンテナ壁とは判定しない
    # (壁・切り欠きは搬入開始 x の制限 transport_x_bounds としてのみ効く)。
    sweep_obstacles = list(geo.packed_obstacles(cdict)) + list(geo.static_obstacles(cdict))

    out = {name: np.zeros(empty.shape, dtype=bool)
           for name in ('covered', 'supported', 'reach_strict')}

    seen = set()
    n_shapes = 0
    for it in remaining:
        lwh = (it['length'], it['width'], it['height'])
        for oi in range(6):
            half = geo.half_extent(lwh, oi)
            key = tuple(max(1, int(math.ceil(2 * h / voxel - 1e-9))) for h in half)
            if key in seen:
                continue
            seen.add(key)
            di, dj, dk = key
            if di > nx or dj > ny or dk > nz:
                continue
            n_shapes += 1
            I, J, K = nx - di + 1, ny - dj + 1, nz - dk + 1

            # --- 最終位置が空か ---
            tot = (A[di:di + I, dj:dj + J, dk:dk + K]
                   - A[0:I, dj:dj + J, dk:dk + K]
                   - A[di:di + I, 0:J, dk:dk + K]
                   - A[di:di + I, dj:dj + J, 0:K]
                   + A[0:I, 0:J, dk:dk + K]
                   + A[0:I, dj:dj + J, 0:K]
                   + A[di:di + I, 0:J, 0:K]
                   - A[0:I, 0:J, 0:K])
            ok = tot == 0
            if not ok.any():
                continue
            out['covered'] |= _positions_to_covered(ok, key, empty.shape)

            # --- 支持 ---
            foot = (Ab[di:di + I, dj:dj + J, :] - Ab[0:I, dj:dj + J, :]
                    - Ab[di:di + I, 0:J, :] + Ab[0:I, 0:J, :])
            need = SUPPORT_RATIO * di * dj
            sup_ok = np.zeros_like(ok)
            below = np.arange(K) - 1
            v = below >= 0
            if v.any():
                sup_ok[:, :, v] = foot[:, :, below[v]] >= need
            sup_ok[:, :, ~v] = True
            ok_sup = ok & sup_ok
            if not ok_sup.any():
                continue
            out['supported'] |= _positions_to_covered(ok_sup, key, empty.shape)

            # --- 厳密な搬入経路(validator.check_transport_path と同じ判定を連続座標で) ---
            pi, pj, pk = np.nonzero(ok_sup)
            z0_arr, z1_arr = _corridor_geometry(cdict, dk, K, voxel)
            mxy = geo.SAFETY_MARGIN_XY
            mz = geo.SWEEP_Z_MARGIN

            x0 = xs[pi] - voxel / 2.0
            x1 = x0 + di * voxel
            y0 = ys[pj] - voxel / 2.0
            y1 = y0 + dj * voxel
            cz0 = z0_arr[pk]; cz1 = z1_arr[pk]

            x_min_w, x_max_w = geo.transport_x_bounds(cdict, di * voxel / 2.0)
            cxp = (x0 + x1) / 2.0
            if x_min_w > x_max_w:
                # この向きは開口部(切り欠きで削られた幅)を通らない = 全位置が到達不能
                continue
            scx = np.clip(cxp, x_min_w, x_max_w)
            needs_x = np.abs(scx - cxp) > 1e-9
            sx0 = scx - di * voxel / 2.0
            sx1 = scx + di * voxel / 2.0
            x2lo = np.minimum(sx0, x0); x2hi = np.maximum(sx1, x1)

            hit_any = np.zeros(pi.shape[0], dtype=bool)
            for (oc, oh) in sweep_obstacles:
                fz = (cz1 + mz > oc[2] - oh[2]) & (oc[2] + oh[2] + mz > cz0)
                if not fz.any():
                    continue
                fy_far = (oc[1] + oh[1] + mxy > y_entry)
                p1 = (fz & (sx1 + mxy > oc[0] - oh[0]) & (oc[0] + oh[0] + mxy > sx0)
                      & (y1 + mxy > oc[1] - oh[1]) & fy_far)
                p2 = (needs_x & fz & (x2hi + mxy > oc[0] - oh[0]) & (oc[0] + oh[0] + mxy > x2lo)
                      & (y1 + mxy > oc[1] - oh[1]) & (oc[1] + oh[1] + mxy > y0))
                hit_any |= (p1 | p2)
                if hit_any.all():
                    break

            if (~hit_any).any():
                m = np.zeros(ok_sup.shape, dtype=bool)
                sel = ~hit_any
                m[pi[sel], pj[sel], pk[sel]] = True
                out['reach_strict'] |= _positions_to_covered(m, key, empty.shape)

    for k in ('covered', 'supported', 'reach_strict'):
        out[k] &= empty
    out['n_shapes'] = n_shapes
    return out


def _sweep_boxes(masks, cdict, key, ok_sup, voxel):
    """ok_sup の各 True 位置について、掃引箱(p1: y掃引 / p2: x掃引)の座標配列を返す。

    fit_and_reach の「厳密な搬入経路」ブロックから、障害物との判定に必要な幾何量だけを
    切り出したもの(判定式は一切変えていない)。戻り値は
    (pi, pj, pk, geom) で、geom は障害物1個との衝突判定に必要な配列の dict。
    """
    xs = masks['xs']; ys = masks['ys']
    di, dj, dk = key
    nx, ny, nz = masks['empty'].shape
    K = nz - dk + 1
    pi, pj, pk = np.nonzero(ok_sup)
    if pi.size == 0:
        return None
    z0_arr, z1_arr = _corridor_geometry(cdict, dk, K, voxel)
    x0 = xs[pi] - voxel / 2.0
    x1 = x0 + di * voxel
    y0 = ys[pj] - voxel / 2.0
    y1 = y0 + dj * voxel
    x_min_w, x_max_w = geo.transport_x_bounds(cdict, di * voxel / 2.0)
    if x_min_w > x_max_w:
        return None
    cxp = (x0 + x1) / 2.0
    scx = np.clip(cxp, x_min_w, x_max_w)
    needs_x = np.abs(scx - cxp) > 1e-9
    sx0 = scx - di * voxel / 2.0
    sx1 = scx + di * voxel / 2.0
    geom = {
        'x0': x0, 'x1': x1, 'y0': y0, 'y1': y1,
        'cz0': z0_arr[pk], 'cz1': z1_arr[pk],
        'sx0': sx0, 'sx1': sx1, 'needs_x': needs_x,
        'x2lo': np.minimum(sx0, x0), 'x2hi': np.maximum(sx1, x1),
        'y_entry': -cdict['width'] / 2.0,
    }
    return pi, pj, pk, geom


def _hits(geom, oc, oh):
    """障害物 (center, half) が掃引箱に当たる位置の bool 配列(fit_and_reach と同式)。"""
    mxy = geo.SAFETY_MARGIN_XY
    mz = geo.SWEEP_Z_MARGIN
    fz = (geom['cz1'] + mz > oc[2] - oh[2]) & (oc[2] + oh[2] + mz > geom['cz0'])
    fy_far = (oc[1] + oh[1] + mxy > geom['y_entry'])
    p1 = (fz & (geom['sx1'] + mxy > oc[0] - oh[0]) & (oc[0] + oh[0] + mxy > geom['sx0'])
          & (geom['y1'] + mxy > oc[1] - oh[1]) & fy_far)
    p2 = (geom['needs_x'] & fz & (geom['x2hi'] + mxy > oc[0] - oh[0])
          & (oc[0] + oh[0] + mxy > geom['x2lo'])
          & (geom['y1'] + mxy > oc[1] - oh[1]) & (oc[1] + oh[1] + mxy > geom['y0']))
    return p1 | p2


def _fit_positions(masks, cdict, item, voxel):
    """荷物 item の全 orientation について (key, ok_sup) を返す(fit_and_reach と同じ判定)。"""
    empty = masks['empty']
    nx, ny, nz = empty.shape
    A = _pad3(~empty)
    solid_below = masks['occupied'] | ~masks['in_container']
    Ab = np.pad(solid_below.astype(np.int32).cumsum(0).cumsum(1), ((1, 0), (1, 0), (0, 0)),
                mode='constant')
    lwh = (item['length'], item['width'], item['height'])
    seen = set()
    out = []
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
        if not ok.any():
            continue
        foot = (Ab[di:di + I, dj:dj + J, :] - Ab[0:I, dj:dj + J, :]
                - Ab[di:di + I, 0:J, :] + Ab[0:I, 0:J, :])
        need = SUPPORT_RATIO * di * dj
        sup_ok = np.zeros_like(ok)
        below = np.arange(K) - 1
        v = below >= 0
        if v.any():
            sup_ok[:, :, v] = foot[:, :, below[v]] >= need
        sup_ok[:, :, ~v] = True
        ok_sup = ok & sup_ok
        if ok_sup.any():
            out.append((key, ok_sup))
    return out


def item_blockers(containers, item, voxel=VOXEL, masks_cache=None):
    """荷物 X ひとつについて「開ければ置けるようになる既配置荷物の集合」を返す。

    Phase29: Phase24/28 が測っていたのは *体積* だけで、封鎖している **荷物の同定** は
    していなかった(監査ツールは経路上の障害物 *個数* までは数えていた)。ここは同じ
    判定式(`_hits` = fit_and_reach の p1|p2)に障害物の identity を持たせただけの拡張である。

    手順:
      1. X が幾何的に収まり支持もある位置(= supported)を全 orientation で列挙する
      2. そのうち搬入経路が塞がれている位置に絞る
      3. **棚(静的障害物)が当たっている位置は除く** —— 棚は順序で動かせないので、
         そこを開けることは原理的にできない
      4. 残った位置のうち **ブロッカー数が最小** の位置を選び、その位置を塞いでいる
         既配置荷物の index 集合を返す

    戻り値 None は「順序の修正では開けられない」(収まる位置が無い/棚が塞いでいる/
    そもそも経路は空いている=別の理由で置けなかった)の意。
    """
    best = None
    for ci, cdict in enumerate(containers):
        if masks_cache is not None and ci in masks_cache:
            masks = masks_cache[ci]
        else:
            masks = build_masks(cdict, voxel=voxel)
            if masks_cache is not None:
                masks_cache[ci] = masks
        if not masks['empty'].any():
            continue
        packed = [it for it in cdict.get('packed_items', [])
                  if it.get('pos') is not None and it.get('orn') is not None]
        obs = [(geo.item_world_aabb(it), int(it['index'])) for it in packed]
        static = list(geo.static_obstacles(cdict))
        for key, ok_sup in _fit_positions(masks, cdict, item, voxel):
            got = _sweep_boxes(masks, cdict, key, ok_sup, voxel)
            if got is None:
                continue
            pi, pj, pk, geom = got
            n = pi.shape[0]
            shelf_hit = np.zeros(n, dtype=bool)
            for (oc, oh) in static:
                shelf_hit |= _hits(geom, oc, oh)
            nobs = np.zeros(n, dtype=np.int32)
            for (oc, oh), _idx in obs:
                nobs += _hits(geom, oc, oh)
            cand = (~shelf_hit) & (nobs >= 1)
            if not cand.any():
                continue
            masked = np.where(cand, nobs, np.iinfo(np.int32).max)
            p = int(np.argmin(masked))       # C順の最初の最小 = 決定的
            cnt = int(nobs[p])
            if best is not None and cnt >= best['n_blockers']:
                continue
            one = {k: (v[p:p + 1] if isinstance(v, np.ndarray) else v)
                   for k, v in geom.items()}
            ids = [idx for (oc, oh), idx in obs if bool(_hits(one, oc, oh)[0])]
            best = {'container_idx': ci, 'n_blockers': cnt, 'blockers': sorted(ids),
                    'n_blocked_positions': int(cand.sum())}
    return best


def item_occupiers(containers, item, voxel=VOXEL, masks_cache=None):
    """荷物 X ひとつについて「X の居場所を奪っている既配置荷物の集合」を返す(Phase34)。

    `item_blockers` との違いは **見ている失敗の種類** である:

      - `item_blockers` は「X は収まるし支持もあるが、**搬入経路**が塞がれている」位置を探す
        (Phase30 の分類 (iii))。掃引箱に当たっている荷物を返す。
      - `item_occupiers` は「X が**そもそも収まらない**」場合に、X が収まりうる位置を
        **占有している**荷物を返す(Phase30 の分類 (i) = 21シーン中10シーンの最大区分)。

    Phase30 §5.1 が「現在の `raw_fit_count` は入るか否かしか持たず、**占有者の identity が
    無い**」と指摘した欠落を埋めるのが本関数である。判定に使う部品(`build_masks` /
    支持判定 / `_pad3` の箱和)はすべて `item_blockers` / `_fit_positions` と共通で、
    新規に書き起こした幾何判定は「候補箱と既配置荷物AABBの重なり」だけである
    (voxel化せず連続座標の区間重なりで判定するので、格子解像度による取りこぼしが無い)。

    手順:
      1. X の各 orientation について、**静的障害物(棚)・容器外に一切かからない**位置を
         列挙する —— 棚・壁は順序では動かせないので、そこに掛かる位置は原理的に不可能
      2. そのうち **現状態で支持がある**位置に絞る(`_fit_positions` と同じ支持判定)。
         支持を現状態基準で見るのは、空中の無意味な位置を候補にしないための足切りである
         (占有者を除去すると支持も消えることはあるが、除去した荷物はロールアウトが
          後段で置き直すので、ここでは近似として現状態の支持を使う)
      3. 各位置について、AABB が重なっている既配置荷物の数を数える
      4. **重なり数が最小(かつ1以上)** の位置を選び、その荷物 index 集合を返す

    戻り値 None は「順序の修正では開けられない」(棚・壁だけが理由で入らない/
    既に空きに収まる=別の理由で置けなかった)の意。
    """
    lwh = (item['length'], item['width'], item['height'])
    best = None
    for ci, cdict in enumerate(containers):
        if masks_cache is not None and ci in masks_cache:
            masks = masks_cache[ci]
        else:
            masks = build_masks(cdict, voxel=voxel)
            if masks_cache is not None:
                masks_cache[ci] = masks
        packed = [it for it in cdict.get('packed_items', [])
                  if it.get('pos') is not None and it.get('orn') is not None]
        if not packed:
            continue
        obs = [(geo.item_world_aabb(it), int(it['index'])) for it in packed]

        nx, ny, nz = masks['empty'].shape
        xs, ys, zs = masks['xs'], masks['ys'], masks['zs']
        # 「順序では動かせない障害物」= 容器外 + 棚(in_container は inside & ~struct)。
        A = _pad3(~masks['in_container'])
        solid_below = masks['occupied'] | ~masks['in_container']
        Ab = np.pad(solid_below.astype(np.int32).cumsum(0).cumsum(1), ((1, 0), (1, 0), (0, 0)),
                    mode='constant')

        seen = set()
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

            # --- 棚・容器外に一切かからない位置 ---
            tot = (A[di:di + I, dj:dj + J, dk:dk + K]
                   - A[0:I, dj:dj + J, dk:dk + K]
                   - A[di:di + I, 0:J, dk:dk + K]
                   - A[di:di + I, dj:dj + J, 0:K]
                   + A[0:I, 0:J, dk:dk + K]
                   + A[0:I, dj:dj + J, 0:K]
                   + A[di:di + I, 0:J, 0:K]
                   - A[0:I, 0:J, 0:K])
            ok = tot == 0
            if not ok.any():
                continue

            # --- 支持(_fit_positions と同式) ---
            foot = (Ab[di:di + I, dj:dj + J, :] - Ab[0:I, dj:dj + J, :]
                    - Ab[di:di + I, 0:J, :] + Ab[0:I, 0:J, :])
            need = SUPPORT_RATIO * di * dj
            sup_ok = np.zeros_like(ok)
            below = np.arange(K) - 1
            v = below >= 0
            if v.any():
                sup_ok[:, :, v] = foot[:, :, below[v]] >= need
            sup_ok[:, :, ~v] = True
            ok = ok & sup_ok
            if not ok.any():
                continue

            # --- 既配置荷物との重なり(連続座標の区間重なり、分離可能) ---
            x_lo = xs[:I] - voxel / 2.0
            y_lo = ys[:J] - voxel / 2.0
            z_lo = zs[:K] - voxel / 2.0
            nocc = np.zeros((I, J, K), dtype=np.int32)
            hits = []
            for (oc, oh), idx in obs:
                mi = (x_lo < oc[0] + oh[0]) & (x_lo + di * voxel > oc[0] - oh[0])
                if not mi.any():
                    continue
                mj = (y_lo < oc[1] + oh[1]) & (y_lo + dj * voxel > oc[1] - oh[1])
                if not mj.any():
                    continue
                mk = (z_lo < oc[2] + oh[2]) & (z_lo + dk * voxel > oc[2] - oh[2])
                if not mk.any():
                    continue
                nocc += (mi[:, None, None] & mj[None, :, None] & mk[None, None, :])
                hits.append((idx, mi, mj, mk))

            cand = ok & (nocc >= 1)
            if not cand.any():
                continue
            masked = np.where(cand, nocc, np.iinfo(np.int32).max)
            p = int(np.argmin(masked))       # C順の最初の最小 = 決定的
            pi, pj, pk = np.unravel_index(p, masked.shape)
            cnt = int(nocc[pi, pj, pk])
            if best is not None and cnt >= best['n_occupiers']:
                continue
            ids = sorted(idx for idx, mi, mj, mk in hits if mi[pi] and mj[pj] and mk[pk])
            best = {'container_idx': ci, 'n_occupiers': cnt, 'occupiers': ids,
                    'n_positions': int(cand.sum())}
    return best


def stall_occupiers(containers, pool_items, voxel=VOXEL):
    """行き詰まり時点のプール全件について占有者を同定し、最小1件を返す(stall_blockers と同型)。"""
    cache: dict = {}
    per_item = []
    for pos, item in enumerate(pool_items):
        r = item_occupiers(containers, item, voxel=voxel, masks_cache=cache)
        per_item.append({'item_index': int(item['index']), 'pool_pos': pos, 'result': r})
    ranked = [r for r in per_item if r['result'] is not None]
    ranked.sort(key=lambda r: (r['result']['n_occupiers'], r['pool_pos']))
    cells = sum(int(m['empty'].size) for m in cache.values())
    return {'candidate': ranked[0] if ranked else None, 'per_item': per_item,
            'grid_cells': cells, 'n_shapes': max(1, len(pool_items))}


def fit_position_counts(containers, items, exclude_ids, voxel=VOXEL):
    """`exclude_ids` を取り除いた状態で、各荷物が「収まり支持もある」位置の数を数える。

    Phase34 の regret 修復オペレータ用。「除去した荷物たちを戻すとき、置き場所の選択肢が
    少ないものから先に戻す(most-constrained-first)」という順序づけに使う。
    戻り値: ({item_index: 位置数}, grid_cells, n_shapes)。
    """
    ex = {int(i) for i in exclude_ids}
    stripped = []
    for c in containers:
        cc = dict(c)
        cc['packed_items'] = [it for it in c.get('packed_items', [])
                              if int(it.get('index', -1)) not in ex]
        stripped.append(cc)
    counts = {int(it['index']): 0 for it in items}
    cells = 0
    shapes = 0
    for cdict in stripped:
        masks = build_masks(cdict, voxel=voxel)
        cells += int(masks['empty'].size)
        if not masks['empty'].any():
            continue
        for it in items:
            n = 0
            for key, ok_sup in _fit_positions(masks, cdict, it, voxel):
                n += int(ok_sup.sum())
                shapes += 1
            counts[int(it['index'])] += n
    return counts, cells, max(1, shapes)


def stall_blockers(containers, pool_items, voxel=VOXEL):
    """行き詰まり時点のプール(=置けなかった荷物)について、ブロッカーを同定する。

    戻り値は「ブロッカー数が最小、同数ならプール順」で選んだ1件(順序修正の対象)と、
    プール全件の内訳。順序修正で開けられる衝突が1件も無ければ candidate は None。
    """
    cache: dict = {}
    per_item = []
    for pos, item in enumerate(pool_items):
        b = item_blockers(containers, item, voxel=voxel, masks_cache=cache)
        per_item.append({'item_index': int(item['index']), 'pool_pos': pos,
                         'result': b})
    ranked = [r for r in per_item if r['result'] is not None]
    ranked.sort(key=lambda r: (r['result']['n_blockers'], r['pool_pos']))
    # コストのユニット換算用(決定的な量: voxel 数 × 評価した荷物数)。
    cells = sum(int(m['empty'].size) for m in cache.values())
    return {'candidate': ranked[0] if ranked else None, 'per_item': per_item,
            'grid_cells': cells, 'n_shapes': max(1, len(pool_items))}


def stall_reachability(containers, remaining, voxel=VOXEL):
    """行き詰まり時点の到達可能性を1スカラーにまとめて返す。

    戻り値 dict:
      empty_volume     : 空き体積 [m^3]
      supported_volume : 「残荷物のどれかが収まり、支持もある」空き体積 [m^3]
      blocked_volume   : そのうち **搬入経路が塞がれている** 体積 = Phase24 の (a) [m^3]
      blocked_ratio    : blocked_volume / supported_volume ([0,1])
      grid_cells       : voxel 数(コスト換算用)
      n_shapes         : 評価したユニーク形状数(コスト換算用)

    blocked_ratio の分母を empty ではなく **supported** にしているのは退化解を避けるため。
    empty で正規化すると「置かない順序ほど空きが大きく、封鎖の割合が小さくなる」ので、
    目的関数が「何も置かない」方向へ引っ張られる(Phase18 で実際に起きた退化解と同型:
    suite_A07 で 40個中1個しか置かない順序が選ばれ fill 28.07->0.00)。
    supported で正規化すれば「置ける見込みのある空間のうち、経路が死んでいる割合」となり、
    置いた量そのものには直接依存しない。
    """
    tot_empty = tot_sup = tot_blocked = 0.0
    cells = 0
    shapes = 0
    v3 = voxel ** 3
    for cdict in containers:
        masks = build_masks(cdict, voxel=voxel)
        cells += masks['empty'].size
        if not masks['empty'].any() or not remaining:
            continue
        out = fit_and_reach(masks, remaining, cdict, voxel=voxel)
        shapes += out['n_shapes']
        tot_empty += float(masks['empty'].sum()) * v3
        tot_sup += float(out['supported'].sum()) * v3
        tot_blocked += float((out['supported'] & ~out['reach_strict']).sum()) * v3
    ratio = tot_blocked / tot_sup if tot_sup > 1e-9 else 0.0
    return {'empty_volume': tot_empty, 'supported_volume': tot_sup,
            'blocked_volume': tot_blocked, 'blocked_ratio': ratio,
            'grid_cells': cells, 'n_shapes': shapes}
