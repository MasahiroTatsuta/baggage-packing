"""
tools/phase24_corridor_audit.py

Phase24 ターゲット1: Phase22 で 21.261 m³(空きの26.7%)と測った
**(a) 支持はあるが搬入経路が塞がれている空間** を分解し、真の大きさを測り直す。

Phase22 からの変更点は3つ。

1. **搬入経路判定を楽観近似から厳密化した。**
   Phase22 は「最終高さのまま +y 方向へ掃引する」近似で、
     ・非直置き(is_resting=False)の候補が START_Z=80mm 持ち上がって掃引されること
     ・掃引に SWEEP_Z_MARGIN=15.5mm の z 余裕・SAFETY_MARGIN_XY=22mm の xy 余裕が要ること
     ・目標 x が搬入可能域 [x_min, x_max](切り欠き cut_x で左が削られる)の外なら、
       いったん x_min/x_max で y 掃引したあと **x 掃引**が要ること
   をいずれも無視していた。本ツールは validator.check_transport_path と同じ順序
   (y掃引 → x掃引)・同じ浮上量(effective_start_z の直置き判定と天井クリップ)で判定する。

2. **(a) を3つの軸で分類する。**
   (i)  帯域: その空間へ到達する配置が is_resting になるか(床直置き / 棚直置き / 非直置き)
   (ii) 塞いでいる障害物の種類: 既配置荷物のみ / 棚(構造物)が絡む / 切り欠きによる x 迂回
   (iii) X レーン別に、経路上にある障害物の個数(1個だけなら順序次第で回避できた可能性が高い)

3. **空隙の「位置」を測る。**
   連結成分ごとの重心(コンテナローカル正規化座標)、空き体積の X/Y/Z 分布、
   および「上部 / 奥 / 切り欠き下 / 手前」の領域別シェア。

Stage A((c) を実コードで測って行き詰まり状態を進める)は Phase22 と同一で、
数値が Phase22 と直接比較できるようにしてある。

使い方:
    PYTHONPATH=. .venv/bin/python tools/phase24_corridor_audit.py \
        --config-path 'configs/gen/suite_*.json' --out results/phase24_void.json

src/ は一切変更しない。agents/ も読み取り専用で使う。
"""
import argparse
import glob
import importlib
import json
import math
import os
import sys
import time
import traceback
from contextlib import redirect_stdout

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

import numpy as np

from src.ground_handling.env import GroundHandlingEnv

VOXEL = 0.025          # [m] Phase22 と同一(数値の直接比較のため)
SUPPORT_RATIO = 0.55   # planner.MIN_UNION_SUPPORT_RATIO と同水準

BAND_FLOOR, BAND_SHELF, BAND_HIGH = 0, 1, 2
BAND_NAMES = {BAND_FLOOR: 'floor_resting', BAND_SHELF: 'shelf_resting', BAND_HIGH: 'lifted'}


# ----------------------------------------------------------------------------
# Stage A: 行き詰まり状態の取得と (c) の測定(Phase22 と同一)
# ----------------------------------------------------------------------------
def run_to_stall(task_config, module_path):
    mod_prefix = '.'.join(module_path.rstrip('/').split('/'))
    agent_mod = importlib.import_module(mod_prefix + '.agent')

    env = GroundHandlingEnv(config=task_config, verbose=False, render_mode=None)
    try:
        env.reset_settings()
        agent = agent_mod.Agent(module_path)
        agent.get_init_states(env.get_init_states())
        all_items = {int(it['index']): it for it in env.get_info_for_optimization()}
        if env.optimize:
            env.set_item_order(list(agent.optimize(env.get_info_for_optimization())))
        env.reset_item_stream()
        obs, info = env.reset(seed=42)
        terminated = truncated = False
        while not terminated and not truncated:
            action = agent.policy(obs)
            obs, reward, terminated, truncated, info = env.step(action)

        container_list = env.container_manager.get_item_info_in_containers()
        packed = set()
        for c in container_list:
            for it in c['packed_items']:
                packed.add(int(it['index']))
        remaining = [all_items[i] for i in sorted(all_items) if i not in packed]
        placed_volume = 0.0
        for c in container_list:
            for it in c['packed_items']:
                placed_volume += it['length'] * it['width'] * it['height']
        return {
            'container_list': container_list,
            'remaining': remaining,
            'optimize': bool(env.optimize),
            'placed_volume': placed_volume,
            'n_packed': len(packed),
            'n_total': len(all_items),
            'container_volume': sum(c['volume'] for c in container_list),
        }
    finally:
        try:
            env.close()
        except Exception:
            pass


def measure_c(state, module_path, densities=(8, 16)):
    """production より強い探索で追加配置できる体積 = (c)。Phase22 と同一実装。"""
    mod_prefix = '.'.join(module_path.rstrip('/').split('/'))
    planner = importlib.import_module(mod_prefix + '.planner')
    simulate = importlib.import_module(mod_prefix + '.simulate')
    geo = importlib.import_module(mod_prefix + '.geometry')

    containers = simulate.clone_containers(state['container_list'])
    prepacked_ids = geo.initial_prepacked_ids(state['container_list'])
    pool = [dict(it) for it in state['remaining']]
    strict_support = not state['optimize']

    saved = planner.RETRY_GRID_DENSITY
    added_vol = 0.0
    try:
        while pool:
            action = None
            for d in densities:
                planner.RETRY_GRID_DENSITY = d
                budget = planner.SearchBudget.from_seconds(40.0)
                action = planner.plan(containers, pool, max_pool_items=None,
                                      strict_support=strict_support,
                                      prepacked_ids=prepacked_ids, budget=budget)
                if action is not None:
                    break
            if action is None:
                break
            item = pool.pop(action['item_idx'])
            cont = containers[action['container_idx']]
            cont['packed_items'].append(simulate._place(cont, item, action))
            added_vol += item['length'] * item['width'] * item['height']
    finally:
        planner.RETRY_GRID_DENSITY = saved

    return {'c_volume': added_vol, 'containers_after': containers, 'remaining_after': pool}


# ----------------------------------------------------------------------------
# Stage B: voxel 分解
# ----------------------------------------------------------------------------
def _axes(cdict, voxel):
    ox = cdict['center'][0]
    L, W, H, th = cdict['length'], cdict['width'], cdict['height'], cdict['thickness']
    xs = np.arange(ox - L / 2 + th + voxel / 2, ox + L / 2 - th, voxel)
    ys = np.arange(-W / 2 + th + voxel / 2, W / 2 - th, voxel)
    zs = np.arange(th + voxel / 2, H - th, voxel)
    return xs, ys, zs


def build_masks(cdict, geo, voxel=VOXEL):
    """in_container / occupied(荷物) / struct(棚) / shape(コンテナ形状の外側) を返す。"""
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
    labels = np.zeros(X.shape, dtype=np.int32)
    lab = 0
    for it in cdict['packed_items']:
        c, h = geo.item_world_aabb(it)
        m = ((np.abs(X - c[0]) <= h[0]) & (np.abs(Y - c[1]) <= h[1]) & (np.abs(Z - c[2]) <= h[2]))
        occ |= m
        lab += 1
        labels[m] = lab
    # 棚は荷物とは別 id を与える(経路上の障害物個数を数えるため)
    labels[struct & (labels == 0)] = lab + 1

    in_c = inside & ~struct
    empty = in_c & ~occ
    return {'xs': xs, 'ys': ys, 'zs': zs, 'inside': inside, 'in_container': in_c,
            'occupied': occ & in_c, 'struct': struct, 'empty': empty, 'labels': labels}


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


def _corridor_geometry(cdict, geo, dk, K, voxel):
    """各 k(荷物底面の voxel 行)について、掃引箱の z 範囲 [z0, z1] と帯域を返す(連続値)。

    validator.check_transport_path と同式:
      real_bottom = th + k*voxel + REST_CLEARANCE   (目標点は支持面から REST_CLEARANCE 浮く)
      is_resting  = ある直置き面 r_z について 0 <= real_bottom - r_z <= 0.05
      effective_start_z = 0(直置き) / START_Z(それ以外, 天井余裕でクリップ)
      sweep_z     = min(ceiling_sweep, world_z + effective_start_z)
    戻り値: z0(K,), z1(K,), band(K,)
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
    on_floor = (d0 >= 0.0) & (d0 <= 0.05)
    on_shelf = (d1 >= 0.0) & (d1 <= 0.05)
    resting = on_floor | on_shelf

    eff = np.where(resting, 0.0, geo.START_Z)
    handled = resting.copy()
    for c_z in (H / 2.0 + buf, H + buf - th):
        clearance = c_z - top
        trig = (~handled) & (clearance >= 0.0) & (clearance < (eff + geo.CEILING_MARGIN))
        eff = np.where(trig, np.maximum(0.0, clearance - geo.CEILING_MARGIN - 0.0005), eff)
        handled = handled | trig

    ceiling_sweep = H + buf - th - h / 2.0 - geo.START_MARGIN
    sweep_z = np.minimum(ceiling_sweep, world_z + eff)
    band = np.where(on_floor, BAND_FLOOR, np.where(on_shelf, BAND_SHELF, BAND_HIGH))
    return sweep_z - h / 2.0, sweep_z + h / 2.0, band


def fit_and_reach(masks, remaining, geo, cdict, voxel=VOXEL):
    """残荷物×全 orientation を全位置で列挙し、covered / supported / reachable と
    (a) の分類マスクを構成する。"""
    empty = masks['empty']
    nx, ny, nz = empty.shape
    xs = masks['xs']; ys = masks['ys']
    y_entry = -cdict['width'] / 2.0

    A = _pad3(~empty)                                  # 最終位置の当たり(Phase22 と同一)
    solid_below = masks['occupied'] | ~masks['in_container']
    Ab = np.pad(solid_below.astype(np.int32).cumsum(0).cumsum(1), ((1, 0), (1, 0), (0, 0)),
                mode='constant')
    # 掃引の障害物 = 既配置荷物 + 棚。validator._move_item はコンテナ壁とは判定しない
    # (壁・切り欠きは搬入開始 x の制限 transport_x_bounds としてのみ効く)。
    sweep_obstacles = [(ab, False) for ab in geo.packed_obstacles(cdict)]
    sweep_obstacles += [(ab, True) for ab in geo.static_obstacles(cdict)]

    out = {name: np.zeros(empty.shape, dtype=bool) for name in
           ('covered', 'supported', 'reach_strict', 'reach_optimistic',
            'blk_item_only', 'blk_shelf', 'blk_xshift',
            'band_floor', 'band_shelf', 'band_high',
            'obs0', 'obs1', 'obs2', 'obs3plus')}

    seen = set()
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
            I, J, K = nx - di + 1, ny - dj + 1, nz - dk + 1

            # --- 最終位置が空か(Phase22 と同一) ---
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

            # --- 支持(Phase22 と同一) ---
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

            # --- 楽観近似(Phase22 と同じ判定。比較用) ---
            corr_opt = (A[di:di + I, dj:dj + J, dk:dk + K]
                        - A[0:I, dj:dj + J, dk:dk + K]
                        - A[di:di + I, dj:dj + J, 0:K]
                        + A[0:I, dj:dj + J, 0:K])
            ok_opt = ok_sup & (corr_opt == 0)
            if ok_opt.any():
                out['reach_optimistic'] |= _positions_to_covered(ok_opt, key, empty.shape)

            # --- 厳密な搬入経路(validator.check_transport_path と同じ判定を連続座標で) ---
            # 支持まで通った位置だけを疎に取り出し、位置ごとに y掃引→x掃引の2箱を
            # 全障害物と AABB 分離判定する(margin_xy=SAFETY_MARGIN_XY, margin_z=SWEEP_Z_MARGIN)。
            pi, pj, pk = np.nonzero(ok_sup)
            z0_arr, z1_arr, band_arr = _corridor_geometry(cdict, geo, dk, K, voxel)
            mxy = geo.SAFETY_MARGIN_XY
            mz = geo.SWEEP_Z_MARGIN

            x0 = xs[pi] - voxel / 2.0
            x1 = x0 + di * voxel
            y0 = ys[pj] - voxel / 2.0
            y1 = y0 + dj * voxel
            cz0 = z0_arr[pk]; cz1 = z1_arr[pk]
            band = band_arr[pk]

            x_min_w, x_max_w = geo.transport_x_bounds(cdict, di * voxel / 2.0)
            cxp = (x0 + x1) / 2.0
            if x_min_w > x_max_w:
                # この向きは開口部(切り欠きで削られた幅)を通らない = 全位置が到達不能
                scx = cxp.copy()
                needs_x = np.ones(pi.shape[0], dtype=bool)
                impossible = True
            else:
                scx = np.clip(cxp, x_min_w, x_max_w)
                needs_x = np.abs(scx - cxp) > 1e-9
                impossible = False
            sx0 = scx - di * voxel / 2.0
            sx1 = scx + di * voxel / 2.0
            x2lo = np.minimum(sx0, x0); x2hi = np.maximum(sx1, x1)

            hit_any = np.zeros(pi.shape[0], dtype=bool)
            hit_item = np.zeros(pi.shape[0], dtype=bool)
            hit_shelf = np.zeros(pi.shape[0], dtype=bool)
            nobs = np.zeros(pi.shape[0], dtype=np.int16)
            if not impossible:
                for (oc, oh), is_shelf in sweep_obstacles:
                    fz = (cz1 + mz > oc[2] - oh[2]) & (oc[2] + oh[2] + mz > cz0)
                    if not fz.any():
                        continue
                    fy_far = (oc[1] + oh[1] + mxy > y_entry)
                    p1 = (fz & (sx1 + mxy > oc[0] - oh[0]) & (oc[0] + oh[0] + mxy > sx0)
                          & (y1 + mxy > oc[1] - oh[1]) & fy_far)
                    p2 = (needs_x & fz & (x2hi + mxy > oc[0] - oh[0]) & (oc[0] + oh[0] + mxy > x2lo)
                          & (y1 + mxy > oc[1] - oh[1]) & (oc[1] + oh[1] + mxy > y0))
                    hit = p1 | p2
                    if not hit.any():
                        continue
                    hit_any |= hit
                    nobs += hit
                    if is_shelf:
                        hit_shelf |= hit
                    else:
                        hit_item |= hit
            else:
                hit_any[:] = True

            def _scatter(sel):
                m = np.zeros(ok_sup.shape, dtype=bool)
                m[pi[sel], pj[sel], pk[sel]] = True
                return m

            if (~hit_any).any():
                out['reach_strict'] |= _positions_to_covered(_scatter(~hit_any), key, empty.shape)
            if hit_any.any():
                for sel, name in (
                        (hit_any & hit_item & ~hit_shelf, 'blk_item_only'),
                        (hit_any & hit_shelf, 'blk_shelf'),
                        (hit_any & needs_x, 'blk_xshift'),
                        (hit_any & (band == BAND_FLOOR), 'band_floor'),
                        (hit_any & (band == BAND_SHELF), 'band_shelf'),
                        (hit_any & (band == BAND_HIGH), 'band_high'),
                        (hit_any & (nobs == 1), 'obs1'),
                        (hit_any & (nobs == 2), 'obs2'),
                        (hit_any & (nobs >= 3), 'obs3plus'),
                        (hit_any & (nobs == 0), 'obs0')):
                    if sel.any():
                        out[name] |= _positions_to_covered(_scatter(sel), key, empty.shape)

    for k in out:
        out[k] &= empty
    return out


def _ray_obstacle_counts(masks):
    """各 voxel について、手前(y=front)からその voxel まで直進したときに横切る
    障害物(荷物・棚)の個数。ラベル画像の y 方向の「立ち上がり」を数える。"""
    lab = masks['labels']
    prev = np.zeros_like(lab)
    prev[:, 1:, :] = lab[:, :-1, :]
    starts = ((lab != 0) & (lab != prev)).astype(np.int32)
    # 自分より手前(排他)の個数 = 累積和を1つずらす
    c = np.cumsum(starts, axis=1)
    out = np.zeros_like(c)
    out[:, 1:, :] = c[:, :-1, :]
    return out


def analyze_scene(task_config, module_path, label):
    mod_prefix = '.'.join(module_path.rstrip('/').split('/'))
    geo = importlib.import_module(mod_prefix + '.geometry')

    state = run_to_stall(task_config, module_path)
    cres = measure_c(state, module_path)

    vv = VOXEL ** 3
    agg = dict(in_container=0.0, empty=0.0, covered=0.0, supported=0.0,
               reach_strict=0.0, reach_optimistic=0.0,
               a_strict=0.0, a_optimistic=0.0,
               a_band_floor=0.0, a_band_shelf=0.0, a_band_high=0.0,
               a_item_only=0.0, a_shelf=0.0, a_xshift=0.0,
               a_obs0=0.0, a_obs1=0.0, a_obs2=0.0, a_obs3plus=0.0)
    ray_hist = [0.0] * 6            # 参考: voxel 自身の高さで手前から直進したとき横切る個数
    lane_a = [0.0] * 10             # X を10レーンに分けた (a) 体積
    lane_empty = [0.0] * 10
    lane_obs1 = [0.0] * 10          # そのレーンで「経路上の障害物1個」で塞がれている (a) 体積
    region = dict(upper=0.0, back=0.0, under_cut=0.0, front=0.0, other=0.0)
    region_a = dict(upper=0.0, back=0.0, under_cut=0.0, front=0.0, other=0.0)
    axis_hist = {'x': [0.0] * 10, 'y': [0.0] * 10, 'z': [0.0] * 10}
    axis_hist_a = {'x': [0.0] * 10, 'y': [0.0] * 10, 'z': [0.0] * 10}
    comps = []
    eff_geom_volume = 0.0

    for cdict in cres['containers_after']:
        masks = build_masks(cdict, geo)
        in_c = masks['in_container']; empty = masks['empty']
        eff_geom_volume += in_c.sum() * vv
        res = fit_and_reach(masks, cres['remaining_after'], geo, cdict)

        a_strict = res['supported'] & ~res['reach_strict']
        a_opt = res['supported'] & ~res['reach_optimistic']

        agg['in_container'] += in_c.sum() * vv
        agg['empty'] += empty.sum() * vv
        agg['covered'] += res['covered'].sum() * vv
        agg['supported'] += res['supported'].sum() * vv
        agg['reach_strict'] += res['reach_strict'].sum() * vv
        agg['reach_optimistic'] += res['reach_optimistic'].sum() * vv
        agg['a_strict'] += a_strict.sum() * vv
        agg['a_optimistic'] += a_opt.sum() * vv

        # (i) 帯域: 低い帯で到達しようとしている空間を優先して数える(排他)
        bf = a_strict & res['band_floor']
        bs = a_strict & res['band_shelf'] & ~bf
        bh = a_strict & ~bf & ~bs
        agg['a_band_floor'] += bf.sum() * vv
        agg['a_band_shelf'] += bs.sum() * vv
        agg['a_band_high'] += bh.sum() * vv

        # (ii) 障害物種別: 「荷物だけで塞がれている」= 順序/配置で開けられる分を優先(排他)
        io = a_strict & res['blk_item_only']
        sh = a_strict & ~io
        agg['a_item_only'] += io.sum() * vv
        agg['a_shelf'] += sh.sum() * vv
        agg['a_xshift'] += (a_strict & res['blk_xshift']).sum() * vv

        # (iii) 経路上の障害物個数(排他: 少ない個数で塞がれている方を優先して数える)
        o1 = a_strict & res['obs1']
        o2 = a_strict & res['obs2'] & ~o1
        o3 = a_strict & res['obs3plus'] & ~o1 & ~o2
        o0 = a_strict & ~o1 & ~o2 & ~o3
        agg['a_obs1'] += o1.sum() * vv
        agg['a_obs2'] += o2.sum() * vv
        agg['a_obs3plus'] += o3.sum() * vv
        agg['a_obs0'] += o0.sum() * vv

        rc = _ray_obstacle_counts(masks)
        for n in range(6):
            m = (rc == n) if n < 5 else (rc >= 5)
            ray_hist[n] += (a_strict & m).sum() * vv
        nx = empty.shape[0]
        edges = np.linspace(0, nx, 11).astype(int)
        for li in range(10):
            sl = slice(edges[li], edges[li + 1])
            lane_a[li] += a_strict[sl].sum() * vv
            lane_empty[li] += empty[sl].sum() * vv
            lane_obs1[li] += o1[sl].sum() * vv

        # (iv) 位置
        xs, ys, zs = masks['xs'], masks['ys'], masks['zs']
        L, W, H = cdict['length'], cdict['width'], cdict['height']
        ox = cdict['center'][0]; th = cdict['thickness']; cut_x = cdict['cut_x']; cut_y = cdict['cut_y']
        xn = (xs - (ox - L / 2)) / L
        yn = (ys + W / 2) / W
        zn = zs / H
        for name, norm, ax in (('x', xn, 0), ('y', yn, 1), ('z', zn, 2)):
            bins = np.clip((norm * 10).astype(int), 0, 9)
            for b in range(10):
                sel = bins == b
                idx = [slice(None)] * 3; idx[ax] = sel
                axis_hist[name][b] += empty[tuple(idx)].sum() * vv
                axis_hist_a[name][b] += a_strict[tuple(idx)].sum() * vv

        Xn = xn[:, None, None]; Yn = yn[None, :, None]; Zn = zn[None, None, :]
        under_cut = (Xn < (th + cut_x) / L) & (Zn < cut_y / H)
        upper = (~under_cut) & (Zn > 2.0 / 3.0)
        back = (~under_cut) & (~upper) & (Yn > 2.0 / 3.0)
        front = (~under_cut) & (~upper) & (Yn < 1.0 / 3.0)
        other = ~(under_cut | upper | back | front)
        for name, m in (('under_cut', under_cut), ('upper', upper), ('back', back),
                        ('front', front), ('other', other)):
            region[name] += (empty & m).sum() * vv
            region_a[name] += (a_strict & m).sum() * vv

        try:
            from scipy import ndimage
            labc, n = ndimage.label(empty)
            if n:
                cnt = np.bincount(labc.ravel())[1:]
                cents = ndimage.center_of_mass(empty, labc, np.arange(1, n + 1))
                for c_i, ct in zip(cnt, cents):
                    comps.append({
                        'volume': float(c_i * vv),
                        'cx': float(np.interp(ct[0], np.arange(len(xn)), xn)),
                        'cy': float(np.interp(ct[1], np.arange(len(yn)), yn)),
                        'cz': float(np.interp(ct[2], np.arange(len(zn)), zn)),
                    })
        except Exception:
            pass

    return {
        'label': label,
        'container_volume': state['container_volume'],
        'placed_volume': state['placed_volume'],
        'n_packed': state['n_packed'], 'n_total': state['n_total'],
        'optimize': state['optimize'],
        'c_volume': cres['c_volume'],
        'n_remaining_after': len(cres['remaining_after']),
        'eff_geom_volume': eff_geom_volume,
        'voxel': agg,
        'ray_hist': ray_hist,
        'lane_a': lane_a, 'lane_empty': lane_empty, 'lane_obs1': lane_obs1,
        'region': region, 'region_a': region_a,
        'axis_hist': axis_hist, 'axis_hist_a': axis_hist_a,
        'components': sorted(comps, key=lambda d: -d['volume'])[:50],
    }


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--config-path', nargs='+', required=True)
    ap.add_argument('--module-path', default='agents/mysolver/')
    ap.add_argument('--optimize-budget', type=float, default=None)
    ap.add_argument('--out', required=True)
    args = ap.parse_args()
    if args.optimize_budget is not None:
        os.environ['MYSOLVER_OPTIMIZE_BUDGET'] = str(args.optimize_budget)

    paths = []
    for pat in args.config_path:
        m = sorted(glob.glob(pat))
        paths.extend(m if m else [pat])

    scenes = {}
    for cp in paths:
        with open(cp) as f:
            cfg = json.load(f)
        for tid, tc in cfg.items():
            label = f'{os.path.basename(cp)}::{tid}'
            t0 = time.time()
            try:
                with open(os.devnull, 'w') as dn, redirect_stdout(dn):
                    r = analyze_scene(tc, args.module_path, label)
            except Exception:
                print(f'[{label}] ERROR {traceback.format_exc().splitlines()[-1]}', flush=True)
                continue
            scenes[label] = r
            v = r['voxel']
            print(f'[{label}] empty={v["empty"]:.2f} sup={v["supported"]:.2f} '
                  f'a_opt={v["a_optimistic"]:.2f} a_strict={v["a_strict"]:.2f} '
                  f'(floor={v["a_band_floor"]:.2f}/shelf={v["a_band_shelf"]:.2f}/'
                  f'lift={v["a_band_high"]:.2f}) item_only={v["a_item_only"]:.2f} '
                  f'({time.time()-t0:.0f}s)', flush=True)
            with open(args.out, 'w') as f:
                json.dump({'scenes': scenes}, f)

    with open(args.out, 'w') as f:
        json.dump({'scenes': scenes}, f)
    print(f'\nwrote {args.out}')


if __name__ == '__main__':
    main()
