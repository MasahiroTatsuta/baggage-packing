"""
tools/phase22_void_audit.py

Phase22 ターゲット1: 行き詰まり時点の「空き体積」を原因別に分解する。

背景:
    Phase21 で fill_strict ≒ 0.63 × (置けた体積/総容積) と確定し、唯一の主レバーは
    「置けた体積」に絞られた。現在の体積利用率は 37.8%(54.327/143.855 m³)、つまり
    **62%が空き**である。本ツールはその62%を以下に分類し体積シェアを出す。

      (c) 到達可能かつ入る荷物が実在するのに探索が見つけられなかった空間
      (b) どの残荷物も寸法的に入らない空間(細切れの空隙)
      (a) 入る荷物はあるが搬入経路・支持が無くて使えない空間
      (d) コンテナ形状・指標定義に由来し構造的に使えない体積

測定の考え方(2段構え):
    **Stage A: (c) は実コードで厳密に測る。**
    行き詰まり状態から、production より強い探索(グリッド密度を上げ、予算を潤沢にした
    `planner.plan`)を影シミュレータ上で回し、追加で置けた体積を (c) とする。
    voxel近似で「置けるはず」と判定するより、実際の合法性判定コードで置けることを
    示すほうが証拠として強い(その分だけ確実に production の取りこぼしである)。

    **Stage B: 残った空きを voxel で分解する。**
    コンテナ内部を等間隔voxelに離散化し、
      ・in_container: 7面すべての内側(切り欠き五角柱)かつ棚を除く
      ・occupied:     配置済み荷物のAABB
    としたうえで、残荷物×全orientationについて3D積分画像で「その向きの直方体が
    完全に空voxelへ収まる位置」を全列挙し、少なくとも1つの配置に覆われるvoxelを
    「寸法的には使える空間」とする。覆われないvoxelが (b) である。

    voxel量子化は保守側に丸める(荷物の寸法はceil、つまり必要voxel数を多めに要求する)。
    したがって (b) は過大評価されうるが、(c)/(a) を過大評価しない側に倒れる。

    (d) は2成分を別立てで報告する:
      ・棚下など「幾何的に薄い」ゾーンの空き体積
      ・**fill_score の分母のかさ上げ**: Container.volume は inner_height を
        `height - thickness - buffer` で計算するが、実際に床面と天井面で挟まれた
        z区間は `height - 2*thickness` しかない。この差の分だけ分母が過大であり、
        fill_score は原理的に100に到達できない。理論上界の算出に必要。

使い方:
    PYTHONPATH=. .venv/bin/python tools/phase22_void_audit.py \
        --config-path 'configs/gen/suite_*.json' --out results/phase22_void.json

src/ は一切変更しない。agents/ も読み取り専用で使う(定数の一時的な引き上げのみ)。
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

VOXEL = 0.025          # [m] 離散化幅。荷物寸法0.2〜0.9mに対し十分細かく、かつ26シーン回せる粒度
SUPPORT_RATIO = 0.55   # planner.MIN_UNION_SUPPORT_RATIO と同水準


# ----------------------------------------------------------------------------
# Stage A: 行き詰まり状態の取得と、(c) の厳密な測定
# ----------------------------------------------------------------------------
def run_to_stall(task_config, module_path):
    """本番と同じ手順でエピソードを走らせ、行き詰まり時点の状態を返す。"""
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
    """production より強い探索で追加配置できる体積 = (c) を実コードで測る。

    `planner.RETRY_GRID_DENSITY` を一時的に引き上げて `planner.plan` を潤沢な予算で回す。
    plan() は「通常密度 -> 優先コンテナ制約の緩和 -> 密度を上げた最終リトライ」の順に
    エスカレートするので、この定数だけを上げれば production の上位互換になる。
    """
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
    added = []
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
            v = item['length'] * item['width'] * item['height']
            added_vol += v
            added.append({'index': int(item['index']), 'volume': v,
                          'density': planner.RETRY_GRID_DENSITY})
    finally:
        planner.RETRY_GRID_DENSITY = saved

    return {'c_volume': added_vol, 'c_items': added,
            'containers_after': containers, 'remaining_after': pool}


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
    """in_container(切り欠き五角柱の内側、棚を除く)と occupied(配置済み荷物)を返す。"""
    xs, ys, zs = _axes(cdict, voxel)
    X, Y, Z = np.meshgrid(xs, ys, zs, indexing='ij')
    pts = np.stack([X.ravel(), Y.ravel(), Z.ravel()], axis=1)

    n_vecs = np.array(cdict['n_vecs']); points = np.array(cdict['points'])
    inside = np.ones(pts.shape[0], dtype=bool)
    for n, p in zip(n_vecs, points):
        inside &= (pts - p) @ n <= 0.0
    in_c = inside.reshape(X.shape)

    struct = np.zeros(X.shape, dtype=bool)
    for center, half in geo.static_obstacles(cdict):
        struct |= ((np.abs(X - center[0]) <= half[0]) &
                   (np.abs(Y - center[1]) <= half[1]) &
                   (np.abs(Z - center[2]) <= half[2]))

    occ = np.zeros(X.shape, dtype=bool)
    for it in cdict['packed_items']:
        c, h = geo.item_world_aabb(it)
        occ |= ((np.abs(X - c[0]) <= h[0]) &
                (np.abs(Y - c[1]) <= h[1]) &
                (np.abs(Z - c[2]) <= h[2]))

    in_c &= ~struct
    empty = in_c & ~occ
    return {'xs': xs, 'ys': ys, 'zs': zs, 'in_container': in_c, 'occupied': occ & in_c,
            'empty': empty, 'struct': struct}


def _window_any(a, size, axis):
    """a を axis 方向に「幅 size の窓のどこかが True か」へ変換する(分離可能な膨張)。

    位置 q に置いた荷物は [q, q+size) を覆うので、voxel p が覆われる条件は
    「区間 [p-size+1, p] のどこかに位置 q が存在する」。累積和の差分で O(N) に計算する。
    """
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
    """左下隅位置の集合 pos を、その直方体が覆う voxel 集合へ展開する。"""
    di, dj, dk = dims
    full = np.zeros(shape, dtype=bool)
    full[:pos.shape[0], :pos.shape[1], :pos.shape[2]] = pos
    out = _window_any(full, di, 0)
    out = _window_any(out, dj, 1)
    out = _window_any(out, dk, 2)
    return out


def fit_coverage(masks, remaining, geo, voxel=VOXEL):
    """残荷物のどれか1つでも収まりうるvoxelの集合(寸法的な可用性)を返す。

    各 (荷物, orientation) について、その直方体が完全に空voxelへ収まる左下隅位置を
    3D積分画像で全列挙し、覆われるvoxelを covered / supported へ OR していく。
    荷物の寸法は ceil で丸める(=必要voxel数を多めに要求する保守側の丸め)。
    """
    empty = masks['empty']
    nx, ny, nz = empty.shape
    blocked = (~empty).astype(np.int32)
    A = np.pad(blocked.cumsum(0).cumsum(1).cumsum(2), ((1, 0), (1, 0), (1, 0)), mode='constant')
    solid_below = masks['occupied'] | ~masks['in_container']
    Ab = np.pad(solid_below.astype(np.int32).cumsum(0).cumsum(1), ((1, 0), (1, 0), (0, 0)),
                mode='constant')

    covered = np.zeros(empty.shape, dtype=bool)
    supported = np.zeros(empty.shape, dtype=bool)
    reachable = np.zeros(empty.shape, dtype=bool)
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
            # 支持: 直下の1層の footprint が SUPPORT_RATIO 以上 solid(k=0 は内床面に接地)
            foot = (Ab[di:di + I, dj:dj + J, :] - Ab[0:I, dj:dj + J, :]
                    - Ab[di:di + I, 0:J, :] + Ab[0:I, 0:J, :])
            need = SUPPORT_RATIO * di * dj
            sup_ok = np.zeros_like(ok)
            below = np.arange(K) - 1
            v = below >= 0
            if v.any():
                sup_ok[:, :, v] = foot[:, :, below[v]] >= need
            sup_ok[:, :, ~v] = True
            covered |= _positions_to_covered(ok, key, empty.shape)
            ok_sup = ok & sup_ok
            if ok_sup.any():
                supported |= _positions_to_covered(ok_sup, key, empty.shape)
                # 搬入経路(y方向掃引): 手前 y=0 から目標yまでの掃引域が空いているか。
                # 最終高さのまま掃引する楽観近似(実際は非直置きだと START_Z 分持ち上がる)。
                corr = (A[di:di + I, dj:dj + J, dk:dk + K]
                        - A[0:I, dj:dj + J, dk:dk + K]
                        - A[di:di + I, dj:dj + J, 0:K]
                        + A[0:I, dj:dj + J, 0:K])
                ok_reach = ok_sup & (corr == 0)
                if ok_reach.any():
                    reachable |= _positions_to_covered(ok_reach, key, empty.shape)
    return covered & empty, supported & empty, reachable & empty


def analyze_scene(task_config, module_path, label):
    mod_prefix = '.'.join(module_path.rstrip('/').split('/'))
    geo = importlib.import_module(mod_prefix + '.geometry')

    state = run_to_stall(task_config, module_path)
    cres = measure_c(state, module_path)

    out = {
        'label': label,
        'container_volume': state['container_volume'],
        'placed_volume': state['placed_volume'],
        'n_packed': state['n_packed'], 'n_total': state['n_total'],
        'c_volume': cres['c_volume'],
        'c_items': cres['c_items'],
        'n_remaining_after': len(cres['remaining_after']),
    }

    # Stage B は「強い探索でも置けなくなった状態」に対して行う
    tot = dict(in_container=0.0, empty=0.0, covered=0.0, supported=0.0, reachable=0.0)
    vv = VOXEL ** 3
    comp_sizes = []
    eff_geom_volume = 0.0
    for cdict in cres['containers_after']:
        masks = build_masks(cdict, geo)
        in_c = masks['in_container']; empty = masks['empty']
        eff_geom_volume += in_c.sum() * vv
        covered, supported, reachable = fit_coverage(masks, cres['remaining_after'], geo)
        tot['in_container'] += in_c.sum() * vv
        tot['empty'] += empty.sum() * vv
        tot['covered'] += covered.sum() * vv
        tot['supported'] += supported.sum() * vv
        tot['reachable'] += reachable.sum() * vv
        # 連結成分サイズ分布
        try:
            from scipy import ndimage
            lab, n = ndimage.label(empty)
            if n:
                cnt = np.bincount(lab.ravel())[1:]
                comp_sizes.extend((cnt * vv).tolist())
        except Exception:
            pass
    out['voxel'] = tot
    out['eff_geom_volume'] = eff_geom_volume
    out['component_volumes'] = sorted(comp_sizes, reverse=True)[:200]
    return out


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
            print(f'[{label}] placed={r["placed_volume"]:.2f} c={r["c_volume"]:.3f} '
                  f'empty={v["empty"]:.2f} cov={v["covered"]:.2f} sup={v["supported"]:.2f} '
                  f'reach={v["reachable"]:.2f} '
                  f'({time.time()-t0:.0f}s)', flush=True)
            with open(args.out, 'w') as f:
                json.dump({'scenes': scenes}, f)

    with open(args.out, 'w') as f:
        json.dump({'scenes': scenes}, f)
    print(f'\nwrote {args.out}')


if __name__ == '__main__':
    main()
