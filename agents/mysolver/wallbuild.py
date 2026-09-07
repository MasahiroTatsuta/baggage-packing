"""WBC — Wall-Building Constructor(2026-09-07 着手、docs/wallbuild_design.md)。

planner の候補生成を Extreme Point 貪欲 → maximal-space + 層(壁)構築へ差し替える第1手。
コンテナを奥(y 最大)→ 手前へ maximal space 単位で埋め、参照空間を常に「最も奥・最も低い・
最も左」から選ぶことで front-to-back を構築規則としてハード保証する。搬入経路は「まだ
埋めていない手前空間」を必ず通れるため、is_valid 死(搬入経路衝突)が構造的に起きない。

配置そのもの(着地 z・支持・内包・搬入経路の検証)は既存 `planner.plan` を region 制約付きで
呼んで流用する(新規の幾何は書かない = Phase86 の教訓)。WBC の寄与は「どの空間を・どの順で
埋めるか」の選択規則。

既定 `MYSOLVER_WBC=0` では `ordering.build_plan` から呼ばれない。
"""
import os
import time

import numpy as np

from . import geometry as geo
from . import planner
from . import simulate

WBC_CAND_ITEMS = int(os.environ.get('MYSOLVER_WBC_CAND_ITEMS', '12'))
WBC_STEP_S = float(os.environ.get('MYSOLVER_WBC_STEP_S', '0.8'))
WBC_W_VOL = float(os.environ.get('MYSOLVER_WBC_W_VOL', '1.0'))
WBC_W_BEHIND = float(os.environ.get('MYSOLVER_WBC_W_BEHIND', '1.0'))
WBC_W_FRONT = float(os.environ.get('MYSOLVER_WBC_W_FRONT', '1.0'))
WBC_MIN_SPACE = float(os.environ.get('MYSOLVER_WBC_MIN_SPACE', '0.02'))   # この辺長未満の空間は無視
WBC_Z_TOL = float(os.environ.get('MYSOLVER_WBC_Z_TOL', '0.06'))           # ref の z 帯からの許容はみ出し
WBC_WALL_BAND = float(os.environ.get('MYSOLVER_WBC_WALL_BAND', '0.0'))    # >0 なら現在の壁帯の厚み(0=自動)
WBC_INIT_BAND_MULT = float(os.environ.get('MYSOLVER_WBC_INIT_BAND_MULT', '1.8'))  # 壁の初期厚み = これ×最小荷物辺
WBC_SLACK_MARGIN = float(os.environ.get('MYSOLVER_WBC_SLACK_MARGIN', '-0.008'))    # 採用する候補の内包スラック上限(実閾値 -0.005 に対し余裕)
WBC_AABB_INFLATE = float(os.environ.get('MYSOLVER_WBC_AABB_INFLATE', '0.0'))    # maximal-space 更新で配置 AABB を各辺これだけ膨張(次の荷物に余裕)
WBC_1C_ONLY = os.environ.get('MYSOLVER_WBC_1C_ONLY', '1') == '1'                  # 単一・非優先コンテナのみ WBC(2c/優先は従来経路)

# space = (ci, x0, y0, z0, x1, y1, z1)  すべて container-local


def _vol(s):
    return max(0.0, s[4] - s[1]) * max(0.0, s[5] - s[2]) * max(0.0, s[6] - s[3])


def _contains(a, b, eps=1e-9):
    """a が b を(ほぼ)完全に包含するか。"""
    return (a[0] == b[0] and a[1] - eps <= b[1] and a[2] - eps <= b[2] and a[3] - eps <= b[3]
            and a[4] + eps >= b[4] and a[5] + eps >= b[5] and a[6] + eps >= b[6])


def _overlap(s, box):
    # タプルは (ci, x0, y0, z0, x1, y1, z1) = index (0, 1,2,3, 4,5,6)。
    return (s[1] < box[4] and box[1] < s[4]
            and s[2] < box[5] and box[2] < s[5]
            and s[3] < box[6] and box[3] < s[6])


def _split_space(s, box):
    """maximal space s から障害物 box(同じ ci・local AABB)の占有分を引いた極大直方体群。
    最大6個(左/右/手前-y/奥+y/下/上)。s と box が重ならなければ [s]。"""
    ci = s[0]
    if not _overlap(s, box):
        return [s]
    x0, y0, z0, x1, y1, z1 = s[1], s[2], s[3], s[4], s[5], s[6]
    bx0, by0, bz0, bx1, by1, bz1 = box[1], box[2], box[3], box[4], box[5], box[6]
    out = []
    if bx0 > x0:
        out.append((ci, x0, y0, z0, min(bx0, x1), y1, z1))            # 左
    if bx1 < x1:
        out.append((ci, max(bx1, x0), y0, z0, x1, y1, z1))            # 右
    if by0 > y0:
        out.append((ci, x0, y0, z0, x1, min(by0, y1), z1))           # 手前
    if by1 < y1:
        out.append((ci, x0, max(by1, y0), z0, x1, y1, z1))           # 奥
    if bz0 > z0:
        out.append((ci, x0, y0, z0, x1, y1, min(bz0, z1)))           # 下
    if bz1 < z1:
        out.append((ci, x0, y0, max(bz1, z0), x1, y1, z1))           # 上
    return [o for o in out if _vol(o) > 1e-9]


def _prune(spaces):
    """極小空間・重複を除き、他に真に包含される空間を除く。"""
    kept = []
    seen = set()
    for s in spaces:
        if s in seen:
            continue
        if min(s[4] - s[1], s[5] - s[2], s[6] - s[3]) < WBC_MIN_SPACE:
            continue
        seen.add(s)
        kept.append(s)
    final = []
    for i, s in enumerate(kept):
        if any(j != i and _contains(kept[j], s) and _vol(kept[j]) >= _vol(s)
               and not (j > i and kept[j] == s) for j in range(len(kept))):
            continue
        final.append(s)
    return final


def _interior_bounds(cont):
    """container-local (= world for y,z / world-ox for x) の内部直方体。
    指示書: place_pos は local、world = (x+ox, y, z)。x は中心0、y/z は world そのもの。"""
    L = cont['length']; W = cont['width']; H = cont['height']
    th = cont.get('thickness', 0.0)
    cy = cont['center'][1]; cz = cont['center'][2]
    # z は world 座標。床上面 = cz - H/2 + th、天井 = cz + H/2。
    return (-L / 2.0 + th, cy - W / 2.0 + th, cz - H / 2.0 + th,
            L / 2.0 - th, cy + W / 2.0 - th, cz + H / 2.0)


def init_maximal_spaces(containers):
    """各コンテナの初期 maximal space 群。cutcorner の斜め面・棚・既積みを障害物として subtract。"""
    S = []
    for ci, cont in enumerate(containers):
        x0, y0, z0, x1, y1, z1 = _interior_bounds(cont)
        spaces = [(ci, x0, y0, z0, x1, y1, z1)]
        obstacles = []
        # cutcorner: x が最小側・z が最大側の楔を矩形で保守近似
        cut_x = cont.get('cut_x', 0.0) or 0.0
        cut_y = cont.get('cut_y', 0.0) or 0.0
        if cut_x > 1e-6 and cut_y > 1e-6:
            obstacles.append((ci, x0, y0, z1 - cut_y, x0 + cut_x, y1, z1))
        # 棚(脇の小棚 + あれば大棚)
        for ab in geo.static_obstacles(cont):
            c, h = np.asarray(ab[0], float), np.asarray(ab[1], float)
            ox = cont['center'][0]
            obstacles.append((ci, c[0] - ox - h[0], c[1] - h[1], c[2] - h[2],
                              c[0] - ox + h[0], c[1] + h[1], c[2] + h[2]))
        # 既積み
        for it in cont.get('packed_items', []):
            aabb = _item_local_aabb(cont, it)
            if aabb is not None:
                obstacles.append(aabb)
        for box in obstacles:
            nxt = []
            for s in spaces:
                nxt.extend(_split_space(s, box))
            spaces = _prune(nxt)
        S.extend(spaces)
    return _prune(S)


def _item_local_aabb(cont, packed_item):
    """packed_items の1要素 → container-local AABB space タプル。"""
    pos = packed_item.get('pos')
    if pos is None:
        return None
    ox = cont['center'][0]
    try:
        R = geo.quat_abs_rotmat(packed_item['orn'])
        hw = R @ np.array([packed_item['length'] / 2.0, packed_item['width'] / 2.0,
                           packed_item['height'] / 2.0])
        hw = np.abs(hw)
    except Exception:
        hw = np.array([packed_item['length'] / 2.0, packed_item['width'] / 2.0,
                       packed_item['height'] / 2.0])
    cx = pos[0] - ox
    ci = None  # 呼び出し側で埋める必要はない: init のループが ci を知っている
    return (0, cx - hw[0], pos[1] - hw[1], pos[2] - hw[2],
            cx + hw[0], pos[1] + hw[1], pos[2] + hw[2])


def _space_fits_any(s, items):
    """s(space タプル)に items のいずれかが(3辺のソート比較で)収まるか。"""
    sd = sorted((s[4] - s[1], s[5] - s[2], s[6] - s[3]))
    for it in items:
        idm = sorted((it['length'], it['width'], it['height']))
        if idm[0] <= sd[0] + 1e-9 and idm[1] <= sd[1] + 1e-9 and idm[2] <= sd[2] + 1e-9:
            return True
    return False


def _layer_score(item, act, ref):
    """壁は背面(ref[5], y_hi)から手前へ積む。背面に密着し、手前へ張り出さない候補を選好。"""
    pp = act['place_pos']
    half = geo.half_extent((item['length'], item['width'], item['height']), act['orientation'])
    vol = item['length'] * item['width'] * item['height']
    back_face = float(pp[1]) + float(half[1])
    front_face = float(pp[1]) - float(half[1])
    gap_behind = max(0.0, ref[5] - back_face)              # 背面からの隙間(小さいほど良い)
    protrude = max(0.0, ref[5] - front_face)               # 手前への張り出し(壁を薄く保つ)
    return (WBC_W_VOL * vol * 1000.0
            - WBC_W_BEHIND * gap_behind * 100.0
            - WBC_W_FRONT * protrude * 50.0)


def _placed_aabb(cont, item, act):
    pp = act['place_pos']
    ox = cont['center'][0]
    half = geo.half_extent((item['length'], item['width'], item['height']), act['orientation'])
    cx = float(pp[0])   # act['place_pos'] は既に container-local
    return (0, cx - half[0], float(pp[1]) - half[1], float(pp[2]) - half[2],
            cx + half[0], float(pp[1]) + half[1], float(pp[2]) + half[2])


WBC_SKYLINE = os.environ.get('MYSOLVER_WBC_SKYLINE', '1') == '1'
WBC_WALL_DEPTH_MULT = float(os.environ.get('MYSOLVER_WBC_WALL_DEPTH_MULT', '3.5'))
WBC_WALL_GAP = float(os.environ.get('MYSOLVER_WBC_WALL_GAP', '0.02'))  # 壁/床から荷物を離すクリアランス(inclusion margin 対策)


def _orients_for_depth(item, max_depth):
    """Y(壁の奥行き)方向の寸法が max_depth 以下になる向き。(orn, (dx,dy,dz)) を Y薄い順で。"""
    out = []
    for o in range(6):
        h = geo.half_extent((item['length'], item['width'], item['height']), o)
        d = (2.0 * h[0], 2.0 * h[1], 2.0 * h[2])
        if d[1] <= max_depth + 1e-6:
            out.append((o, d, float(h[1])))
    out.sort(key=lambda t: t[1][1])          # Y が薄い向きを優先
    return out


def _pack_wall(cont, ci, y_back, depth, x_lo, x_hi, z_lo, z_hi, items, prepacked_ids,
               strict_support):
    """1つの壁帯 [y_back-depth, y_back] × X[x_lo,x_hi] × Z[z_lo,z_hi] を X-Z 2D skyline で密に詰める。
    items(体積降順)から入るものを bottom-left で置き、planner.validate_planned_xy で最終検証。
    戻り値: (placements[list of dict], used_ids[set])。"""
    # skyline: X 軸上の区分 [(xs, xe, top_z), ...]、初期は床フラット。
    sky = [(x_lo, x_hi, z_lo)]
    placements = []
    used = set()
    _diag = {'no_orient': 0, 'no_slot': 0, 'validate_none': 0, 'tried': 0}
    progressed = True
    while progressed and items:
        progressed = False
        for it in list(items):
            if it['index'] in used:
                continue
            _diag['tried'] += 1
            best = None                     # (top_z_after, xs, orn, dxyz)
            _os = _orients_for_depth(it, depth)
            if not _os:
                _diag['no_orient'] += 1
            for orn, dxyz, hy in _os:
                w, h = dxyz[0], dxyz[2]
                if w > (x_hi - x_lo) + 1e-6 or h > (z_hi - z_lo) + 1e-6:
                    continue
                # skyline 上で、幅 w が収まる各開始 x について、その区間の最大 top_z を求める
                xs = x_lo
                while xs + w <= x_hi + 1e-6:
                    seg_top = z_lo
                    covered = False
                    for (sxs, sxe, st) in sky:
                        if sxe <= xs + 1e-9 or sxs >= xs + w - 1e-9:
                            continue
                        seg_top = max(seg_top, st)
                        covered = True
                    if not covered:
                        seg_top = z_lo
                    if seg_top + h <= z_hi + 1e-6:
                        cand = (seg_top + h, xs, orn, dxyz, seg_top, hy)
                        if best is None or (cand[0], cand[1]) < (best[0], best[1]):
                            best = cand
                    xs += max(0.02, w * 0.34)
            if best is None:
                if _os:
                    _diag['no_slot'] += 1
                continue
            top_after, xs, orn, dxyz, seg_top, hy = best
            hx, hzz = dxyz[0] / 2.0, dxyz[2] / 2.0
            lx = xs + hx
            ly = y_back - hy - WBC_WALL_GAP  # 背面密着(壁から少し離す = inclusion margin 対策)
            lz = seg_top + hzz + geo.REST_CLEARANCE   # skyline shelf(床 or 下の荷物)に載る
            half = np.array([hx, hy, hzz], dtype=np.float64)
            wp = geo.local_to_world(cont, (lx, ly, lz))[None, :]
            # 着地 z / 支持は skyline(shelf)が保証。ここでは内包 + 搬入経路だけ確認。
            if not bool(geo.check_inclusion_batch(cont, half, wp)[0]):
                _diag['validate_none'] += 1
                continue
            _obst = [(np.asarray(pit['pos'], float),
                      np.abs(geo.quat_abs_rotmat(pit['orn']) @ np.array(
                          [pit['length'] / 2, pit['width'] / 2, pit['height'] / 2])))
                     for pit in cont.get('packed_items', [])]
            if not bool(planner.transport_legal_batch(cont, half, wp, obstacles=_obst)[0]):
                _diag['validate_none'] += 1
                continue
            act = {'place_pos': np.array([lx, ly, lz], dtype=float), 'orientation': int(orn),
                   'container_idx': 0}
            cont['packed_items'].append(simulate._place(cont, dict(it), act))
            placements.append({'index': int(it['index']), 'container_idx': int(ci),
                               'place_pos': (float(lx), float(ly), float(lz)),
                               'orientation': int(orn)})
            used.add(it['index'])
            # skyline 更新: [xs, xs+w] 区間を top_after に持ち上げる
            xe = xs + dxyz[0]
            nsky = []
            for (sxs, sxe, st) in sky:
                if sxe <= xs + 1e-9 or sxs >= xe - 1e-9:
                    nsky.append((sxs, sxe, st))
                    continue
                if sxs < xs:
                    nsky.append((sxs, xs, st))
                if sxe > xe:
                    nsky.append((xe, sxe, st))
            nsky.append((xs, xe, top_after))
            sky = sorted(nsky)
            progressed = True
    if os.environ.get('MYSOLVER_WBC_DEBUG', '0') == '1':
        import sys
        print(f'[WBC]   _pack_wall diag={_diag}', file=sys.stderr)
    return placements, used


def _wbc_plan_skyline(container_list, items_by_index, item_list, lookahead_k, budget, wall_deadline):
    """壁カーソルを奥→前へ進め、各壁帯を _pack_wall(X-Z skyline)で密に詰める。"""
    conts = simulate.clone_containers(container_list)
    cont = conts[0]
    ci = 0
    prepacked_ids = geo.initial_prepacked_ids(conts)
    x0, y0, z0, x1, y1, z1 = _interior_bounds(cont)
    remaining = sorted(item_list, key=lambda it: -(it['length'] * it['width'] * it['height']))
    remaining = [dict(it) for it in remaining]
    min_dim = min(min(it['length'], it['width'], it['height']) for it in remaining)
    wall_depth = max(WBC_WALL_DEPTH_MULT * min_dim, min_dim + 0.03)
    _dbg = os.environ.get('MYSOLVER_WBC_DEBUG', '0') == '1'
    plan: list[dict] = []
    y_back = y1
    _nwall = 0
    while (remaining and y_back - z0 * 0 - y0 > 0.02 and not budget.exhausted()
           and time.perf_counter() < wall_deadline):
        depth = min(wall_depth, y_back - y0)
        g = WBC_WALL_GAP
        placements, used = _pack_wall(cont, ci, y_back, depth, x0 + g, x1 - g, z0 + g, z1 - g,
                                      remaining, prepacked_ids, strict_support=False)
        _nwall += 1
        if _dbg:
            import sys
            print(f'[WBC] wall#{_nwall} y[{y_back-depth:.2f},{y_back:.2f}] depth={depth:.2f} '
                  f'placed={len(placements)} plan_total={len(plan)+len(placements)} '
                  f'remaining={len(remaining)-len(used)}', file=sys.stderr)
        plan.extend(placements)
        remaining = [it for it in remaining if it['index'] not in used]
        y_back -= depth if placements else max(depth, min_dim + 0.03)
        if _nwall > 60:
            break
    if _dbg:
        import sys
        print(f'[WBC] FINAL(skyline) plan={len(plan)}/{len(item_list)} walls={_nwall} '
              f'remaining={len(remaining)}', file=sys.stderr)
    return plan


def wbc_plan(container_list, items_by_index, item_list, lookahead_k, budget, wall_deadline=None):
    """戻り値: plan_entries のリスト。anytime(budget / wall_deadline 枯渇で打ち切り)。"""
    if not container_list or not item_list:
        return []
    if WBC_1C_ONLY and (len(container_list) != 1
                        or any(c.get('is_prioritized') for c in container_list)):
        return []                       # 2c / 優先コンテナは従来経路(simulate_order)に委ねる
    if wall_deadline is None:
        wall_deadline = time.perf_counter() + 1e9

    if WBC_SKYLINE:
        return _wbc_plan_skyline(container_list, items_by_index, item_list, lookahead_k,
                                 budget, wall_deadline)
    conts = simulate.clone_containers(container_list)
    S = init_maximal_spaces(conts)
    remaining = sorted(item_list, key=lambda it: -(it['length'] * it['width'] * it['height']))
    remaining = [dict(it) for it in remaining]
    plan: list[dict] = []
    min_item_dim = min(min(it['length'], it['width'], it['height']) for it in remaining)
    _dbg = os.environ.get('MYSOLVER_WBC_DEBUG', '0') == '1'
    if _dbg:
        import sys
        for ci_, c_ in enumerate(conts):
            print(f'[WBC] cont{ci_} interior={tuple(round(v,3) for v in _interior_bounds(c_))} '
                  f'shelf={c_.get("shelf")} cut=({c_.get("cut_x")},{c_.get("cut_y")}) '
                  f'prepacked={len(c_.get("packed_items",[]))}', file=sys.stderr)
        for s in sorted(S, key=lambda s: (-s[2], s[3], s[1]))[:8]:
            print(f'[WBC]   init space ci{s[0]} '
                  f'x[{s[1]:.2f},{s[4]:.2f}] y[{s[2]:.2f},{s[5]:.2f}] z[{s[3]:.2f},{s[6]:.2f}]',
                  file=sys.stderr)

    # 壁カーソル: コンテナごとに「現在埋めている壁の背面 y」。コンテナ背面から前へ進む。
    # 壁帯 = [wall_back - wall_depth, wall_back]。wall_depth はその壁の最初の荷物が決める
    # (未確定なら最大荷物 1 個ぶんを仮に使う)。帯に届く空間だけ ref 候補、帯が埋まったら前進。
    wall_back = {ci: c['center'][1] + c['width'] / 2.0 for ci, c in enumerate(conts)}
    wall_depth: dict = {ci: None for ci in range(len(conts))}
    _band_init = max(WBC_INIT_BAND_MULT * min_item_dim, min_item_dim + 0.02)
    _front = {ci: c['center'][1] - c['width'] / 2.0 for ci, c in enumerate(conts)}
    _stall = {'no_usable': 0, 'no_ref': 0, 'no_best_planNone': 0, 'no_best_slack': 0,
              'no_best_z': 0, 'no_best_other': 0, 'iters': 0}
    while remaining and not budget.exhausted() and time.perf_counter() < wall_deadline:
        _stall['iters'] += 1
        # usable = 残り荷物の**どれか**が入る空間(top-K だけで見ると大型が入らない空間を
        # 誤って捨て、小物用のスリットを活かせず早期に止まる)。
        usable = [s for s in S if _space_fits_any(s, remaining)]
        if not usable:
            _stall['no_usable'] += 1
            break
        ref = None
        ci = None
        for _adv in range(256):
            cand = []
            for s in usable:
                d = wall_depth[s[0]] if wall_depth[s[0]] else _band_init
                if s[5] >= wall_back[s[0]] - d - 1e-6:
                    cand.append(s)
            if cand:
                ref = min(cand, key=lambda s: (s[3], -s[5], s[1]))
                ci = ref[0]
                break
            # 帯内に何も無い → 壁を1枚ぶん前進(contiguous に保つ)。前面に達したら打ち切り。
            advanced = False
            for k in wall_back:
                step = wall_depth[k] if wall_depth[k] else _band_init
                if wall_back[k] - step > _front[k] - 1e-6:
                    wall_back[k] = wall_back[k] - step
                    wall_depth[k] = None
                    advanced = True
            if not advanced:
                # 前面近くの薄い残り: 帯制限を外して全 usable から選ぶ
                ref = min(usable, key=lambda s: (s[3], -s[5], s[1]))
                ci = ref[0]
                break
        if ref is None:
            _stall['no_ref'] += 1
            break
        band = wall_depth[ci] if wall_depth[ci] else _band_init
        # region の y: 荷物**中心**が入る範囲。中心を [wall_back-band, wall_back] に絞ると
        # band より深い荷物が1つも入らない(band < 2*half_y で範囲が反転)。中心の下限を
        # 大きく緩め、背面密着は _layer_score(gap_behind/protrude)に任せる。
        region = (ref[1], max(ref[2], wall_back[ci] - 2.0 * band - _band_init), ref[3],
                  ref[4], min(ref[5], wall_back[ci] + 0.02), ref[6])
        # ref に3辺で収まる候補だけを、体積降順で最大 WBC_CAND_ITEMS 個試す。
        cand_items = [it for it in remaining if _space_fits_any(ref, [it])][:WBC_CAND_ITEMS]
        if not cand_items:
            cand_items = remaining[:WBC_CAND_ITEMS]
        best = None
        _r = {'planNone': 0, 'z': 0, 'slack': 0}
        for item in cand_items:
            if budget.exhausted() or time.perf_counter() >= wall_deadline:
                break
            try:
                act = planner.plan([conts[ci]], [dict(item)], max_pool_items=None,
                                   budget=budget.child_seconds(WBC_STEP_S), region=region)
            except Exception:
                act = None
            if act is None:
                _r['planNone'] += 1
                continue
            zc = float(act['place_pos'][2])
            if zc < ref[3] - WBC_Z_TOL or zc > ref[6] + WBC_Z_TOL:
                _r['z'] += 1
                continue
            # 実 evaluator の厳しい内包マージン(-0.005)を落とす候補は最初から採らない
            _half = geo.half_extent((item['length'], item['width'], item['height']),
                                    int(act['orientation']))
            _wp = geo.local_to_world(conts[ci], act['place_pos'])[None, :]
            if float(geo.inclusion_slack_batch(conts[ci], _half, _wp)[0]) > WBC_SLACK_MARGIN:
                _r['slack'] += 1
                continue
            sc = _layer_score(item, act, ref)
            if best is None or sc > best[0]:
                best = (sc, item, act)
        if best is None:
            if _r['planNone'] >= len(cand_items):
                _stall['no_best_planNone'] += 1
            elif _r['slack'] > _r['z'] and _r['slack'] > 0:
                _stall['no_best_slack'] += 1
            elif _r['z'] > 0:
                _stall['no_best_z'] += 1
            else:
                _stall['no_best_other'] += 1
            if _dbg and sum(v for k, v in _stall.items() if k.startswith('no_best')) <= 6:
                import sys
                print(f'[WBC] stall@plan={len(plan)} ref y[{ref[2]:.2f},{ref[5]:.2f}] '
                      f'z[{ref[3]:.2f},{ref[6]:.2f}] band={band:.2f} wb={wall_back[ci]:.2f} '
                      f'#cand={len(cand_items)} planNone={_r["planNone"]} z={_r["z"]} '
                      f'slack={_r["slack"]} |S|={len(S)}', file=sys.stderr)
            S = [s for s in S if s is not ref]
            continue
        _, item, act = best
        act = {'place_pos': np.asarray(act['place_pos'], dtype=float),
               'orientation': int(act['orientation']), 'container_idx': 0}
        _ihalf = geo.half_extent((item['length'], item['width'], item['height']), act['orientation'])
        if wall_depth[ci] is None:
            # この壁の最初の荷物が壁の厚みを決める(背面 wall_back からの深さ)
            wall_depth[ci] = max(0.05, wall_back[ci] - (float(act['place_pos'][1]) - float(_ihalf[1])))
        if _dbg and len(plan) < 999:
            import sys
            _wp = geo.local_to_world(conts[ci], act['place_pos'])[None, :]
            _slk = float(geo.inclusion_slack_batch(conts[ci], _ihalf, _wp)[0])
            print(f'[WBC] #{len(plan)} ref ci{ci} y[{ref[2]:.2f},{ref[5]:.2f}] z[{ref[3]:.2f},{ref[6]:.2f}] '
                  f'band={band:.2f} wb={wall_back[ci]:.2f} wd={wall_depth[ci]:.2f} '
                  f'-> item{item["index"]} orn{act["orientation"]} pos=({act["place_pos"][0]:.2f},'
                  f'{act["place_pos"][1]:.2f},{act["place_pos"][2]:.2f}) slk={_slk:.3f} |S|={len(S)}',
                  file=sys.stderr)
        placed = simulate._place(conts[ci], dict(item), act)
        conts[ci]['packed_items'].append(placed)
        pp = act['place_pos']
        plan.append({'index': int(item['index']), 'container_idx': int(ci),
                     'place_pos': (float(pp[0]), float(pp[1]), float(pp[2])),
                     'orientation': int(act['orientation'])})
        remaining = [it for it in remaining if it['index'] != item['index']]
        box = _placed_aabb(conts[ci], item, act)
        _inf = WBC_AABB_INFLATE
        box = (ci, box[1] - _inf, box[2] - _inf, box[3] - _inf,
               box[4] + _inf, box[5] + _inf, box[6] + _inf)  # 数mm膨張(次の荷物に余裕)
        nxt = []
        for s in S:
            nxt.extend(_split_space(s, box) if s[0] == ci else [s])
        S = _prune(nxt)

    if _dbg:
        import sys
        print(f'[WBC] FINAL offline plan={len(plan)} / total={len(item_list)} '
              f'(remaining={len(remaining)}, |S|={len(S)}) stall={_stall}', file=sys.stderr)
    return plan
