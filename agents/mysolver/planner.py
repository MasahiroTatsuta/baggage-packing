"""
候補生成＋スコアリング。各ステップで
  (pool内の各item) × (orientation 0..5) × (候補位置)
を評価し、合法な手のうち最良のものを1つ返す。

候補位置は「床/棚/既配置荷物の上面のどこかに乗せる」を区別せず、XY位置ごとに
その真下にある一番高い(かつ荷物側の90%以上が乗る)支持面を探して着地高さ(landing z)を
決める、いわゆる skyline(高さマップ)方式で統一している。XY候補自体は
  1) コンテナ床面をカバーする粗いグリッド
  2) 既配置荷物・壁のAABBの角に接する Extreme Point(隙間に隙間なく詰めるためのアンカー点)
の合成で作る。Extreme Point により「グリッドの目には引っかからないがピッタリ収まる隙間」を拾い、
グリッドにより「Extreme Pointだけでは見つからない広い空きスペース」を拾う。

合法性は geometry.py の関数で validator.py と同等のロジック(内包・搬入経路衝突・支持面)を
配置前に自己再現して判定する。合法手が1つも無い場合のみ None を返す。

優先荷物・ソフト貨物の「下敷き」を評価スコアに任せず、候補生成の段階で
「非優先(非ソフト)荷物を優先(ソフト)荷物の上に乗せる」候補そのものを作らないことで
ハード制約として回避する(置く順序に関わらず下敷きは発生しない)。
"""
import time
import numpy as np

from . import geometry as geo

MAX_POOL_ITEMS = 20
GRID_MARGIN = 0.02
# 荷物どうしのExtreme Point生成に使うクリアランス。衝突判定(_apply_obstacle_filters)は
# geo.SAFETY_MARGIN_XY(0.022)以上離れていないと「衝突」扱いにするため、これより
# 小さいクリアランスでアンカーを作ると、生成元の荷物自身との衝突判定で毎回弾かれてしまう。
# そのため必ず SAFETY_MARGIN_XY より広めに取る。
EP_ITEM_CLEARANCE = geo.SAFETY_MARGIN_XY + 0.006
CONTACT_EPS = 0.03          # 壁・他の荷物への「接触」とみなす隙間の許容値(EP_ITEM_CLEARANCEより広く取る)
MIN_SUPPORT_RATIO = 0.9     # 荷物の底面がこれだけ支持面に乗っていれば安定とみなす


def _unique_orientations(lwh):
    seen = {}
    for orn_idx in range(6):
        half = tuple(np.round(geo.half_extent(lwh, orn_idx), 5))
        if half not in seen:
            seen[half] = orn_idx
    return list(seen.values())


def _grid_xy(container, nx=31, ny=23):
    length = container['length']; width = container['width']
    x_lo = -length / 2.0 + GRID_MARGIN
    x_hi = length / 2.0 - GRID_MARGIN
    y_lo = -width / 2.0 + GRID_MARGIN
    y_hi = width / 2.0 - GRID_MARGIN
    xs = np.linspace(x_lo, x_hi, nx)
    ys = np.linspace(y_lo, y_hi, ny)
    xx, yy = np.meshgrid(xs, ys, indexing='ij')
    return xx.ravel(), yy.ravel()


def _rect_overlap_ratio_batch(cx, cy, hx, hy, ocx, ocy, ohx, ohy):
    ox = np.maximum(0.0, np.minimum(cx + hx, ocx + ohx) - np.maximum(cx - hx, ocx - ohx))
    oy = np.maximum(0.0, np.minimum(cy + hy, ocy + ohy) - np.maximum(cy - hy, ocy - ohy))
    area = (2 * hx) * (2 * hy)
    return (ox * oy) / max(area, 1e-9)


def _collect_obstacles(container):
    """衝突判定用(可否のみ、属性は問わない)の (center, half_ext) 一覧"""
    return geo.packed_obstacles(container) + geo.static_obstacles(container)


def _landing_supports(container):
    """
    着地面候補として使える (center, half_ext, is_prioritized, is_soft) 一覧。
    棚などの構造物は誰の上にも中立(is_prioritized=is_soft=False)として扱う。
    """
    supports = []
    for item in container.get('packed_items', []):
        if item.get('pos') is None or item.get('orn') is None:
            continue
        center, half = geo.item_world_aabb(item)
        supports.append((center, half, item.get('is_prioritized', False), item.get('is_soft', False)))
    for center, half in geo.static_obstacles(container):
        supports.append((center, half, False, False))
    return supports


def _extreme_points(container, half, obstacles):
    """
    壁・既配置荷物(障害物)のAABBの角に、新しい荷物(半寸法 half)がぴったり接する位置を
    アンカー候補として列挙する(Extreme Point法)。荒いグリッドでは拾いきれない隙間を拾うため。
    """
    length = container['length']; width = container['width']; thickness = container['thickness']
    ox = container['center'][0]

    x_lo = -length / 2.0 + thickness + GRID_MARGIN + half[0]
    x_hi = length / 2.0 - thickness - GRID_MARGIN - half[0]
    y_lo = -width / 2.0 + thickness + GRID_MARGIN + half[1]
    y_hi = width / 2.0 - thickness - GRID_MARGIN - half[1]
    if x_lo > x_hi or y_lo > y_hi:
        return set()

    points = {(x_lo, y_lo), (x_lo, y_hi), (x_hi, y_lo), (x_hi, y_hi)}

    for center, oh in obstacles:
        cx, cy = center[0] - ox, center[1]
        hx, hy = oh[0], oh[1]
        candidates = [
            (cx - hx - half[0] - EP_ITEM_CLEARANCE, cy - hy),
            (cx - hx - half[0] - EP_ITEM_CLEARANCE, cy + hy),
            (cx + hx + half[0] + EP_ITEM_CLEARANCE, cy - hy),
            (cx + hx + half[0] + EP_ITEM_CLEARANCE, cy + hy),
            (cx - hx, cy - hy - half[1] - EP_ITEM_CLEARANCE),
            (cx + hx, cy - hy - half[1] - EP_ITEM_CLEARANCE),
            (cx - hx, cy + hy + half[1] + EP_ITEM_CLEARANCE),
            (cx + hx, cy + hy + half[1] + EP_ITEM_CLEARANCE),
        ]
        for cxp, cyp in candidates:
            if x_lo - 1e-6 <= cxp <= x_hi + 1e-6 and y_lo - 1e-6 <= cyp <= y_hi + 1e-6:
                points.add((round(float(cxp), 5), round(float(cyp), 5)))

    return points


def _candidate_xy(container, half, obstacles):
    grid_x, grid_y = _grid_xy(container)
    pts = set(zip(np.round(grid_x, 5).tolist(), np.round(grid_y, 5).tolist()))
    pts |= _extreme_points(container, half, obstacles)
    if not pts:
        return np.zeros((0, 2), dtype=np.float64)
    return np.array(sorted(pts), dtype=np.float64)


def _apply_obstacle_filters(world_pos, half, obstacles, x_lo_arr, x_hi_arr, y_lo_arr, y_hi_arr, z_center):
    """
    world_pos: (N,3) 最終目標点。x_lo_arr..z_center: 搬入経路(掃引)の外接範囲。
    戻り値: 衝突していない(合法)候補の bool マスク (N,)
    """
    n = world_pos.shape[0]
    ok = np.ones(n, dtype=bool)

    min_final = world_pos - half[None, :]
    max_final = world_pos + half[None, :]

    z_lo = np.full(n, z_center - half[2])
    z_hi = np.full(n, z_center + half[2])
    min_sweep = np.stack([x_lo_arr - half[0], y_lo_arr - half[1], z_lo], axis=1)
    max_sweep = np.stack([x_hi_arr + half[0], y_hi_arr + half[1], z_hi], axis=1)

    for center, ohalf in obstacles:
        collide_final = geo.box_overlap_batch(min_final, max_final, center, ohalf)
        collide_sweep = geo.box_overlap_batch(min_sweep, max_sweep, center, ohalf)
        ok &= ~collide_final
        ok &= ~collide_sweep
    return ok


def _contact_bonus(container, half, world_x, world_y, world_z, obstacles):
    """
    壁・他の荷物に「接している」候補ほど隙間なく詰められるため加点する。
    同じ高さ帯(Zが重なる)にあり、かつXまたはY方向で隙間eps以内に接する場合に加点。
    """
    length = container['length']; width = container['width']; thickness = container['thickness']
    ox = container['center'][0]

    x_wall_lo = -length / 2.0 + thickness + ox
    x_wall_hi = length / 2.0 - thickness + ox
    y_wall_lo = -width / 2.0 + thickness
    y_wall_hi = width / 2.0 - thickness

    touch = np.zeros(world_x.shape[0])
    touch += np.abs((world_x - half[0]) - x_wall_lo) < CONTACT_EPS
    touch += np.abs((world_x + half[0]) - x_wall_hi) < CONTACT_EPS
    touch += np.abs((world_y - half[1]) - y_wall_lo) < CONTACT_EPS
    touch += np.abs((world_y + half[1]) - y_wall_hi) < CONTACT_EPS

    for center, oh in obstacles:
        z_overlap = (world_z - half[2] < center[2] + oh[2]) & (world_z + half[2] > center[2] - oh[2])
        x_touch = (np.abs((world_x - half[0]) - (center[0] + oh[0])) < CONTACT_EPS) | \
                  (np.abs((world_x + half[0]) - (center[0] - oh[0])) < CONTACT_EPS)
        y_touch = (np.abs((world_y - half[1]) - (center[1] + oh[1])) < CONTACT_EPS) | \
                  (np.abs((world_y + half[1]) - (center[1] - oh[1])) < CONTACT_EPS)
        touch += (z_overlap & (x_touch | y_touch)).astype(float)

    return touch


def _score(container, local_x, local_y, world_z, half, item, support_ratio, contact_bonus):
    length = container['length']; width = container['width']; height = container['height']
    z_term = -world_z * 12.0
    # back_termを最優先の位置決定要因にする(奥から手前へ順に詰め、自ら搬入経路を塞がないため)。
    # support/contactのような小さな差でこれが覆らないよう、他項より大きい重みを与える。
    back_term = ((local_y + width / 2.0) / max(width, 1e-6)) * 2.0
    edge_term = (np.abs(local_x) / max(length / 2.0, 1e-6)) * 0.3
    support_term = support_ratio * 1.0
    contact_term = contact_bonus * 0.6
    prio_term = 4.0 if (item.get('is_prioritized', False) and container.get('is_prioritized', False)) else 0.0
    # 底面が狭く背が高い(倒れやすい)向きを強く避ける
    base_half = max(half[0], half[1])
    stability_penalty = max(0.0, half[2] - base_half) * 20.0
    # cogタイブレーク: 他項がほぼ互角の候補間でのみ効く程度の小さな重みで、
    # 「重い荷物ほど低い位置」をわずかに優先する(z_term程は支配的にしない)。
    mass_norm = min(item.get('mass', 1.0), 15.0) / 15.0
    height_ratio = np.clip(world_z / max(height, 1e-6), 0.0, 1.0)
    cog_term = (1.0 - height_ratio) * mass_norm * 1.2
    return z_term + back_term + edge_term + support_term + contact_term + prio_term - stability_penalty + cog_term


def _evaluate_candidates(container, item, half, obstacles, supports, candidate_xy, deadline):
    """
    候補XY一覧について、乗せられる一番高い支持面(landing z)を求め、内包・搬入経路衝突を
    チェックしたうえで最良の1候補を返す。合法な候補が無ければ None。
    """
    if time.perf_counter() > deadline or candidate_xy.shape[0] == 0:
        return None

    ox = container['center'][0]
    thickness = container['thickness']
    height = container['height']
    buffer = container.get('buffer', 0.0)

    local_x = candidate_xy[:, 0]
    local_y = candidate_xy[:, 1]
    world_x = local_x + ox
    world_y = local_y
    n = local_x.shape[0]

    item_is_prioritized = item.get('is_prioritized', False)
    item_is_soft = item.get('is_soft', False)

    landing_top = np.full(n, thickness)   # 床の上面をベースラインとする
    landing_ratio = np.ones(n)
    for center, oh, sup_prioritized, sup_soft in supports:
        # 非優先(非ソフト)荷物が優先(ソフト)荷物の上に乗るのはハード禁止(下敷き防止)
        if (sup_prioritized and not item_is_prioritized) or (sup_soft and not item_is_soft):
            continue
        top = center[2] + oh[2]
        ratio = _rect_overlap_ratio_batch(world_x, world_y, half[0], half[1], center[0], center[1], oh[0], oh[1])
        better = (ratio >= MIN_SUPPORT_RATIO) & (top > landing_top)
        landing_top = np.where(better, top, landing_top)
        landing_ratio = np.where(better, ratio, landing_ratio)

    world_z = landing_top + half[2] + geo.REST_CLEARANCE
    ceiling_limit = height - thickness - geo.START_MARGIN
    valid_h = (world_z + half[2]) <= ceiling_limit
    world_pos = np.stack([world_x, world_y, world_z], axis=1)

    incl = geo.check_inclusion_batch(container, half, world_pos)
    base_legal = incl & valid_h
    if not np.any(base_legal):
        return None

    # 直置き面(床 or 棚上面)なら浮上なし、それ以外(荷物の上)は搬入時に少し浮かせてから下ろす
    resting_values = [thickness, height / 2.0 + thickness + buffer]
    is_resting = np.zeros(n, dtype=bool)
    for rv in resting_values:
        is_resting |= np.isclose(landing_top, rv, atol=1e-3)
    ceiling_sweep = height - thickness - half[2] - geo.START_MARGIN
    sweep_z = np.where(is_resting, world_z, np.minimum(world_z + geo.START_Z, ceiling_sweep))

    x_min_local, x_max_local = geo.transport_x_bounds(container, half[0])
    x_min_local -= ox; x_max_local -= ox
    start_x_local = np.clip(local_x, x_min_local, x_max_local)
    start_x_world = start_x_local + ox

    y_entry = -container['width'] / 2.0
    # phase1: y方向掃引 (x=搬入時のx固定)
    y1_lo = np.minimum(y_entry, local_y); y1_hi = np.maximum(y_entry, local_y)
    x1_lo = start_x_world; x1_hi = start_x_world
    legal1 = _apply_obstacle_filters(world_pos, half, obstacles, x1_lo, x1_hi, y1_lo, y1_hi, sweep_z)

    # phase2: x方向掃引 (y=target_y固定)
    x2_lo = np.minimum(start_x_world, world_x); x2_hi = np.maximum(start_x_world, world_x)
    y2_lo = world_y; y2_hi = world_y
    legal2 = _apply_obstacle_filters(world_pos, half, obstacles, x2_lo, x2_hi, y2_lo, y2_hi, sweep_z)

    legal = base_legal & legal1 & legal2
    if not np.any(legal):
        return None

    contact = _contact_bonus(container, half, world_x, world_y, world_z, obstacles)
    scores = _score(container, local_x, local_y, world_z, half, item, landing_ratio, contact)
    scores = np.where(legal, scores, -np.inf)
    best_i = int(np.argmax(scores))
    if not legal[best_i]:
        return None

    return {
        'score': float(scores[best_i]),
        'local_pos': np.array([local_x[best_i], local_y[best_i], world_z[best_i]], dtype=np.float32),
    }


def _search_best(container_list, pool_list, n_pool, deadline, enforce_priority_container,
                  has_prioritized_container, rng=None, score_noise=0.0):
    """
    (container × pool item × orientation × 候補位置) を総当たりし、合法な手のうち最良を返す。
    enforce_priority_container=True の間は、優先コンテナが存在するのに優先荷物を非優先
    コンテナへ置く候補そのものを生成しない(placement_score維持のためのハード優先)。
    rng/score_noise は offline の順序探索(複数リスタート)でのみ使う微小ノイズで、
    online呼び出し(デフォルト rng=None)には一切影響しない。
    """
    best_overall = None
    for container in container_list:
        if time.perf_counter() > deadline:
            break
        container_is_prioritized = container.get('is_prioritized', False)
        obstacles = _collect_obstacles(container)
        supports = _landing_supports(container)

        for pool_idx in range(n_pool):
            if time.perf_counter() > deadline:
                break
            item = pool_list[pool_idx]
            if enforce_priority_container and has_prioritized_container \
                    and item.get('is_prioritized', False) and not container_is_prioritized:
                continue
            lwh = (item['length'], item['width'], item['height'])

            for orn_idx in _unique_orientations(lwh):
                if time.perf_counter() > deadline:
                    break
                half = geo.half_extent(lwh, orn_idx)
                candidate_xy = _candidate_xy(container, half, obstacles)
                r = _evaluate_candidates(container, item, half, obstacles, supports, candidate_xy, deadline)
                if r is None:
                    continue

                score = r['score']
                if rng is not None and score_noise > 0.0:
                    score = score + float(rng.normal(0.0, score_noise))

                if best_overall is None or score > best_overall['score']:
                    best_overall = {
                        'score': score,
                        'local_pos': r['local_pos'],
                        'item_idx': pool_idx,
                        'container_idx': container['index'],
                        'orientation': orn_idx,
                    }
    return best_overall


def plan(container_list: list[dict], pool_list: list[dict], time_budget: float = 5.5,
         max_pool_items: int | None = MAX_POOL_ITEMS, rng=None, score_noise: float = 0.0) -> dict | None:
    """
    max_pool_items: online(agent.policy)は既定のMAX_POOL_ITEMSで呼ぶ。offlineの順序探索
    (ordering.build_order)は None を渡し、プール全件(=候補となる全未配置荷物)から
    その時点で最良の1手を選べるようにする(cf. simulate.greedy_construct_order)。
    """
    start = time.perf_counter()
    deadline = start + time_budget

    n_pool = len(pool_list) if max_pool_items is None else min(len(pool_list), max_pool_items)
    has_prioritized_container = any(c.get('is_prioritized', False) for c in container_list)

    best_overall = _search_best(container_list, pool_list, n_pool, deadline, enforce_priority_container=True,
                                 has_prioritized_container=has_prioritized_container, rng=rng, score_noise=score_noise)
    if best_overall is None and has_prioritized_container and time.perf_counter() <= deadline:
        # 優先コンテナ限定では合法手が全く無かった場合のみ、非優先コンテナも含めて再探索する
        # (それ以上待っても優先コンテナに入らない荷物を無駄に足止めしないため)。
        best_overall = _search_best(container_list, pool_list, n_pool, deadline, enforce_priority_container=False,
                                     has_prioritized_container=has_prioritized_container, rng=rng, score_noise=score_noise)

    if best_overall is None:
        return None

    return {
        'item_idx': best_overall['item_idx'],
        'container_idx': best_overall['container_idx'],
        'place_pos': best_overall['local_pos'].astype(np.float32),
        'orientation': best_overall['orientation'],
    }
