"""
候補生成＋スコアリング。各ステップで
  (pool内の各item) × (orientation 0..5) × (候補位置: 床グリッド/棚グリッド/既配置荷物の上)
を評価し、合法な手のうち最良のものを1つ返す。

合法性は geometry.py の関数で validator.py と同等のロジック(内包・搬入経路衝突・支持面)を
配置前に自己再現して判定する。合法手が1つも無い場合のみ None を返す。
"""
import time
import numpy as np

from . import geometry as geo

MAX_POOL_ITEMS = 20
GRID_MARGIN = 0.02


def _unique_orientations(lwh):
    seen = {}
    for orn_idx in range(6):
        half = tuple(np.round(geo.half_extent(lwh, orn_idx), 5))
        if half not in seen:
            seen[half] = orn_idx
    return list(seen.values())


def _grid_xy(container, nx=19, ny=15):
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
    return geo.packed_obstacles(container) + geo.static_obstacles(container)


def _exposed_supports(container):
    packed = [it for it in container.get('packed_items', []) if it.get('pos') is not None and it.get('orn') is not None]
    aabbs = [geo.item_world_aabb(it) for it in packed]
    exposed = []
    for i in range(len(packed)):
        c, h = aabbs[i]
        top_z = c[2] + h[2]
        covered = False
        for j in range(len(packed)):
            if i == j:
                continue
            c2, h2 = aabbs[j]
            bottom_j = c2[2] - h2[2]
            if abs(bottom_j - top_z) < 0.012 and _rect_overlap_ratio_batch(
                    c[0], c[1], h[0], h[1], c2[0], c2[1], h2[0], h2[1]) > 0.05:
                covered = True
                break
        if not covered:
            exposed.append((packed[i], c, h))
    return exposed


def _score(container, local_x, local_y, world_z, half, item, support_ratio):
    length = container['length']; width = container['width']
    z_term = -world_z * 12.0
    back_term = ((local_y + width / 2.0) / max(width, 1e-6)) * 0.5
    edge_term = (np.abs(local_x) / max(length / 2.0, 1e-6)) * 0.3
    support_term = support_ratio * 3.0
    prio_term = 4.0 if (item.get('is_prioritized', False) and container.get('is_prioritized', False)) else 0.0
    # 底面が狭く背が高い(倒れやすい)向きを強く避ける
    base_half = max(half[0], half[1])
    stability_penalty = max(0.0, half[2] - base_half) * 20.0
    return z_term + back_term + edge_term + support_term + prio_term - stability_penalty


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


def _evaluate_floor_or_shelf(container, item, orn_idx, half, obstacles, is_shelf, deadline):
    if time.perf_counter() > deadline:
        return None

    thickness = container['thickness']; height = container['height']
    ox = container['center'][0]

    local_x, local_y = _grid_xy(container)
    if is_shelf:
        world_z = height / 2.0 + thickness + half[2] + geo.REST_CLEARANCE
    else:
        world_z = thickness + half[2] + geo.REST_CLEARANCE

    world_x = local_x + ox
    world_y = local_y
    world_pos = np.stack([world_x, world_y, np.full_like(world_x, world_z)], axis=1)

    incl = geo.check_inclusion_batch(container, half, world_pos)
    if not np.any(incl):
        return None

    # 搬入経路: Y方向(手前→目標)、続けてX方向(目標)。床/棚に直置きなので浮上なし(sweep_z=world_z)。
    x_min_local, x_max_local = geo.transport_x_bounds(container, half[0])
    x_min_local -= ox
    x_max_local -= ox
    start_x_local = np.clip(local_x, x_min_local, x_max_local)
    start_x_world = start_x_local + ox

    y_entry = -container['width'] / 2.0
    # phase1: y方向掃引 (x=start_x固定)
    y1_lo = np.minimum(y_entry, local_y)
    y1_hi = np.maximum(y_entry, local_y)
    x1_lo = start_x_world
    x1_hi = start_x_world

    legal1 = _apply_obstacle_filters(world_pos, half, obstacles, x1_lo, x1_hi, y1_lo, y1_hi, world_z)

    # phase2: x方向掃引 (y=target_y固定)
    x2_lo = np.minimum(start_x_world, world_x)
    x2_hi = np.maximum(start_x_world, world_x)
    y2_lo = world_y
    y2_hi = world_y
    legal2 = _apply_obstacle_filters(world_pos, half, obstacles, x2_lo, x2_hi, y2_lo, y2_hi, world_z)

    legal = incl & legal1 & legal2
    if not np.any(legal):
        return None

    if is_shelf:
        sc, sh = geo.big_shelf_aabb(container)
        support_ratio = _rect_overlap_ratio_batch(world_x, world_y, half[0], half[1], sc[0], sc[1], sh[0], sh[1])
        legal &= support_ratio >= 0.9
        if not np.any(legal):
            return None
    else:
        support_ratio = np.ones_like(world_x)

    scores = _score(container, local_x, local_y, world_z, half, item, support_ratio)
    scores = np.where(legal, scores, -np.inf)
    best_i = int(np.argmax(scores))
    if not legal[best_i]:
        return None

    return {
        'score': float(scores[best_i]),
        'local_pos': np.array([local_x[best_i], local_y[best_i], world_z], dtype=np.float32),
    }


def _evaluate_stack(container, item, orn_idx, half, obstacles, exposed, deadline):
    if time.perf_counter() > deadline:
        return None
    ox = container['center'][0]
    best = None
    for support_item, sc, sh in exposed:
        if support_item.get('is_soft', False):
            continue  # ソフトの上には何も積まない
        offsets = [(0.0, 0.0)]
        dx = max(0.0, sh[0] - half[0]) * 0.85
        dy = max(0.0, sh[1] - half[1]) * 0.85
        if dx > 1e-6 or dy > 1e-6:
            offsets += [(dx, dy), (-dx, dy), (dx, -dy), (-dx, -dy)]

        for ddx, ddy in offsets:
            cx = sc[0] + ddx
            cy = sc[1] + ddy
            cz = sc[2] + sh[2] + half[2] + geo.REST_CLEARANCE

            support_ratio = _rect_overlap_ratio_batch(
                np.array([cx]), np.array([cy]), half[0], half[1], sc[0], sc[1], sh[0], sh[1])[0]
            if support_ratio < 0.9:
                continue

            world_pos = np.array([[cx, cy, cz]])
            incl = geo.check_inclusion_batch(container, half, world_pos)
            if not incl[0]:
                continue

            sweep_z = min(cz + geo.START_Z, container['height'] - container['thickness'] - half[2] - geo.START_MARGIN)

            x_min_local, x_max_local = geo.transport_x_bounds(container, half[0])
            local_x = cx - ox
            start_x_local = float(np.clip(local_x, x_min_local - ox, x_max_local - ox))
            start_x_world = start_x_local + ox

            y_entry = -container['width'] / 2.0
            y1_lo = np.array([min(y_entry, cy)]); y1_hi = np.array([max(y_entry, cy)])
            x1_lo = np.array([start_x_world]); x1_hi = np.array([start_x_world])
            legal1 = _apply_obstacle_filters(world_pos, half, obstacles, x1_lo, x1_hi, y1_lo, y1_hi, sweep_z)

            x2_lo = np.array([min(start_x_world, cx)]); x2_hi = np.array([max(start_x_world, cx)])
            y2_lo = np.array([cy]); y2_hi = np.array([cy])
            legal2 = _apply_obstacle_filters(world_pos, half, obstacles, x2_lo, x2_hi, y2_lo, y2_hi, sweep_z)

            # 最終位置自体の衝突 (支持物とは接触許容、他とはmargin要)
            legal_final = _apply_obstacle_filters(
                world_pos, half, obstacles,
                np.array([cx]), np.array([cx]), np.array([cy]), np.array([cy]), cz)

            if not (legal1[0] and legal2[0] and legal_final[0]):
                continue

            score = _score(container, local_x, cy, cz, half, item, support_ratio)
            if best is None or score > best['score']:
                best = {'score': float(score), 'local_pos': np.array([local_x, cy, cz], dtype=np.float32)}
    return best


def plan(container_list: list[dict], pool_list: list[dict], time_budget: float = 5.5) -> dict | None:
    start = time.perf_counter()
    deadline = start + time_budget

    best_overall = None
    n_pool = min(len(pool_list), MAX_POOL_ITEMS)

    for container in container_list:
        if time.perf_counter() > deadline:
            break
        obstacles = _collect_obstacles(container)
        exposed = _exposed_supports(container)

        for pool_idx in range(n_pool):
            if time.perf_counter() > deadline:
                break
            item = pool_list[pool_idx]
            lwh = (item['length'], item['width'], item['height'])

            for orn_idx in _unique_orientations(lwh):
                if time.perf_counter() > deadline:
                    break
                half = geo.half_extent(lwh, orn_idx)

                candidates = []
                r = _evaluate_floor_or_shelf(container, item, orn_idx, half, obstacles, False, deadline)
                if r is not None:
                    candidates.append(r)
                if container.get('shelf', False):
                    r = _evaluate_floor_or_shelf(container, item, orn_idx, half, obstacles, True, deadline)
                    if r is not None:
                        candidates.append(r)
                r = _evaluate_stack(container, item, orn_idx, half, obstacles, exposed, deadline)
                if r is not None:
                    candidates.append(r)

                for c in candidates:
                    if best_overall is None or c['score'] > best_overall['score']:
                        best_overall = {
                            'score': c['score'],
                            'local_pos': c['local_pos'],
                            'item_idx': pool_idx,
                            'container_idx': container['index'],
                            'orientation': orn_idx,
                        }

    if best_overall is None:
        return None

    return {
        'item_idx': best_overall['item_idx'],
        'container_idx': best_overall['container_idx'],
        'place_pos': best_overall['local_pos'].astype(np.float32),
        'orientation': best_overall['orientation'],
    }
