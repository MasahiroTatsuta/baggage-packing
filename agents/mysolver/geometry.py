"""
コンテナ/荷物の幾何・座標変換・合法性判定（validator.py の判定を配置前に自己再現する）。

前提: pool_list / container_list には validator.py の内部パラメータ
(inclusion_margin, safety_margin, start_z, ceiling_margin, start_margin ...) は
含まれない。CLAUDE_CODE_指示書.md に明記された既定値を保守的な側に寄せて採用し、
実際の値が多少ずれても「合法判定される手は本当に合法」であることを優先する。
"""
import numpy as np
import pybullet as p

from src.ground_handling.utils import ORNS, get_half_ext

# --- 保守的な既定値 (指示書 記載値をベースに安全側へ) ---
INCLUSION_MARGIN = -0.006      # 実際は -0.005〜0.02 程度。より厳しい(狭い)側を採用。
SAFETY_MARGIN_XY = 0.022       # 実際は 0.015 程度。横方向は少し余裕を持たせる。
Z_TOUCH_EPS = -0.0015          # 上下方向は「接触」を許容(margin<0で厳密な貫通のみ検出)
START_Z = 0.08                 # 非直置き時の搬入時浮上量
START_MARGIN = 0.01
CEILING_MARGIN = 0.02
REST_CLEARANCE = 0.016         # 接地/積み上げ目標zに与える内側クリアランス。
                                # inclusion_marginが負のため、境界に厳密接触(dot=0)する
                                # 目標だと内包判定に落ちる。displacement_threshold(数十cm)は
                                # 十分大きいため、この程度の浮きは沈み込みで解消される。


def local_to_world(container: dict, local_pos) -> np.ndarray:
    ox = container['center'][0]
    return np.array([local_pos[0] + ox, local_pos[1], local_pos[2]], dtype=np.float64)


def world_to_local(container: dict, world_pos) -> np.ndarray:
    ox = container['center'][0]
    return np.array([world_pos[0] - ox, world_pos[1], world_pos[2]], dtype=np.float64)


def half_extent(lwh, orn_idx: int) -> np.ndarray:
    return np.array(get_half_ext([lwh[0], lwh[1], lwh[2]], orn_idx), dtype=np.float64)


def check_inclusion_batch(container: dict, half: np.ndarray, world_pos: np.ndarray, margin: float = INCLUSION_MARGIN) -> np.ndarray:
    """
    world_pos: shape (N,3)。 各候補についてコンテナ内包判定(validator.check_inclusion と同式)。
    戻り値: shape (N,) bool
    """
    n_vecs = np.array(container['n_vecs'])          # (F,3)
    points = np.array(container['points'])          # (F,3)
    bonus = np.dot(np.abs(n_vecs), half)             # (F,)
    # (N,F) = sum_axis3( n_vecs[f] * (pos[n]-points[f]) )
    diff = world_pos[:, None, :] - points[None, :, :]     # (N,F,3)
    dots = np.einsum('nfc,fc->nf', diff, n_vecs) + bonus[None, :]
    return np.all(dots <= margin, axis=1)


def quat_abs_rotmat(orn) -> np.ndarray:
    m = np.array(p.getMatrixFromQuaternion(orn), dtype=np.float64).reshape(3, 3)
    return np.abs(m)


def item_world_aabb(item: dict):
    """既配置荷物のワールドAABB(中心, 半寸)を実姿勢(orn)から保守的に算出する。"""
    pos = np.array(item['pos'], dtype=np.float64)
    half_local = np.array([item['length'] / 2.0, item['width'] / 2.0, item['height'] / 2.0])
    absR = quat_abs_rotmat(item['orn'])
    half_world = absR @ half_local
    return pos, half_world


def small_shelf_aabb(container: dict):
    length = container['length']; width = container['width']; height = container['height']
    thickness = container['thickness']; cut_x = container['cut_x']
    ox = container['center'][0]
    center = np.array([-length / 2.0 + cut_x / 2.0 + thickness + ox, 0.0, height / 2.0 + thickness / 2.0])
    half = np.array([cut_x / 2.0, width / 2.0 - thickness, thickness / 2.0])
    return center, half


def big_shelf_aabb(container: dict):
    length = container['length']; width = container['width']; height = container['height']
    thickness = container['thickness']
    ox = container['center'][0]
    center = np.array([ox, width / 4.0, height / 2.0 + thickness / 2.0])
    half = np.array([length / 2.0 - thickness / 2.0, width / 4.0 - thickness, thickness / 2.0])
    return center, half


def static_obstacles(container: dict):
    """コンテナに常設される構造物(脇の小さい棚、あれば大棚)を障害物AABBとして返す。"""
    obstacles = [small_shelf_aabb(container)]
    if container.get('shelf', False):
        obstacles.append(big_shelf_aabb(container))
    return obstacles


def packed_obstacles(container: dict):
    obstacles = []
    for item in container.get('packed_items', []):
        if item.get('pos') is None or item.get('orn') is None:
            continue
        obstacles.append(item_world_aabb(item))
    return obstacles


def box_overlap_batch(min1: np.ndarray, max1: np.ndarray, center2: np.ndarray, half2: np.ndarray,
                       margin_xy: float = SAFETY_MARGIN_XY, margin_z: float = Z_TOUCH_EPS) -> np.ndarray:
    """
    min1, max1: shape (N,3) 候補側(点 or 掃引区間)のAABB範囲。
    center2, half2: shape (3,) 障害物1個のAABB。
    戻り値: shape (N,) bool。True=衝突(margin込みで重なる)。
    """
    margin = np.array([margin_xy, margin_xy, margin_z])
    min2 = center2 - half2
    max2 = center2 + half2
    sep = (max1 + margin[None, :] <= min2[None, :]) | (max2[None, :] + margin[None, :] <= min1)
    separated_any_axis = np.any(sep, axis=1)
    return ~separated_any_axis


def rect_overlap_ratio(cx1, cy1, hx1, hy1, cx2, cy2, hx2, hy2) -> float:
    """XY平面での矩形重なり面積 / 矩形1の面積。"""
    ox = max(0.0, min(cx1 + hx1, cx2 + hx2) - max(cx1 - hx1, cx2 - hx2))
    oy = max(0.0, min(cy1 + hy1, cy2 + hy2) - max(cy1 - hy1, cy2 - hy2))
    area1 = (2 * hx1) * (2 * hy1)
    if area1 <= 1e-9:
        return 0.0
    return (ox * oy) / area1


def transport_x_bounds(container: dict, half_x: float):
    length = container['length']; thickness = container['thickness']; cut_x = container['cut_x']
    ox = container['center'][0]
    x_min = -length / 2.0 + thickness + cut_x + half_x + START_MARGIN + ox
    x_max = length / 2.0 - thickness - half_x - START_MARGIN + ox
    return x_min, x_max
