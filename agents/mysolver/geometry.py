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
# 本家 fill_score(evaluator.calculate_fill_rate) は配置後に settle_wait_step=300 の物理演算を
# 経た「最終静止姿勢」の8角点で inclusion_margin(実際は -0.005 程度、負=厳密内包)を再チェックする。
# 後続荷物の投入で既配置荷物がわずかに押される/沈み込むだけで、壁ギリギリ(dot≈0)の配置は
# 数mmの沈降で簡単に外側へはみ出し fill 対象から脱落する。これを防ぐため、実margin(-0.005)より
# 更に厳しい側へ寄せ、沈降マージンを確保する。
# (実験的には -0.015 付近までは配置数への影響は軽微だが、-0.016 付近から特定コンテナ形状
#  (cut corner付近)で候補が急減する崖があるため、余裕を見て崖のかなり手前に留める。)
INCLUSION_MARGIN = -0.012      # 実際は -0.005〜0.02 程度。沈降ドリフト分の余裕を追加。
SAFETY_MARGIN_XY = 0.022       # 実際は 0.015 程度。横方向は少し余裕を持たせる。
Z_TOUCH_EPS = -0.0015          # 「最終着地位置」専用。支持面へのz方向の厳密接触(隙間ゼロ)を
                                # 誤って衝突扱いしないための負マージン(margin<0で貫通のみ検出)。
SWEEP_Z_MARGIN = 0.0155        # 「搬入経路の掃引」専用。実margin(0.015)に対しZ_TOUCH_EPSは
                                # 実質バッファ無し(むしろ負)で、掃引中に他の荷物のすぐ上を
                                # かすめて real validator 側でのみ衝突判定される事故があった
                                # (実測距離0.0149での搬入失敗を確認)。着地目標の直下の支持面
                                # (隙間は必ずREST_CLEARANCE=0.016)を誤って衝突扱いしない上限は
                                # 0.016なので、値は [0.015, 0.016) の窓に入っている必要がある。
                                # Phase11: 旧値 0.014 はこの窓の下限(実margin 0.015)を割り込んで
                                # おり、「隙間 14〜15mm の掃引」を自分だけ合法と誤判定していた。
                                # 実測でこの窓の事故が再発(before: 距離0.01479 で1件、after:
                                # 0.01433 で3件、いずれもエピソード即終了)したため、窓の中央
                                # 0.0155 へ引き上げる(両側に 0.5mm の余裕があり浮動小数誤差
                                # (~1e-9)に対して十分)。
DIRECT_SUPPORT_Z_TOL = 0.019   # 「最終着地位置」で、ある障害物が自分の直下の支持面(REST_CLEARANCE
                                # =0.016だけ隙間を空けて乗っている対象)かどうかを判定する許容誤差。
OBSTACLE_Z_MARGIN = 0.02       # 「最終着地位置」で、直下の支持面”以外”の障害物(横・真上の棚など)
                                # に対して要求するz方向margin。Z_TOUCH_EPSは支持面接触専用の特例で、
                                # それ以外にまで適用すると「棚のすぐ下に潜り込む」ような、real
                                # safety_margin(0.015)を割り込む配置を合法と誤判定する
                                # (実測: 棚とitemの隙間14mmでreal validator側の衝突を確認)。
START_Z = 0.08                 # 非直置き時の搬入時浮上量
START_MARGIN = 0.01
CEILING_MARGIN = 0.02
REST_CLEARANCE = 0.016         # 接地/積み上げ目標zに与える内側クリアランス。
                                # inclusion_marginが負のため、境界に厳密接触(dot=0)する
                                # 目標だと内包判定に落ちる。displacement_threshold(数十cm)は
                                # 十分大きいため、この程度の浮きは沈み込みで解消される。

# --- fill算出時のリスク評価用(Phase6: sim-to-realギャップ対策) ---
# 本家 evaluator の inclusion_margin(実際の値。configでは -0.005 が使われる)。
# INCLUSION_MARGIN(-0.012)より緩いため、配置時点でのslack(inclusion_slack_batchの値)が
# これより十分小さければ、多少の沈降ドリフトがあってもfill集計に残る可能性が高いと判断する。
REAL_INCLUSION_MARGIN = -0.005
# real margin からさらにこれだけ余裕(slackがREAL_INCLUSION_MARGIN-SAFE_SLACK以下)があれば
# 「安全」、real marginぎりぎり(slack>=REAL_INCLUSION_MARGIN)なら「危険」とみなす連続評価の幅。
SAFE_SLACK = 0.02


def local_to_world(container: dict, local_pos) -> np.ndarray:
    ox = container['center'][0]
    return np.array([local_pos[0] + ox, local_pos[1], local_pos[2]], dtype=np.float64)


def world_to_local(container: dict, world_pos) -> np.ndarray:
    ox = container['center'][0]
    return np.array([world_pos[0] - ox, world_pos[1], world_pos[2]], dtype=np.float64)


def half_extent(lwh, orn_idx: int) -> np.ndarray:
    return np.array(get_half_ext([lwh[0], lwh[1], lwh[2]], orn_idx), dtype=np.float64)


def inclusion_slack_batch(container: dict, half: np.ndarray, world_pos: np.ndarray) -> np.ndarray:
    """
    world_pos: shape (N,3)。各候補について、全面のうち最も厳しい(壁に最も近い)
    dot値(validator.check_inclusion と同式)を返す。小さい(より負)ほど壁からの
    余裕が大きく、real evaluatorの厳しいinclusion_margin(-0.005程度)や配置後の
    沈降ドリフトに対して安全であることを意味する。戻り値: shape (N,) float
    """
    n_vecs = np.array(container['n_vecs'])          # (F,3)
    points = np.array(container['points'])          # (F,3)
    bonus = np.dot(np.abs(n_vecs), half)             # (F,)
    # (N,F) = sum_axis3( n_vecs[f] * (pos[n]-points[f]) )
    diff = world_pos[:, None, :] - points[None, :, :]     # (N,F,3)
    dots = np.einsum('nfc,fc->nf', diff, n_vecs) + bonus[None, :]
    return np.max(dots, axis=1)


def check_inclusion_batch(container: dict, half: np.ndarray, world_pos: np.ndarray, margin: float = INCLUSION_MARGIN) -> np.ndarray:
    """
    world_pos: shape (N,3)。 各候補についてコンテナ内包判定(validator.check_inclusion と同式)。
    戻り値: shape (N,) bool
    """
    return inclusion_slack_batch(container, half, world_pos) <= margin


def fill_risk_factor(slack):
    """
    inclusion_slack_batch の値(壁に最も近い面のdot値)を [0,1] のfill期待係数に変換する。
    slack <= REAL_INCLUSION_MARGIN - SAFE_SLACK (壁から十分離れている) -> 1.0 (満額)
    slack >= REAL_INCLUSION_MARGIN (real evaluatorの基準ですら際どい) -> 0.0 (沈降ドリフトで
    fill集計から漏れる可能性が高いとみなし、offline探索の目的関数・online配置スコアの両方で
    このような際どい配置を割り引く)。線形補間。
    """
    return np.clip((REAL_INCLUSION_MARGIN - slack) / SAFE_SLACK, 0.0, 1.0)


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
    buffer = container.get('buffer', 0.0)
    center = np.array([-length / 2.0 + cut_x / 2.0 + thickness + ox, 0.0, height / 2.0 + thickness / 2.0 + buffer])
    half = np.array([cut_x / 2.0, width / 2.0 - thickness, thickness / 2.0])
    return center, half


def big_shelf_aabb(container: dict):
    length = container['length']; width = container['width']; height = container['height']
    thickness = container['thickness']
    ox = container['center'][0]
    buffer = container.get('buffer', 0.0)
    center = np.array([ox, width / 4.0, height / 2.0 + thickness / 2.0 + buffer])
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
                       margin_xy: float = SAFETY_MARGIN_XY, margin_z=Z_TOUCH_EPS) -> np.ndarray:
    """
    min1, max1: shape (N,3) 候補側(点 or 掃引区間)のAABB範囲。
    center2, half2: shape (3,) 障害物1個のAABB。
    margin_z: スカラー、または候補ごとに変えたい場合は shape (N,) の配列
    (例: 「この障害物が自分の直下の支持面かどうか」で許容誤差を変える場合)。
    戻り値: shape (N,) bool。True=衝突(margin込みで重なる)。
    """
    min2 = center2 - half2
    max2 = center2 + half2
    margin_z_arr = np.asarray(margin_z)
    sep_x = (max1[:, 0] + margin_xy <= min2[0]) | (max2[0] + margin_xy <= min1[:, 0])
    sep_y = (max1[:, 1] + margin_xy <= min2[1]) | (max2[1] + margin_xy <= min1[:, 1])
    sep_z = (max1[:, 2] + margin_z_arr <= min2[2]) | (max2[2] + margin_z_arr <= min1[:, 2])
    separated_any_axis = sep_x | sep_y | sep_z
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
