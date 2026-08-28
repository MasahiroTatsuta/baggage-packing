"""
コンテナ/荷物の幾何・座標変換・合法性判定（validator.py の判定を配置前に自己再現する）。

前提: pool_list / container_list には validator.py の内部パラメータ
(inclusion_margin, safety_margin, start_z, ceiling_margin, start_margin ...) は
含まれない。CLAUDE_CODE_指示書.md に明記された既定値を保守的な側に寄せて採用し、
実際の値が多少ずれても「合法判定される手は本当に合法」であることを優先する。
"""
import os

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
# Phase75: 公式値(-0.005)が判明した後も指示書ベースの安全側推測(-7mm、実測検証なし)の
# ままだったため、env化して本番スイープ可能にする(SAFETY_MARGIN_XYのPhase60と同じ扱い)。
# 既定値 -0.012 は不変(このコミット単体では挙動を変えない)。公式値ちょうど(-0.005)は
# 使わないこと(上記の崖は逆方向だが、閾値そのものでは余裕がゼロになる)。
INCLUSION_MARGIN = float(os.environ.get('MYSOLVER_INCLUSION_MARGIN', '-0.012'))      # 実際は -0.005〜0.02 程度。沈降ドリフト分の余裕を追加。
# Phase60: 公式値0.015が確定した後も、指示書記載値をベースにした安全側推測(+7mm、
# 実測による検証は一度も行われていない)のままだったため、env化してスイープ可能にする。
# 既定値0.022は不変(このコミット単体では挙動を変えない)。0.015ちょうどは使わないこと
# (公式の閾値そのもので余裕がゼロになる)。
SAFETY_MARGIN_XY = float(os.environ.get('MYSOLVER_SAFETY_MARGIN_XY', '0.022'))  # 実際は 0.015 程度。横方向は少し余裕を持たせる。
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
# Phase75: 現行値 0.016 は実測の裏づけなく安全側に置かれた自制(SWEEP_Z_MARGIN等の
# 窓の上限として引用されるが、この値自体が最適である確証はない)。env化して本番で
# 直接試せるようにする。既定値 0.016 は不変(このコミット単体では挙動を変えない)。
REST_CLEARANCE = float(os.environ.get('MYSOLVER_REST_CLEARANCE', '0.016'))         # 接地/積み上げ目標zに与える内側クリアランス。
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


def inclusion_slack_batch(container: dict, half: np.ndarray, world_pos: np.ndarray,
                           floor_only: bool = False) -> np.ndarray:
    """
    world_pos: shape (N,3)。各候補について、全面のうち最も厳しい(壁に最も近い)
    dot値(validator.check_inclusion と同式)を返す。小さい(より負)ほど壁からの
    余裕が大きく、real evaluatorの厳しいinclusion_margin(-0.005程度)や配置後の
    沈降ドリフトに対して安全であることを意味する。戻り値: shape (N,) float

    floor_only=True(Phase31, 既定False): 全面の最悪値ではなく、内床面(法線が最も
    下向きの面 = argmin(n_vecs[:,2]))だけの dot値を返す。壁・天井・切り欠きへの
    接近は無視し、床からの浮きだけを見る。**hard legality判定(check_inclusion_batch)
    には使わないこと**——側壁・天井を突き抜けた候補を誤って合法とみなしうるため、
    このフラグを使うのは risk_vol(offlineの候補順序選択指標)側に限定する
    (results/phase31_report.md 参照)。
    """
    n_vecs = np.array(container['n_vecs'])          # (F,3)
    points = np.array(container['points'])          # (F,3)
    bonus = np.dot(np.abs(n_vecs), half)             # (F,)
    # (N,F) = sum_axis3( n_vecs[f] * (pos[n]-points[f]) )
    diff = world_pos[:, None, :] - points[None, :, :]     # (N,F,3)
    dots = np.einsum('nfc,fc->nf', diff, n_vecs) + bonus[None, :]
    if floor_only:
        floor_idx = int(np.argmin(n_vecs[:, 2]))     # 最も下向きの法線 = 内床面
        return dots[:, floor_idx]
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
    """既配置荷物のワールドAABB(中心, 半寸)を実姿勢(orn)から保守的に算出する。

    Phase19(ターゲット2): 荷物のpos/ornは配置時に一度だけ設定され、以降変更されない
    (agents/mysolver 内で再代入する箇所は simulate.py の配置処理1箇所のみで、既存の
    要素を後から書き換える経路は無い)。そのため計算結果を item dict にメモ化してよい。
    offline探索(simulate.py)は同一の既配置荷物(dictオブジェクトそのもの)に対し
    毎ステップ(=毎 planner.plan 呼び出し)このAABBを再計算していたが、その大部分は
    quat_abs_rotmat(pybulletのクォータニオン→行列変換呼び出し)の再実行だった。
    キャッシュは出力を一切変えない(近似ではなく厳密な記憶化)。
    """
    cached = item.get('_aabb_cache')
    if cached is not None:
        return cached
    pos = np.array(item['pos'], dtype=np.float64)
    half_local = np.array([item['length'] / 2.0, item['width'] / 2.0, item['height'] / 2.0])
    absR = quat_abs_rotmat(item['orn'])
    half_world = absR @ half_local
    item['_aabb_cache'] = (pos, half_world)
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
    """
    Phase19(ターゲット2): container dict に増分キャッシュ(_packed_obstacle_cache)を
    載せ、前回この関数を呼び出した時点から新たに packed_items の末尾に追加された荷物
    だけAABBを計算する(Extreme Point法の"増分更新": 障害物集合を毎回スキャンし直すの
    ではなく前回の集合+差分で構築する)。

    simulate.py の offline探索は container dict を1回 clone_containers で複製した後、
    以降は同一の packed_items リストオブジェクトへ .append() するだけで単調増加させる
    (agents/mysolver 内でリストを丸ごと差し替える箇所は無い)ため、下のidentityチェックが
    常に安全に成立する。online(agent.py)は毎ステップ observation から container dict を
    丸ごと再構築するため packed_items のリストオブジェクトが呼び出しごとに異なり、
    identityチェックが必ず不一致になって安全にフルスキャンへフォールバックする
    (誤ったキャッシュヒットは構造的に起こり得ない)。
    """
    items = container.get('packed_items', [])
    cache = container.get('_packed_obstacle_cache')
    if cache is not None and cache['src'] is items and cache['n'] <= len(items):
        obstacles = cache['list']
        start = cache['n']
    else:
        obstacles = []
        cache = {'src': items, 'n': 0, 'list': obstacles}
        container['_packed_obstacle_cache'] = cache
        start = 0
    for item in items[start:]:
        if item.get('pos') is None or item.get('orn') is None:
            continue
        obstacles.append(item_world_aabb(item))
    cache['n'] = len(items)
    return list(obstacles)


# --- Phase18: 積み上げの静的安定 幾何代理(影シミュレータのorder選択用) ---
# 実 stability_score は shake test(pybullet の物理演算)でしか測れないが、offline探索
# (ordering.build_order)は影シミュレータ(simulate.py)上でしか順序を評価できない。
# 物理シミュレーションをそのまま足すのは高コストなため、静的力学から導ける必要条件2つを
# 幾何だけで近似する:
#   (a) 上下質量比: 上の荷物の質量が、直下で支持している既配置荷物群の合計質量に対して
#       どれだけ重いか(heavy-over-light の度合い)。
#   (b) 支持点凸包へのCoG投影: 剛体の静的安定の必要条件そのもの(接触点が張る支持多角形の
#       外に重心があれば転倒モーメントが生じる)。CoGが支持多角形の境界にどれだけ近いか
#       (=転倒までの余裕)を連続量として使う。
# 床/棚に直接乗る配置(直下に既配置荷物が無い)は、支持面がコンテナ底面/棚上面という
# 常にCoGを覆う広い剛体面なので、常に安定側(リスク0)として扱う。
#
# Phase18実測(tools診断、旧gen_2containers_priorityの探索中に生成された全候補順序を計測):
# planner._evaluate_candidates の候補legality(MIN_UNION_SUPPORT_RATIO・span・centroid offset)
# が既に「あからさまに際どい支持」を候補生成の時点でハード排除しているため、(a)(b)を単純な
# 二値(違反/非違反)にすると閾値(質量比3.0倍・CoGがhull外)に一度も抵触せず
# (実測: 122回の積み上げ全てで質量比<=1.63・CoGはhull境界から常に正の余裕を持つ)、
# 目的関数への寄与が恒等的にゼロになり探索に一切影響しなかった。二値の必要条件としては
# 満たされていても、「閾値ぎりぎり」ほど実物理(摩擦・剛性)的には不利という直感
# (geo.fill_risk_factor が壁際配置を連続的に割り引くのと同じ発想)に基づき、[0,1] の
# 連続リスクへ変更する: 質量比・CoG余裕とも「閾値に対する消費割合」を返し、閾値に近づくほど
# 1へ、余裕があるほど0へ連続的に近づける。
STACKING_MASS_RATIO = 3.0            # 上の荷物質量 <= この倍率 * 直下支持荷物群の合計質量 が「安全」の目安
STACKING_CONTACT_Z_TOL = 0.02        # 「同じ高さ帯の支持」とみなすz許容差(planner.SUPPORT_LEVEL_TOLと同値)
STACKING_COG_SAFE_MARGIN = 0.05      # CoGが支持多角形境界からこれだけ内側(m)にあれば安全(リスク0)


def convex_hull_2d(points: np.ndarray) -> np.ndarray:
    """Andrew's monotone chain。points: (N,2) -> 凸包頂点(CCW順、(M,2))。
    scipy不使用(理由は本セクション冒頭のコメント参照)。"""
    pts = np.unique(np.asarray(points, dtype=np.float64), axis=0)
    if pts.shape[0] <= 2:
        return pts

    def cross(o, a, b):
        return (a[0] - o[0]) * (b[1] - o[1]) - (a[1] - o[1]) * (b[0] - o[0])

    order = np.lexsort((pts[:, 1], pts[:, 0]))
    pts = pts[order]
    lower = []
    for pt in pts:
        while len(lower) >= 2 and cross(lower[-2], lower[-1], pt) <= 0:
            lower.pop()
        lower.append(pt)
    upper = []
    for pt in pts[::-1]:
        while len(upper) >= 2 and cross(upper[-2], upper[-1], pt) <= 0:
            upper.pop()
        upper.append(pt)
    return np.array(lower[:-1] + upper[:-1])


def point_in_convex_polygon(hull: np.ndarray, point, tol: float = 1e-6) -> bool:
    """hull は convex_hull_2d の戻り値(CCW)。頂点数<3(点・線分に退化)も扱う。"""
    return signed_distance_to_convex_polygon(hull, point) >= -tol


def signed_distance_to_convex_polygon(hull: np.ndarray, point) -> float:
    """
    hull(convex_hull_2d の戻り値、CCW)の境界までの符号付き距離[m]を返す。
    正 = 内側にその距離だけ余裕がある、負 = 外側にその距離だけはみ出している。
    厳密には「各辺を含む直線までの符号付き距離の最小値」であり、最近傍特徴が頂点の場合は
    真の最短距離よりわずかに小さい(=より安全側に出る)近似になるが、連続なリスク指標としては
    十分(点/線分に退化した場合は面としての内部を持たないため、内側扱いにはしない)。
    """
    n = hull.shape[0]
    px, py = point
    if n == 0:
        return -np.inf
    if n == 1:
        return -float(np.hypot(hull[0, 0] - px, hull[0, 1] - py))
    if n == 2:
        ax, ay = hull[0]; bx, by = hull[1]
        abx, aby = bx - ax, by - ay
        length = float(np.hypot(abx, aby))
        if length < 1e-12:
            return -float(np.hypot(ax - px, ay - py))
        return -abs((abx * (py - ay) - aby * (px - ax)) / length)
    dists = []
    for i in range(n):
        ax, ay = hull[i]
        bx, by = hull[(i + 1) % n]
        abx, aby = bx - ax, by - ay
        length = float(np.hypot(abx, aby))
        if length < 1e-12:
            continue
        dists.append((abx * (py - ay) - aby * (px - ax)) / length)
    return min(dists) if dists else 0.0


def stacking_instability_risk(existing_items: list[dict], placed_item: dict, item_mass: float,
                               mass_ratio: float = STACKING_MASS_RATIO,
                               z_tol: float = STACKING_CONTACT_Z_TOL,
                               cog_safe_margin: float = STACKING_COG_SAFE_MARGIN) -> tuple[bool, float]:
    """
    placed_item(pos/orn確定済み)の直下に既配置荷物(existing_items。placed_item自身は
    含まない)があるか(is_stacked)と、あるならその積み上げの静的安定リスク(risk, [0,1])を
    1回の接触計算で判定する。risk は (a)質量比 (b)CoG投影余裕 のうち悪い方
    (max、=より危険な側)を採用する。
    床/棚に直接乗る(直下に既配置荷物が無い)場合は is_stacked=False, risk=0.0
    (=常に安定側、分母にも数えない)。
    戻り値: (is_stacked, risk)
    """
    pos, half = item_world_aabb(placed_item)
    item_bottom = pos[2] - half[2]
    contacts = []
    support_mass = 0.0
    for other in existing_items:
        if other.get('pos') is None or other.get('orn') is None:
            continue
        o_pos, o_half = item_world_aabb(other)
        top = o_pos[2] + o_half[2]
        if abs(top - item_bottom) > z_tol:
            continue
        x_lo = max(pos[0] - half[0], o_pos[0] - o_half[0])
        x_hi = min(pos[0] + half[0], o_pos[0] + o_half[0])
        y_lo = max(pos[1] - half[1], o_pos[1] - o_half[1])
        y_hi = min(pos[1] + half[1], o_pos[1] + o_half[1])
        if x_hi - x_lo > 1e-6 and y_hi - y_lo > 1e-6:
            contacts.append((x_lo, x_hi, y_lo, y_hi))
            support_mass += other.get('mass', 0.0)
    if not contacts:
        return False, 0.0
    risk_mass = np.clip((item_mass / support_mass) / mass_ratio if support_mass > 0 else 1.0, 0.0, 1.0)
    pts = []
    for x_lo, x_hi, y_lo, y_hi in contacts:
        pts.extend([(x_lo, y_lo), (x_lo, y_hi), (x_hi, y_lo), (x_hi, y_hi)])
    hull = convex_hull_2d(np.array(pts))
    margin = signed_distance_to_convex_polygon(hull, (pos[0], pos[1]))
    risk_cog = float(np.clip(1.0 - margin / cog_safe_margin, 0.0, 1.0))
    return True, max(float(risk_mass), risk_cog)


def initial_prepacked_ids(container_list) -> dict:
    """各コンテナについて、エピソード開始時(get_init_states時点)から既に存在していた
    既配置荷物のindex集合を返す({container_index: frozenset(item_index)})。

    Phase15 ターゲット1: corridor_penalty(搬入経路保護)が「初期状態から積まれていた
    層」と「自分が今回のエピソードで積んだ層」を区別するために使う。既積み層の天面は
    不揃いなことが多く、これを他の障害物と同様に min_top_behind の算出に使うと、
    最低天面を過剰に守って既積み層の上への積み上げそのものを抑制してしまう
    (pre-packedシーンでの実測: results/phase15_report.md 参照)。
    """
    result = {}
    for c in container_list or []:
        ids = {item['index'] for item in c.get('packed_items', []) if item.get('pos') is not None}
        result[c.get('index')] = frozenset(ids)
    return result


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
