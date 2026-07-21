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
# Phase8: 探索グリッドの細分化。_search_best の通常探索(1手ごとに毎回呼ぶ)で使う既定密度。
# 密度1(31x23グリッド)は粗く、荷物どうしの隙間にぴったり収まる細いXY位置をExtreme Point法
# だけでは拾いきれない場合がある。密度2は計測上、online呼び出し(pool<=MAX_POOL_ITEMS=20)
# では0.35s->0.9s程度への増加に留まり、policy_timeout(8s)・実際の呼び出し予算(5.5s)に
# 対して十分な余裕がある(探索は deadline を自己チェックして安全に打ち切るため、万一時間が
# 足りない状況でもクラッシュや予算超過はしない)。
BASE_GRID_DENSITY = 2
# Phase7由来の「合法手0件時の最終リトライ」の密度。BASE_GRID_DENSITYを底上げしたことに
# 合わせて、通常探索よりさらに一段細かく最後の望みを探れるよう底上げする。
RETRY_GRID_DENSITY = 4
# Phase9: 「奥から手前への層規律」を back_term の重みではなく探索の構造そのものとして担保する。
# コンテナをY方向(手前=-width/2 〜 奥=+width/2)に n_y_slices 個の層へ分割し、まず「奥側から
# level+1 個分の層」だけを合法候補の対象にする。その層内でpool全件×全orientationを試し、
# 1つでも合法候補があればそれで確定し、それより手前の層は一切開放しない(=手前を空けておく)。
# 奥側の層に本当に置ける手が無くなった場合にのみ、次の層(1つ手前)を開放してリトライする。
# 最終level(=n_y_slices-1)は全開放(従来の全域探索と同じ)なので、真に空間的行き詰まりの場合の
# 挙動(Noneを返す)は変えない。back_termの重みを崖のある値まで上げずとも、搬入経路と衝突しうる
# 「奥がまだ空いているのに手前を先に埋める」配置そのものを生成しなくなる。
Y_SLICE_COUNT = 2
Y_SLICE_EPS = 0.01
# 荷物どうしのExtreme Point生成に使うクリアランス。衝突判定(_apply_obstacle_filters)は
# geo.SAFETY_MARGIN_XY(0.022)以上離れていないと「衝突」扱いにするため、これより
# 小さいクリアランスでアンカーを作ると、生成元の荷物自身との衝突判定で毎回弾かれてしまう。
# そのため必ず SAFETY_MARGIN_XY より広めに取る。
EP_ITEM_CLEARANCE = geo.SAFETY_MARGIN_XY + 0.006
CONTACT_EPS = 0.03          # 壁・他の荷物への「接触」とみなす隙間の許容値(EP_ITEM_CLEARANCEより広く取る)
MIN_SUPPORT_RATIO = 0.9     # 荷物の底面がこれだけ支持面に乗っていれば(重心条件を問わず)安定とみなす
# Phase11(ターゲット2): 「複数の支持面にまたがって乗る」着地の解禁。
#
# Phase10までの着地面判定は「単一の支持体に MIN_SUPPORT_RATIO 以上乗る」場合しか着地高さを
# 上げなかった。そのため、既積み荷物が小さい箱の集まり(積付済み初期状態は典型的にこれ)だと
# どの1個の上にも90%乗れず、既積み層の上面が丸ごと使えない空間になっていた
# (実測 tools/diagnose_prepacked.py: 既積み層の上に着地できる候補XYは全体の 2.0%(P01)/
#  7.3%(P06) しかなく、union判定にすると 6.6%/24.7% へ 2.2〜3.3倍に増える)。
#
# 物理的には「4個の箱の上にまたがって乗る」は完全に安定である。安定性の本質は接触面積比
# ではなく「支持点が荷物の底面を広く・偏りなく囲んでいるか」なので、
#   (1) 同じ高さ帯にある支持体の重なり面積の合計比 >= MIN_UNION_SUPPORT_RATIO
#   (2) 接触領域の外接矩形が底面の各軸を MIN_SUPPORT_SPAN_RATIO 以上またぐ(=端に寄っていない)
#   (3) 接触面積の重心が底面中心から MAX_SUPPORT_CENTROID_OFFSET(半寸法比)以内
# の3条件で判定する((2)(3)が「角にちょこんと乗る」不安定配置を排除する)。
MIN_UNION_SUPPORT_RATIO = 0.55
SUPPORT_LEVEL_TOL = 0.02        # 「同じ高さ帯の支持面」とみなす上面zの許容差
MIN_SUPPORT_SPAN_RATIO = 0.6    # 接触領域の外接矩形が底面の各軸方向をまたぐ最小割合
MAX_SUPPORT_CENTROID_OFFSET = 0.15  # 接触面積重心の底面中心からのずれ(半寸法に対する比)
# Phase9: 層規律導入により、非優先荷物が優先荷物のすぐ側面に密接して置かれやすくなった結果、
# 揺れ試験(stability_score算出)時の沈み込み・傾きで優先荷物の上に非優先荷物が乗り上げてしまい
# placement_scoreを損なう実例が確認された(候補生成時は「優先荷物の上を着地面にしない」を
# 徹底しているが、真横に隙間なく置かれた場合の物理的な傾き・接触までは防げない)。
# 優先荷物のAABBの周囲に追加のクリアランスを設け、非優先荷物がそのすぐ側面に密着する候補
# そのものを作らないことで、置く順序や重み調整に頼らずハード制約として回避する。
PRIORITY_CLEARANCE_XY = 0.05
PRIORITY_CLEARANCE_Z = 0.05


def _unique_orientations(lwh):
    seen = {}
    for orn_idx in range(6):
        half = tuple(np.round(geo.half_extent(lwh, orn_idx), 5))
        if half not in seen:
            seen[half] = orn_idx
    return list(seen.values())


def _grid_xy(container, nx=31, ny=23, density: int = 1):
    length = container['length']; width = container['width']
    x_lo = -length / 2.0 + GRID_MARGIN
    x_hi = length / 2.0 - GRID_MARGIN
    y_lo = -width / 2.0 + GRID_MARGIN
    y_hi = width / 2.0 - GRID_MARGIN
    xs = np.linspace(x_lo, x_hi, nx * density)
    ys = np.linspace(y_lo, y_hi, ny * density)
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


def _candidate_xy(container, half, obstacles, grid_density: int = 1):
    grid_x, grid_y = _grid_xy(container, density=grid_density)
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

    item_bottom = min_final[:, 2]
    for center, ohalf in obstacles:
        # 最終着地点は、この障害物が「自分の直下の支持面(REST_CLEARANCEだけ隙間を空けて
        # 乗っている対象)」である候補に限り、z方向の厳密接触(隙間ゼロ)を許すデフォルトmargin
        # (Z_TOUCH_EPS)で判定する。それ以外(横から接近・真上の棚の下に潜り込む等)は
        # 実margin相当(OBSTACLE_Z_MARGIN)を要求する(一律Z_TOUCH_EPSにしていたため、
        # 棚のすぐ下に潜り込む配置を合法と誤判定していた実例を修正)。
        obstacle_top = center[2] + ohalf[2]
        is_direct_support = np.abs(item_bottom - obstacle_top) < geo.DIRECT_SUPPORT_Z_TOL
        margin_z_final = np.where(is_direct_support, geo.Z_TOUCH_EPS, geo.OBSTACLE_Z_MARGIN)
        # 掃引(搬入経路の移動中)は「別の荷物のすぐ上をかすめる」際の実余裕を確保するため、
        # z方向により大きい margin(SWEEP_Z_MARGIN)を要求する。
        collide_final = geo.box_overlap_batch(min_final, max_final, center, ohalf, margin_z=margin_z_final)
        collide_sweep = geo.box_overlap_batch(min_sweep, max_sweep, center, ohalf, margin_z=geo.SWEEP_Z_MARGIN)
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


def _score(container, local_x, local_y, world_z, half, item, support_ratio, contact_bonus, slack):
    length = container['length']; width = container['width']; height = container['height']
    z_term = -world_z * 12.0
    # 壁ギリギリ(real evaluatorのinclusion_margin付近)の配置は、後続荷物の投入や自身の
    # 沈み込みでfill集計(本家evaluatorの厳しい判定)から漏れやすい。他項がほぼ互角な
    # 候補間でのみ効く程度の小さな重みで、壁からの余裕(slackがより負)を優先する。
    boundary_term = geo.fill_risk_factor(slack) * 1.5
    # back_termを「同じ着地高さの候補同士」では最優先の位置決定要因にする
    # (奥から手前へ順に詰め、自ら搬入経路を塞がないため)。
    # support/contact(最大でも1.0+0.6*4=3.4程度)がこの差を覆さないよう、
    # 同高度内での最大差がそれらを上回るだけの重みを与える。
    # 一方、床(低いz)と積み上げ(高いz)の間の差(z_termで2.4以上)は逆転させない。
    # Phase6実験: 5.9以上だとgen_manyitems_patternAのstability_scoreが94.39まで落ちる
    # (=97以上維持の制約に抵触)崖があり、5.8以下だとgen_2containers_priorityの完走に
    # 必要な順序を貪欲構築が見失い100%→82.9%placedまで落ちる崖もある(奥行き選好が
    # 弱まり、後続の押し込み経路を残せなくなるため)。両者を満たす5.85を採用する。
    back_term = ((local_y + width / 2.0) / max(width, 1e-6)) * 5.85
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
    return (z_term + back_term + edge_term + support_term + contact_term + prio_term
            - stability_penalty + cog_term + boundary_term)


def _evaluate_candidates(container, item, half, obstacles, supports, candidate_xy, deadline, stats=None):
    """
    候補XY一覧について、乗せられる一番高い支持面(landing z)を求め、内包・搬入経路衝突を
    チェックしたうえで最良の1候補を返す。合法な候補が無ければ None。

    stats: 診断用(tools/diagnose_stall.py 専用)。None以外を渡すと、Noneを返す直前に
    「どの段階で全滅したか」をカウントする。本番のonline呼び出し(agent.policy)では
    常にNoneのままなので、このオプションは通常経路の速度・挙動に影響しない。
    """
    if time.perf_counter() > deadline:
        return None
    if candidate_xy.shape[0] == 0:
        if stats is not None:
            stats['no_xy'] = stats.get('no_xy', 0) + 1
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

    # --- 着地面(skyline)の決定 ---
    # pass1: XYで少しでも重なる支持体はすべて「その上に乗るしかない」障害なので、
    #        重なる支持体の上面の最大値が着地上面になる(重なりが無ければ床)。
    item_area = max(4.0 * half[0] * half[1], 1e-12)
    landing_top = np.full(n, thickness)
    cache = []
    for center, oh, sup_prioritized, sup_soft in supports:
        x_lo = np.maximum(world_x - half[0], center[0] - oh[0])
        x_hi = np.minimum(world_x + half[0], center[0] + oh[0])
        y_lo = np.maximum(world_y - half[1], center[1] - oh[1])
        y_hi = np.minimum(world_y + half[1], center[1] + oh[1])
        ow = x_hi - x_lo
        oh_ = y_hi - y_lo
        touch = (ow > 1e-6) & (oh_ > 1e-6)
        if not np.any(touch):
            continue
        top = center[2] + oh[2]
        landing_top = np.where(touch & (top > landing_top), top, landing_top)
        forbidden = (sup_prioritized and not item_is_prioritized) or (sup_soft and not item_is_soft)
        cache.append((top, touch, x_lo, x_hi, y_lo, y_hi, ow, oh_, forbidden))

    # pass2: 着地上面と同じ高さ帯にある支持体だけを「接触している支持」として集計する。
    on_floor = landing_top <= thickness + 1e-9
    sum_ratio = np.zeros(n)
    sum_area = np.zeros(n)
    cen_x = np.zeros(n)
    cen_y = np.zeros(n)
    span_x_lo = np.full(n, np.inf); span_x_hi = np.full(n, -np.inf)
    span_y_lo = np.full(n, np.inf); span_y_hi = np.full(n, -np.inf)
    forbidden_hit = np.zeros(n, dtype=bool)
    for top, touch, x_lo, x_hi, y_lo, y_hi, ow, oh_, forbidden in cache:
        at_level = touch & (np.abs(top - landing_top) <= SUPPORT_LEVEL_TOL) & ~on_floor
        if not np.any(at_level):
            continue
        if forbidden:
            # 非優先(非ソフト)荷物が優先(ソフト)荷物の上に乗るのはハード禁止(下敷き防止)
            forbidden_hit |= at_level
            continue
        area = np.where(at_level, ow * oh_, 0.0)
        sum_area += area
        sum_ratio += area / item_area
        cen_x += area * (x_lo + x_hi) * 0.5
        cen_y += area * (y_lo + y_hi) * 0.5
        span_x_lo = np.where(at_level, np.minimum(span_x_lo, x_lo), span_x_lo)
        span_x_hi = np.where(at_level, np.maximum(span_x_hi, x_hi), span_x_hi)
        span_y_lo = np.where(at_level, np.minimum(span_y_lo, y_lo), span_y_lo)
        span_y_hi = np.where(at_level, np.maximum(span_y_hi, y_hi), span_y_hi)

    safe_area = np.maximum(sum_area, 1e-12)
    off_x = np.abs(cen_x / safe_area - world_x) / max(half[0], 1e-9)
    off_y = np.abs(cen_y / safe_area - world_y) / max(half[1], 1e-9)
    span_ok = (((span_x_hi - span_x_lo) >= MIN_SUPPORT_SPAN_RATIO * 2.0 * half[0]) &
               ((span_y_hi - span_y_lo) >= MIN_SUPPORT_SPAN_RATIO * 2.0 * half[1]))
    balanced = span_ok & (off_x <= MAX_SUPPORT_CENTROID_OFFSET) & (off_y <= MAX_SUPPORT_CENTROID_OFFSET)
    stacked_ok = (sum_ratio >= MIN_SUPPORT_RATIO) | ((sum_ratio >= MIN_UNION_SUPPORT_RATIO) & balanced)
    support_ok = on_floor | (stacked_ok & ~forbidden_hit)
    landing_ratio = np.where(on_floor, 1.0, np.minimum(sum_ratio, 1.0))

    world_z = landing_top + half[2] + geo.REST_CLEARANCE
    ceiling_limit = height - thickness - geo.START_MARGIN
    valid_h = (world_z + half[2]) <= ceiling_limit
    world_pos = np.stack([world_x, world_y, world_z], axis=1)

    slack = geo.inclusion_slack_batch(container, half, world_pos)
    incl = slack <= geo.INCLUSION_MARGIN
    base_legal = incl & valid_h & support_ok

    # Phase11: fill期待値の評価は「目標点」ではなく「沈降後の静止姿勢」の slack で行う。
    # 目標zは支持面から geo.REST_CLEARANCE(16mm)だけ浮かせた点だが、配置後の物理演算で
    # 荷物は必ず支持面まで落ちる。本家 evaluator は静止後の8角点を inclusion_margin=-0.005 で
    # 判定するため、床直置きの荷物は「底面が内床面と一致 -> dot≈0 > -0.005」で必ず
    # fill集計から脱落する(実測: 既積み6個だけの初期状態の fill_score は 0.00)。
    # 目標点の slack(=-0.016)で risk を測ると床置きを 0.55 の期待値で過大評価してしまうため、
    # 沈降後(z を REST_CLEARANCE だけ下げた点)の slack を fill リスク評価に使う。
    settled_pos = world_pos.copy()
    settled_pos[:, 2] -= geo.REST_CLEARANCE
    settled_slack = geo.inclusion_slack_batch(container, half, settled_pos)

    if not item_is_prioritized:
        min_final = world_pos - half[None, :]
        max_final = world_pos + half[None, :]
        for center, oh, sup_prioritized, _ in supports:
            if not sup_prioritized:
                continue
            too_close = geo.box_overlap_batch(min_final, max_final, center, oh,
                                               margin_xy=PRIORITY_CLEARANCE_XY, margin_z=PRIORITY_CLEARANCE_Z)
            base_legal = base_legal & ~too_close

    if not np.any(base_legal):
        if stats is not None:
            if not np.any(support_ok):
                stats['fail_support'] = stats.get('fail_support', 0) + 1
            elif not np.any(incl):
                stats['fail_inclusion'] = stats.get('fail_inclusion', 0) + 1
            elif not np.any(valid_h):
                stats['fail_ceiling'] = stats.get('fail_ceiling', 0) + 1
            else:
                stats['fail_inclusion_and_ceiling'] = stats.get('fail_inclusion_and_ceiling', 0) + 1
        return None

    # 直置き面(床 or 棚上面)なら浮上なし、それ以外(荷物の上)は搬入時に少し浮かせてから下ろす
    resting_values = [thickness, height / 2.0 + thickness + buffer]
    is_resting = np.zeros(n, dtype=bool)
    for rv in resting_values:
        is_resting |= np.isclose(landing_top, rv, atol=1e-3)

    # validator.check_transport_path と同式の「浮上量(effective_start_z)クリップ」。
    # 常設の小棚(と大棚)は概ね height/2 付近にあるため、非直置き(浮上あり)の掃引が
    # その高さ帯を大きくまたぐ場合、本家は浮上量を天井余裕(ceiling_margin)まで切り詰める。
    # ここを単純な「コンテナ天井のみ」の上限にしていると、本家より高い(=より安全に見える)
    # sweep_z を使ってしまい、実際には安全マージンを割り込む候補を合法と誤判定しうる
    # (実測: 掃引がこの中間高さをまたいだ候補で real validator 側の距離0.0149mの衝突を確認)。
    top_z = world_z + half[2]
    effective_start = np.where(is_resting, 0.0, geo.START_Z)
    handled = is_resting.copy()
    for c_z in (height / 2.0 + buffer, height + buffer - thickness):
        clearance = c_z - top_z
        trigger = (~handled) & (clearance >= 0.0) & (clearance < (effective_start + geo.CEILING_MARGIN))
        clipped = np.maximum(0.0, clearance - geo.CEILING_MARGIN - 0.0005)
        effective_start = np.where(trigger, clipped, effective_start)
        handled = handled | trigger

    ceiling_sweep = height + buffer - thickness - half[2] - geo.START_MARGIN
    sweep_z = np.minimum(ceiling_sweep, world_z + effective_start)

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
        if stats is not None:
            survivors = base_legal.copy()
            if not np.any(survivors & legal1):
                stats['fail_transport_y'] = stats.get('fail_transport_y', 0) + 1
            elif not np.any(survivors & legal1 & legal2):
                stats['fail_transport_x'] = stats.get('fail_transport_x', 0) + 1
        return None

    if stats is not None:
        stats['success'] = stats.get('success', 0) + 1

    contact = _contact_bonus(container, half, world_x, world_y, world_z, obstacles)
    scores = _score(container, local_x, local_y, world_z, half, item, landing_ratio, contact, settled_slack)
    scores = np.where(legal, scores, -np.inf)
    best_i = int(np.argmax(scores))
    if not legal[best_i]:
        return None

    return {
        'score': float(scores[best_i]),
        'local_pos': np.array([local_x[best_i], local_y[best_i], world_z[best_i]], dtype=np.float32),
        'slack': float(settled_slack[best_i]),
    }


def _y_slice_bounds(container, n_slices: int):
    """奥(+width/2)から手前(-width/2)へ向けて、levelごとに開放するy下限(手前側の境界)。

    level=0 が最も奥側だけを開放した最狭状態、level=n_slices-1 は全開放(従来と同じ全域)。
    """
    width = container['width']
    step = width / max(n_slices, 1)
    bounds = []
    for level in range(n_slices):
        if level >= n_slices - 1:
            bounds.append(-width / 2.0 - 1.0)  # 全開放。境界の浮動小数誤差を避け十分大きく余裕を取る
        else:
            bounds.append(width / 2.0 - (level + 1) * step)
    return bounds


def _apply_y_slice_filter(candidate_xy, half_y, y_active_lo):
    """候補のうち、手前側の端(local_y - half_y)がy_active_lo以上(=開放層内)のものだけを残す。"""
    if candidate_xy.shape[0] == 0:
        return candidate_xy
    keep = (candidate_xy[:, 1] - half_y) >= (y_active_lo - Y_SLICE_EPS)
    return candidate_xy[keep]


def _search_best(container_list, pool_list, n_pool, deadline, enforce_priority_container,
                  has_prioritized_container, rng=None, score_noise=0.0, stats=None,
                  grid_density: int = BASE_GRID_DENSITY, n_y_slices: int = Y_SLICE_COUNT,
                  reserve_priority_container: bool = False):
    """
    (container × pool item × orientation × 候補位置) を総当たりし、合法な手のうち最良を返す。
    enforce_priority_container=True の間は、優先コンテナが存在するのに優先荷物を非優先
    コンテナへ置く候補そのものを生成しない(placement_score維持のためのハード優先)。
    reserve_priority_container=True の間は、その逆向きの「席取り」も行う(Phase11)。
    ただしこちらは候補を消すのではなく、候補を2段の tier に分けて tier を score より優先する
    ランキングにする:
      tier 0 = 非優先荷物が非優先コンテナに入る / 優先荷物が優先コンテナに入る(望ましい)
      tier 1 = 非優先荷物が優先コンテナに入る(優先コンテナの容積・搬入経路を潰す)
    tier 0 の合法手が1つでもあれば必ずそちらを選び、tier 0 が皆無のときだけ tier 1 に落ちる。
    「探索段を1つ増やす」実装にすると、探索段どうしが同じ time_budget を食い合って後段が
    時間切れで打ち切られ、かえって合法手を取り逃す(実測: P03 で 21個→14個配置に悪化)ため、
    同一パス内のランキングとして実装し追加コストを0にしている。
    rng/score_noise は offline の順序探索(複数リスタート)でのみ使う微小ノイズで、
    online呼び出し(デフォルト rng=None)には一切影響しない。
    stats: tools/diagnose_stall.py 専用の診断カウンタ(Noneなら何もしない)。
    grid_density: 候補XYグリッドの密度倍率。既定は BASE_GRID_DENSITY。plan()が通常探索で
    全滅した場合のみ、残り時間予算内でさらに密度を上げた最終リトライに使う(Phase7: 「合法手
    なし」と誤って諦める頻度を減らし、agent.pyの無検証フォールバック=即死へ落ちる回数を
    減らすため)。
    n_y_slices: Phase9の層規律の分割数。コンテナごとに独立して「まだ奥に置けるなら手前は
    使わない」を保証するため、コンテナのループの内側でlevelを0から昇順に試し、合法候補が
    見つかった時点でそのコンテナの手番を確定する(それより手前の層は開放しない)。
    """
    best_overall = None
    for container in container_list:
        if time.perf_counter() > deadline:
            break
        container_is_prioritized = container.get('is_prioritized', False)
        obstacles = _collect_obstacles(container)
        supports = _landing_supports(container)
        y_bounds = _y_slice_bounds(container, n_y_slices)
        # (pool_idx, orn_idx) -> (half, 全域候補xy)。層のlevelを上げてもgrid/extreme point自体は
        # 変わらないため、y絞り込みだけをlevelごとにやり直せるようキャッシュして再計算を避ける。
        candidate_cache: dict = {}

        container_best = None
        for level_idx, y_active_lo in enumerate(y_bounds):
            if time.perf_counter() > deadline:
                break
            # 最終levelは全開放(従来の全域探索と同値)。y絞り込みのマスク生成・コピーは
            # 候補配列サイズ分のコストがかかるため、無駄なオーバーヘッドを避けるため省略する
            # (n_y_slices<=1の場合は常にここに該当し、実質従来のplanner.pyと同じ速度になる)。
            is_fully_open = level_idx == len(y_bounds) - 1
            level_best = None
            for pool_idx in range(n_pool):
                if time.perf_counter() > deadline:
                    break
                item = pool_list[pool_idx]
                item_is_prio = item.get('is_prioritized', False)
                if enforce_priority_container and has_prioritized_container \
                        and item_is_prio and not container_is_prioritized:
                    continue
                # tier 0 が1つでもあれば tier 1 は絶対に選ばれない(席取りのハード優先)。
                tier = 1 if (reserve_priority_container and container_is_prioritized
                             and not item_is_prio) else 0
                lwh = (item['length'], item['width'], item['height'])

                for orn_idx in _unique_orientations(lwh):
                    if time.perf_counter() > deadline:
                        break
                    cache_key = (pool_idx, orn_idx)
                    if cache_key not in candidate_cache:
                        half = geo.half_extent(lwh, orn_idx)
                        full_xy = _candidate_xy(container, half, obstacles, grid_density=grid_density)
                        candidate_cache[cache_key] = (half, full_xy)
                    half, full_xy = candidate_cache[cache_key]
                    candidate_xy = full_xy if is_fully_open else _apply_y_slice_filter(full_xy, half[1], y_active_lo)
                    r = _evaluate_candidates(container, item, half, obstacles, supports, candidate_xy, deadline, stats=stats)
                    if r is None:
                        continue

                    score = r['score']
                    if rng is not None and score_noise > 0.0:
                        score = score + float(rng.normal(0.0, score_noise))

                    rank = (-tier, score)
                    if level_best is None or rank > level_best['rank']:
                        level_best = {
                            'rank': rank,
                            'score': score,
                            'local_pos': r['local_pos'],
                            'item_idx': pool_idx,
                            'container_idx': container['index'],
                            'orientation': orn_idx,
                            'slack': r['slack'],
                        }
            if level_best is not None:
                container_best = level_best
                break  # このコンテナは現在開放中の層内で置けるため、より手前の層は開放しない

        if container_best is not None:
            if best_overall is None or container_best['rank'] > best_overall['rank']:
                best_overall = container_best
    return best_overall


def plan(container_list: list[dict], pool_list: list[dict], time_budget: float = 5.5,
         max_pool_items: int | None = MAX_POOL_ITEMS, rng=None, score_noise: float = 0.0,
         stats=None, info: dict | None = None) -> dict | None:
    """
    max_pool_items: online(agent.policy)は既定のMAX_POOL_ITEMSで呼ぶ。offlineの順序探索
    (ordering.build_order)は None を渡し、プール全件(=候補となる全未配置荷物)から
    その時点で最良の1手を選べるようにする(cf. simulate.greedy_construct_order)。
    stats: tools/diagnose_stall.py が「なぜ全滅したか」を調べるための診断カウンタ辞書
    (省略時Noneで、本番のonline/offline呼び出しには一切影響しない)。
    info: 選ばれた最良候補の付帯情報('slack'=壁からの余裕)を書き戻す辞書(省略時None)。
    simulate.py が offline探索の目的関数(risk調整済みvolume)を計算するために使う。
    actionの実キー({item_idx,container_idx,place_pos,orientation})には含めない
    (env側のフォーマットチェックを壊さないため)。
    """
    start = time.perf_counter()
    deadline = start + time_budget

    n_pool = len(pool_list) if max_pool_items is None else min(len(pool_list), max_pool_items)
    has_prioritized_container = any(c.get('is_prioritized', False) for c in container_list)
    has_plain_container = any(not c.get('is_prioritized', False) for c in container_list)

    # Phase11(ターゲット1): 優先コンテナの「席取り」。
    # placement_score が減点される唯一の実測要因は「優先コンテナが満杯/搬入不能になった後に
    # 到着した優先荷物が非優先コンテナへ回される」ケースだった(D03/P03の実測: 減点2件とも
    # 下敷きではなく wrong-container、いずれも優先コンテナ側は fail_transport_y で合法手ゼロ)。
    # その時点で優先コンテナには非優先荷物が先に入り込んで容積・搬入経路を潰していたため、
    # 「非優先コンテナが1台でもあるなら、非優先荷物はそちらを使い切るまで優先コンテナに
    # 入れない」を reserve_priority_container で担保する(_search_best の tier 参照)。
    # tier は同一パス内のランキングなので探索コストは増えず、tier 0 の合法手が皆無なら
    # 自動的に tier 1(=優先コンテナ)へ落ちるため「置けたはずの荷物を置けなくする」ことは
    # 無い(=配置数・fillを構造的に減らさない)。
    best_overall = _search_best(container_list, pool_list, n_pool, deadline, enforce_priority_container=True,
                                 has_prioritized_container=has_prioritized_container, rng=rng,
                                 score_noise=score_noise, stats=stats,
                                 reserve_priority_container=has_prioritized_container and has_plain_container)
    if best_overall is None and has_prioritized_container and time.perf_counter() <= deadline:
        # 優先コンテナ限定では合法手が全く無かった場合のみ、非優先コンテナも含めて再探索する
        # (それ以上待っても優先コンテナに入らない荷物を無駄に足止めしないため)。
        best_overall = _search_best(container_list, pool_list, n_pool, deadline, enforce_priority_container=False,
                                     has_prioritized_container=has_prioritized_container, rng=rng, score_noise=score_noise,
                                     stats=stats)

    if best_overall is None and time.perf_counter() <= deadline:
        # Phase7: 通常密度のグリッド+Extreme Pointで合法候補が1つも無かった場合の最終リトライ。
        # 「本当に空間的行き詰まり」なのか「粗いグリッドが偶然、荷物が収まる細い隙間を
        # 拾えなかっただけ」なのかを区別せず一律諦めると、agent.py側は合法性チェックを
        # 一切行わない無検証フォールバックに落ちてsudden death(即座にエピソード終了、
        # 残り全荷物を失う)になる。同じ margin(=real validatorに対して安全側)のまま
        # グリッド密度だけを上げ、残り時間予算内で「本当に置ける場所が無いか」をもう一段
        # 丁寧に探す。時間予算(deadline)は呼び出し元が渡した time_budget のままで、
        # 新たに延長はしない(policy_timeout=8sに対する安全マージンを保つため)。
        best_overall = _search_best(container_list, pool_list, n_pool, deadline, enforce_priority_container=False,
                                     has_prioritized_container=has_prioritized_container, rng=rng, score_noise=score_noise,
                                     stats=stats, grid_density=RETRY_GRID_DENSITY)

    if best_overall is None:
        return None

    if info is not None:
        info['slack'] = best_overall['slack']

    return {
        'item_idx': best_overall['item_idx'],
        'container_idx': best_overall['container_idx'],
        'place_pos': best_overall['local_pos'].astype(np.float32),
        'orientation': best_overall['orientation'],
    }
