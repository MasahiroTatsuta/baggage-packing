"""
optimize() 用のオフライン・シャドウシミュレータ。

pybullet の物理演算(重力・接触)は使わず、planner.py と全く同じ幾何・合法性・スコアリング
ロジックだけを使って「この順序/この貪欲戦略で実際に詰めたら何個(どれだけの体積)置けるか」を
高速に見積もる。online の policy() は毎ステップ observation から状態を再構築するだけの
純関数であり、内部的には geometry.py の保守的判定だけで合否を決めているため、この
シャドウシミュレータで得られる配置結果は実際の policy() ループの挙動を高い精度で再現する
(実機の物理沈み込みや衝撃までは再現しないが、順序の善し悪しを比較する探索の評価関数としては
十分な精度)。

container dict / item dict は get_init_states() / get_info_for_optimization() が返す
生の dict をそのまま複製して使う。既配置荷物の姿勢は ORNS[orn_idx] に対応する
クオータニオンを直接組み立てて付与するため、pybullet の物理クライアントは一切不要。
"""
import os

import numpy as np
import pybullet as p

from . import geometry as geo
from . import planner
from src.ground_handling.utils import ORNS

_ORN_QUATS = [p.getQuaternionFromEuler(e) for e in ORNS]

# Phase28: 到達可能性評価(reach.stall_reachability)1回あたりのコストを、SearchBudget の
# ユニット系へ換算する係数。生コストは「voxel格子数 × 評価したユニーク形状数」にほぼ比例する
# (どちらも numpy のベクトル演算の反復回数を決める量)。planner.CANDIDATE_BUILD_COST と
# 同じ考え方で、実測時間 × UNITS_PER_SEC が同じユニット数になるよう較正する。
# 較正値の実測根拠は results/phase28_report.md §3。3シーン×2反復の実測(voxel=0.10)は
#   A01: 3990cells×93shapes=0.082s / A02: 3990×215=0.118s / A03: 3990×87=0.074s
# で、集計 (総units / 総(cells*shapes)) = 2.69。planner.UNITS_PER_SEC とは逆に、この定数は
# **大きめに置くほど保守的**(名目予算を早めに使い切る=壁時計の安全弁を踏みにくい)なので、
# 集計値をわずかに上回る 3.0 を採用する。
REACH_UNIT_COST = float(os.environ.get('MYSOLVER_REACH_UNIT_COST', '3.0'))

# Phase31: risk_vol(候補順序を選ぶ目的関数)の壁からの余裕をどの面から測るかを切り替える。
# 既定 'all' は Phase6 以来の「全面の最悪値」(壁・天井・切り欠きも含む inclusion_slack_batch)。
# Phase21 の全数監査(results/phase21_report.md)は574件の実際の脱落原因が
# floor 99.7% / back 0.3% で、側壁・天井・切り欠きによる脱落は実測0件だったと確定している。
# つまり 'all' は実際にはほぼ起きない側壁接近まで強く罰しており、壁際まで詰めた
# 密な(=真のfillが高い)配置を誤って割り引いている可能性がある(results/phase31_report.md)。
# 'floor' は geo.inclusion_slack_batch(floor_only=True) で内床面だけを見る。
# **この値は risk_vol(=候補順序の選択指標)にのみ影響する。** _score() の boundary_term
# (online policy() / offline構築の両方が使う配置スコア)や check_inclusion_batch(hard
# legality)は一切変更しないため、「どこに何を置くか」(=各候補順序の実際の配置内容)は
# 既定のまま完全に不変で、「作り終えた複数の候補順序のどれを勝者に選ぶか」だけが変わる。
# Phase33: ローカルA/Bは t=1.177(単体)で採否基準未達だったが、Phase32の機序検証
# (勝者交代5シーン中4シーンで新勝者の方が過大評価が少ない)を根拠に既定を 'floor' へ
# 切り替えてpublicで直接検証した。実測 53.64328945200516 はベースライン53.64比で
# +0.003ptとノイズ床(判定基準 -0.1〜+0.3)の内側だったため、既定を 'all' に戻して
# この路線を不採用で確定した(results/phase33_report.md §1.3)。
RISK_SLACK_FACES = os.environ.get('MYSOLVER_RISK_SLACK_FACES', 'all')


def clone_containers(container_list: list[dict]) -> list[dict]:
    """container dict のリストを、packed_items も含めて浅くない複製にする。"""
    cloned = []
    for c in container_list:
        cc = dict(c)
        cc['packed_items'] = [dict(item) for item in c.get('packed_items', [])]
        cloned.append(cc)
    return cloned


def _place(container: dict, item: dict, action: dict) -> dict:
    """action(planner.plan の戻り値)に従い item を仮想的に配置し、packed_items に追加できる
    dict を返す(pos/orn を実配置相当の値で埋める)。"""
    ox = container['center'][0]
    lp = action['place_pos']
    placed = dict(item)
    placed['pos'] = (float(lp[0]) + ox, float(lp[1]), float(lp[2]))
    placed['orn'] = _ORN_QUATS[action['orientation']]
    return placed


# ---------------------------------------------------------------------------
# Phase49(作業1): cog(質量加重重心)の影シミュレータ代理。
#
# 正規化式は tools/scorer.py::Scorer._get_floor_ceil_z / calculate_cog_score から
# **そのまま移植**(独自の式は作らない、という指示どおり)。本物は real Container
# オブジェクトの属性アクセス(.points/.n_vecs/.center)・pybulletの実姿勢
# (item.get_pose)を使うのに対し、こちらは observation の dict 表現
# (container['points']/container['n_vecs']/container['center']、
# item['pos'])を読む点だけが違う——CLAUDE_CODE_指示書.md §2が示す観測dictの
# 契約上、container dict は本物のContainer.create()と同じ points/n_vecs を
# 持つため、床/天井面の推定ロジックも含めて完全に同一の計算になる。
#
# 本物との違いは「real physics(pybulletの沈降)後の位置」ではなく「候補構築時点の
# pos(=plannerが狙った着地点)」を使うことだけ(simulate.pyの他の全指標と同じ性質の
# sim-to-realギャップ)。この代理が本物のcog_score順位をどれだけ保つかはPhase49
# 作業2でSpearman相関により検証する(結果次第でこの先には進まない)。
# ---------------------------------------------------------------------------
def _floor_ceil_z(container: dict) -> tuple[float, float]:
    """tools/scorer.py::Scorer._get_floor_ceil_z と同一の正規化式。"""
    center = container['center']
    height = container['height']
    floor_z = center[2] - height / 2.0
    ceil_z = center[2] + height / 2.0
    for pt, nv in zip(container.get('points', []), container.get('n_vecs', [])):
        if abs(nv[0]) < 1e-6 and abs(nv[1]) < 1e-6:
            if nv[2] < -0.5:
                floor_z = pt[2]
            elif nv[2] > 0.5:
                ceil_z = pt[2]
    return floor_z, ceil_z


def cog_proxy_score(containers: list[dict]) -> float:
    """tools/scorer.py::Scorer.calculate_cog_score と同一の質量加重・正規化式。
    container['packed_items'] の各要素の pos[2](世界座標z)と mass だけを使う。"""
    total_mass = 0.0
    weighted_h = 0.0
    for container in containers:
        packed = container.get('packed_items', [])
        if not packed:
            continue
        floor_z, ceil_z = _floor_ceil_z(container)
        effective_height = max(ceil_z - floor_z, 1e-6)
        for item in packed:
            pos = item.get('pos')
            if pos is None:
                continue
            mass = item.get('mass', 0.0)
            normalized_h = (pos[2] - floor_z) / effective_height
            normalized_h = min(max(normalized_h, 0.0), 1.0)
            weighted_h += mass * normalized_h
            total_mass += mass

    if total_mass == 0:
        return 100.0

    avg_h = weighted_h / total_mass
    return min(max(100.0 * (1.0 - avg_h), 0.0), 100.0)


def simulate_order(container_list: list[dict], items_by_index: dict[int, dict], order: list[int],
                    lookahead_k: int, budget: planner.SearchBudget, per_step_time_budget: float = 0.7,
                    prepacked_ids: dict | None = None,
                    stability_weight: float = 1.0,
                    reach_info: dict | None = None,
                    stall_info: dict | None = None,
                    resume_state: dict | None = None,
                    snapshot_after: int | None = None,
                    snapshot_out: dict | None = None,
                    snapshots_out: dict | None = None,
                    contrib_out: list | None = None,
                    compute_cog_proxy: bool = False) -> tuple:
    """
    online の ItemStreamManager(lookahead_k個のプールを毎ステップ最大まで補充)と同じ
    プール管理則で、順序 order 通りに荷物を流し込みながら planner.plan を毎ステップ呼ぶ。
    実際の policy() 呼び出しと同じ既定(max_pool_items=既定値)で呼ぶことで、
    「このorderを実機に渡したら何個置けるか」の妥当な見積もりになる。

    per_step_time_budget=0.7: Phase8で planner.BASE_GRID_DENSITY を1→2に上げたことに
    合わせて引き上げた値(旧値0.35は、pool=MAX_POOL_ITEMS・密度1の1手評価に要する実測時間
    (約0.35s)に由来する較正値だったため、密度を上げた分だけ比例して増やし、lookahead_k>1
    のシーン(pool>1)でここが毎回途中打ち切りになり評価が不正確になるのを防ぐ)。
    Phase17以降、この「秒」は planner.UNITS_PER_SEC で決定的にユニット数へ換算される
    名目値であり、実際の打ち切りは壁時計ではなく消費ユニット数で決まる。

    budget: planner.SearchBudget。この検証1回に使えるユニット予算(親=optimize全体の予算)。

    stability_weight: Phase18。積み上げの幾何代理リスク(geo.stacking_instability_risk)を
    risk調整済み体積へ割り引く強さ([0,1]のクリップ後の乗数 (1 - stability_weight*risk) を
    その荷物自身の寄与にだけ掛ける)。**個々の荷物の寄与を超えては割り引けない**設計が
    重要: 初版は「平均リスク率 * コンテナ総容積」を目的関数から一律に減算する加算ペナルティ
    だったが、bulky系(荷物が大きく、積み上げなしでは大半が入らない)シーンで実際の
    達成可能な体積(risk_adjusted_volume)よりペナルティのほうが大きくなり、「何も置かない
    ほうが目的関数上は得」という退化解に収束する重大な回帰を引き起こした(実測:
    suite_A07_1c_40_bulky で fill_strict 28.07->0.00, 40個中1個しか置かない順序が選ばれた)。
    荷物ごとの寄与に対してのみ割り引く(=常に0以上)設計にすることで、「置かないほうが得」
    という退化解を構造的に排除する。

    戻り値: (配置できた item index のリスト(配置順), 配置できた体積の合計,
             risk調整済み体積の合計, placement違反率, stability平均リスク)。
    placement違反率は tools/scorer.calculate_placement_score と同じ定義の
    「優先コンテナがあるのに非優先コンテナへ入った優先手荷物の数 / 配置された優先手荷物の数」
    (既積みの優先手荷物も分母・分子に含む)。Phase11: 順序探索がこの違反を明示的に避けられる
    ようにするため(placement_score は離散ペナルティで、順序次第で 100 に戻せる)。
    risk調整済み体積は、各配置の壁からの余裕(geo.inclusion_slack_batch)を
    geo.fill_risk_factor で [0,1] に変換し、さらに積み上げの安定リスクで割り引いた体積の
    総和。real evaluator の厳しいinclusion_marginぎりぎりで壁ぎわに配置された荷物は、
    実機の沈降ドリフトでfill集計から漏れるリスクが高いとみなして割り引く
    (Phase6: sim-to-realギャップ対策)。offline探索(ordering.build_order)はこの値を
    目的関数の主指標として使う。
    stability平均リスクは geo.stacking_instability_risk (Phase18: 実stability_score(shake
    test)には無い、静的安定の必要条件の幾何代理) が返す [0,1] の連続リスクを積み上げ
    (床/棚への直置きでない配置)についてのみ平均したもの(直置きは常に安定側で0扱い・
    分母にも含めない)。診断・報告用に返すだけで、目的関数自体はこの平均値ではなく
    上記の荷物ごとの割引(risk調整済み体積に織り込み済み)を使う。
    実測(旧gen_2containers_priorityの探索で生成された全候補順序)では候補生成側
    (planner._evaluate_candidates)の合法性判定が既に「あからさまに際どい支持」をハード
    排除しているため、二値の違反フラグにすると閾値に一度も抵触せず目的関数への寄与が
    恒等的にゼロになった。そのため「閾値に対する消費割合」を連続量として使う設計にしている
    (geo.fill_risk_factor が壁際配置を連続的に割り引くのと同じ発想)。
    Phase17までの目的関数(risk調整済み体積 − placementペナルティ)には stability の代理が
    一切無く、目的関数上は改善するが実stabilityが悪化する順序に探索が収束する問題があった
    (results/phase17_report.md §3.5)。

    Phase33(調査専用フック、既定無効): `resume_state`/`snapshot_after`/`snapshot_out` は
    ALNSの「接頭辞再開」が実装可能かを検証するための足場。**どちらも既定Noneで、指定しない限り
    Phase32までの経路とビット単位で同一**(この3引数を一切使わない呼び出しは無変更)。
    `snapshot_after=k` を渡すと、k個目を配置した直後(refill後)の完全な内部状態
    (containers/pool/残order/累積量)を `snapshot_out` に書き込んでそこでループを止める。
    `resume_state`(`snapshot_out`と同じ形の dict)を渡すと、その状態から続きを流し込む
    (containers/order を最初から構築し直さない)。resume_state を使う呼び出しでは
    container_list/order 引数は無視される。

    Phase34(ALNS用フック、既定無効): `snapshots_out` に dict を渡すと、**ロールアウトを
    止めずに**すべての配置数 k について同じ形の状態を `{k: state}` として記録する
    (ALNS が任意の接頭辞から再開できるようにするため。1回のロールアウトで全 k 分の
    スナップショットが揃うので、反復ごとに接頭辞を作り直す必要が無い)。
    `contrib_out` にリストを渡すと `(item index, risk割引率)` を配置順に記録する
    (ALNS の worst removal が「最も割引された=無駄の大きい配置」を選ぶのに使う)。
    どちらも既定 None で、渡さない呼び出しは Phase33 までの経路と完全に同一である。

    Phase49(作業1、既定無効): `compute_cog_proxy=True` を渡すと、戻り値の末尾に
    `cog_proxy_score(containers)`(質量加重重心の代理スコア、tools/scorer.py::
    calculate_cog_score と同一の正規化式)を追加した **6要素タプル** を返す。
    **既定Falseのときは戻り値は従来どおり5要素タプルのまま**(既存の呼び出し側は
    5値でunpackしており、6要素に変えると壊れるため)。
    """
    if resume_state is not None:
        containers = resume_state['containers']
        has_prio_container = any(c.get('is_prioritized', False) for c in containers)
        n_prio_placed = resume_state['n_prio_placed']
        n_prio_misrouted = resume_state['n_prio_misrouted']
        _src = list(resume_state['remaining_order'])
        pool = list(resume_state['pool'])
        placed_ids = list(resume_state['placed_ids'])
        placed_volume = resume_state['placed_volume']
        risk_adjusted_volume = resume_state['risk_adjusted_volume']
        n_stacked = resume_state['n_stacked']
        stacking_risk_sum = resume_state['stacking_risk_sum']
        _cur = 0

        def refill():
            # Phase34: 旧実装は iter() を使っていたが、複数スナップショットでは
            # 「残りの order」を **消費せずに** 読み出す必要があるため、明示的な
            # リスト+カーソルへ置き換えた(引き込む順序も個数も従来と完全に同一)。
            nonlocal _cur
            while len(pool) < lookahead_k and _cur < len(_src):
                pool.append(dict(items_by_index[_src[_cur]]))
                _cur += 1
    else:
        containers = clone_containers(container_list)
        has_prio_container = any(c.get('is_prioritized', False) for c in containers)
        n_prio_placed = 0
        n_prio_misrouted = 0
        for c in containers:
            for it in c.get('packed_items', []):
                if it.get('is_prioritized', False):
                    n_prio_placed += 1
                    if has_prio_container and not c.get('is_prioritized', False):
                        n_prio_misrouted += 1
        _src = list(order)
        pool = []
        _cur = 0

        def refill():
            nonlocal _cur
            while len(pool) < lookahead_k and _cur < len(_src):
                pool.append(dict(items_by_index[_src[_cur]]))
                _cur += 1

        refill()
        placed_ids: list[int] = []
        placed_volume = 0.0
        risk_adjusted_volume = 0.0
        n_stacked = 0
        stacking_risk_sum = 0.0

    def _snapshot():
        return {
            'containers': clone_containers(containers),
            'pool': [dict(it) for it in pool],
            'remaining_order': list(_src[_cur:]),
            'placed_ids': list(placed_ids),
            'placed_volume': placed_volume,
            'risk_adjusted_volume': risk_adjusted_volume,
            'n_stacked': n_stacked,
            'stacking_risk_sum': stacking_risk_sum,
            'n_prio_placed': n_prio_placed,
            'n_prio_misrouted': n_prio_misrouted,
            'budget_used': budget.used,
        }

    while pool:
        if snapshots_out is not None and len(placed_ids) not in snapshots_out:
            snapshots_out[len(placed_ids)] = _snapshot()
        if (snapshot_after is not None and snapshot_out is not None
                and len(placed_ids) == snapshot_after):
            snapshot_out.update(_snapshot())
            break
        # Phase17: 壁時計ではなく親の残ユニットで打ち切る(同一入力なら同じ手数・同じ結果)。
        if budget.exhausted():
            break
        info: dict = {}
        action = planner.plan(containers, pool, info=info, prepacked_ids=prepacked_ids,
                               budget=budget.child_seconds(per_step_time_budget))
        if action is None:
            # Phase29: 行き詰まった瞬間の状態(その時点のコンテナと、置けなかったプール)を
            # 呼び出し側へ渡す。**dict を渡さなければ何も起きない**(既定 None)ので、
            # Phase28 までの経路とビット単位で同一。containers/pool はこのループを抜けた
            # あとは読み取り専用にしかならないため、複製せず参照をそのまま渡す。
            if stall_info is not None:
                stall_info['containers'] = containers
                stall_info['pool'] = list(pool)
                stall_info['n_placed'] = len(placed_ids)
                stall_info['stalled'] = True
            break
        item = pool.pop(action['item_idx'])
        container = containers[action['container_idx']]
        placed = _place(container, item, action)
        # Phase18: 積む前(=直下候補になりうる既配置荷物だけ)の packed_items に対して判定する。
        is_stacked, stacking_risk = geo.stacking_instability_risk(
            container['packed_items'], placed, item.get('mass', 0.0))
        container['packed_items'].append(placed)
        if is_stacked:
            n_stacked += 1
            stacking_risk_sum += stacking_risk
        placed_ids.append(item['index'])
        if item.get('is_prioritized', False):
            n_prio_placed += 1
            if has_prio_container and not container.get('is_prioritized', False):
                n_prio_misrouted += 1
        item_volume = item['length'] * item['width'] * item['height']
        placed_volume += item_volume
        # Phase18: 積み上げリスクは「この荷物自身の寄与」だけを割り引く(0未満にはしない)。
        # コンテナ総容積に対する加算ペナルティにすると、達成可能な体積より罰則が大きくなる
        # シーンで「何も置かない」ほうが目的関数上有利になる退化解を生む(関数docstring参照)。
        stability_discount = max(0.0, 1.0 - stability_weight * stacking_risk)
        # Phase20(ターゲット2、既定では無効): fill計上の期待値を「配置目標点」ではなく
        # 「沈降後の静止姿勢」の slack で評価する案。planner.USE_SETTLED_SLACK が False の
        # ときは settled_slack は計算すらされず None なので、Phase19 と完全に同じ式になる。
        # 採用を見送った理由は planner.USE_SETTLED_SLACK のコメントと
        # results/phase20_report.md §3 を参照(較正は改善するが順位=意思決定を変えないため)。
        risk_slack = info.get('settled_slack') if planner.USE_SETTLED_SLACK else None
        if risk_slack is None:
            risk_slack = info.get('slack', geo.REAL_INCLUSION_MARGIN)
        if RISK_SLACK_FACES == 'floor':
            # Phase31: risk_vol選択指標だけを内床面基準で測り直す。位置そのものは
            # planner.plan が既に決めた placed['pos'] を使うだけなので、構築(どの位置に
            # 置くか)は一切変えていない。
            floor_half = geo.half_extent(
                (item['length'], item['width'], item['height']), action['orientation'])
            floor_slack = geo.inclusion_slack_batch(
                container, floor_half, np.array([placed['pos']], dtype=np.float64),
                floor_only=True)
            risk_slack = float(floor_slack[0])
        # **式の形と評価順序は Phase33 までと1文字も変えないこと**(浮動小数点の加算・乗算は
        # 結合則が成り立たないため、括弧のくくり直しだけでビット単位一致が壊れる)。
        risk_adjusted_volume += (item_volume * geo.fill_risk_factor(risk_slack)
                                  * stability_discount)
        if contrib_out is not None:
            # 診断用の副産物なので、こちらは本計算とは独立に別式で求めてよい。
            contrib_out.append((int(item['index']),
                                float(geo.fill_risk_factor(risk_slack)) * stability_discount))
        refill()

    # Phase28: 行き詰まり時点の到達可能性を **1回だけ** 測る。
    #
    # corridor_penalty(planner._corridor_excess)は候補1手ごとに、その時点の障害物に対して
    # しか罰則を課せないため「これから積み上がる荷物が作る通路」を守れない(時間方向の myopia、
    # results/phase24_report.md §2.3)。ここは順序を最後まで流し切った**後**なので、
    # 「その順序が最終的に自己封鎖したか」を直接観測できる —— myopia の原因そのものを外す。
    #
    # 毎ステップ回すと1ロールアウトのコストが跳ね上がり、同じ予算で回せる順序候補が減って
    # 逆効果になる(Phase22 の RETRY_GRID_DENSITY 4->8 と同型の失敗)。必ずこの1回だけにすること。
    # reach_info=None(既定)なら計算自体を行わないので、Phase27 までと完全に等価な no-op。
    if reach_info is not None:
        from . import reach as _reach
        remaining_items = list(pool)
        seen_idx = {int(it['index']) for it in remaining_items}
        for idx in _src[_cur:]:       # プールにまだ引き込まれていない残りの荷物
            if idx not in seen_idx:
                remaining_items.append(items_by_index[idx])
                seen_idx.add(idx)
        stats = _reach.stall_reachability(containers, remaining_items)
        reach_info.update(stats)
        # コストをユニット予算へ計上する。壁時計ではなく決定的な量(格子数×形状数)で課金する
        # ので、Phase17 で確立した決定性は保たれる。ここを計上しないと「ユニットは増えないのに
        # 壁時計だけ伸びる」状態になり、非常用安全弁(hard_deadline)を踏んで決定性が壊れる
        # (beam_construct_order の clone_containers に関する注意書きと同じ理由)。
        budget.spend(REACH_UNIT_COST * stats['grid_cells'] * max(1, stats['n_shapes']))

    violation_ratio = n_prio_misrouted / n_prio_placed if n_prio_placed else 0.0
    stability_risk_ratio = stacking_risk_sum / n_stacked if n_stacked else 0.0
    if compute_cog_proxy:
        # Phase49(作業1): 既定Falseのときはこの分岐に入らず、従来の5要素タプルのまま
        # (呼び出し側のビット単位不変を保つ)。
        return (placed_ids, placed_volume, risk_adjusted_volume, violation_ratio,
                stability_risk_ratio, cog_proxy_score(containers))
    return placed_ids, placed_volume, risk_adjusted_volume, violation_ratio, stability_risk_ratio


def _state_key(containers):
    """部分解の同一性キー。荷物の集合と実際の姿勢が同じなら、詰めた順番が違っても同一状態。

    ビームが同一状態のコピーで埋まると実質 b=1 の貪欲へ退化するため、これで重複を排除する。
    """
    parts = []
    for c in containers:
        for it in c['packed_items']:
            p = it['pos']
            parts.append((int(it['index']), round(p[0], 4), round(p[1], 4), round(p[2], 4)))
    parts.sort()
    return tuple(parts)


def beam_construct_order(container_list: list[dict], item_list: list[dict], budget: planner.SearchBudget,
                          per_step_time_budget: float = 3.0, rng=None, score_noise: float = 0.0,
                          shuffle_ties: bool = False, window: int | None = None,
                          prepacked_ids: dict | None = None, beam_width: int = 1,
                          top_k: int | None = None) -> list[int]:
    """Phase23: greedy_construct_order を幅 beam_width のビームサーチへ一般化したもの。

    各ステップで、ビーム内の各部分解について planner.plan_topk で上位 top_k 手を展開し、
    「そこまでに積んだ risk調整済み体積」で上位 beam_width 個を残す。この評価量は
    build_order の目的関数(simulate_order が返す risk_adjusted_volume)と同じ定義なので、
    ビームの枝刈り基準と最終的な採否基準が一致する。

    **beam_width=1 かつ top_k=1 は greedy_construct_order と同一の手順に退化する**
    (同じ探索を同じ順序で呼び、常に最良手1つだけを採用する)。実装の正しさは
    「b=1 の出力が greedy_construct_order と完全一致すること」で担保する。

    予算は planner.SearchBudget のユニットで管理し、いつ打ち切られても
    「それまでに構築できた順序 + 残りを末尾に付けた完全順列」を返す(anytime)。
    """
    if top_k is None:
        top_k = max(1, beam_width)

    # Phase23(修正): ビームの枝刈り基準を build_order の目的関数と厳密に一致させる。
    # 初版は risk調整済み体積だけで枝を選んでいたため、優先コンテナを非優先荷物で潰す枝を
    # 咎められず、実測で D03 の placement_score が 100->92.86 に落ちた(b=2)。
    # build_order の目的関数は「risk調整済み体積 − PLACEMENT_PENALTY_WEIGHT × 総容積 × 違反率」
    # なので、同じ定義の placement ペナルティをビームのスコアにも入れる
    # (重みは ordering 側の既存定数をそのまま使い、新しい調整パラメータは増やさない)。
    from . import ordering as _ordering
    total_container_volume = sum(c.get('volume', 0.0) for c in container_list)
    placement_w = getattr(_ordering, 'PLACEMENT_PENALTY_WEIGHT', 0.5)
    has_prio_container = any(c.get('is_prioritized', False) for c in container_list)

    def _prio_counts(containers):
        """simulate_order と同一定義(既積みの優先手荷物も分母・分子に含む)。"""
        placed = mis = 0
        for c in containers:
            for it in c['packed_items']:
                if it.get('is_prioritized', False):
                    placed += 1
                    if has_prio_container and not c.get('is_prioritized', False):
                        mis += 1
        return placed, mis

    def _objective(risk_vol, n_prio, n_mis):
        ratio = (n_mis / n_prio) if n_prio else 0.0
        return risk_vol - placement_w * total_container_volume * ratio

    base = clone_containers(container_list)
    remaining = {item['index']: dict(item) for item in item_list}
    if shuffle_ties and rng is not None:
        keys = list(remaining.keys())
        rng.shuffle(keys)
        remaining = {k: remaining[k] for k in keys}

    # state: containers / remaining(dict) / order(list) / score(float) / key(部分解の同一性)
    _p0, _m0 = _prio_counts(base)
    beam = [{'containers': base, 'remaining': remaining, 'order': [], 'risk_vol': 0.0,
             'n_prio': _p0, 'n_mis': _m0, 'score': _objective(0.0, _p0, _m0),
             'key': _state_key(base)}]
    best = beam[0]

    while True:
        if budget.exhausted():
            break
        # 展開候補は (親, action) のまま集め、**上位b個に絞ってから初めてコンテナを複製する**。
        # 素直に子ごとに clone_containers すると 1ステップあたり b*top_k 回の複製が発生し、
        # その費用はユニット予算に計上されないため、b を上げると壁時計だけが膨らんで
        # 非常用安全弁(hard_deadline)を踏み、Phase17 で確保した決定性を壊してしまう。
        # 子のスコアも同一性キーも action だけから増分計算できるので、複製は b 回で足りる。
        cands = []
        any_open = False
        for st in beam:
            if not st['remaining']:
                continue
            any_open = True
            pool = list(st['remaining'].values())
            if window is not None:
                pool = pool[:window]
            acts = planner.plan_topk(st['containers'], pool, top_k,
                                     budget.child_seconds(per_step_time_budget),
                                     max_pool_items=None, rng=rng, score_noise=score_noise,
                                     prepacked_ids=prepacked_ids)
            for a in acts:
                item = pool[a['item_idx']]
                cont = st['containers'][a['container_idx']]
                ox = cont['center'][0]
                lp = a['place_pos']
                pos = (float(lp[0]) + ox, float(lp[1]), float(lp[2]))
                vol = item['length'] * item['width'] * item['height']
                risk_slack = a.get('settled_slack') if planner.USE_SETTLED_SLACK else None
                if risk_slack is None:
                    risk_slack = a.get('slack', geo.REAL_INCLUSION_MARGIN)
                risk_vol = st['risk_vol'] + vol * float(geo.fill_risk_factor(risk_slack))
                n_prio, n_mis = st['n_prio'], st['n_mis']
                if item.get('is_prioritized', False):
                    n_prio += 1
                    if has_prio_container and not cont.get('is_prioritized', False):
                        n_mis += 1
                score = _objective(risk_vol, n_prio, n_mis)
                key = tuple(sorted(st['key'] + ((int(item['index']), round(pos[0], 4),
                                                  round(pos[1], 4), round(pos[2], 4)),)))
                cands.append((score, key, st, a, item, risk_vol, n_prio, n_mis))
        if not any_open or not cands:
            break
        cands.sort(key=lambda t: t[0], reverse=True)
        beam = []
        seen = set()
        for score, key, st, a, item, risk_vol, n_prio, n_mis in cands:
            if key in seen:
                continue
            seen.add(key)
            conts = clone_containers(st['containers'])
            cont = conts[a['container_idx']]
            cont['packed_items'].append(_place(cont, item, a))
            rem = dict(st['remaining'])
            del rem[item['index']]
            beam.append({'containers': conts, 'remaining': rem,
                         'order': st['order'] + [item['index']],
                         'risk_vol': risk_vol, 'n_prio': n_prio, 'n_mis': n_mis,
                         'score': score, 'key': key})
            if len(beam) >= max(1, beam_width):
                break
        if beam[0]['score'] > best['score'] or not best['order']:
            best = beam[0]

    # 「最も多く積めた」ではなく「risk調整済み体積が最大」の部分解を採用する
    # (build_order の目的関数と一致させる)。
    for st in beam:
        if st['score'] > best['score']:
            best = st
    order = list(best['order'])
    if best['remaining']:
        order.extend(best['remaining'].keys())
    return order


def greedy_construct_order(container_list: list[dict], item_list: list[dict], budget: planner.SearchBudget,
                            per_step_time_budget: float = 3.0, rng=None, score_noise: float = 0.0,
                            shuffle_ties: bool = False, window: int | None = None,
                            prepacked_ids: dict | None = None) -> list[int]:
    """
    「今置ける中で一番良い荷物・向き・位置」を毎回選び直す貪欲構築(オフライン限定, フル情報)。
    lookahead=1(パターンA)ではこの構築順そのものが online の実行結果と一致する
    (pool=1個の planner.plan は、pool内に他の候補が無いだけで同じ探索・同じ位置を返すため)。
    lookahead>1(パターンB)でも「常に最良の1手から詰める」近似順序として有効に働く。

    予算切れ、または合法手が尽きた場合は、残りの荷物インデックスを(必要なら shuffle して)
    そのまま末尾に付け足し、必ず全 item index を過不足なく含む順列を返す。

    budget: planner.SearchBudget。この1リスタートに使えるユニット予算(親=optimize全体)。
    Phase17: 旧実装は壁時計 deadline で打ち切っていたため、同じ入力でも実行環境の速度で
    「何手目まで構築できたか」が変わり、返す順序自体が run ごとに違っていた。
    """
    containers = clone_containers(container_list)
    remaining = {item['index']: dict(item) for item in item_list}
    order: list[int] = []

    if shuffle_ties and rng is not None:
        keys = list(remaining.keys())
        rng.shuffle(keys)
        remaining = {k: remaining[k] for k in keys}

    while remaining:
        # Phase17: 壁時計ではなく親の残ユニットで打ち切る。1リスタートが「どの手で止まるか」が
        # 実行環境の速度に依らず決まるため、同じ seed_items/window なら常に同じ順序を返す。
        if budget.exhausted():
            break
        pool = list(remaining.values())
        if window is not None:
            pool = pool[:window]
        action = planner.plan(containers, pool, max_pool_items=None,
                               rng=rng, score_noise=score_noise, prepacked_ids=prepacked_ids,
                               budget=budget.child_seconds(per_step_time_budget))
        if action is None:
            break
        item = pool[action['item_idx']]
        del remaining[item['index']]
        container = containers[action['container_idx']]
        container['packed_items'].append(_place(container, item, action))
        order.append(item['index'])

    if remaining:
        order.extend(remaining.keys())

    return order
