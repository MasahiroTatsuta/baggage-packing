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
import pybullet as p

from . import geometry as geo
from . import planner
from src.ground_handling.utils import ORNS

_ORN_QUATS = [p.getQuaternionFromEuler(e) for e in ORNS]


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


def simulate_order(container_list: list[dict], items_by_index: dict[int, dict], order: list[int],
                    lookahead_k: int, budget: planner.SearchBudget, per_step_time_budget: float = 0.7,
                    prepacked_ids: dict | None = None,
                    stability_weight: float = 1.0) -> tuple[list[int], float, float]:
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
    """
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
    idx_iter = iter(order)
    pool: list[dict] = []

    def refill():
        while len(pool) < lookahead_k:
            try:
                pool.append(dict(items_by_index[next(idx_iter)]))
            except StopIteration:
                return

    refill()
    placed_ids: list[int] = []
    placed_volume = 0.0
    risk_adjusted_volume = 0.0
    n_stacked = 0
    stacking_risk_sum = 0.0

    while pool:
        # Phase17: 壁時計ではなく親の残ユニットで打ち切る(同一入力なら同じ手数・同じ結果)。
        if budget.exhausted():
            break
        info: dict = {}
        action = planner.plan(containers, pool, info=info, prepacked_ids=prepacked_ids,
                               budget=budget.child_seconds(per_step_time_budget))
        if action is None:
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
        risk_adjusted_volume += (item_volume * geo.fill_risk_factor(risk_slack)
                                  * stability_discount)
        refill()

    violation_ratio = n_prio_misrouted / n_prio_placed if n_prio_placed else 0.0
    stability_risk_ratio = stacking_risk_sum / n_stacked if n_stacked else 0.0
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

    base = clone_containers(container_list)
    remaining = {item['index']: dict(item) for item in item_list}
    if shuffle_ties and rng is not None:
        keys = list(remaining.keys())
        rng.shuffle(keys)
        remaining = {k: remaining[k] for k in keys}

    # state: containers / remaining(dict) / order(list) / score(float) / key(部分解の同一性)
    beam = [{'containers': base, 'remaining': remaining, 'order': [], 'score': 0.0,
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
                score = st['score'] + vol * float(geo.fill_risk_factor(risk_slack))
                key = tuple(sorted(st['key'] + ((int(item['index']), round(pos[0], 4),
                                                  round(pos[1], 4), round(pos[2], 4)),)))
                cands.append((score, key, st, a, item))
        if not any_open or not cands:
            break
        cands.sort(key=lambda t: t[0], reverse=True)
        beam = []
        seen = set()
        for score, key, st, a, item in cands:
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
