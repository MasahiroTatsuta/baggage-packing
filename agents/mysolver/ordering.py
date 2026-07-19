"""
optimize() 用の積付順序決定。

パターンA(look_ahead=1)では online 側に「どの荷物を選ぶか」の自由度が無く、置いた結果
(置けた個数・充填体積)は事実上この順序だけで決まる。そのため単純なソートではなく、
simulate.py の自前シミュレータ(pybullet不使用・planner.pyと同一の幾何/合法性判定)で
実際に「その順序で詰めたら何個置けるか」を検証しながら、より良い順序を探索する。

戦略:
  1. まず決定的ヒューリスティック順(体積・質量ベースのソート)を1つ用意する(常に安全な
     フォールバックであり、探索が万一何も改善できなくてもこれを返せる)。
  2. planner.plan を「その時点の未配置荷物」をプールとして呼ぶ貪欲構築
     (simulate.greedy_construct_order)を、プール幅(window)を変えながら複数回行う。
     lookahead=1では、この構築順が online 実行の結果とほぼ一致する(pool内に他候補が
     無いだけで同じ探索・同じ位置を返すため)。
     実験的には「毎回“残り全件”から最良の1手を選ぶ(window無制限)」よりも、
     「元のストリーム順で見えている手前の数十件だけから選ぶ」方が良い順序になりやすい
     (無制限だと単発最適を食い潰し、後段の荷物の置き場所を却って狭めてしまう一種の
     視野依存トンネルビジョンが起きるため)。そのため window 幅も探索対象にする。
  3. 時間が残っていれば、tie-break をランダム化した貪欲構築を繰り返しリスタートし、
     simulate_order (lookahead_k 込みで online と同じ条件) で評価した「risk調整済み配置体積」
     (壁ぎわで沈降ドリフトによりfill集計から漏れやすい配置を割り引いた体積)が
     一番良いものを採用する。
  4. 180s のタイムアウト(→デフォルト順)は絶対に踏まないよう、time_budget に対して
     十分なマージンを取って必ず打ち切る。
"""
import time

import numpy as np

from . import simulate

# optimize() 全体の壁時計制限(180s)に対する、実際に使う探索時間予算。
# Phase6: 15/30/60/120/165秒でスイープ計測した結果、影シミュレータ上のrisk調整済み体積を
# 目的関数にしても(→ordering._better参照)fillは予算に対して単調には改善せず、
# 30秒付近をピークに120秒まで悪化し165秒でわずかに持ち直す、という非単調な挙動が残った
# (sample_config::000, gen_shelf_patternAで確認。探索を伸ばすほど、影シミュレータ上は
# 良く見えるが実物理の沈降・回転までは再現できない配置を選びやすくなるsim-to-realギャップが
# 完全には消えないため)。指示書の方針(単調にできない場合は安全側で固定予算を採用してよい)に
# 従い、170s上限に対して余裕を持たせつつ実測ピークの30秒を採用する
# (180sタイムアウトへの安全マージンは別途 optimize_time_budget 呼び出し側で確保する)。
DEFAULT_TIME_BUDGET = 30.0
# 1回の貪欲構築の1ステップ(planner.plan 1呼び出し)に許す上限。
PER_STEP_TIME_BUDGET = 3.0
# simulate_order による検証(online と同じ lookahead_k プールでの再現)に許す上限。
MAX_VALIDATE_SLICE = 12.0
# 最終的な締切ぎりぎりまで新しいリスタートを始めないための安全マージン。
FINAL_MARGIN = 6.0
# 貪欲構築の1回あたりの最低保証時間。実行環境の負荷変動(CPU競合等)で1ステップが
# 想定より遅くなっても、window探索の各候補が「時間切れによる尻切れ」で不当に低評価
# されないための下限(=最悪ケースでも構築が自然完了(合法手が尽きる)まで待てるようにする)。
MIN_CONSTRUCT_SLICE = 20.0
# 貪欲構築時にプールとして見せる「window(手前から何件か)」の候補。
# None は無制限(残り全件)。
WINDOW_CANDIDATES = [15, 20, 25, 30, None]


def order_items(item_list: list[dict]) -> list[int]:
    """決定的ヒューリスティック順(探索の初期シード兼、最終フォールバック)。

    大きく重いものを土台として先に、ソフト・小物は後段(隙間埋め)に回す。
    planner.py が「非優先(非ソフト)荷物を優先(ソフト)荷物の上に乗せない」というハード制約を
    候補生成時に強制するため、順序に関わらず下敷きは発生しない。そのため純粋に
    「詰めやすさ」を優先してよい。
    """
    def volume_of(item: dict) -> float:
        return item.get('volume', item['length'] * item['width'] * item['height'])

    def sort_key(item: dict):
        return (
            1 if item.get('is_soft', False) else 0,
            -volume_of(item),
            -item.get('mass', 0.0),
            0 if item.get('is_prioritized', False) else 1,
        )

    sorted_items = sorted(item_list, key=sort_key)
    return [item['index'] for item in sorted_items]


# Phase7: 「足切り突破シーン数の最大化」を最優先目標にするための二段構え目的関数。
#
# README「評価指標」節: 手荷物を一定数以上コンテナに積載できていないと fill_score 以外の
# 4指標が0になる(足切り)。本番の閾値は非公開だが、tools/scorer.py の cutoff_sensitivity
# (感度分析: 絶対個数10/15/20、総荷物数比30%/50%、体積比30%/50%)のうち、いずれの仮説でも
# 「配置個数が少なすぎるシーンは4指標を丸ごと失う」という結論は変わらない。
# Phase6までは risk調整済み体積(fill寄り)を主指標・配置数をタイブレークにしていたが、
# これは「fillを僅かに上げるために配置個数を犠牲にする」順序を積極的に選好してしまい、
# 足切りぎりぎり/未満のシーンをさらに悪化させる向きに働く。
# Phase7では「まず足切り相当の個数を確保し、その後で体積(fillへの寄与)を最適化する」
# 二段構えに変更する: スコアを (min(配置数, ASSUMED_CUTOFF_TARGET), risk調整済み体積, 配置数)
# の辞書式(タプル)比較にする。
#   - 配置数が ASSUMED_CUTOFF_TARGET 未満のシーンでは、体積差より配置数の増加を常に優先する
#     (=足切り未達を最優先で救う)。
#   - ASSUMED_CUTOFF_TARGET 以上を確保できたシーンでは、それ以上個数を積み増すより
#     体積(fillへの寄与)を優先する(=無闇に個数だけ稼いで質を落とさない)。
#   - 体積も同点なら配置数が多い方を採用する(最終タイブレーク)。
# ASSUMED_CUTOFF_TARGET 自体は非公開閾値の代用の"安全側"な仮定であり、閾値の実値が
# これより低くても高くても、「配置数が少ないほど優先的に救う」という単調な挙動は変わらず
# 安全に働く(閾値の正確な値を言い当てる必要はない)。
ASSUMED_CUTOFF_RATIO = 0.5      # 感度分析の比率仮説(30%/50%)のうち安全側の50%を採用
ASSUMED_CUTOFF_MIN_COUNT = 10   # 感度分析の絶対個数仮説(10/15/20)のうち最も緩い10を下限に採用


def cutoff_target(total_items: int) -> int:
    """このシーンで「足切りを確実に超えた」とみなす配置個数の仮の目標値。

    総荷物数が少ないシーン(数個〜十数個)では ASSUMED_CUTOFF_RATIO*total が
    ASSUMED_CUTOFF_MIN_COUNT を下回りうるが、絶対個数の仮説(10個以上)も無視できないため
    max を取る。ただし目標が総荷物数を超えることはない(min で頭打ち)。
    """
    if total_items <= 0:
        return 0
    target = max(ASSUMED_CUTOFF_MIN_COUNT, int(np.ceil(ASSUMED_CUTOFF_RATIO * total_items)))
    return min(target, total_items)


def _better(candidate: tuple[int, float, int], current: tuple[int, float, int]) -> bool:
    """(min(配置数,目標), risk調整済み体積, 配置数) の辞書式比較。"""
    return candidate > current


def build_order(item_list: list[dict], container_list: list[dict] | None, lookahead_k: int | None,
                 time_budget: float = DEFAULT_TIME_BUDGET) -> list[int]:
    start = time.perf_counter()
    deadline = start + time_budget

    # time_budget を短く指定した開発時(local_evalの--optimize-budget等)でも、
    # 定数を固定のままだと「残り時間 < MIN_CONSTRUCT_SLICE」で即打ち切りになり、
    # 貪欲構築が一度も走らずヒューリスティック順のままになってしまう。
    # 本番相当(165s+)では従来の定数と一致するよう min() で頭打ちにしつつ、
    # 短い time_budget では予算に比例させて必ず何回かは構築が回るようにする。
    min_construct_slice = min(MIN_CONSTRUCT_SLICE, max(1.0, time_budget * 0.15))
    final_margin = min(FINAL_MARGIN, max(0.2, time_budget * 0.05))
    max_validate_slice = min(MAX_VALIDATE_SLICE, max(0.5, time_budget * 0.3))

    heuristic_order = order_items(item_list)

    if not container_list or not item_list:
        return heuristic_order
    k = max(1, int(lookahead_k or 1))

    items_by_index = {item['index']: item for item in item_list}
    best_order = heuristic_order
    best_score = (-1, -1.0, -1)
    target = cutoff_target(len(item_list))

    def validate(order: list[int]) -> tuple[int, float, int]:
        now = time.perf_counter()
        if now > deadline:
            return (-1, -1.0, -1)
        vdeadline = min(deadline, now + max_validate_slice)
        placed_ids, placed_volume, risk_adjusted_volume = simulate.simulate_order(
            container_list, items_by_index, order, k, vdeadline)
        count = len(placed_ids)
        return (min(count, target), risk_adjusted_volume, count)

    try:
        score = validate(heuristic_order)
        if _better(score, best_score):
            best_order, best_score = heuristic_order, score
    except Exception:
        pass

    rng = np.random.default_rng(0)
    all_indices = set(items_by_index.keys())

    def try_construct(window, use_noise, slice_budget):
        nonlocal best_order, best_score
        if slice_budget <= 0:
            return
        slice_deadline = time.perf_counter() + slice_budget
        try:
            order = simulate.greedy_construct_order(
                container_list, item_list, slice_deadline,
                per_step_time_budget=PER_STEP_TIME_BUDGET,
                rng=rng if use_noise else None,
                score_noise=0.35 if use_noise else 0.0,
                shuffle_ties=use_noise,
                window=window,
            )
            if set(order) == all_indices:
                score = validate(order)
                if _better(score, best_score):
                    best_order, best_score = order, score
        except Exception:
            pass

    # フェーズ1: window幅を変えた決定的(ノイズ無し)貪欲構築を一通り試す。
    # 各候補には「残り時間 ÷ 残り候補数」を均等配分する(先の候補が早く自然終了すれば、
    # 後の候補により多くの時間が回る)。CPU競合等で1ステップが遅くなっても、構築が
    # 「合法手が尽きる」まで自然完了できるだけの最低時間(MIN_CONSTRUCT_SLICE)は必ず確保し、
    # 尻切れによる不当な低評価(=品質比較にならない)を避ける。
    pending_windows = list(WINDOW_CANDIDATES)
    while pending_windows:
        now = time.perf_counter()
        remaining = deadline - final_margin - now
        if remaining < min_construct_slice:
            break
        window = pending_windows.pop(0)
        slice_budget = max(min_construct_slice, remaining / (len(pending_windows) + 1))
        slice_budget = min(slice_budget, remaining)
        try_construct(window, use_noise=False, slice_budget=slice_budget)

    # フェーズ2: 残り時間でランダム化(shuffle+noise)リスタートを繰り返し、
    # window もランダムに振って多様性を確保する。
    while True:
        now = time.perf_counter()
        remaining = deadline - final_margin - now
        if remaining < min_construct_slice:
            break
        window = WINDOW_CANDIDATES[int(rng.integers(0, len(WINDOW_CANDIDATES)))]
        try_construct(window, use_noise=True, slice_budget=min(remaining, min_construct_slice * 2))

    return best_order
