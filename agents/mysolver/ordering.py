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
import os
import time

import numpy as np

from . import geometry as geo
from . import planner
from . import simulate

# optimize() 全体の壁時計制限(180s、本番の実効上限は170s)に対する、実際に使う探索時間予算。
# Phase6: 15/30/60/120/165秒でスイープ計測した結果、非単調な挙動(30秒付近がピークで
# 120秒まで悪化)が見られたため、当時は安全側で30秒に固定していた。
# Phase16(ターゲット1): Phase8以降の目的関数変更・バグ修正を経た現行コードで
# 30/60/120/165秒を26シーン中optimize有効な21シーン×3回平均で再計測したところ、
# fill_strict は 22.93→23.31→24.08→24.35 と単調改善、placement_score も
# 99.32→99.87→100.00→99.91 とほぼ単調(ノイズ内)で、Phase6のような集計レベルの
# 非単調な悪化は再現しなかった(results/phase16_report.md §1参照)。fill_loose は
# 40.15→38.91→37.25→37.40 と120秒付近で下げ止まり、120→165の追加ゲインは
# fill_strict/fill_looseとも小さい(それぞれ+0.27/+0.15、いずれもこの測定のノイズ幅
# 0.3〜0.6pt程度に収まる)。一方、P02(pre-packed×offline optimize)は
# 26.89(120s)→22.04(165s)と単一シーンでは非単調に大きく振れており(build_orderの
# 単一グローバルRNGが壁時計依存の消費個数で分岐するため、詳細はphase16_report.md §2)、
# 165秒まで攻めるより120秒に留めるほうが安全と判断し、120秒を新しいデフォルトに採用する
# (170s上限に対して50秒の余裕を残す)。
DEFAULT_TIME_BUDGET = 120.0
# 1回の貪欲構築の1ステップ(planner.plan 1呼び出し)に許す上限。
PER_STEP_TIME_BUDGET = 3.0
# simulate_order による検証(online と同じ lookahead_k プールでの再現)に許す上限。
MAX_VALIDATE_SLICE = 12.0
# 最終的な締切ぎりぎりまで新しいリスタートを始めないための安全マージン。
# Phase17: 原理的には MAX_VALIDATE_SLICE 以上が望ましい。これより小さいと「最後のリスタートの
# validate だけが残予算で切り詰められる」ため、そのリスタートの評価値が総予算に依存し、
# 予算単調性が最後の1件だけ崩れる(検証(b)で残った非単調7件の原因の1つ)。
# ただし 12.0 に上げると CONSTRUCT_SLICE(20.0)と合わせて1リスタートの起動に32秒必要になり、
# **budget=30 ではリスタートが1回も回らなくなる**(実測: opt 3.73s、ヒューリスティック順のまま)。
# 本フェーズの計測はすべて 6.0 で実施しており、12.0 では掃引の30秒点の意味が変わってしまうため
# 6.0 のまま据え置く。次フェーズで CONSTRUCT_SLICE の引き下げとセットで再検討すること
# (旧6シーンでは 6.0/12.0 の結果は完全に一致し、この値による差は出なかった: §3.5)。
FINAL_MARGIN = 6.0
# 貪欲構築1回(=1リスタート)に配る予算[名目秒]。
#
# Phase17(ターゲット2): **総予算に依存しない固定値**であることが本質的に重要。
# ターゲット1(決定化)だけでは予算に対する単調性は回復しなかった(実測: シーン単位の
# スプレッド平均 5.33->6.76 と、むしろ拡大)。原因は、旧実装がこの配分を
# 「残り予算 ÷ 残り候補数」で決めており、しかも下限自体も time_budget に比例させていたため、
# **総予算が変わるとリスタート i に配られる予算も変わり、結果として順序そのものが変わる**
# ことにあった。つまり予算を増やすと「同じ系列をより多く辿る」のではなく
# 「まるごと別の系列を辿る」ため、改善するか悪化するかが事実上ランダムになる。
#
# 固定値にすると、リスタート i の結果は総予算に依存しなくなり、小さい予算の系列が
# 大きい予算の系列の**接頭辞**になる。build_order は best-of-N なので、N が増えれば
# 目的関数は単調に改善する(tools/phase17_restart_trace.py が接頭辞性を直接検査する)。
CONSTRUCT_SLICE = 20.0
# フェーズ2(ランダムリスタート)の1回あたり予算は従来どおりフェーズ1の2倍。
PHASE2_SLICE_FACTOR = 2.0
# 貪欲構築時にプールとして見せる「window(手前から何件か)」の候補。
# None は無制限(残り全件)。
WINDOW_CANDIDATES = [15, 20, 25, 30, None]
# Phase17: optimize() 全体の非常用の最終安全弁(壁時計、秒)。本番の optimization_timeout
# (180s、実効上限170s)を絶対に踏まないための保険であり、通常は発火しない。
#
# 探索の打ち切り自体は planner.SearchBudget の「消費ユニット」で決まる(=決定的)。
# 本環境より遅いマシンでは同じユニット数の消化に時間がかかるが、その場合でもこの安全弁が
# 発火するまでは最後まで決定的に探索しきる(=マシン速度に依らず同じ順序を返す)。
# 逆に本環境より速いマシンでは同じ探索を短時間で終える(安全側)。
HARD_WALL_LIMIT = 165.0
# time_budget を短く指定した開発時(--optimize-budget 30 等)にも比例した安全弁を掛ける。
# 較正誤差(実効速度が想定の 1/1.4 まで落ちる)まではユニット予算を使い切れる余裕になる。
HARD_WALL_FACTOR = 1.4


def _volume_of(item: dict) -> float:
    return item.get('volume', item['length'] * item['width'] * item['height'])


# Phase9: 単一の決定的順序(体積優先)だけでは局所解に落ちる(gen_sizevariety_patternCで
# 未配置44個中28個は順序さえ変えれば置けていた実測から確認)。そのため「詰めやすさ」の
# 仮説が異なる複数戦略で候補順序を作り、実際にシミュレートしたfillで比較して選ぶ
# (単一ヒューリスティックへの依存・そのヒューリスティックがハマる回帰を防ぐ)。
# 各戦略とも is_soft を最後段・is_prioritized を先頭寄りにする制約は共通(下敷き防止・優先度)で、
# 変えるのは「同条件内で何を先に置くか」の一次キーのみ。
def _strategy_volume_desc(item_list: list[dict]) -> list[dict]:
    """体積優先: 大きく重いものを土台として先に置く(従来のorder_items相当)。"""
    def key(item: dict):
        return (
            1 if item.get('is_soft', False) else 0,
            -_volume_of(item),
            -item.get('mass', 0.0),
            0 if item.get('is_prioritized', False) else 1,
        )
    return sorted(item_list, key=key)


def _strategy_count_first(item_list: list[dict]) -> list[dict]:
    """個数優先: 小さい荷物から先に確保し、置ける個数そのものを早期に積み増す。"""
    def key(item: dict):
        return (
            1 if item.get('is_soft', False) else 0,
            _volume_of(item),
            item.get('mass', 0.0),
            0 if item.get('is_prioritized', False) else 1,
        )
    return sorted(item_list, key=key)


def _strategy_big_first(item_list: list[dict]) -> list[dict]:
    """大物優先: 単一寸法が最も大きい(=後回しにするほど置き場所を選ぶ)荷物から先に確保する。"""
    def max_dim(item: dict) -> float:
        return max(item['length'], item['width'], item['height'])

    def key(item: dict):
        return (
            1 if item.get('is_soft', False) else 0,
            -max_dim(item),
            -_volume_of(item),
            0 if item.get('is_prioritized', False) else 1,
        )
    return sorted(item_list, key=key)


def _strategy_layer_first(item_list: list[dict]) -> list[dict]:
    """層優先: 低く・底面が広い荷物から並べ、各層内で下から上へ積みやすい面を先に作る。"""
    def footprint(item: dict) -> float:
        return item['length'] * item['width']

    def key(item: dict):
        return (
            1 if item.get('is_soft', False) else 0,
            item['height'],
            -footprint(item),
            0 if item.get('is_prioritized', False) else 1,
        )
    return sorted(item_list, key=key)


# フェーズ1で必ずWINDOW_CANDIDATES全通りを試す「基準」戦略(従来Phase6-8で調整されてきたのは
# これ単独のため、まずこれを従来通り手厚く探索し回帰しないようにする)。他戦略はフェーズ2の
# ランダムリスタートでのみ種として使い、フェーズ1の時間配分は一切変えない。
STRATEGIES = [_strategy_volume_desc, _strategy_count_first, _strategy_big_first, _strategy_layer_first]


def order_items(item_list: list[dict]) -> list[int]:
    """決定的ヒューリスティック順(探索の初期シード兼、最終フォールバック)。

    大きく重いものを土台として先に、ソフト・小物は後段(隙間埋め)に回す。
    planner.py が「非優先(非ソフト)荷物を優先(ソフト)荷物の上に乗せない」というハード制約を
    候補生成時に強制するため、順序に関わらず下敷きは発生しない。そのため純粋に
    「詰めやすさ」を優先してよい。
    """
    sorted_items = _strategy_volume_desc(item_list)
    return [item['index'] for item in sorted_items]


# Phase8: 目的関数を体積(risk調整済みfill)優先に戻す。
#
# Phase7で導入した「まず足切り相当の個数を確保してから体積を最適化する」二段構え
# ((min(配置数,目標), 体積, 配置数) の辞書式比較)は、本番提出でスコアを動かさなかった
# (足切り=個数不足仮説は棄却)。一方、提出実績とローカル平均fillは強く連動している
# (fill 20.05→24.01点、22.77→31.26点、24.40→37.31点。fillが横ばいの回はスコアも
# 横ばい)。回帰では score ≒ 3*fill - 36 となり、目標60点にはfill≈32が必要
# (Phase7時点で24.4)。そのためPhase8では体積(risk調整済み)を単独の主指標に戻す。
# ただし配置数を意図的に犠牲にする理由も無いため、体積が同点の場合のみ配置数が多い方を
# 選ぶ弱いタイブレークとして残す(体積を上げるための積極的な個数犠牲は選好しない)。
ASSUMED_CUTOFF_RATIO = 0.5      # 感度分析の比率仮説(30%/50%)のうち安全側の50%を採用
ASSUMED_CUTOFF_MIN_COUNT = 10   # 感度分析の絶対個数仮説(10/15/20)のうち最も緩い10を下限に採用


def cutoff_target(total_items: int) -> int:
    """このシーンで「足切りを確実に超えた」とみなす配置個数の仮の目標値。

    現在は目的関数には使わない(Phase8で体積優先に戻したため)。診断・報告用に残す。
    """
    if total_items <= 0:
        return 0
    target = max(ASSUMED_CUTOFF_MIN_COUNT, int(np.ceil(ASSUMED_CUTOFF_RATIO * total_items)))
    return min(target, total_items)


def _better(candidate: tuple[float, int], current: tuple[float, int]) -> bool:
    """(placement込みの調整体積, 配置数) の辞書式比較。体積を主指標、配置数は同点タイブレークのみ。"""
    return candidate > current


# Phase11(ターゲット1): 順序探索の目的関数に placement_score を組み込む重み。
#
# fill_score      = 100 * counted_volume / total_container_volume
# placement_score = 100 * (1 - 違反数/配置された優先手荷物数)
# なので「placement を1pt落とすことと fill を1pt落とすこと」を同価値とみなすなら、
# 違反率 r の損失は体積換算で r * total_container_volume に等しい(重み1.0)。
# 本番の指標重みは非公開だが、Phase10 の重み推定では placement は全提出で100のまま
# 識別できなかった一方、fill は public を支配していた。fill を毀損してまで placement を
# 取りにいかないよう、等価重み(1.0)ではなく保守的に 0.5 を採用する
# (=placement 1pt は fill 0.5pt 相当。D03 の 20pt 減点は fill 10pt 相当の価値)。
PLACEMENT_PENALTY_WEIGHT = 0.5

# Phase18: 順序探索の目的関数に stability の幾何代理(geo.stacking_instability_risk)を
# 組み込む重み。Phase17 で決定性を確保したことで gen_2containers_priority の探索が
# 「影シミュレータの目的関数(risk調整済み体積 − placementペナルティ)は改善するが実
# stability は悪化する」順序に常に収束することが判明した(results/phase17_report.md §3.5)。
# 目的関数に stability の代理が一切無かったことが原因。
#
# 初版は PLACEMENT_PENALTY_WEIGHT と同じ構造(平均リスク[0,1] * total_container_volume の
# 加算ペナルティ)を試したが、bulky系シーン(荷物が大きく、積み上げなしでは大半が入らない)で
# 「達成可能な体積よりペナルティのほうが大きい」ため何も置かない退化解に収束する重大な回帰を
# 引き起こした(実測: suite_A07_1c_40_bulky で fill_strict 28.07->0.00)。26シーンスイートでの
# 過適合チェック(指示3)で発見。simulate.simulate_order 側で「荷物ごとの寄与を超えては
# 割り引かない」設計に直してある(simulate.pyのdocstring参照)ため、この重みは
# 「stacking_instability_risk=1.0(閾値到達)のときその荷物の寄与を何割引くか」という
# [0,1]に近いスケールの意味に変わった(1.0=リスク最大でその荷物の寄与を0にする)。
#
# 再較正(bulky系の退化を修正した後): gen_2containers_priority + suite_A07_1c_40_bulky の
# 2シーンだけを見る掃引では W<=1.0 で無効果(gen_2containers_priorityは18.24/96.87のまま)、
# W=2.0 で修正(20.54/98.23)、W=3.0〜5.0 はプラトー(A07はむしろ改善: 27.18->30.35)に見えたため、
# 一度は 2.5 を採用候補にした。
#
# **しかし指示3の26シーンスイート検証(過適合チェック)で重大な回帰が見つかり、不採用にした**
# (results/phase18_report.md §3参照)。W=2.5 で fill_strict がスイート平均 24.37→22.93(−1.44、
# ノイズ床±0.26を大きく超える)に悪化し、suite_D02_A_1c_40_prioheavy_nocont は
# fill_strict 32.42→14.06(−18.36)、suite_C02_2c_55_shelfprio は placement_score が
# 100→85.71 に落ちる(制約違反)。原因を切り分けるため W を 1.1〜2.5 の範囲で細かく
# 掃引したところ、次の「崖の不一致」が判明した:
#   - gen_2containers_priority は W=1.1(未修正: 18.24)→W=1.2(修正: 20.54)の間に不連続に切り替わる。
#   - suite_D02 は W=1.1 の時点で既に破綻しており(14.06)、W=1.2〜2.5 でも変化しない
#     (=gen_2containers_priorityが直る前に既に壊れている。両者を同時に満たす区間が存在しない)。
#   - suite_C02 は W=1.2/1.4 で 24.70(無傷)なのに W=1.3 だけ 27.14 に跳ねる(非単調)。
#     目的関数への滑らかな重みづけのつもりが、貪欲構築+リスタートという離散探索に対しては
#     「別の局所解へ飛び移るスイッチ」として作用するため、重みに対して滑らかに応答しない。
# この2点(a: シーン間で質量比の「安全な尺度」が大きく異なり閾値を共有できない、
# b: 離散探索の出力は重みに対して非連続)から、**単一の大域重みでは較正不能**と判断した。
# Phase14の教訓(重みには崖がある)がここでも再現しており、しかも今回はスイープで崖を
# 避ける「安全な水準」自体が存在しないケースだった。
#
# 結論: 幾何代理(geo.stacking_instability_risk)自体・discount機構(荷物ごとにのみ働き
# 加算ペナルティにしない設計)は次フェーズ(trust region + ρ-test)でも土台として使える見込みが
# あるため実装は残すが、**デフォルトの重みは 0.0(=無効・Phase17と完全に同じ挙動)に戻す**。
# 有効化したいときは環境変数 MYSOLVER_STABILITY_W で明示的に上書きすること。
STABILITY_PENALTY_WEIGHT = float(os.environ.get('MYSOLVER_STABILITY_W', '0.0'))

# Phase23: 貪欲構築をビームサーチへ一般化したときの幅。
#
# Phase20/22 の測定が同じ結論を指していた: 現在の探索は「見た候補の中ではほぼ最良を選べて
# いる」(native regret 1.14pt)が「良い候補を生成できていない」(候補順序による実fill
# スプレッド 19.19pt、行き詰まりからの取りこぼし (c) の7割が荷物選択の自由度由来)。
# そこで狙うのは候補の選び方ではなく**候補の作り方**であり、1本の貪欲構築を幅bのビームへ
# 広げる。b=1 は greedy_construct_order と完全に同一の手順に退化する(検証済み)。
#
# 幅bとリスタート回数はトレードオフになる: 1構築あたりのコストがb倍になるので、
# 同じ総予算で回せるリスタート数は 1/b になる。Phase22 §3.3 で「1手あたりコストを増やすと
# 予算内で回れる組合せが減って逆効果」という失敗を実証済みなので、値は掃引で決める。
# Phase23 の掃引結果(8シーン -> 26シーンで確定):
#   b=1 23.74 / b=2 D03のplacementが92.86で制約違反 / b=3 **26シーンで fill_strict +1.95** /
#   b=5 C02のplacementが85.71で制約違反
# 制約(placement/soft 全シーン満点)を満たす b>1 は b=3 のみで、26シーンで
# fill_strict 23.74->25.68 / fill_loose 36.65->39.13、採点モデル換算 public +0.87pt。
# b=1 は greedy_construct_order と完全に同一(18/18 の順序一致で検証済み)。
BEAM_WIDTH = int(os.environ.get('MYSOLVER_BEAM_WIDTH', '3'))


def build_order(item_list: list[dict], container_list: list[dict] | None, lookahead_k: int | None,
                 time_budget: float = DEFAULT_TIME_BUDGET) -> list[int]:
    start = time.perf_counter()
    # Phase17: 探索の総量は「秒」ではなく決定的なユニット数で持つ。壁時計は
    # 非常用の最終安全弁(hard_deadline)としてのみ使い、通常は発火しない。
    hard_deadline = start + min(HARD_WALL_LIMIT, time_budget * HARD_WALL_FACTOR)
    total_budget = planner.SearchBudget.from_seconds(time_budget, hard_deadline=hard_deadline)

    # Phase17(ターゲット2): 1リスタートあたりの配分・検証枠・最終マージンはすべて
    # **総予算に依存しない固定値**にする(理由は CONSTRUCT_SLICE のコメント参照)。
    # 旧実装はこれらを time_budget に比例させており、それ自体が予算非単調性の原因だった。
    # (以下はすべて「名目秒」。planner.UNITS_PER_SEC で決定的にユニットへ換算する。)
    #
    # 例外は「1リスタート分すら入らないほど短い開発用予算」の場合だけ。ここだけは
    # 何も構築せずヒューリスティック順を返してしまうのを避けるため予算に比例させる
    # (本フェーズの掃引水準 30/60/120/165 はすべて固定値側に入るので、単調性の
    #  検証には影響しない)。
    construct_slice = CONSTRUCT_SLICE if time_budget >= CONSTRUCT_SLICE else max(1.0, time_budget * 0.5)
    final_margin = FINAL_MARGIN if time_budget >= CONSTRUCT_SLICE else max(0.2, time_budget * 0.05)
    max_validate_slice = MAX_VALIDATE_SLICE
    u = planner.UNITS_PER_SEC
    # Phase23: ビーム幅bのとき1構築あたりの探索量はb倍になるため、1リスタートの枠もb倍にする
    # (=各ビーム状態が従来の貪欲1本と同じ計算量を受け取る)。その分リスタート回数は 1/b に減り、
    # 「幅を取るか、試行回数を取るか」というトレードオフが予算配分として素直に表現される。
    construct_units = construct_slice * u * max(1, BEAM_WIDTH)
    phase2_units = construct_units * PHASE2_SLICE_FACTOR
    final_margin_units = final_margin * u

    heuristic_order = order_items(item_list)

    if not container_list or not item_list:
        return heuristic_order
    k = max(1, int(lookahead_k or 1))

    items_by_index = {item['index']: item for item in item_list}
    best_order = heuristic_order
    # Phase11: placement ペナルティで目的関数が負になりうるため、初期値は -inf にする
    # (旧 -1.0 のままだと、全候補が負スコアのシーンで貪欲構築の結果が一切採用されない)。
    best_score = None

    total_container_volume = sum(c.get('volume', 0.0) for c in container_list)
    # Phase15(ターゲット1): container_list はこの時点でまだ初期状態(get_init_states直後、
    # 何も配置していない)なので、ここから「エピソード開始時に既に積まれていた荷物」の
    # identityを求めておける(simulate.pyのclone_containersは深く複製するが、既積み荷物の
    # indexはそのまま引き継がれるので、以降の全シミュレーション呼び出しに対して有効)。
    prepacked_ids = geo.initial_prepacked_ids(container_list)

    def validate(order: list[int]) -> tuple[float, int] | None:
        """戻り値 None は「予算切れで評価できなかった」の意(比較対象にしない)。"""
        if total_budget.exhausted():
            return None
        # Phase18: stability の幾何代理は risk_adjusted_volume 自体(荷物ごとの割引)に
        # 織り込み済み(simulate.simulate_order の stability_weight 引数)。ここで返る
        # stability_risk_ratio は診断用(coverageの平均リスク)であり目的関数には使わない。
        placed_ids, placed_volume, risk_adjusted_volume, violation_ratio, stability_risk_ratio = \
            simulate.simulate_order(
                container_list, items_by_index, order, k,
                total_budget.child_seconds(max_validate_slice), prepacked_ids=prepacked_ids,
                stability_weight=STABILITY_PENALTY_WEIGHT)
        count = len(placed_ids)
        penalty = PLACEMENT_PENALTY_WEIGHT * total_container_volume * violation_ratio
        return (risk_adjusted_volume - penalty, count)

    try:
        score = validate(heuristic_order)
        if score is not None and (best_score is None or _better(score, best_score)):
            best_order, best_score = heuristic_order, score
    except Exception:
        pass

    # Phase9: 体積優先1本の決定的順序だけに頼ると局所解に落ちて回帰しうる(実測: 未配置44個中
    # 28個は順序を変えれば置けていた)。仮説の異なる複数戦略それぞれの並び順を、貪欲構築の
    # 「種」として後段のtry_constructに渡す(構築を挟まない素の順序をここで直接validateすると、
    # max_validate_slice分の時間を戦略数だけ倍取りしてしまい、本来の主眼である貪欲構築+
    # window探索フェーズの時間を圧迫してしまうため、素の順序自体はここでは評価しない)。
    strategy_orders = [(fn.__name__, fn(item_list)) for fn in STRATEGIES]

    rng = np.random.default_rng(0)
    all_indices = set(items_by_index.keys())

    def try_construct(seed_items, window, use_noise, slice_units):
        nonlocal best_order, best_score
        if slice_units <= 0:
            return
        try:
            order = simulate.beam_construct_order(
                container_list, seed_items, total_budget.child(slice_units),
                per_step_time_budget=PER_STEP_TIME_BUDGET,
                rng=rng if use_noise else None,
                score_noise=0.35 if use_noise else 0.0,
                shuffle_ties=use_noise,
                window=window,
                prepacked_ids=prepacked_ids,
                beam_width=BEAM_WIDTH,
            )
            if set(order) == all_indices:
                score = validate(order)
                if score is not None and (best_score is None or _better(score, best_score)):
                    best_order, best_score = order, score
        except Exception:
            pass

    # フェーズ1: window幅を変えた決定的(ノイズ無し)貪欲構築を一通り試す(体積優先のみ)。
    # これはPhase6-8で個別に調整されてきた基準戦略・基準配分そのものであり、他戦略を混ぜて
    # 時間配分を変えると(戦略数倍に時間を奪われ、結果としてこのフェーズ自体が薄まる)回帰する
    # ことをPhase9の実験で確認したため、ここは従来通り体積優先1本のみで手厚く行う。
    # Phase17(ターゲット2): 各リスタートには**固定量**(construct_units)を配る。
    # 旧実装の「残り予算 ÷ 残り候補数」は総予算に依存するため、予算を変えると
    # リスタート i の結果そのものが変わってしまい、単調性が原理的に成立しなかった。
    #
    # あわせて「満額の枠が入らないなら新しいリスタートを始めない」ことで、
    # 尻切れリスタート(=総予算によって内容が変わる唯一の残りケース)も排除する。
    # これにより小さい予算の系列は大きい予算の系列の**接頭辞**になり、
    # build_order は「決定的な系列の先頭 N 個の argmax」= N に対して単調になる。
    default_items = strategy_orders[0][1]
    pending_windows = list(WINDOW_CANDIDATES)
    while pending_windows:
        if total_budget.exhausted():   # 非常用安全弁が発火した場合のみ真になりうる
            break
        if total_budget.remaining() < construct_units + final_margin_units:
            break
        window = pending_windows.pop(0)
        try_construct(default_items, window, use_noise=False, slice_units=construct_units)

    # フェーズ2: 残り予算でランダム化(shuffle+noise)リスタートを繰り返し、
    # window と戦略の両方をランダムに振って多様性を確保する(単一戦略への依存を避ける)。
    while True:
        if total_budget.exhausted():   # 非常用安全弁が発火した場合のみ真になりうる
            break
        if total_budget.remaining() < phase2_units + final_margin_units:
            break
        window = WINDOW_CANDIDATES[int(rng.integers(0, len(WINDOW_CANDIDATES)))]
        _, seed_items = strategy_orders[int(rng.integers(0, len(strategy_orders)))]
        try_construct(seed_items, window, use_noise=True, slice_units=phase2_units)

    return best_order
