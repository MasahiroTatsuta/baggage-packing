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

from . import alns
from . import geometry as geo
from . import planner
from . import reach
from . import replica
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

# ---------------------------------------------------------------------------
# Phase28: 自己封鎖した順序を割り引く重み(到達可能性の項)
# ---------------------------------------------------------------------------
# Phase24 が特定した corridor_penalty の構造的限界(時間方向の myopia: 既に置かれた障害物の
# 上の通路しか守れないが、封鎖の主因である中間高さ帯・上部の通路はその後の積み上げで初めて
# 生まれる)を、**順序探索の側**で外すための項。影シミュレータは順序を最後まで流し切るので、
# 行き詰まり時点で「支持はあるが搬入経路が死んでいる空間」= Phase24 の (a) を事後に観測できる。
# 判定は agents/mysolver/reach.py(tools/phase24_corridor_audit.py からの忠実な移植)。
#
# **既定 0.0(無効)**。0 のときは reach_info を作らないので計算経路にすら入らず、
# Phase27 までの出力とビット単位で同一(決定的5シーンで検証済み)。
#
# 適用は加算ペナルティではなく **乗算の割引**((1 - W*blocked_ratio) を risk調整済み体積に
# 掛ける)。Phase18 で加算ペナルティが「達成可能な体積より罰則が大きく、何も置かないほうが
# 得」という退化解を生んだ(suite_A07 で fill 28.07->0.00)ため、その構造を繰り返さない。
#
# ---------------------------------------------------------------------------
# 【結果: 不採用】W=0.25 の26シーン測定で fill_strict 24.413 -> 24.763(+0.350)、
# σ=1.784 / SE=0.350 / **t=+1.000**。動いたのは **26シーン中1シーン(A03 +9.10)** だけで、
# 残り25シーンはビット単位で不変だった(results/phase28_report.md)。
#
#  ・**t=1.000 は構造的な上限**: 25シーンが0・1シーンだけが d のとき mean=d/26、SE=d/26 と
#    なり t は d に依らず 1.000 に固定される。単一シーン効果はどれだけ大きくても t>2 を
#    原理的に通過できない(Phase25a の採否基準が意図どおり働いた例)。
#  ・**重み区間が存在しない**(本フェーズの主成果、tools/phase28_rerank.py で定量化):
#    「採用される順序が変わる最小の W」はシーンごとに 0.23 / 0.39 / 1.04 / 2.29 と10倍
#    散らばる。一方 A03 は W=0.5〜1.0 に崖を持ち W=1.0 で 11.55(ベースライン17.78以下)。
#    **A03 が壊れる W < A02 が反応し始める W(1.04)** なので、広く効かせる W と
#    どのシーンも壊さない W の共通区間が無い。Phase18 と同型の結論が別の項で再現した。
#  ・**構造的な原因**: 再ランクが起きると必ず配置個数の少ない順序が選ばれる
#    (A02 21->4, A03 26->10, A04 28->19)。荷物を置くたびに通路を塞ぐ可能性が生まれるので、
#    「通路を開けておく」と「たくさん詰める」は原理的に競合する。分母を empty ではなく
#    supported にしたことで最悪の退化(fill 0 崩壊)は防げたが、弱い偏りは消せなかった。
#  ・コストは実測 +9.9%(optimize有効21シーン平均 +7.75s)かかっており、効果が無い以上は純損失。
#
# Phase24 の具体案2「ビームサーチのスコアに同じ項を入れる」は、閾値のばらつきが**項の性質**
# であって入れる場所の問題ではないため、同じ重み区間問題に必ずぶつかる。推奨しない。
# ---------------------------------------------------------------------------
REACH_WEIGHT = float(os.environ.get('MYSOLVER_REACH_WEIGHT', '0.0'))

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
# Phase25a: 既定を Phase22 相当(b=1)へ戻した。理由は Phase25a の採否基準の改訂:
# 26シーン平均の改善だけでなく、シーン別効果の標準偏差σ・SE=σ/√26・t値で判定すべきところ、
# Phase23の b=3 は +1.95±1.20(t=1.63、t>2に届かず)で本来は不採用相当の分散だった。
# 実際の提出フィードバックも public -0.45 で悪化しており、σが大きい変更は26シーン平均が
# 改善して見えても隠しテスト(別シーン集合)には一般化しないことが裏付けられた
# (results/phase25a_report.md)。b=3以上を再検討する場合は必ずシーン数を増やしてSEを
# 下げてから判定すること。
BEAM_WIDTH = int(os.environ.get('MYSOLVER_BEAM_WIDTH', '1'))

# ---------------------------------------------------------------------------
# Phase29: 衝突駆動リスタート(conflict-driven ordering repair)
# ---------------------------------------------------------------------------
# Phase24(候補側で構築中に効かせる)と Phase28(順序側で事後に効かせる)は、いずれも
# **到達可能性を目的関数の項として足す**路線であり、どちらも「シーン横断で使える重みの区間が
# 存在しない」という同じ壁で失敗した(Phase18・Phase26 と合わせて3敗)。
#
# ここは目的関数を一切変えない。**重みを導入しない**ので崖も尺度問題も起きない。
# 変えるのは「次に試す順序をどう作るか」だけである:
#
#   1. ロールアウトが行き詰まったら、その瞬間の状態(simulate.simulate_order の stall_info)を取る
#   2. 置けなかった荷物 X について、掃引経路を塞いでいる既配置荷物 Y1..Yn を同定する
#      (reach.item_blockers。判定式は Phase24 の監査ツールと同一で、障害物に identity を
#       持たせただけ。棚が塞いでいる位置は「順序では動かせない」として除外する)
#   3. **X を Y1..Yn すべてより前へ動かした**順序を作って評価しなおす
#      (1手スワップではなく多手移動。Phase24 が見積もった単発スワップの上限
#        (経路上の障害物1個のケース=(a)の23.6%)を超えて、障害物3個以上の55%にも届きうる)
#   4. 改善しなければその修正は捨て、次の衝突へ移る
#
# 予算について: リスタート回数は**増やさない**。フェーズ1/フェーズ2のループは
# 「満額の枠が入らないなら新しいリスタートを始めない」ため、必ず端数の予算が余って捨てられて
# いる(実測 §ステップ1)。修正の試行はこの **余り** の中だけで行うので、
# 総予算も、既存のリスタートの内容も、その決定的な系列も一切変わらない
# (=Phase17 で確立した「小さい予算の系列は大きい予算の系列の接頭辞」も保たれる)。
#
# **既定は無効(0)**。無効時は stall_info を渡さないので simulate 側の計算経路にも入らず、
# Phase28 までの出力とビット単位で同一(決定的5シーン+A01-A03 の 8/8 で検証済み)。
#
# ---------------------------------------------------------------------------
# 【結果: 不採用】到達したのは **26シーン中2シーン**(C03, P05)だけで、採点指標が動いたのは
# **1シーン**(C03: fill_strict 32.208 -> 34.904)。26シーン平均 +0.104 / σ=0.529 / SE=0.104 /
# **t=+1.000** で、2×SE(0.207)を超えない(results/phase29_report.md)。
#
#  ・**主因は Phase24 の (a) の読み違い**(本フェーズ最大の成果): (a)=搬入経路が封鎖された
#    空き 29.993 m³ は「残荷物の **どれか** が入る空間」の合計である。しかし評価は sudden death
#    なので、エピソードを止めるのは常に **次の1個(X)** だけ。実測では **21シーン中13シーンで
#    X はそもそもどこにも入らない**(支持付きで収まる位置がゼロ。voxel を 0.10->0.05->0.025 と
#    細かくしてもゼロのまま)。つまり (a) の大半は「X 以外の荷物のための空間」で、
#    X の順序を直しても取りに行けない。**Phase24 の 13.14pt は順序修正の射程内に無い。**
#  ・衝突が同定できた6シーンでも、4シーンではどの修正手も悪化した(-0.50〜-12.51pt)。
#    X を前に出すと、開く通路より壊れる層のほうが大きい。目的関数はこれを正しく全て棄却した。
#  ・**機構自体は動いている**: C03 で実測 +2.696pt、P05 の delay_blockers で実測 +4.799pt。
#    効かない理由は機構ではなく母数なので、ブロッカー同定(reach.item_blockers)は残す。
#  ・副産物: P05 では目的関数が採用した手が実機 ±0.000 で、棄却した手が +4.799 だった。
#    影シミュレータは P05 の base を placed 19個と予測するが実機は28個。Phase20 は
#    「代理誤差はあるが順位=意思決定は変わらない」と結論していたが、**1手差の粒度では
#    順位が変わる**。次に代理精度を触るならこれが具体的な足がかりになる。
# ---------------------------------------------------------------------------
REPAIR = os.environ.get('MYSOLVER_REPAIR', '0') == '1'
# 1回の build_order で試す修正の上限(予算が先に尽きるのが普通なので、暴走止めの意味合い)。
REPAIR_MAX = int(os.environ.get('MYSOLVER_REPAIR_MAX', '12'))
# ブロッカー同定に使う voxel。Phase28 の集計用既定(0.10)は「残荷物のどれかが入る空間」を
# 測るには足りるが、**特定の1個の荷物**が入る位置を数えるには粗すぎる(voxel は少しでも
# 重なれば occupied 側に倒れるので、細い隙間が丸ごと消える)。実測 tools/phase29_diag.py:
# D01 は 0.10 で sup=0(=修正不能)だが 0.05 で sup=2・0.025 で sup=12 と、いずれも
# 既配置荷物だけが塞いでいる位置だった。1回あたり数十msなので 0.05 を既定にする。
REPAIR_VOXEL = float(os.environ.get('MYSOLVER_REPAIR_VOXEL', '0.05'))
_DEBUG = os.environ.get('MYSOLVER_DEBUG_BUDGET') == '1'

# ---------------------------------------------------------------------------
# Phase34: ALNS(破壊 → 修復)を接頭辞再開の上で回す
# ---------------------------------------------------------------------------
# Phase29(衝突駆動リスタート)が26シーン中2シーンにしか届かなかった原因は2つあった:
#   (a) 対象が (iii) 搬入経路の封鎖だけで、Phase30 の最大区分 (i) 幾何で入らない
#       (10/21シーン、残体積の54.8%)に届いていなかった
#   (b) 1試行が「順序を最初から全部評価し直す」コストで、端数予算に数回しか入らなかった
# Phase34 は (a) を occupier removal(reach.item_occupiers)で、(b) を接頭辞再開
# (Phase33 で 9/9 ビット単位一致・1反復コストは全構築の1.6〜4.8%)で外す。
# 設計と正しさの議論は agents/mysolver/alns.py の docstring を参照。
#
# **既定は無効(0)**。無効時はスナップショットも stall_info も収集しないので、
# 計算経路にすら入らず Phase33 までの出力とビット単位で同一
# (決定的5シーン + A01-A03 で検証: results/phase34_report.md)。
#
# 予算について: Phase29 と同じく **リスタート回数も総ユニット予算も増やさない**。
# フェーズ1/2 は「満額の枠が入らないなら新しいリスタートを始めない」ため必ず端数が
# 余って捨てられており、ALNS の反復はその端数の中だけで回す。反復回数は固定値ではなく
# 「端数に収まるだけ」の anytime 設計にする(端数はシーンによって C02 1.7s 〜 D01 39.7s と
# 20倍以上ばらつくため、固定回数では意味を成さない)。
ALNS = os.environ.get('MYSOLVER_ALNS', '0') == '1'
# 1回の build_order で試す反復の上限(通常は予算が先に尽きる。暴走止め)。
ALNS_MAX = int(os.environ.get('MYSOLVER_ALNS_MAX', '64'))
# 占有者・ブロッカー同定の voxel。占有者の同定は「そこに何が居るか」という嵩の問いなので、
# Phase29 がブロッカー用に 0.05 まで細かくした理由(細い隙間が消えると収まる位置を
# 見落とす)は当てはまらない。既定は reach.VOXEL と同じ 0.10。
ALNS_VOXEL = float(os.environ.get('MYSOLVER_ALNS_VOXEL', '0.10'))
# worst removal で外す個数。
ALNS_WORST_Q = int(os.environ.get('MYSOLVER_ALNS_WORST_Q', '3'))

# ゲート1(到達シーン数)の計測用。build_order 1回ぶんの診断を書き出すだけで、
# 探索の挙動には一切影響しない(tools/phase34_gate1.py が読む)。
ALNS_STATS: dict = {}

# ---------------------------------------------------------------------------
# Phase35: ρ-test(複製評価器による受理ゲート)
# ---------------------------------------------------------------------------
# Phase34 が測った決定的な事実: ALNS が採用した手は**定義上すべて代理目的関数を改善して
# いる**のに、実fillの改善は7シーン中4シーン、代理gainと実fill差の順位相関は ρ=−0.321。
# 代理 = 実 + ノイズ なら、代理gainが大きい手を選ぶことは**ノイズが正に大きい手を選ぶこと**
# であり、現在の代理関数を山登りするあらゆる手法(Phase24/28/29/34 の4連敗)が失敗する。
#
# ここは代理を良くするのではなく、**代理を信じない**。候補順序を最後に
# 本物と同じ pybullet/validator/evaluator(agents/mysolver/replica.py)で実際に走らせ、
# **実 fill の argmax** を勝者にする。構築ロジックには一切触れない。
#
# 適用範囲: `replica.is_applicable`(既積み荷物が無いシーンだけ)。既積みがあると
# 復元できない物理の内部状態のせいで本物とずれることを実測済み(replica.py の docstring)。
#
# 予算: **総予算は増やさない**(Phase25b で飽和済み・増やすと悪化)。複製評価に回す分だけ
# 構築の予算を減らす。減らす副作用が小さいことは Phase25b のスイープが示している
# (1.55e7 → 2.0e7 で public 53.61 → 53.64 と、25%減らしても 0.03 しか動かない)。
# 【採用】26シーンA/B(1.55e7)で fill_strict 24.413 → 26.263(**+1.850**)、σ=3.060 /
# SE=0.600 / **t=3.082** と採用基準 t>2 を通過し、**悪化したシーンが1件も無かった**
# (改善9 / 不変17)。Phase23 以来はじめて t>2 を満たした変更である
# (results/phase35_report.md §3)。既定を有効にする。
REPLICA_SELECT = os.environ.get('MYSOLVER_REPLICA_SELECT', '1') == '1'
# 実評価に回す候補数(上位K件、代理スコア降順)。**固定値**にしてあるのは決定性のため
# (「残り時間で入るだけ」にすると машина速度で結果が変わり、Phase17 で確保した
#  決定性が壊れる)。壁時計の保険は別途 deadline で持つ。
REPLICA_TOPK = int(os.environ.get('MYSOLVER_REPLICA_TOPK', '4'))
# 複製評価のために取り置く壁時計[秒]。構築側はこの分だけ早く切り上げる。
REPLICA_RESERVE_S = float(os.environ.get('MYSOLVER_REPLICA_RESERVE_S', '45.0'))
REPLICA_STATS: dict = {}


def _advance_before(order: list[int], x: int, blockers: list[int]) -> list[int] | None:
    """x を「order 上で最も早いブロッカー」の直前へ移動した順序を返す。

    これが本フェーズの主役の多手移動である(「1手入れ替え」ではない)。x を1つ前へ出すと、
    その間にあった荷物はすべて1つ後ろへずれる。
    """
    pos = {v: i for i, v in enumerate(order)}
    if x not in pos:
        return None
    tgt = min((pos[b] for b in blockers if b in pos), default=None)
    if tgt is None or tgt >= pos[x]:
        return None
    rest = [v for v in order if v != x]
    return rest[:tgt] + [x] + rest[tgt:]


def _advance_to_front(order: list[int], x: int, blockers: list[int]) -> list[int] | None:
    """x を先頭へ移動する(「全ブロッカーより前」の最も強い形)。

    lookahead_k>1 のシーンでは、プールの中から planner が置く順を選ぶので、**ブロッカーが
    ストリーム順では x より後ろに居ることがある**(置かれた順と流れてきた順が一致しない)。
    この場合 `_advance_before` も `_delay_blockers` も「もう x のほうが前にある」と判断して
    None を返してしまい、修正手が1つも作れない。実測(tools/phase29_repair_probe.py)では
    C03 がこれに該当し、唯一作れるこの手が代理fill +5.79pt だった。
    """
    if x not in order or order[0] == x:
        return None
    return [x] + [v for v in order if v != x]


def _delay_blockers(order: list[int], x: int, blockers: list[int]) -> list[int] | None:
    """ブロッカー群を x の直後へまとめて移動した順序を返す(相対順序は保つ)。

    `_advance_before` と同じ「x が全ブロッカーより前」を達成する別経路。x を前に出すと
    x より前の積み方まで丸ごと変わってしまうのに対し、こちらは x までの積み方を保ったまま
    ブロッカーだけを後ろへ回すので、破壊が小さい。どちらが効くかは事前には決められないため
    両方を決定的な順番で試す。
    """
    bset = {b for b in blockers if b in set(order)}
    if not bset:
        return None
    pos = {v: i for i, v in enumerate(order)}
    if x not in pos or all(pos[b] > pos[x] for b in bset):
        return None
    bs = [v for v in order if v in bset]
    rest = [v for v in order if v not in bset]
    i = rest.index(x)
    return rest[:i + 1] + bs + rest[i + 1:]


def build_order(item_list: list[dict], container_list: list[dict] | None, lookahead_k: int | None,
                 time_budget: float = DEFAULT_TIME_BUDGET) -> list[int]:
    start = time.perf_counter()
    # Phase35: 複製評価器を使うシーンでは、その分の**壁時計**を先に取り置き、構築側の
    # ユニット予算と壁時計の両方を減らす。総予算を増やさない(Phase25b: 飽和済み)ための処置。
    use_replica = False
    if REPLICA_SELECT and container_list:
        try:
            use_replica = replica.is_applicable(container_list)
        except Exception:
            use_replica = False
    reserve_s = REPLICA_RESERVE_S if use_replica else 0.0
    # **取り置きは壁時計からだけ引き、ユニット予算からは引かない。**
    #
    # 初版は `time_budget - reserve` でユニット予算そのものを削っていたが、これは誤りだった。
    # 構築の大半のシーンは壁時計ではなく **ユニット予算を使い切って先に終わる**
    # (Phase31 実測で optimize の壁時計は mean 51s / max 152s、上限165sに対して余裕がある)。
    # そのためユニット予算を削ると、**取り置きを実際には使い切らないシーンでも**構築の質が
    # 落ちる。実測(§3.2)では A07 が構築側 −10.74、D01 が −10.77 とこの副作用で大きく損した。
    # 壁時計だけを縮めれば、ユニット予算で先に終わるシーンは**一切損をせず**、
    # 壁際まで走るシーンだけが取り置きぶん早く切り上げられる。
    build_budget_s = time_budget
    hard_deadline = start + min(HARD_WALL_LIMIT - reserve_s,
                                 build_budget_s * HARD_WALL_FACTOR)
    total_budget = planner.SearchBudget.from_seconds(build_budget_s, hard_deadline=hard_deadline)

    # Phase17(ターゲット2): 1リスタートあたりの配分・検証枠・最終マージンはすべて
    # **総予算に依存しない固定値**にする(理由は CONSTRUCT_SLICE のコメント参照)。
    # 旧実装はこれらを time_budget に比例させており、それ自体が予算非単調性の原因だった。
    # (以下はすべて「名目秒」。planner.UNITS_PER_SEC で決定的にユニットへ換算する。)
    #
    # 例外は「1リスタート分すら入らないほど短い開発用予算」の場合だけ。ここだけは
    # 何も構築せずヒューリスティック順を返してしまうのを避けるため予算に比例させる
    # (本フェーズの掃引水準 30/60/120/165 はすべて固定値側に入るので、単調性の
    #  検証には影響しない)。
    construct_slice = (CONSTRUCT_SLICE if build_budget_s >= CONSTRUCT_SLICE
                       else max(1.0, build_budget_s * 0.5))
    final_margin = (FINAL_MARGIN if build_budget_s >= CONSTRUCT_SLICE
                    else max(0.2, build_budget_s * 0.05))
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

    # Phase29: 1回の validate が実際に消費したユニット(名目秒ではなく実測)。
    # 修正フェーズが「端数の予算で1回ぶん回るか」を判定するのに使う。MAX_VALIDATE_SLICE(12s)は
    # あくまで上限であって実費ではない(実測 0.37〜8.84 名目秒とシーンで20倍以上ちがう)。
    max_validate_units = [0.0]

    def validate(order: list[int], stall_info: dict | None = None,
                  slice_units: float | None = None,
                  snapshots_out: dict | None = None,
                  contrib_out: list | None = None,
                  resume_state: dict | None = None) -> tuple[float, int] | None:
        """戻り値 None は「予算切れで評価できなかった」の意(比較対象にしない)。"""
        if total_budget.exhausted():
            return None
        used_before = total_budget.used
        # Phase18: stability の幾何代理は risk_adjusted_volume 自体(荷物ごとの割引)に
        # 織り込み済み(simulate.simulate_order の stability_weight 引数)。ここで返る
        # stability_risk_ratio は診断用(coverageの平均リスク)であり目的関数には使わない。
        # Phase28: REACH_WEIGHT>0 のときだけ到達可能性を測る(既定0は dict を渡さないので
        # simulate_order 側の計算経路にも入らず、Phase27 までとビット単位で同一)。
        reach_info: dict | None = {} if REACH_WEIGHT > 0.0 else None
        placed_ids, placed_volume, risk_adjusted_volume, violation_ratio, stability_risk_ratio = \
            simulate.simulate_order(
                container_list, items_by_index, order, k,
                (total_budget.child(slice_units) if slice_units is not None
                 else total_budget.child_seconds(max_validate_slice)),
                prepacked_ids=prepacked_ids,
                stability_weight=STABILITY_PENALTY_WEIGHT, reach_info=reach_info,
                stall_info=stall_info, snapshots_out=snapshots_out, contrib_out=contrib_out,
                resume_state=resume_state)
        max_validate_units[0] = max(max_validate_units[0], total_budget.used - used_before)
        count = len(placed_ids)
        penalty = PLACEMENT_PENALTY_WEIGHT * total_container_volume * violation_ratio
        # Phase28: 自己封鎖した順序を割り引く。**加算ペナルティにしてはならない**。
        # Phase18 の実測(suite_A07 で 40個中1個しか置かない順序が選ばれ fill 28.07->0.00)が
        # 示したとおり、達成可能な体積より罰則が大きくなるシーンでは「何も置かないほうが
        # 目的関数上は得」という退化解に収束する。ここは stability_discount と同じ
        # **乗算による割引**にして、常に 0 以上・達成体積を超えて罰しない形にする。
        # blocked_ratio の分母を supported にしてある点も同じ目的(reach.py のdocstring参照)。
        if reach_info:
            discount = max(0.0, 1.0 - REACH_WEIGHT * reach_info.get('blocked_ratio', 0.0))
            return (risk_adjusted_volume * discount - penalty, count)
        return (risk_adjusted_volume - penalty, count)

    # Phase29: 最良順序が「どこで・何に阻まれて」行き詰まったかを持ち回る(REPAIR/ALNS時のみ)。
    best_stall: dict | None = None
    # Phase34: 最良順序の「全配置数ぶんのスナップショット」と「荷物ごとのrisk割引率」。
    # **スナップショットの採取はユニット予算を消費しない**ので、ALNS の有効/無効で
    # フェーズ1/2 のリスタート系列は完全に同一のまま(追加されるのは複製の壁時計だけ)。
    best_snaps: dict = {}
    best_contribs: list = []
    # Phase35: 複製評価器で選び直すための候補プール((代理スコア, 順序))。
    # 収集はリストへの append だけで、探索の挙動にも予算にも一切影響しない。
    cand_pool: list = []

    def _extras():
        """ALNS 有効時だけ収集する追加情報の入れ物(無効時は全て None = 経路に入らない)。"""
        if not ALNS:
            return ({} if REPAIR else None), None, None
        return {}, {}, []

    try:
        stall, snaps, contribs = _extras()
        score = validate(heuristic_order, stall, snapshots_out=snaps, contrib_out=contribs)
        if score is not None and use_replica:
            cand_pool.append((score, list(heuristic_order)))
        if score is not None and (best_score is None or _better(score, best_score)):
            best_order, best_score, best_stall = heuristic_order, score, stall
            best_snaps, best_contribs = (snaps or {}), (contribs or [])
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
        nonlocal best_order, best_score, best_stall, best_snaps, best_contribs
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
                stall, snaps, contribs = _extras()
                score = validate(order, stall, snapshots_out=snaps, contrib_out=contribs)
                if score is not None and use_replica:
                    cand_pool.append((score, list(order)))
                if score is not None and (best_score is None or _better(score, best_score)):
                    best_order, best_score, best_stall = order, score, stall
                    best_snaps, best_contribs = (snaps or {}), (contribs or [])
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

    # フェーズ3(Phase29): 衝突駆動の順序修正。
    #
    # ここへ来た時点で「新しいリスタート1回分(construct+validate)には足りない端数の予算」が
    # 必ず残っており、従来はそれを捨てていた。修正の試行は validate 1回だけで済む
    # (構築をやり直さない)ので、この端数に収まる。**リスタート回数も総予算も増やさない。**
    if os.environ.get('MYSOLVER_DEBUG_BUDGET') == '1':
        print(f'[budget] limit={total_budget.limit / u:.1f}s used={total_budget.used / u:.1f}s '
              f'remaining={total_budget.remaining() / u:.1f}s '
              f'(1リスタート={construct_units / u:.1f}s+検証, 端数の下限={final_margin_units / u:.1f}s, '
              f'validate実費最大={max_validate_units[0] / u:.2f}s)', flush=True)

    if REPAIR:
        # 1回の修正に必要な予算 = validate 1回の**実費**(このシーンでの実測最大)。
        # MAX_VALIDATE_SLICE(12s)は上限であって実費ではないので、それで判定すると
        # 実費 1.5s のシーンでも「端数が足りない」と誤判定して修正が一度も回らない。
        max_validate_units_cap = max_validate_slice * u
        tried: set[tuple[int, ...]] = {tuple(best_order)}
        skip_items: set[int] = set()
        n_repairs = 0
        while n_repairs < REPAIR_MAX:
            if total_budget.exhausted():
                break
            # FINAL_MARGIN は「最後のリスタートの validate だけが残予算で切り詰められると、
            # そのリスタートの評価値が総予算に依存してしまう」ことを防ぐための取り置きである
            # (FINAL_MARGIN のコメント参照)。修正フェーズには当てはまらない: 切り詰められた
            # 評価は体積が過小に出るだけで、採用は厳密改善のときにしか起きないため、
            # **安全側にしか倒れない**。壁時計の安全弁とも無関係(総ユニット予算は不変で、
            # 従来 捨てていた端数を使うだけ。実測の壁時計は 75〜112s で上限 165s に対し余裕がある)。
            # 実測: この取り置きを残すと C03(端数8.5s・validate実費7.34s)と
            # P05(9.4s・7.09s)がどちらも1回も試行できず、修正の到達シーンが 2 -> 0 になる。
            avail = total_budget.remaining()
            if avail < max_validate_units[0]:
                break
            if not best_stall or not best_stall.get('stalled'):
                break   # 行き詰まらずに全件流し切った(=衝突が無い)
            pool = [it for it in best_stall.get('pool', [])
                    if int(it['index']) not in skip_items]
            if not pool:
                break
            try:
                sb = reach.stall_blockers(best_stall['containers'], pool, voxel=REPAIR_VOXEL)
            except Exception:
                break
            # 到達可能性の計算コストも決定的な量でユニット予算へ計上する(壁時計だけが伸びて
            # 非常用安全弁を踏むのを防ぐ。simulate.REACH_UNIT_COST と同じ扱い)。
            total_budget.spend(simulate.REACH_UNIT_COST * sb['grid_cells'] * sb['n_shapes'])
            cand = sb['candidate']
            if cand is None:
                break   # 順序では開けられない(棚が塞いでいる/そもそも収まらない)
            x = cand['item_index']
            blockers = cand['result']['blockers']
            if _DEBUG:
                print(f'[repair] 衝突: X={x} をブロックしているのは {blockers} '
                      f'(塞がれた位置 {cand["result"]["n_blocked_positions"]}箇所, '
                      f'残予算 {total_budget.remaining() / u:.1f}s)', flush=True)
            skip_items.add(x)   # 同じ X で堂々巡りしない(改善すれば下で解除する)
            improved = False
            for maker in (_advance_before, _delay_blockers, _advance_to_front):
                avail = total_budget.remaining()
                if avail < max_validate_units[0]:
                    break
                cur_best = best_order
                repaired = maker(cur_best, x, blockers)
                if repaired is None or tuple(repaired) in tried:
                    continue
                tried.add(tuple(repaired))
                n_repairs += 1
                stall = {}
                try:
                    # 途中で切られた評価は体積が過小に出るだけで、採用は厳密改善のときしか
                    # 起きないため、誤って悪い順序を採用することはない(安全側に倒れる)。
                    score = validate(repaired, stall,
                                     slice_units=min(max_validate_units_cap, avail))
                except Exception:
                    break
                if _DEBUG:
                    print(f'[repair]   {maker.__name__}: score={score} (現best={best_score})',
                          flush=True)
                if score is not None and (best_score is None or _better(score, best_score)):
                    best_order, best_score, best_stall = repaired, score, stall
                    improved = True
                    break
            if improved:
                # 改善したら新しい行き詰まり地点から測り直す(X も再度対象に戻す)。
                skip_items.clear()

    # フェーズ4(Phase34): ALNS(破壊 → 修復)を接頭辞再開の上で回す。
    #
    # ここへ来た時点で残っているのは「新しいリスタート1回分(構築+検証)には足りない端数」
    # だけである。ALNS の1反復は **末尾だけの再評価** なので(構築をやり直さない、しかも
    # 接頭辞は再計算しない)この端数に収まる —— Phase33 の実測で全構築1回の 1.6〜4.8%。
    # リスタート回数も総ユニット予算も一切増やさない。
    if ALNS:
        stats: dict = {
            'enabled': True,
            'n_items': len(best_order),
            'fraction_s': total_budget.remaining() / u,
            'validate_units_s': max_validate_units[0] / u,
            'n_snapshots': len(best_snaps),
            'n_eval': 0, 'n_accept': 0,
            'ops': {op: {'tried': 0, 'found': 0, 'evaluated': 0, 'accepted': 0}
                    for op in alns.OPS},
            'iter_s': [], 'gain': 0.0, 'stopped': None,
        }
        score0 = best_score
        skip_by_op: dict = {op: set() for op in alns.OPS}
        tried_orders: set = {tuple(best_order)}
        fit_cache: dict = {}
        n_total = len(best_order)
        op_i = 0
        consecutive_fail = 0
        while stats['n_eval'] < ALNS_MAX and consecutive_fail < len(alns.OPS):
            if total_budget.exhausted():
                stats['stopped'] = 'budget_exhausted'
                break
            if not best_stall or not best_stall.get('stalled'):
                stats['stopped'] = 'no_stall'      # 全件流し切った = 壊すべき衝突が無い
                break
            if not best_snaps:
                stats['stopped'] = 'no_snapshots'
                break
            op = alns.OPS[op_i % len(alns.OPS)]
            op_i += 1
            stats['ops'][op]['tried'] += 1

            # --- 破壊: 外す荷物の集合を決める ---
            try:
                if op == alns.OP_OCCUPIER:
                    x, removed, info = alns.destroy_occupier(best_stall, ALNS_VOXEL, skip_by_op[op])
                elif op == alns.OP_BLOCKER:
                    x, removed, info = alns.destroy_blocker(best_stall, ALNS_VOXEL, skip_by_op[op])
                else:
                    x, removed, info = alns.destroy_worst(best_stall, best_contribs,
                                                          ALNS_WORST_Q, skip_by_op[op])
            except Exception:
                consecutive_fail += 1
                continue
            # 幾何判定のコストも決定的な量(格子数×形状数)でユニット予算へ計上する
            # (壁時計だけが伸びて非常用安全弁を踏むのを防ぐ。Phase29 と同じ扱い)。
            if info.get('grid_cells'):
                total_budget.spend(simulate.REACH_UNIT_COST * info['grid_cells']
                                    * info.get('n_shapes', 1))
            if x is None or not removed:
                consecutive_fail += 1
                continue
            consecutive_fail = 0
            stats['ops'][op]['found'] += 1
            skip_by_op[op].add(x)      # 同じ X で堂々巡りしない(改善したら下で解除)

            r_ids = [x] + [i for i in removed if i != x]
            # 「プールに入っている荷物は動かさない」制約を満たす最大の k
            # (=再開位置を最も後ろに取る=1反復が最も安い)。alns.py の docstring 参照。
            k = alns.choose_snapshot_k(best_snaps, best_order, r_ids)
            if k is None:
                continue
            snap = best_snaps[k]
            # 1反復の見積り = validate 実費 × 末尾の割合。端数がこれを下回ったら打ち切る。
            est = max_validate_units[0] * max(0.05, len(snap['remaining_order']) / max(1, n_total))

            improved = False
            for rep in ('greedy', 'regret'):
                if total_budget.remaining() < est:
                    stats['stopped'] = 'fraction_exhausted'
                    break
                if rep == 'greedy':
                    r_ordered = alns.repair_greedy(r_ids, items_by_index)
                else:
                    ck = (k, tuple(sorted(r_ids)))
                    if ck not in fit_cache:
                        try:
                            counts, cells, shapes = reach.fit_position_counts(
                                best_stall['containers'],
                                [items_by_index[i] for i in r_ids], r_ids, voxel=ALNS_VOXEL)
                        except Exception:
                            break
                        total_budget.spend(simulate.REACH_UNIT_COST * cells * shapes)
                        fit_cache[ck] = counts
                    r_ordered = alns.repair_regret(r_ids, fit_cache[ck])

                new_order, new_tail = alns.build_new_order(best_order, snap, r_ordered)
                if len(new_order) != n_total or tuple(new_order) in tried_orders:
                    continue
                tried_orders.add(tuple(new_order))

                rs = alns.make_resume_state(snap, new_tail, simulate.clone_containers)
                stall2: dict = {}
                snaps2: dict = {}
                contribs2: list = []
                used0 = total_budget.used
                try:
                    # 途中で切られた評価は体積が過小に出るだけで、採用は厳密改善のときにしか
                    # 起きないため安全側に倒れる(Phase29 と同じ議論)。
                    score = validate(new_order, stall2,
                                     slice_units=min(max_validate_slice * u,
                                                     total_budget.remaining()),
                                     snapshots_out=snaps2, contrib_out=contribs2,
                                     resume_state=rs)
                except Exception:
                    break
                stats['n_eval'] += 1
                stats['ops'][op]['evaluated'] += 1
                stats['iter_s'].append((total_budget.used - used0) / u)
                if _DEBUG:
                    print(f'[alns] {op}/{rep}: X={x} 外す={removed} k={k} '
                          f'score={score} (現best={best_score}, 残 {total_budget.remaining() / u:.1f}s)',
                          flush=True)
                if score is not None and (best_score is None or _better(score, best_score)):
                    old_order = best_order
                    best_order, best_score = new_order, score
                    best_stall = stall2
                    # 接頭辞が一致しているので k 以下のスナップショットはそのまま使い回せる
                    # (末尾の並びだけ差し替える)。k 以上は今の再開ロールアウトが記録済み。
                    best_snaps = alns.refresh_snapshots(best_snaps, k, old_order, new_order, snaps2)
                    best_contribs = best_contribs[:k] + contribs2
                    stats['n_accept'] += 1
                    stats['ops'][op]['accepted'] += 1
                    for s in skip_by_op.values():
                        s.clear()
                    improved = True
                    break
            if improved:
                continue
        if stats['stopped'] is None:
            stats['stopped'] = ('iter_cap' if stats['n_eval'] >= ALNS_MAX else 'no_candidate')
        if score0 is not None and best_score is not None:
            stats['gain'] = best_score[0] - score0[0]
        ALNS_STATS.clear()
        ALNS_STATS.update(stats)

    # フェーズ5(Phase35): ρ-test —— 代理の argmax ではなく **実 fill の argmax** で選び直す。
    #
    # ここまでで作った候補順序を、本物と同じ pybullet/validator/evaluator(replica.py)で
    # 実際に走らせ、実 fill が最大のものを勝者にする。構築は一切変えていない。
    # 予算は冒頭で取り置いた reserve_s(壁時計)の中だけで使う。
    if use_replica and cand_pool:
        rstats: dict = {'enabled': True, 'n_cand': len(cand_pool), 'reserve_s': reserve_s,
                        'evaluated': 0, 'changed': False, 'rows': [], 'stopped': None}
        # 代理スコア降順に並べ、重複順序を除いて上位K件だけ実評価する。
        seen: set = set()
        ranked = []
        for sc, od in sorted(cand_pool, key=lambda t: t[0], reverse=True):
            key = tuple(od)
            if key in seen:
                continue
            seen.add(key)
            ranked.append((sc, od))
        ranked = ranked[:max(1, REPLICA_TOPK)]
        rstats['n_ranked'] = len(ranked)
        deadline = start + min(HARD_WALL_LIMIT, time_budget * HARD_WALL_FACTOR)
        best_real = None
        try:
            with replica.ReplicaEvaluator(container_list, k,
                                          prepacked_ids=prepacked_ids) as rep:
                for rank, (sc, od) in enumerate(ranked):
                    if time.perf_counter() >= deadline:
                        rstats['stopped'] = 'wall_deadline'
                        break
                    got = rep.evaluate(item_list, od, deadline=deadline)
                    if got is None:
                        rstats['stopped'] = 'wall_deadline'
                        break
                    rstats['evaluated'] += 1
                    rstats['rows'].append({'rank': rank, 'surrogate': sc[0],
                                            'real_fill': got['fill'],
                                            'num_placed': got['num_placed']})
                    # 同点なら代理順位が上(=rank が小さい)ほうを残す = 決定的
                    if best_real is None or got['fill'] > best_real[0] + 1e-12:
                        best_real = (got['fill'], od, rank)
        except Exception:
            rstats['stopped'] = 'error'
            best_real = None
        if best_real is not None:
            rstats['winner_rank'] = best_real[2]
            rstats['winner_fill'] = best_real[0]
            if best_real[2] != 0:
                # 代理の1位とは違う候補が実評価で勝った = ρ-test が効いた瞬間
                rstats['changed'] = True
                best_order = best_real[1]
        if rstats['stopped'] is None:
            rstats['stopped'] = 'done'
        REPLICA_STATS.clear()
        REPLICA_STATS.update(rstats)

    return best_order
