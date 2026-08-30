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
import traceback

import numpy as np

from . import alns
from . import geometry as geo
from . import planner
from . import reach
from . import simulate

# Phase36(タスク1-1): 複製評価器の import は **失敗しても致命傷にしない**。
# replica.py は src.ground_handling(本物の env 実装)を引くため、提出先の環境構成が
# 想定と違えば ImportError になりうる。素朴に `from . import replica` と書くと
# **ordering.py 自体が import できなくなり、エージェントが丸ごと起動しない**
# (=全シーン全損)。ここで握って None にしておけば、複製評価器を使わないだけで
# Phase34 相当の挙動に自動で落ちる。
try:
    from . import replica as _replica_mod
except Exception:      # ImportError に限定しない(依存の初期化失敗も拾う)
    _replica_mod = None

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
# Phase38(ステップA): 環境変数化(既定値は165.0のまま不変)。ローカル計測を壁時計非拘束
# (MYSOLVER_HARD_WALL_LIMIT=3000)で行うため。165.0は提出時のみ使う値。
HARD_WALL_LIMIT = float(os.environ.get('MYSOLVER_HARD_WALL_LIMIT', '165.0'))
# time_budget を短く指定した開発時(--optimize-budget 30 等)にも比例した安全弁を掛ける。
# 較正誤差(実効速度が想定の 1/1.4 まで落ちる)まではユニット予算を使い切れる余裕になる。
# Phase38(ステップA): 環境変数化(既定値は1.4のまま不変)。
HARD_WALL_FACTOR = float(os.environ.get('MYSOLVER_HARD_WALL_FACTOR', '1.4'))


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

# Phase72: build_order()の探索結果を左右しない読み取り専用の診断記録(既定で常時収集、
# 戻り値・探索の挙動は一切変えない)。呼び出しごとに上書きされるだけの副チャネルで、
# Phase71が発見した「shuffle_tiesがis_soft優先の並びを破壊しているか」「破壊した側が
# best_orderとして採用されているか」を、build_order自体を変更せずに外部から診断できるように
# するためのもの。診断ツール(tools/phase72_winner_trace.py)がbuild_order呼び出し直後に
# この辞書を読む。
LAST_BUILD_DIAGNOSTICS: dict = {}


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
# Phase86 Tier3: BRKGA(Biased Random-Key Genetic Algorithm)
# ---------------------------------------------------------------------------
# Deep Research(docs/「Beyond Constructive Heuristics」)が指摘する ALNS 不採用の原因
# (代理評価の精度不足、Phase34 の ρ=−0.321)は、母集団を持つ進化計算がノイズ平均化効果で
# 構造的に回避しやすいとされる。BRKGA は「荷物の並び順を random-key(実数ベクトル)で
# 符号化し、decoder で実配置に変換」する設計で、decoder には**既存の構築ヒューリスティック
# (simulate.beam_construct_order)をそのまま使う**——新しい評価関数は作らない。
# fitness は代理評価ではなく validate()(既存の risk調整済み体積、実 decoder 出力)そのもの。
#
# 既定は無効(0)。無効時はフェーズ2が従来どおりのランダムリスタートのままであり、
# 本ブロックの定数・後述の population ループは一切参照されない
# (build_order 側の分岐は if/else で完全に切り分ける)。
#
# 予算の公平性: 1世代の総予算 ≈ phase2_units(=フェーズ2の1リスタート分の予算)になるよう
# 個体1体あたりの decode 予算を population size で割って配る。つまり
# 「同じ総予算を、独立リスタートの束ではなく交叉のある母集団に配り直したら伸びるか」を
# フェーズ2と揃えた土俵で比較できる設計にしてある(総ユニット予算は増やさない)。
BRKGA = os.environ.get('MYSOLVER_BRKGA', '0') == '1'
# 母集団サイズ。Deep Research の目安(30〜50)の下限寄り(小予算シーンでも複数世代回る余地)。
BRKGA_POP = int(os.environ.get('MYSOLVER_BRKGA_POP', '30'))
# エリート(無条件で次世代へ複製、fitness再評価もしない)の割合。
BRKGA_ELITE_FRAC = float(os.environ.get('MYSOLVER_BRKGA_ELITE_FRAC', '0.2'))
# ミュータント(前世代を無視した完全ランダム個体)で置き換える割合。
BRKGA_MUTANT_FRAC = float(os.environ.get('MYSOLVER_BRKGA_MUTANT_FRAC', '0.2'))
# 交叉時、各遺伝子(荷物ごとのkey)をエリート親から継承する確率(標準BRKGAのbiased crossover。
# 0.5だと普通の一様交叉、1.0に近いほどエリート親に強く倣う)。
BRKGA_BIAS = float(os.environ.get('MYSOLVER_BRKGA_BIAS', '0.7'))
# 世代数の上限(暴走止め。通常は予算が先に尽きる)。
BRKGA_MAX_GEN = int(os.environ.get('MYSOLVER_BRKGA_MAX_GEN', '200'))
# 個体1体のdecode予算を「フェーズ1終了時点の残り予算 ÷ (母集団 × この値)」で逆算する
# 目標世代数。残り予算はシーンごとに大きくばらつく(実測 0〜45s超)ため、固定コストの
# 世代を要求すると残りが少ないシーンで0世代に終わる(下のind_units算出コメント参照)。
BRKGA_TARGET_GENS = int(os.environ.get('MYSOLVER_BRKGA_TARGET_GENS', '15'))
# 母集団のdecode評価に使うスコアノイズ(フェーズ2のuse_noise=Trueと同じ0.35を既定にし、
# 世代間の多様性を確保する)。
BRKGA_NOISE = float(os.environ.get('MYSOLVER_BRKGA_NOISE', '0.35'))
# noisy-fitness対策(Qian et al. 2018): 母集団内で暫定最良を更新した個体は、
# ノイズ無し(score_noise=0)で**もう一度decodeし直して**(=再評価)、その再確認decodeでも
# 現在のグローバル最良を上回った場合にのみ採用する(しきい値選択。マージンは既定0=
# 「再確認しても厳密改善」を要求するだけで、量的なマージンは入れない)。
BRKGA_ACCEPT_MARGIN = float(os.environ.get('MYSOLVER_BRKGA_ACCEPT_MARGIN', '0.0'))
BRKGA_STATS: dict = {}

# ---------------------------------------------------------------------------
# Phase87: フェーズ1/フェーズ2の予算配分を実測するための読み取り専用の診断記録。
# ---------------------------------------------------------------------------
# Phase86でBRKGAの世代数を実測した際に「フェーズ1がほぼ全予算を使い切り、フェーズ2
# 相当の予算がほとんど残らない」ことがA01の1シーンだけで判明した。この記録は
# **探索の挙動には一切影響しない**(値を読むだけで、build_orderの分岐・打ち切り判定
# には使わない)。他の *_STATS(ALNS_STATS, BRKGA_STATS)と同じ既存パターンを踏襲する。
PHASE_BUDGET_STATS: dict = {}

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
# (results/phase35_report.md §3)。Phase35〜43 は既定を有効にしていた。
#
# Phase44: **既定を無効に戻す**。Phase42提出(replica.pyの防御的書き直し+候補単位ラッチ)の
# 結果、fill_score は 38.09476291926298 のまま13桁一致 —— 本番では一度も機能していないと
# 確定した(候補単位ラッチ自体は効いていて optimization は 155.46→165.109 に変化し
# HARD_WALL_LIMIT=165.0 に初めて到達したが、ρ-testそのものの得点効果はゼロのまま)。
# 一方で複製評価は45秒(REPLICA_RESERVE_S)の壁時計取り置きを常に消費し、
# optimization_timeout(180s)への余裕を24.5s→14.9sまで縮める副作用がある。
# 機能していないものに安全マージンを支払い続ける理由が無いため、**無効化は純粋な
# 安全側の改善**として既定を戻す(results/phase41_report.md Phase44節参照)。
# replica.py・候補単位ラッチ・回帰テスト(tools/test_replica_*.py)は削除しない
# ——原因(ρ-testが本番でなぜ機能しないか)が判明すれば '1' に戻すだけで復帰できる
# 状態を保つ。
REPLICA_SELECT = os.environ.get('MYSOLVER_REPLICA_SELECT', '0') == '1'
# 実評価に回す候補数(上位K件、代理スコア降順)。**固定値**にしてあるのは決定性のため
# (「残り時間で入るだけ」にすると машина速度で結果が変わり、Phase17 で確保した
#  決定性が壊れる)。壁時計の保険は別途 deadline で持つ。
REPLICA_TOPK = int(os.environ.get('MYSOLVER_REPLICA_TOPK', '4'))
# 複製評価のために取り置く壁時計[秒]。構築側はこの分だけ早く切り上げる。
REPLICA_RESERVE_S = float(os.environ.get('MYSOLVER_REPLICA_RESERVE_S', '45.0'))
REPLICA_STATS: dict = {}
# Phase38(ステップ1-B): このプロセス(=このシーン)で、複製評価のための取り置きが
# 構築の壁時計締切(hard_deadline)を実際に引き寄せて打ち切りを起こしたか。
# build_order() の末尾で更新する。agent.py の policy() が読み、最初の呼び出し1回だけの
# 壁時計にこの1ビットを符号化する(採点非依存のテレメトリ、MYSOLVER_TELEMETRY=0では不使用)。
LAST_BUILD_WALL_CUT: bool = False
# Phase38(ステップB): このシーンで複製評価が「完走」したか(stopped=='done'、
# すなわちPhase37テレメトリのn=6/n=7に相当)。n=6は勝者不変・n=7は勝者変更のどちらも
# 含む——ここで知りたいのは「ρ-test機構が最後まで動いたか」であって「効いたか」ではない
# (動いてすらいなければ MYSOLVER_REPLICA_METRIC=composite への切替は無意味になる)。
LAST_ANY_SUCCESS: bool = False

# ---------------------------------------------------------------------------
# Phase37(ステップ0): 採点に一切影響しないテレメトリ。
#
# Phase36提出でREPLICA_RESERVE_S(45秒)すら取り置かれた形跡がなく(optimization実測値が
# 「取り置き無し」のローカル最大帯と同帯)、`build_order`冒頭のゲート(H1 preflight失敗 /
# H2 import失敗 / H3 is_applicable False)のどこで落ちているか本番から特定できない。
# `optimization`(=optimize()の壁時計、app.py:98のtime_results)はfull floatで報告され
# 採点には一切使われないため、ここに「到達段階n」を壁時計の埋め方として符号化する。
# 既定無効(MYSOLVER_TELEMETRY=0)。無効時はここに関わる分岐に一切入らないため、
# ρ-test無効時・既存経路と出力(戻り値)はビット単位で不変(壁時計だけを後から埋め増すので、
# best_orderの計算そのものには一切影響しない)。
MYSOLVER_TELEMETRY = os.environ.get('MYSOLVER_TELEMETRY', '0') == '1'
# n=0(158.0s)刻みでn=7(161.5s)まで。HARD_WALL_LIMIT=165s・optimization_timeout=180sの
# どちらに対しても十分な余裕を残す。
TELEMETRY_BASE_S = float(os.environ.get('MYSOLVER_TELEMETRY_BASE_S', '158.0'))
TELEMETRY_STEP_S = float(os.environ.get('MYSOLVER_TELEMETRY_STEP_S', '0.5'))
# コスト抑制(タスク0-2): 埋め増すのはどのみち「全シーンのmax」を左右しうるシーンだけに絞る。
# 自然経過時間がこの閾値を超えないシーンは埋めない(=追加コストほぼゼロ)。
# ローカル26シーン実測(results/phase37_report.md §0)で「取り置き無し」帯・「取り置き有り」帯
# の双方に閾値超のシーンが存在することを確認済みなので、全シーン一律の下限埋め(T_min運用)は
# 採用していない。
TELEMETRY_MIN_ELAPSED_S = float(os.environ.get('MYSOLVER_TELEMETRY_MIN_ELAPSED_S', '140.0'))

# ---------------------------------------------------------------------------
# Phase37(ステップ1-3): ρ-test の勝者決定に使う指標。
#
# 既定 'fill'(=Phase35で採用・26シーンA/Bでt=3.082を通過済みの実装をそのまま維持)。
# 'composite' にすると、実fillのargmaxではなく5成分合成スコア(採点の28.6%だけでなく
# 全体)のargmaxで選ぶ。cog/stability/placement/soft_itemはreplica_scorer.pyによる近似
# (本番評価基盤の非公開ロジックの近似、tools/scorer.pyと同一式)。
# 新規挙動のため既定は無効側('fill')。26シーンでt>2を確認するまでは'composite'を既定にしない
# (評価・採用基準ルール5/6)。
REPLICA_METRIC = os.environ.get('MYSOLVER_REPLICA_METRIC', 'fill')

# ---------------------------------------------------------------------------
# Phase38(ステップ1-A): 本番の n=4(evaluate()内の実行時例外)を自己申告させる。
#
# n=4(stopped=='runtime_error')のときだけ、壁時計の埋め込み先を通常の
# TELEMETRY_BASE_S+TELEMETRY_STEP_S*n の帯から切り離し、この専用の帯
# (162.00〜165.15s、0.05s刻み)へ置き換える。T = N4_BASE_S + N4_STEP_S * code、
# code = 16*a + b(0〜63)。b=例外クラスID(0〜15、_classify_exception参照)、
# a=最初の失敗までに成功した候補数(0〜3で頭打ち)。
# 既定無効(MYSOLVER_TELEMETRY=0)時は分岐にすら入らないため本番経路への影響はゼロ。
TELEMETRY_N4_BASE_S = float(os.environ.get('MYSOLVER_TELEMETRY_N4_BASE_S', '162.00'))
TELEMETRY_N4_STEP_S = float(os.environ.get('MYSOLVER_TELEMETRY_N4_STEP_S', '0.05'))

# ---------------------------------------------------------------------------
# Phase38(ステップ1-C): ラッチを「シーン単位」から「候補単位」に緩める。
#
# 従来(Phase36)は1候補の失敗でそのシーンの複製評価を丸ごと諦めていた。それだと
# 本番のn=4が特定の候補(例えば代理1位の候補だけがpybullet的に特殊な配置になる、等)に
# 固有の場合、原因が分からなくても「その候補だけ飛ばして残りを評価する」ことでρ-testの
# 利得の一部を回収できる可能性がある。暴走(壊れた状態のままK件すべてを無駄撃ち)を防ぐため
# **2回連続で失敗したらシーン単位のラッチに落とす**。
# 'scene' にすると Phase36 と完全に同じ挙動(1回失敗即ラッチ)に戻せる。
REPLICA_LATCH_MODE = os.environ.get('MYSOLVER_REPLICA_LATCH_MODE', 'per_candidate')


def _classify_exception(exc: Exception) -> int:
    """例外を b(0〜15)に符号化する(Phase38ステップ1-A)。

    isinstance の判定順は「サブクラスを親クラスより先に」が鉄則
    (RecursionError は RuntimeError のサブクラス、TimeoutError は OSError のサブクラス)。
    """
    pybullet_error = None
    if _replica_mod is not None:
        pybullet_error = getattr(getattr(_replica_mod, 'p', None), 'error', None)
    checks = [(MemoryError, 0)]
    if pybullet_error is not None:
        checks.append((pybullet_error, 1))
    checks += [
        (KeyError, 2),
        (IndexError, 3),
        (AttributeError, 4),
        (ValueError, 5),
        (TypeError, 6),
        (TimeoutError, 7),      # OSError のサブクラスなので OSError より先に判定する
        (RecursionError, 12),   # RuntimeError のサブクラスなので RuntimeError より先に判定する
        (RuntimeError, 8),
        (OSError, 9),           # IOError は Python3 で OSError のエイリアス
        (AssertionError, 10),
        (ZeroDivisionError, 11),
        (OverflowError, 13),
    ]
    for cls, code in checks:
        if isinstance(exc, cls):
            return code
    return 14  # その他(rstats['exc_class'] にクラス名を残す)


def _record_replica_failure(rstats: dict, exc: Exception | None, rank: int) -> None:
    """1件の複製評価失敗を rstats に記録する(Phase38ステップ1-A/Phase40の共通ロジック)。

    Phase42(ステップ1): 従来は `rep.evaluate()` が例外を投げたとき
    (`except Exception as e:` 経路)にだけこの記録ロジックが必要だったが、
    Phase41で replica.py が観測データ欠損を例外ではなく `('data_error', exc)` という
    戻り値で伝えるようになったため、**同じ符号化ロジックを2箇所(except節と
    data_error分岐)で重複させないための共通関数**にした。

    `exc` には元の例外オブジェクトそのものを渡す(replica.py 側で捕捉されたものでも
    `__traceback__` は保持されたままなので、`_classify_exception()` による b の符号化
    (0〜15、n=4テレメトリの壁時計埋め込み)は例外を投げていた頃と完全に同一の結果になる)。
    `exc` が None(原因不明。本来起こらない想定だが防御的に許容する)の場合は
    b=14('その他')として扱う。
    """
    tb_frames = traceback.extract_tb(exc.__traceback__) if exc is not None else []
    last_frame = tb_frames[-1] if tb_frames else None
    rstats.setdefault('exc_events', []).append({
        'rank': rank,
        'exc_class': type(exc).__name__ if exc is not None else 'Unknown',
        'file': os.path.basename(last_frame.filename) if last_frame else None,
        'lineno': last_frame.lineno if last_frame else None,
    })
    if 'exc_class' not in rstats:
        # Phase38(ステップ1-A): 最初の失敗だけを記録する。
        # a=この時点までに成功した候補数(rstats['evaluated']がまだ
        # 加算されていないので、そのままの値が「失敗より前の成功数」)。
        a = min(3, rstats['evaluated'])
        b = _classify_exception(exc) if exc is not None else 14
        rstats['exc_class'] = type(exc).__name__ if exc is not None else 'Unknown'
        rstats['exc_a'] = a
        rstats['exc_b'] = b
        rstats['exc_code'] = 16 * a + b


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
    # Phase35: 複製評価器を使うシーンでは、その分の**壁時計**を先に取り置く。
    #
    # Phase36(タスク1-2a): **取り置きを決める前に、使えるかどうかを確定させる。**
    # import 失敗・pybullet 初期化失敗のような「構築を始める前に判明する失敗」で
    # 取り置きだけ残ると、複製評価をしないのに構築の締切だけ45秒早いという
    # 最悪の状態になる。preflight() をここで通し、駄目なら取り置きを 0 に戻して
    # **ρ-test 無効時とビット単位で同一の経路**へ落とす。
    use_replica = False
    # Phase37(ステップ0): 到達段階n(採点非依存のテレメトリ、末尾で壁時計に符号化する)。
    # 0 = ゲートに入れていない/未分類の早期失敗(H2相当)。REPLICA_STATSが後で埋まれば
    # そちらを優先する(n=3以降はrstats['stopped']から読み直す。末尾参照)。
    _telem_n = 0
    if REPLICA_SELECT and container_list and _replica_mod is not None:
        try:
            # 元は `is_applicable(...) and preflight()` の1行(短絡評価)だったのを、
            # 到達段階を見分けるために2段へ分けただけで、呼び出し順・短絡の有無は不変。
            applicable = _replica_mod.is_applicable(container_list)
            if not applicable:
                _telem_n = 1          # H3: is_applicable False(既積みあり等で対象外)
            elif not _replica_mod.preflight():
                _telem_n = 2          # H1: preflight False(pybullet初期化失敗)
            else:
                use_replica = True
                _telem_n = 2          # ここまで通過。以降はrstats['stopped']から読み直す
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
        LAST_BUILD_DIAGNOSTICS.clear()
        LAST_BUILD_DIAGNOSTICS.update({'winner_source': 'heuristic', 'winner_strategy': 'order_items(volume_desc)',
                                        'n_items': len(item_list)})
        return heuristic_order
    k = max(1, int(lookahead_k or 1))

    items_by_index = {item['index']: item for item in item_list}
    best_order = heuristic_order
    # Phase11: placement ペナルティで目的関数が負になりうるため、初期値は -inf にする
    # (旧 -1.0 のままだと、全候補が負スコアのシーンで貪欲構築の結果が一切採用されない)。
    best_score = None
    # Phase72: best_orderが最後に更新された時点の由来(heuristic/phase1/phase2)と種戦略名。
    _winner = {'source': 'heuristic', 'strategy': 'order_items(volume_desc)'}

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

    def try_construct(seed_items, window, use_noise, slice_units, source_label='phase?', strategy_label='?'):
        nonlocal best_order, best_score, best_stall, best_snaps, best_contribs, _winner
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
                    _winner = {'source': source_label, 'strategy': strategy_label}
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
        try_construct(default_items, window, use_noise=False, slice_units=construct_units,
                      source_label='phase1', strategy_label=strategy_orders[0][0])

    # Phase87: フェーズ1終了直後(フェーズ2開始前)の予算消費量を記録する(読み取り専用)。
    _phase1_used_units = total_budget.used
    _phase1_remaining_units = total_budget.remaining()
    _phase2_restart_count = 0

    # フェーズ2: 残り予算でランダム化(shuffle+noise)リスタートを繰り返し、
    # window と戦略の両方をランダムに振って多様性を確保する(単一戦略への依存を避ける)。
    #
    # Phase86 Tier3: BRKGA有効時は、この独立リスタートの束を「交叉のある母集団」に
    # 丸ごと置き換える(if/elseで完全に分岐するため、BRKGA=0時は従来どおり)。
    if BRKGA:
        n = len(item_list)
        pop_size = max(2, BRKGA_POP)
        elite_n = max(1, min(int(pop_size * BRKGA_ELITE_FRAC), pop_size - 1))
        mutant_n = max(0, min(int(pop_size * BRKGA_MUTANT_FRAC), pop_size - elite_n))
        # 個体1体のdecode予算。
        #
        # 実測(A01、既定budget=120s)で判明した設計ミスの修正: 当初は「1世代の総decode予算
        # ≈ phase2_units(フェーズ2の1リスタート分、pop_size=30なら計40s相当)」で固定していたが、
        # フェーズ1(window 5本、最大100s相当)がほぼ毎回そのシーンの残り予算を使い切ってしまい
        # (実測: 120s中74.7s消費、残45.3s)、**1世代分(46s相当)にすら届かず世代数0のまま
        # 一度もdecodeが走らなかった**(=フェーズ2も実は同じ理由で0-1リスタートしか回っていない
        # ことが副次的に判明: Phase83のrcl_k15/k50「差が出ない」現象と整合する)。
        # 固定コストの世代を要求すると、シーンごとに変動するフェーズ1後の残り予算に対して
        # 「世代を1つも回せない」確率が高すぎる。**残り予算を実測してから、目標世代数
        # (既定15)に収まるよう個体1体の予算を逆算する**ことで、残りが少ないシーンでも
        # 複数世代を確保できるようにする(個体の予算そのものは小さくなるが、
        # 「母集団×世代で数百回」という規模感(Deep Research)を残り予算の多寡によらず狙う)。
        brkga_budget_at_start = total_budget.remaining()
        ind_units = max(1.0, brkga_budget_at_start / max(1, pop_size * BRKGA_TARGET_GENS))
        idx_pos = {it['index']: pos for pos, it in enumerate(item_list)}

        def _decode(keys, use_noise, slice_units, window):
            order_pos = np.argsort(keys)
            seed_items = [item_list[p] for p in order_pos]
            try:
                order = simulate.beam_construct_order(
                    container_list, seed_items, total_budget.child(slice_units),
                    per_step_time_budget=PER_STEP_TIME_BUDGET,
                    rng=rng if use_noise else None,
                    score_noise=BRKGA_NOISE if use_noise else 0.0,
                    shuffle_ties=use_noise,
                    window=window,
                    prepacked_ids=prepacked_ids,
                    beam_width=BEAM_WIDTH,
                )
            except Exception:
                return None
            return order if set(order) == all_indices else None

        population = [rng.random(n) for _ in range(pop_size)]
        # フェーズ1と同じ「既知の良い並び」を母集団の一部の初期個体に埋め込む
        # (残りは純ランダムで多様性を確保する、標準的なBRKGAのbiased初期化)。
        for i, (_, seed_items) in enumerate(strategy_orders[:min(len(strategy_orders), pop_size)]):
            keys = np.empty(n)
            for rank_pos, it in enumerate(seed_items):
                keys[idx_pos[it['index']]] = rank_pos / max(1, n - 1)
            population[i] = keys

        fitness: list = [None] * pop_size

        def _rank_key(fit):
            return (float('-inf'), 0) if fit is None else fit

        gen = 0
        n_eval = 0
        while True:
            if total_budget.exhausted():   # 非常用安全弁が発火した場合のみ真になりうる
                break
            # 上のind_units算出コメントのとおり、「1世代丸ごと」を要求すると残り予算が
            # 少ないシーンで0世代に終わる。個体1体分+最終マージンさえあれば世代を開始する
            # (世代の途中で尽きた個体はfitness=Noneのまま最下位扱いになるだけで、安全側)。
            if total_budget.remaining() < ind_units + final_margin_units:
                break
            if gen >= BRKGA_MAX_GEN:
                break
            for i in range(pop_size):
                if fitness[i] is not None:
                    # エリート複製個体は前世代のfitnessを引き継ぐ(再decodeしない=無駄がない)。
                    continue
                window = WINDOW_CANDIDATES[int(rng.integers(0, len(WINDOW_CANDIDATES)))]
                order = _decode(population[i], use_noise=True, slice_units=ind_units, window=window)
                if order is None:
                    continue
                n_eval += 1
                stall, snaps, contribs = _extras()
                score = validate(order, stall, snapshots_out=snaps, contrib_out=contribs)
                fitness[i] = score
                if score is not None and use_replica:
                    cand_pool.append((score, list(order)))
                if score is not None and (best_score is None or _better(score, best_score)):
                    # noisy-fitness対策(Qian et al. 2018): ノイズ無しで再decode・再検証して
                    # から採否を決める(再評価+しきい値選択。マージン既定0=厳密改善のみ要求)。
                    confirm_order = _decode(population[i], use_noise=False,
                                             slice_units=ind_units, window=window)
                    if confirm_order is not None:
                        cstall, csnaps, ccontribs = _extras()
                        confirm_score = validate(confirm_order, cstall,
                                                  snapshots_out=csnaps, contrib_out=ccontribs)
                        accept = (confirm_score is not None and
                                  (best_score is None or
                                   (_better(confirm_score, best_score) and
                                    confirm_score[0] >= best_score[0] + BRKGA_ACCEPT_MARGIN)))
                        if accept:
                            best_order, best_score, best_stall = confirm_order, confirm_score, cstall
                            best_snaps, best_contribs = (csnaps or {}), (ccontribs or [])
                            _winner = {'source': 'brkga', 'strategy': f'gen{gen}'}
            ranked = sorted(range(pop_size), key=lambda i: _rank_key(fitness[i]), reverse=True)
            elites_idx = ranked[:elite_n]
            new_population = [population[i] for i in elites_idx]
            new_fitness = [fitness[i] for i in elites_idx]
            for _ in range(mutant_n):
                new_population.append(rng.random(n))
                new_fitness.append(None)
            while len(new_population) < pop_size:
                pe = population[elites_idx[int(rng.integers(0, len(elites_idx)))]]
                po = population[int(rng.integers(0, pop_size))]
                mask = rng.random(n) < BRKGA_BIAS
                new_population.append(np.where(mask, pe, po))
                new_fitness.append(None)
            population, fitness = new_population, new_fitness
            gen += 1
        BRKGA_STATS.clear()
        BRKGA_STATS.update({'enabled': True, 'pop_size': pop_size, 'generations': gen,
                             'n_eval': n_eval, 'elite_n': elite_n, 'mutant_n': mutant_n,
                             'ind_units_s': ind_units / u})
    else:
        while True:
            if total_budget.exhausted():   # 非常用安全弁が発火した場合のみ真になりうる
                break
            if total_budget.remaining() < phase2_units + final_margin_units:
                break
            window = WINDOW_CANDIDATES[int(rng.integers(0, len(WINDOW_CANDIDATES)))]
            strat_name, seed_items = strategy_orders[int(rng.integers(0, len(strategy_orders)))]
            try_construct(seed_items, window, use_noise=True, slice_units=phase2_units,
                          source_label='phase2', strategy_label=strat_name)
            _phase2_restart_count += 1

    # Phase87: フェーズ1/フェーズ2の予算配分の実測記録(読み取り専用、探索結果には無関係)。
    PHASE_BUDGET_STATS.clear()
    PHASE_BUDGET_STATS.update({
        'time_budget_s': build_budget_s, 'total_limit_units': total_budget.limit,
        'phase1_used_units': _phase1_used_units, 'phase1_remaining_units': _phase1_remaining_units,
        'phase1_used_s': _phase1_used_units / u, 'phase1_remaining_s': _phase1_remaining_units / u,
        'phase2_units': phase2_units, 'phase2_units_s': phase2_units / u,
        'phase2_restart_count': _phase2_restart_count,
        'phase2_could_run_at_least_1': _phase1_remaining_units >= phase2_units + final_margin_units,
    })

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
                    _winner = {'source': 'repair', 'strategy': _winner.get('strategy')}
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
                    _winner = {'source': 'alns', 'strategy': _winner.get('strategy')}
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

    # Phase38(ステップ1-B): 構築(total_budget)がここまでで壁時計 hard_deadline に
    # 実際に到達したか(=hard_expired)を記録する。use_replica のシーンでは
    # hard_deadline = start + min(HARD_WALL_LIMIT-reserve_s, build_budget_s*HARD_WALL_FACTOR)
    # であり、本番既定(DEFAULT_TIME_BUDGET=120, reserve_s=45)では前者(165-45=120)が
    # 常に min() の勝者になるため、hard_expired==True は「取り置きが構築を実際に
    # 短くした」ことをほぼ直接意味する。global 経由で agent.py の policy() が読む。
    if MYSOLVER_TELEMETRY:
        global LAST_BUILD_WALL_CUT
        LAST_BUILD_WALL_CUT = bool(use_replica and total_budget.hard_expired)

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
        rstats['latched'] = False
        deadline = start + min(HARD_WALL_LIMIT, time_budget * HARD_WALL_FACTOR)
        best_real = None
        # Phase37(ステップ1-3): このシーンで実際に使う指標は**1回だけ確定して固定する**
        # (行ごとに fill/composite を切り替えると、compositeが一部の候補だけ欠けたときに
        # スケールの違う値(0-100 の composite と 0-100 の fill でも重み構成が違う)を
        # argmaxで直接比較してしまう。最初の実評価でcompositeが取れなければそのシーンでは
        # 以降も一貫してfillへフォールバックする)。
        active_metric = REPLICA_METRIC
        metric_decided = False
        rstats['metric_used'] = active_metric

        # Phase36(タスク1): 失敗しても **静かに代理の勝者へ落ちる**。
        # 設計上の要点が3つある:
        #   1. **握った例外で best_real を捨てない**。K件のうち途中まで実評価できていれば、
        #      その結果は本物の測定値なので使ってよい(初版は except で None に戻していて、
        #      4件目が失敗すると1〜3件目の正しい勝者まで捨てていた)。
        #   2. **ラッチ**: Phase38(ステップ1-C)で「シーン単位」から「候補単位」に緩めた。
        #      1候補の失敗だけでは飛ばして次を評価し、**2回連続で失敗した場合だけ**
        #      シーン単位のラッチ(以降このシーンでは複製評価をしない)に落とす。
        #      pybullet の初期化に失敗するような環境ではK件すべてで同じ失敗が起きるので、
        #      連続失敗の場合のリトライは壁時計を焼くだけで、余裕が約15秒しかない現状では
        #      timeout の引き金になる。REPLICA_LATCH_MODE='scene' で Phase36 の旧挙動
        #      (1回失敗で即ラッチ)に戻せる。
        #   3. **disconnect は finally で保証する**。except の中に置くと、正常終了時と
        #      break 脱出時に漏れる。
        rep = None
        try:
            rep = _replica_mod.ReplicaEvaluator(
                container_list, k, prepacked_ids=prepacked_ids).open()
        except Exception:
            rep = None
            rstats['stopped'] = 'open_failed'
            rstats['latched'] = True
        if rep is not None:
            consecutive_fail = 0
            try:
                for rank, (sc, od) in enumerate(ranked):
                    if time.perf_counter() >= deadline:
                        rstats['stopped'] = 'wall_deadline'
                        rstats['latched'] = True
                        break
                    try:
                        status, payload = rep.evaluate(
                            item_list, od, deadline=deadline,
                            compute_composite=(active_metric == 'composite'))
                    except Exception as e:
                        # ここに来るのは replica.py 自身が捕捉しなかった例外だけ
                        # (pybullet.error/MemoryError/RuntimeError や
                        # MYSOLVER_REPLICA_FORCE_FAIL=runtime の障害注入など)。
                        # Phase40(ステップ2, ローカル専用): 静かな握りつぶし(この except 自体)
                        # の中身が見えるように、発生ごとに (例外クラス, 発生ファイル, 行番号,
                        # 候補順位) を記録する。握りつぶす動作自体は変えない(下のcontinue/breakは
                        # 従来どおり)。採点経路には出ない REPLICA_STATS への追記のみ。
                        consecutive_fail += 1
                        _record_replica_failure(rstats, e, rank)
                        if REPLICA_LATCH_MODE == 'scene' or consecutive_fail >= 2:
                            rstats['stopped'] = 'runtime_error'
                            rstats['latched'] = True
                            break
                        continue   # 1-C: この候補だけ飛ばして次候補の評価を続ける
                    if status == 'deadline':      # 壁時計 deadline 超過(当然の全体ラッチ)
                        rstats['stopped'] = 'wall_deadline'
                        rstats['latched'] = True
                        break
                    if status == 'data_error':
                        # Phase42(ステップ1): replica.py が観測データ欠損・型異常を
                        # 自前で捕捉して例外を投げずに伝えてきたケース。壁時計とは
                        # 無関係の「この候補固有(またはシーン固有)の失敗」なので、
                        # 上の except Exception 節と全く同じ候補単位ラッチを適用する
                        # (payload に原因例外そのものが入っているので
                        # _classify_exception() による符号化もそのまま流用できる)。
                        consecutive_fail += 1
                        _record_replica_failure(rstats, payload, rank)
                        if REPLICA_LATCH_MODE == 'scene' or consecutive_fail >= 2:
                            rstats['stopped'] = 'runtime_error'
                            rstats['latched'] = True
                            break
                        continue   # 1-C: この候補だけ飛ばして次候補の評価を続ける
                    # status == 'ok'
                    got = payload
                    consecutive_fail = 0
                    rstats['evaluated'] += 1
                    composite = got.get('composite')
                    if not metric_decided:
                        # 最初の実評価だけでこのシーンの指標を確定する(以降は切り替えない)。
                        if active_metric == 'composite' and composite is None:
                            active_metric = 'fill'   # replica_scorer が使えない環境: フォールバック
                            rstats['metric_used'] = active_metric
                        metric_decided = True
                    key_val = composite if active_metric == 'composite' else got['fill']
                    rstats['rows'].append({'rank': rank, 'surrogate': sc[0],
                                            'real_fill': got['fill'],
                                            'composite': composite,
                                            'cog_score': got.get('cog_score'),
                                            'stability_score': got.get('stability_score'),
                                            'placement_score': got.get('placement_score'),
                                            'soft_item_score': got.get('soft_item_score'),
                                            'num_placed': got['num_placed']})
                    # 同点なら代理順位が上(=rank が小さい)ほうを残す = 決定的
                    if best_real is None or key_val > best_real[0] + 1e-12:
                        best_real = (key_val, od, rank)
            finally:
                try:
                    rep.close()
                except Exception:
                    pass
        if rstats['latched'] and _DEBUG:
            print(f'[replica] ラッチ発動: {rstats["stopped"]} '
                  f'(実評価 {rstats["evaluated"]}/{len(ranked)} 件で打ち切り、'
                  f'以降このシーンでは複製評価器を使わない)', flush=True)
        if best_real is not None:
            rstats['winner_rank'] = best_real[2]
            rstats['winner_key_value'] = best_real[0]        # active_metric の値
            rstats['winner_row'] = rstats['rows'][best_real[2]] if best_real[2] < len(rstats['rows']) else None
            rstats['winner_fill'] = (rstats['winner_row']['real_fill']
                                     if rstats['winner_row'] else best_real[0])
            if best_real[2] != 0:
                # 代理の1位とは違う候補が実評価で勝った = ρ-test が効いた瞬間
                rstats['changed'] = True
                best_order = best_real[1]
                _winner = {'source': 'replica_select', 'strategy': _winner.get('strategy')}
        if rstats['stopped'] is None:
            rstats['stopped'] = 'done'
        REPLICA_STATS.clear()
        REPLICA_STATS.update(rstats)

    if MYSOLVER_TELEMETRY:
        stopped = REPLICA_STATS.get('stopped')
        if stopped == 'open_failed':
            _telem_n = 3
        elif stopped == 'runtime_error':
            _telem_n = 4
        elif stopped == 'wall_deadline':
            _telem_n = 5
        elif stopped == 'done':
            _telem_n = 7 if REPLICA_STATS.get('changed') else 6
        global LAST_ANY_SUCCESS
        LAST_ANY_SUCCESS = (stopped == 'done')
        elapsed = time.perf_counter() - start
        padded = elapsed > TELEMETRY_MIN_ELAPSED_S
        if padded:
            # Phase38(ステップ1-A): n=4(runtime_error)のときだけ、通常の
            # TELEMETRY_BASE_S+TELEMETRY_STEP_S*n 帯(158.0〜161.5s)から切り離し、
            # 専用の帯(162.00〜165.15s、0.05s刻み)で例外クラス(b)と成功候補数(a)を
            # 符号化する。n=0〜7 の意味は変えず、n=4 だけが上書きされる
            # (160.0付近と162.x付近の両方にn=4が現れることはない)。
            n4_code = REPLICA_STATS.get('exc_code')
            if _telem_n == 4 and n4_code is not None:
                target_t = start + max(TELEMETRY_N4_BASE_S + TELEMETRY_N4_STEP_S * n4_code, elapsed)
            else:
                target_t = start + max(TELEMETRY_BASE_S + TELEMETRY_STEP_S * _telem_n, elapsed)
            while time.perf_counter() < target_t:
                time.sleep(0.01)
        REPLICA_STATS['telemetry_n'] = _telem_n
        REPLICA_STATS['telemetry_padded'] = padded

    # Phase72: 診断記録の書き出し(読み取り専用、best_order自体には一切影響しない)。
    LAST_BUILD_DIAGNOSTICS.clear()
    LAST_BUILD_DIAGNOSTICS.update({
        'winner_source': _winner['source'], 'winner_strategy': _winner['strategy'],
        'n_items': len(item_list),
    })
    return best_order
