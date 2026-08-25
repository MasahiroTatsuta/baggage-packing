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
import functools
import os
import time

import numpy as np

from . import geometry as geo

MAX_POOL_ITEMS = 20
GRID_MARGIN = 0.02
# Phase8: 探索グリッドの細分化。_search_best の通常探索(1手ごとに毎回呼ぶ)で使う既定密度。
# 密度1(31x23グリッド)は粗く、荷物どうしの隙間にぴったり収まる細いXY位置をExtreme Point法
# だけでは拾いきれない場合がある。密度2は計測上、online呼び出し(pool<=MAX_POOL_ITEMS=20)
# では0.35s->0.9s程度への増加に留まり、policy_timeout(8s)・実際の呼び出し予算(5.5s)に
# 対して十分な余裕がある(探索は SearchBudget を自己チェックして安全に打ち切るため、万一
# 予算が足りない状況でもクラッシュや予算超過はしない)。
BASE_GRID_DENSITY = 2
# Phase7由来の「合法手0件時の最終リトライ」の密度。BASE_GRID_DENSITYを底上げしたことに
# 合わせて、通常探索よりさらに一段細かく最後の望みを探れるよう底上げする。
# Phase22: この値を上げても意味が無いことを実測で確認済み。既定 4 のまま据え置くこと。
#
# 行き詰まり状態から強い探索を回すと 26シーン合計 6.932 m^3(fill換算 +4.82pt)を追加配置
# できるため、当初これを「グリッド解像度の取りこぼし」と解釈して 4->8 を検証した。しかし
# 内訳を切り分けると解釈が誤りだった(results/phase22_report.md §3.2):
#   ・密度 4 のままでも同じ 4.018 m^3 に到達する(8/16 に上げても1立方センチも増えない)
#     = **解像度は律速ではない**
#   ・一方 pool を online と同じ lookahead_k 個に制限すると 4.018 -> 1.256 と 69% 減る
#     = 取りこぼしの正体は「どの荷物を置くか選べる自由度」= 順序側の問題
# 26シーンA/B でも 4->8 は fill_strict 23.74->23.45(改善0/悪化3/同値23)で不採用。
# 悪化3件は全て optimize 無効シーンで、密度を上げるとユニット消費が増え policy 予算内で
# 回れる(荷物×向き)の組合せが減り、密度4なら見つかった手を取り逃すため。
# MYSOLVER_RETRY_DENSITY は A/B 再現用のフックであり、既定値以外での運用は想定しない。
RETRY_GRID_DENSITY = int(os.environ.get('MYSOLVER_RETRY_DENSITY', '4'))
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
# ---------------------------------------------------------------------------
# Phase26: 壁積み(wall stacking)
# ---------------------------------------------------------------------------
# Phase9 の層規律(上記)は「奥側から level+1 個分の層だけを開放する」という構造そのもので、
# 分割数を増やせばそのまま「奥から手前へ壁を1枚ずつ積む」構築になる。実際 Phase9 は
# 固定4分割を試しているが**悪化して差し戻された**。README §課題表の記録によれば理由は
# 「小物が最も狭い層を埋めて逆効果」——つまり width/4 = 0.3625m の層には大きい荷物が
# そもそも入らず、貪欲が「その層に入る小物」だけを選んで奥を小物で埋め尽くし、
# 後続の大物の置き場所を潰したためである。
#
# 本フェーズはこの失敗要因を「壁の厚みが荷物サイズ分布と無関係な固定値だったこと」と
# 特定し、**厚みをプールの荷物サイズ分布から決める**ことで構造的に回避する。
# 荷物は6向きすべてが試されるので、「ある荷物が厚み T の壁に入りうる」条件は
#     min(length, width, height) <= T
# である(最小辺をy方向に向ければよい)。したがってプール内の最小辺の分布の
# WALL_QUANTILE 分位点を T に取れば、**プールの WALL_QUANTILE 割の荷物はどれも壁1枚に
# 収まる**ことが保証され、Phase9 の「大物が入らない層」は発生しない。
# 分割数は「厚みが T を下回らない」最大値、すなわち floor(width / T) とする
# (ceil にすると width/n < T となり保証が壊れる)。
#
# ---------------------------------------------------------------------------
# 【結果: 不採用】26シーンで fill_strict 24.413 -> 21.457(**-2.956**)、
# σ=6.580 / SE=1.290 / **t=-2.291**。採否基準 t>2 を**負の方向に**超えており、
# 「効果なし」ではなく統計的に有意な悪化である(results/phase26_report.md)。
#
# 失敗の内訳が2つの独立した知見を与えた:
#
#  (1) 実装上の欠陥: 壁の厚みを「その時点のプール」から計算していたため、offline の
#      貪欲構築で大物から消費されると残りプールの分位点が下がり、**終盤に壁が勝手に
#      薄くなって Phase9 の失敗モードが再発する**。初期壁枚数 n0 で層別すると
#      「壁積みが発動しないはずの n0=2 群」が最悪(-4.64, t=-2.27)という、欠陥を直接
#      指す結果になった。壁の厚みはシーンの属性でありプールの属性ではないので、
#      本来は初期の全荷物リストから一度だけ決めて固定すべきだった。
#  (2) ただし修正しても勝てる見込みは無い: 汚染を受けていない n0>=3 群も
#      -1.51(σ=5.99, t=-0.94)で符号は負。n0=2 群が完全に無変化になったとしても
#      26シーン平均は概算 -0.82 で符号は負のまま。
#
# 機序: fill_**loose** は +1.246 と増えており(A01 +23.11)、壁積みは実際により多くの
# 体積を詰め込んでいる。しかし loose-strict のギャップが 12.18 -> 16.38(+4.20)に開く
# ——増分がコンテナ境界から15mm以内に集中し、沈降後の厳マージン判定で計上から漏れる。
# 壁積みは荷物を壁面へ押し付ける方向に働くため inclusion_slack がほぼ0の配置が構造的に
# 増え、既存の boundary_term(fill_risk_factor * 1.5)では抑えきれなかった。
# なお緩レジームでも t=+0.729 と基準に遠く届かないため、**本番の内包判定レジームが
# 厳/緩どちらであっても不採用の結論は変わらない**。
#
# Y_SLICE 構造への介入は Phase9/13/14 に続き**4連敗**。今回は失敗要因(固定厚み)を
# 特定したうえで設計し直しても負けたので、この構造への介入は打ち切ること。
# ---------------------------------------------------------------------------
# 既定は無効(WALL_MODE=False)。無効時は n_y_slices=Y_SLICE_COUNT のままで、
# 追加の計算も一切行わないため Phase25a のアンカー構成とビット単位で同一の出力になる。
WALL_MODE = os.environ.get('MYSOLVER_WALL_MODE', '0') != '0'
# 壁の厚みに使う「プール内の最小辺」の分位点。1.0 にすると「全荷物が壁1枚に収まる」を
# 厳密に保証できるが、外れ値1個で厚みが決まってしまい実質 Y_SLICE_COUNT=2 に退化する
# (26シーンの実測で max(min辺) は 0.30〜0.68m、width=1.45m に対し floor は大半が2)。
WALL_QUANTILE = float(os.environ.get('MYSOLVER_WALL_Q', '0.9'))
# 分割数の上限。level ループは合法手が見つかった時点で break するが、コンテナが奥から
# 埋まりきると「level 0..k-1 を空振りしてから level k で見つける」ため、1手あたりの
# 評価コストが最悪 O(分割数) 倍になる。予算(SearchBudget)は決定的に消費されるので
# 安全性は損なわれないが、同じ名目予算で回れる組合せが減る(Phase22 §3.3 の失敗と同型)
# ため上限を設ける。
WALL_MAX_SLICES = int(os.environ.get('MYSOLVER_WALL_MAX', '6'))
# Phase14(ターゲット2): 「大棚のあるコンテナでは Y_SLICE の奥層が棚の占有域とほぼ一致するので
# 分割自体をやめる」案を shelf系7シーン×3回で実測したが、fill_strict 23.24->21.49 /
# fill_loose 32.24->29.43 と両レジームで明確に悪化した(run間stdは0.43/0.31なのでノイズではない)。
# 標的だった gen_shelf_patternA 自体も 25.23->21.34 と悪化しており、仮説「棚の領域を先に埋めさせる
# 規律が行き詰まりの原因」は棄却された。Y_SLICE 構造への介入は Phase9・Phase13 に続き3連敗。
# 詳細は results/phase14_report.md §5。
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
# Phase13(ターゲット2): B01_1c_40_plain の stability(94.20 < 97制約)回収。
#
# phase11 §5.2 で「union のしきい値を全シーン一律に締めるのは割に合わない」(B01 fill -10.51,
# P06 fill -6.11)ことは実測済みなので、しきい値自体は変えない。代わりに、
# 「offline optimize が無効(config.agent.optimize=False)」なシーンに限定して、より保守的な
# union支持しきい値を使う(strict_support、agent.py が init_states['optimize'] から判定して
# planner.plan に渡す)。この条件に該当するのは本スイートでは B01-B04・P04 の5シーンのみで、
# 他21シーンは一切影響を受けない(=fillへの副作用が構造的に他シーンへ波及しない)。
# 根拠: offline optimize が有効なシーン(パターンA等)は simulate.py の影シミュレータで
# 順序をあらかじめ検証してから本番実行するため、union支持による際どい積み上げも「その順序で
# 本当に置けるか」を事前にふるいにかけられる。一方 optimize=False(パターンB、lookahead=10)
# ではその事前検証が一切無く、際どい支持のまま積み上げが実行時に初めて試される。B01の
# stability低下がパターンB群で層別最大(98.57→97.56, phase11 §4.2)だったことと整合する仮説。
MIN_UNION_SUPPORT_RATIO_STRICT = 0.75
MIN_SUPPORT_SPAN_RATIO_STRICT = 0.75
MAX_SUPPORT_CENTROID_OFFSET_STRICT = 0.10
# Phase9: 層規律導入により、非優先荷物が優先荷物のすぐ側面に密接して置かれやすくなった結果、
# 揺れ試験(stability_score算出)時の沈み込み・傾きで優先荷物の上に非優先荷物が乗り上げてしまい
# placement_scoreを損なう実例が確認された(候補生成時は「優先荷物の上を着地面にしない」を
# 徹底しているが、真横に隙間なく置かれた場合の物理的な傾き・接触までは防げない)。
# 優先荷物のAABBの周囲に追加のクリアランスを設け、非優先荷物がそのすぐ側面に密着する候補
# そのものを作らないことで、置く順序や重み調整に頼らずハード制約として回避する。
PRIORITY_CLEARANCE_XY = 0.05
PRIORITY_CLEARANCE_Z = 0.05
# Phase14: 搬入経路の詰まり(fail_transport_y)対策 —— 「階段状スカイライン」の選好。
#
# 荷物は必ず手前(y=-width/2)から入り、直置き面(床・棚上面)の 0〜50mm 上に底面が来る候補は
# 最終高さのまま、それ以外は START_Z だけ浮上した高さで +y 方向へ掃引される
# (_evaluate_candidates の is_resting / sweep_z 参照)。したがって、ある候補が
# 「自分と同じXレーンにあり、かつ自分より奥にある既配置物・棚の“最も低い天面”」より高く
# 手前側に立つと、その低い天面の上に後から荷物を差し込む経路を物理的に塞ぐ。
# 逆に「手前にあるものほど天面が低い」階段状のプロファイルを保てば、奥の残容積への通路は
# 原理的に塞がれない。この超過量(m)に比例したペナルティを候補スコアに加える。
#
# 奥に何も無い場合(守るべき通路が存在しない)と、奥が既に天井付近まで埋まっている場合
# (通路を残しても何も入らない)はペナルティなし。
#
# 重みは決定的なパターンBシーン(B01-B04, P04)での掃引で決めた。不感帯なし版は重みに対して
# fill が非単調(W=2.5で19.46、W=3.0で23.21)にカオス的に振れ、全水準の平均が無変更と同じで
# 「効果ゼロ+攪拌」でしかなかった。不感帯を入れた版は滑らかな単峰になり、W∈{3,6,12}の全てで
# 両レジームのfillが無変更を上回る(W=6でstrict 22.33->24.38, loose 30.52->32.19)。
# 台地の中央にあたる 6.0 を採用する(W=24 で崖があるため上端には寄せない)。
CORRIDOR_WEIGHT = float(os.environ.get('MYSOLVER_CORRIDOR_W', '6.0'))

# Phase20(ターゲット2): 影シミュレータの fill 計上期待値を、配置目標点ではなく
# **沈降後の静止姿勢**の slack で評価するかどうか。
#
# 物理的にはこちらが正しい(本家 evaluator は settle_wait_step=300 の物理演算後の8角点を
# 判定するので、支持面から REST_CLEARANCE だけ浮いた目標点で測るのは評価する姿勢の誤り)。
# 実際、有効にすると較正は大幅に良くなる:
#   ・計上割合の予測誤差   +0.141 -> -0.049 (絶対値65%減)
#   ・fill の系統バイアス   +4.02pt -> -1.75pt
# **しかし順位相関(=build_orderが実際に使う情報)は改善しなかった**:
#   ・Spearman(native) 0.710 -> 0.690、改善したのは 8シーン中 2つだけ
#   ・26シーン: fill_strict +0.13(ノイズ内)/ fill_loose -1.00(悪化)/
#               stability 98.30->98.18 かつ suite_A01 が 98.41->96.35 で新規に制約違反
# 理由は results/phase20_report.md §3 に詳述: 代理誤差は「シーンごとの一定の下駄
# (較正誤差)」と「候補ごとのばらつき(弁別誤差)」に分解でき、本修正が消したのは前者
# だけだった(シーン内ばらつきは 0.075 -> 0.075 と不変)。build_order は同一シーン内の
# **順位**しか使わないため、全候補に共通の下駄は定義上どんな決定も変えない。
#
# よって既定は False(Phase19と数学的にも計算量的にも完全に等価な no-op)。
# 実装と計測基盤は次フェーズ(trust region + ρ-test)の土台として残す。
USE_SETTLED_SLACK = os.environ.get('MYSOLVER_SETTLED_SLACK', '0') != '0'
# 「奥にある」と判定する y の許容誤差。障害物の手前面が候補の奥面とほぼ一致する(隙間なく
# 密着して並んでいる)場合も「奥にある」とみなす。
CORRIDOR_Y_EPS = 0.02
# 奥の最低天面から天井までがこの高さ未満なら、そこにはもう何も入らないので通路を守らない。
CORRIDOR_MIN_HEADROOM = 0.10
# 「まだ通れる」高さ差の不感帯[m]。奥の天面 T_b の上に後から荷物を着地させる場合、その荷物は
# 底面が T_b + REST_CLEARANCE の位置を目標にし、直置き面でなければ更に START_Z だけ浮上した
# 高さで掃引される(_evaluate_candidates の sweep_z 参照)。掃引時に要求される z 方向の余裕が
# SWEEP_Z_MARGIN なので、手前の天面が T_b + (REST_CLEARANCE + START_Z - SWEEP_Z_MARGIN) までなら
# 経路は物理的に生きている。この範囲内の高さ差までペナルティを課すと、通路を全く塞いでいない
# 候補どうしの順位まで揺さぶることになり、貪欲の選択が無意味に攪拌される(Phase14 の初版が
# まさにこれで、重みに対して fill が非単調・カオス的に振れた)。
CORRIDOR_DEADBAND = float(os.environ.get(
    'MYSOLVER_CORRIDOR_DB', str(geo.REST_CLEARANCE + geo.START_Z - geo.SWEEP_Z_MARGIN)))

# Phase24(ターゲット2): 上の不感帯 0.0805 は「後から差し込む荷物が START_Z=80mm 浮上して
# 掃引される」ことを前提にした値だが、その前提が成り立たない天面が存在する。
#
# validator.check_transport_path は「底面が直置き面(床 / 棚上面)の 0〜50mm 上」なら
# effective_start_z=0(浮上なし)にする。奥の天面 T_b の上に着地する荷物の底面は
# T_b + REST_CLEARANCE なので、
#     T_b + REST_CLEARANCE ∈ [r_z, r_z + 0.05]   (r_z ∈ {thickness, height/2+thickness+buffer})
# を満たす天面 —— つまり **床そのもの・棚上面そのもの(および直上 34mm 以内)** の上に置く
# 荷物は浮上せず、最終高さのまま掃引される。この場合に通路が生きている条件は
#     手前の天面 <= T_b + (REST_CLEARANCE - SWEEP_Z_MARGIN) = T_b + 0.0005
# であり、実効的な不感帯はほぼゼロになる。現行実装は床レベル・棚上面レベルの通路に対しても
# 一律 80.5mm の猶予を与えており、**その帯の通路を塞ぐ候補にペナルティがかからない**。
#
# Phase24 の搬入経路監査(tools/phase24_corridor_audit.py)で、封鎖されている空間のうち
# この「直置き帯」が占めるシェアを実測したうえで、不感帯を天面の種類から一意に決める。
# どちらの値も REST_CLEARANCE / START_Z / SWEEP_Z_MARGIN の3定数から導かれるもので、
# 新しい調整パラメータではない(Phase14 の設計方針をそのまま帯域別に一般化しただけ)。
CORRIDOR_DEADBAND_REST = geo.REST_CLEARANCE - geo.SWEEP_Z_MARGIN
CORRIDOR_DEADBAND_LIFT = geo.REST_CLEARANCE + geo.START_Z - geo.SWEEP_Z_MARGIN
# 'uniform' = Phase23 までと完全に同一(全天面へ CORRIDOR_DEADBAND を適用)。
# 'surface' = 天面が直置き面かどうかで不感帯を切り替える。
CORRIDOR_DB_MODE = os.environ.get('MYSOLVER_CORRIDOR_DB_MODE', 'uniform')

# ---------------------------------------------------------------------------
# Phase17: 探索打ち切りの決定化(壁時計 -> 評価コスト(ユニット))
# ---------------------------------------------------------------------------
# Phase16 §2.3 で特定した通り、_search_best / _evaluate_candidates が
# time.perf_counter() を多重にチェックして打ち切っていたため、「どこまで候補を評価してから
# 諦めるか」が実行環境の速度・瞬間的なCPU負荷で変わっていた。これが
#   (a) 同一シーン・同一予算の反復間ノイズ(P02で fill_strict std ±5.14)
#   (b) シーン単位の optimize予算 非単調性(スプレッド最大13.4pt)
# の共通の根本原因だった。上位層(build_orderのリスタート選択)だけを決定化しても
# 解決しないことは Phase16 で実証済み。
#
# 対策: 打ち切り条件を「消費した評価コスト(ユニット)」で表現する。壁時計は
#   (1) 呼び出し元が秒で表現した予算をユニット数へ換算する較正定数 UNITS_PER_SEC
#   (2) 本番タイムアウトを絶対に踏まないための非常用の最終安全弁 (hard_deadline)
# の2つにのみ残す。(1)は定数なので、同一入力なら消費ユニット列は完全に同じになり、
# マシン速度に依らず同一の出力が得られる。(2)は通常発火しない(発火した場合のみ
# 決定性が失われるが、その場合でも制約遵守が優先される)。
#
# 1回の _evaluate_candidates のコストは、候補XY数 n_xy に対する numpy のベクトル演算を
# supports(pass1/pass2) と obstacles(最終位置・掃引2phase) の個数だけ繰り返す形なので、
#     units = n_xy * (n_supports + n_obstacles + COST_CONST)
# でよく近似できる。COST_CONST は n_sup/n_obs に依らない固定コスト(着地面計算・
# inclusion判定・スコアリング)を obstacles 何個分に相当するかで表した係数。
COST_CONST = 8.0
# 候補XY集合の構築(_candidate_xy: グリッド生成 + Extreme Point 列挙 + set/sort)のコスト。
# 評価そのものではないが (pool_idx, orn) ごとに1回走る無視できない固定費なので、
# 同じユニット系で計上する(生コスト = グリッド点数 + 障害物あたり8点の生成コスト)。
# tools/phase17_probe.py の実測では **候補集合の構築のほうが評価より重い**
# (plan()時間の内訳は _candidate_xy 58% / _evaluate_candidates 40%、
#  1回あたり 3.41ms vs 2.09ms、しかも呼び出し回数もほぼ同数 26420 vs 29994)。
# 係数は「両者が同じ units/sec になる」ように決める:
#   評価 1.63e7 units/s ÷ 候補構築 8.94e5 生units/s = 18.2
# Phase19(ターゲット3): ターゲット1(到達不能候補の早期枝刈り)・ターゲット2(候補構築の
# 増分キャッシュ)で候補構築側の生コストが大きく下がったため、Phase17時点の値(18.2)は
# 「評価側と候補構築側が同じunits/secになる」較正としては古くなっている(同じ手法で
# 測り直すと 評価1.211e7 units/s ÷ 候補構築1.401e6 生units/s = 8.64 になる)。
#
# **しかし 8.64 への変更は不採用にした**: 26シーンスイートでの過適合チェック(指示3)で
# `suite_C02_2c_55_shelfprio` の placement_score が 100→85.71 に落ちる回帰を検出した
# (Phase18 §3で同種の回帰が不採用の決め手になったのと同じ基準)。原因を切り分けたところ、
# `CANDIDATE_BUILD_COST` と `UNITS_PER_SEC` を**どちらか一方だけ**変更する分には
# C02 は placement=100 のまま(むしろ fill も 24.70→25.19/22.38 と改善側)だが、
# **両方を同時に変更したときだけ** 85.71 に落ちる「崖の組み合わせ」であることが分かった
# (`(UNITS_PER_SEC, CANDIDATE_BUILD_COST)` = (1.05e7,18.2)→100.00, (1.55e7,8.64)→100.00,
#  (1.05e7,8.64)→**85.71**。旧6シーン+B01-B04/P04では全パターンで差が出なかったため
# 旧6シーンだけの検証では発見できなかった)。Phase16以来くり返し観測されている
# 「探索パラメータは連続的なダイヤルではなく別の局所解へのスイッチとして作用する」現象
# (results/phase16_report.md〜phase18_report.md)がここでも再現している。
# 指示にあった再較正対象は `UNITS_PER_SEC` のみなので、`CANDIDATE_BUILD_COST` は
# Phase17の値のまま据え置く(不要な変更を追加してリスクを増やさない)。
CANDIDATE_BUILD_COST = 18.2
# 本環境(2コア)で実測した「1秒あたりに消化できるユニット数」。tools/phase17_probe.py で
# 6シーン分の (n_xy, n_sup, n_obs, 実時間) と _candidate_xy の実時間を収集し、
#     UNITS_PER_SEC = 総ユニット / plan() の総壁時計時間
# として求めた(候補構築等のオーバーヘッド込みで較正するため、分母は探索全体の時間)。
# この定数を「実機より小さめ(=保守的)」に置くと、同じ名目秒の予算に対して実所要時間が
# 短くなる方向にずれるだけで、決定性も制約遵守も損なわれない。逆に大きすぎると名目予算より
# 長く走り、非常用安全弁が発火して決定性を失う。
#
# Phase19(ターゲット3)で CANDIDATE_BUILD_COST=18.2(据え置き、上記)のまま測り直したところ、
# 旧6シーンの集計は 1.66e7 units/s(§3.2参照。COST_CONST=8固定、CANDIDATE_BUILD_COST=18.2
# 固定で total_costed_units/plan_time を再計算した値)だった。ターゲット1・2で
# 「同じ壁時計予算で消化できる評価回数」自体は増えている(=同じnominalユニット予算に対する
# 実所要時間が延びうる、§1.5参照)ため、Phase17時点の1.55e7のままでは名目秒あたりの
# 実所要時間が長くなる方向にずれる。Phase17と同じ「集計よりわずかに小さい値を採用する」
# 方針で 1.05e7 を採用する(CANDIDATE_BUILD_COSTは変えず、この定数だけを下げることで
# 名目秒と実秒の対応をPhase17と同水準に近づける較正)。
#
# Phase25a: 1.55e7(Phase17時点の値)への復帰を26シーンA/Bで再検証したが不採用にした。
# 26シーン平均は fill_strict 23.74->24.41(+0.68)と改善して見えたが、シーン別効果は
# σ=5.07・SE=0.99・t=0.68(採否基準t>2に対し大幅未達)で、符号が+10.74〜-9.73と
# シーンごとに激しく反転していた。安全弁(HARD_WALL_LIMIT=165s)は1.55e7でも発火せず
# (26シーン中 optimize 最大114.68s、うち最重量6シーンの2反復再測定でfill_strict std=0.0000
# =決定性は保たれる、results/phase25a_report.md)、決定性・タイムアウト面では1.55e7も
# 安全だが、この分散の大きさでは26シーンの平均改善が隠しテスト(別シーン集合)に
# 一般化する保証がない(Phase23 b=3の教訓と同型)。1.05e7を据え置き、以後の変更は
# この値をアンカーにdiffで評価する。
UNITS_PER_SEC = float(os.environ.get('MYSOLVER_UNITS_PER_SEC', '2.00e7'))
# 非常用安全弁の壁時計チェック間隔(exhausted() 呼び出し回数)。毎回 perf_counter() を
# 呼ぶとホットループのオーバーヘッドになるため間引く。決定性には影響しない
# (安全弁が発火しない限り結果に関与しないため)。
HARD_CHECK_INTERVAL = 64


class SearchBudget:
    """探索の打ち切りを『消費ユニット』で決める決定的な予算。

    limit:          このスコープで使えるユニット数(決定的)。
    hard_deadline:  非常用の最終安全弁(壁時計、絶対時刻)。None なら無し。
    parent:         親スコープ。spend は親にも伝播し、exhausted は親も見る
                    (1手の予算 ⊂ 1リスタートの予算 ⊂ optimize全体の予算 のような入れ子)。

    exhausted() は「これ以上新しい評価を始めない」判定。1回の評価が limit を多少
    超過することは許す(超過量は1回の _evaluate_candidates 分=高々数ms相当で有界)。
    """

    __slots__ = ('limit', 'used', 'hard_deadline', 'parent', '_probe', 'hard_expired')

    def __init__(self, limit: float, hard_deadline: float | None = None, parent=None):
        self.limit = float(limit)
        self.used = 0.0
        self.hard_deadline = hard_deadline
        self.parent = parent
        self._probe = 0
        self.hard_expired = False

    @classmethod
    def from_seconds(cls, seconds: float, hard_deadline: float | None = None, parent=None):
        return cls(seconds * UNITS_PER_SEC, hard_deadline=hard_deadline, parent=parent)

    def child(self, limit: float):
        """このスコープの残量を超えない子スコープを作る。"""
        return SearchBudget(min(limit, self.remaining()), parent=self)

    def child_seconds(self, seconds: float):
        return self.child(seconds * UNITS_PER_SEC)

    def remaining(self) -> float:
        r = self.limit - self.used
        if self.parent is not None:
            r = min(r, self.parent.remaining())
        return r

    def spend(self, units: float) -> None:
        self.used += units
        if self.parent is not None:
            self.parent.spend(units)

    def exhausted(self) -> bool:
        if self.used >= self.limit:
            return True
        if self.hard_expired:
            return True
        if self.hard_deadline is not None:
            self._probe += 1
            if self._probe >= HARD_CHECK_INTERVAL:
                self._probe = 0
                if time.perf_counter() > self.hard_deadline:
                    self.hard_expired = True
                    return True
        if self.parent is not None and self.parent.exhausted():
            return True
        return False


def _eval_units(n_xy: int, n_sup: int, n_obs: int) -> float:
    return n_xy * (n_sup + n_obs + COST_CONST)


def _unique_orientations(lwh):
    seen = {}
    for orn_idx in range(6):
        half = tuple(np.round(geo.half_extent(lwh, orn_idx), 5))
        if half not in seen:
            seen[half] = orn_idx
    return list(seen.values())


@functools.lru_cache(maxsize=64)
def _grid_point_frozenset(length: float, width: float, density: int) -> frozenset:
    """
    Phase19(ターゲット2): _candidate_xy が毎回組み立てていた
    「グリッド生成(np.meshgrid)+丸め+set化」は (length, width, density) だけで決まる
    純粋な計算(荷物・障害物には一切依存しない)にもかかわらず、(pool_idx, orn_idx) の
    キャッシュミスのたびに(=1ステップにつき最大 pool×orientation 回)フルスキャンで
    再構築されていた。値ベースでメモ化し、同じ (length, width, density) の組に対しては
    grid点集合そのもの(数値・丸め誤差込みで既存コードと完全に同一)を再利用する。
    online(agent.py)は毎ステップ container dict を再構築するが、コンテナの寸法自体は
    エピソード中不変なので、寸法値をキーにしたこのキャッシュは online/offline のどちらでも
    安全に効く(dict identityではなく値でキーイングしているため、container dict が
    毎回新しいオブジェクトでもヒットする)。
    """
    x_lo = -length / 2.0 + GRID_MARGIN
    x_hi = length / 2.0 - GRID_MARGIN
    y_lo = -width / 2.0 + GRID_MARGIN
    y_hi = width / 2.0 - GRID_MARGIN
    xs = np.linspace(x_lo, x_hi, 31 * density)
    ys = np.linspace(y_lo, y_hi, 23 * density)
    xx, yy = np.meshgrid(xs, ys, indexing='ij')
    return frozenset(zip(np.round(xx.ravel(), 5).tolist(), np.round(yy.ravel(), 5).tolist()))


def _rect_overlap_ratio_batch(cx, cy, hx, hy, ocx, ocy, ohx, ohy):
    ox = np.maximum(0.0, np.minimum(cx + hx, ocx + ohx) - np.maximum(cx - hx, ocx - ohx))
    oy = np.maximum(0.0, np.minimum(cy + hy, ocy + ohy) - np.maximum(cy - hy, ocy - ohy))
    area = (2 * hx) * (2 * hy)
    return (ox * oy) / max(area, 1e-9)


def _collect_obstacles(container):
    """衝突判定用(可否のみ、属性は問わない)の (center, half_ext) 一覧"""
    return geo.packed_obstacles(container) + geo.static_obstacles(container)


def _collect_corridor_obstacles(container, prepacked_ids):
    """Phase15(ターゲット1): corridor_penalty の min_top_behind 計算専用の障害物一覧。

    エピソード開始時から既に積まれていた荷物(prepacked_ids)を除外する。合法性判定
    (衝突・支持面)には一切影響しない(_collect_obstacles は変更しない、別関数)。
    棚などの静的構造物は常に含める(初期状態かどうかを問わず搬入経路を塞ぐ実体だから)。
    """
    pre_ids = prepacked_ids.get(container.get('index')) if prepacked_ids else None
    if not pre_ids:
        return _collect_obstacles(container)
    obstacles = []
    for item in container.get('packed_items', []):
        if item.get('pos') is None or item.get('orn') is None:
            continue
        if item.get('index') in pre_ids:
            continue
        obstacles.append(geo.item_world_aabb(item))
    obstacles += geo.static_obstacles(container)
    return obstacles


def _landing_supports(container):
    """
    着地面候補として使える (center, half_ext, is_prioritized, is_soft, is_shelf) 一覧。
    棚などの構造物は誰の上にも中立(is_prioritized=is_soft=False)として扱う。
    is_shelf=True は geo.static_obstacles(棚)由来であることを示すフラグ(Phase15
    ターゲット2)。_evaluate_candidates の着地面計算で「棚の下の床」と「棚の上」を
    独立した2つのコンパートメントとして扱うために使う。
    """
    # Phase19(ターゲット2): geo.packed_obstacles と同じ増分キャッシュを、荷物側の
    # (is_prioritized, is_soft) 込みの support タプルに対しても適用する(理由・安全性は
    # geo.packed_obstacles のdocstring参照。online/offlineの区別も同じロジックで成立する)。
    items = container.get('packed_items', [])
    cache = container.get('_landing_support_cache')
    if cache is not None and cache['src'] is items and cache['n'] <= len(items):
        supports = cache['list']
        start = cache['n']
    else:
        supports = []
        cache = {'src': items, 'n': 0, 'list': supports}
        container['_landing_support_cache'] = cache
        start = 0
    for item in items[start:]:
        if item.get('pos') is None or item.get('orn') is None:
            continue
        center, half = geo.item_world_aabb(item)
        supports.append((center, half, item.get('is_prioritized', False), item.get('is_soft', False), False))
    cache['n'] = len(items)
    supports = list(supports)
    # Phase15(ターゲット2)バグ修正: geo.static_obstacles() は全コンテナに常設される
    # small_shelf_aabb(脇の小さいledge)と、shelf=Trueのコンテナだけに存在する
    # big_shelf_aabb(大棚)の両方を返す。両者を一律 is_shelf=True にすると、棚を
    # 持たないコンテナでも small_shelf を「棚」とみなして下/上コンパートメント分割が
    # 誤発火し、無関係なシーン(例: B01)まで着地面計算が変わってしまう(実測で発覚)。
    # is_shelf=True は big_shelf_aabb 由来の場合だけに限定する。
    supports.append((*geo.small_shelf_aabb(container), False, False, False))
    if container.get('shelf', False):
        supports.append((*geo.big_shelf_aabb(container), False, False, True))
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


# Phase26(フォールバック版): 候補生成の「走査順」だけを壁の外側(奥)から内側(手前)へ変え、
# 選択ロジック(_evaluate_candidates の argmax)は現状維持する案の検証用フック。
# 'back_first' で候補配列を y 降順(奥が先)に並べ替える。既定 'default' は従来どおり
# (x,y) 昇順。§results/phase26_report.md §5 のとおり、この変更は数学的にほぼ no-op である
# (_evaluate_candidates は全候補をベクトル演算でスコアリングして argmax を取るため、
#  行の並び替えは「スコアが厳密に同値な候補が複数ある場合にどれが選ばれるか」しか変えない)。
CANDIDATE_ORDER = os.environ.get('MYSOLVER_CAND_ORDER', 'default')


def _candidate_xy(container, half, obstacles, grid_density: int = 1):
    grid_pts = _grid_point_frozenset(container['length'], container['width'], grid_density)
    pts = grid_pts | _extreme_points(container, half, obstacles)
    if not pts:
        return np.zeros((0, 2), dtype=np.float64)
    if CANDIDATE_ORDER == 'back_first':
        # 奥(y大)から手前へ、同一yでは |x| の小さい順(中央から外へ)
        return np.array(sorted(pts, key=lambda p: (-p[1], abs(p[0]), p[0])), dtype=np.float64)
    return np.array(sorted(pts), dtype=np.float64)


# ---------------------------------------------------------------------------
# Phase19(ターゲット1): 到達不能候補の早期枝刈り
# ---------------------------------------------------------------------------
# tools/diagnose_stall.py 相当の停止時失敗内訳計測で、_evaluate_candidates が棄却する
# 候補の大半が legal1(y方向搬入スイープの掃引衝突、_apply_obstacle_filters phase1)由来
# だった(fail_transport_y、gen_shelf_patternAで62.3%)。この判定は候補の着地高さ
# (world_z、ひいてはそこから決まる sweep_z)に依存するため、素朴には「着地面評価(landing_top
# の算出)より前」には判定できないように見える。
#
# しかし sweep_z は「landing_top(常に thickness 以上)+ half[2] + REST_CLEARANCE」に
# 0以上の浮上量を足し、最後に ceiling_sweep で頭打ちにする値なので、候補の (x,y) に一切
# 依存しない決定的な範囲 [SWEEP_Z_MIN, SWEEP_Z_MAX] に必ず収まる(下記 _y_sweep_range 参照)。
# したがって「その範囲全体が、XY方向で重なる既存障害物のz遮断区間(_apply_obstacle_filters
# の phase1 と同一の margin_xy=SAFETY_MARGIN_XY・margin_z=SWEEP_Z_MARGIN)で切れ目なく
# 覆われている」候補は、実際の着地高さが何であれ legal1=False が確定する。これは近似ではなく
# 厳密な必要条件の判定であり、後段の判定結果(=最終出力)を一切変えない
# (tools/phase17_dump.py の digest 一致で検証済み。§results/phase19_report.md T1)。
def _y_sweep_range(container, half):
    """候補の(x,y)によらない sweep_z の到達可能範囲 [z_min, z_max] を返す。

    z_min: landing_top>=thickness (常に成立) から、床置きが on_floor 相当の is_resting
           (effective_start=0) になるため達成できる真の最小値。
    z_max: _evaluate_candidates の sweep_z = min(ceiling_sweep, world_z+effective_start) が
           常に頭打ちにする ceiling_sweep(候補位置に依存しない)。
    z_max <= z_min の場合(この向きの荷物がそもそも高さ的に入り得ない)は None を返し、
    呼び出し元は枝刈りをスキップする(安全側: 判定を real evaluator に委ねる)。
    """
    thickness = container['thickness']; height = container['height']
    buffer = container.get('buffer', 0.0)
    z_min = thickness + half[2] + geo.REST_CLEARANCE
    z_max = height + buffer - thickness - half[2] - geo.START_MARGIN
    if z_max <= z_min:
        return None
    return z_min, z_max


def _y_sweep_unreachable_mask(container, half, candidate_xy, obstacles):
    """
    候補ごとに「到達可能などの sweep_z を選んでも legal1(y搬入スイープ)が必ず失敗する」かを
    厳密に判定する。obstacles は _evaluate_candidates の legal1/legal2 判定と同一の一覧
    (_collect_obstacles の戻り値)をそのまま渡すこと。

    アルゴリズム: 各障害物の z 遮断区間([obstacle z範囲] ± half[2] ± SWEEP_Z_MARGIN、
    [z_min,z_max] にクリップ)を求め、区間の下端でソートしてから候補ごとに「区間の合併が
    [z_min,z_max] を隙間なく覆うか」を古典的な線分被覆走査で判定する(候補ごとに異なるのは
    どの障害物がXYで重なる=有効かだけなので、有効フラグを numpy でベクトル化しつつ同じ
    走査順序を全候補で共有することで O(候補数 × 障害物数) に収める)。

    戻り値: shape (N,) bool。True = 「どんな着地高さでもY搬入不可能」と確定した候補。
    """
    n = candidate_xy.shape[0]
    if n == 0 or not obstacles:
        return np.zeros(n, dtype=bool)

    zr = _y_sweep_range(container, half)
    if zr is None:
        return np.zeros(n, dtype=bool)
    z_min, z_max = zr

    ox = container['center'][0]
    width = container['width']
    local_x = candidate_xy[:, 0]; local_y = candidate_xy[:, 1]

    x_min_local, x_max_local = geo.transport_x_bounds(container, half[0])
    x_min_local -= ox; x_max_local -= ox
    start_x_world = np.clip(local_x, x_min_local, x_max_local) + ox

    y_entry = -width / 2.0
    y1_lo = np.minimum(y_entry, local_y); y1_hi = np.maximum(y_entry, local_y)
    sx_lo = start_x_world - half[0]; sx_hi = start_x_world + half[0]
    sy_lo = y1_lo - half[1]; sy_hi = y1_hi + half[1]

    blocked = []
    for center, oh in obstacles:
        lo = max(z_min, center[2] - oh[2] - half[2] - geo.SWEEP_Z_MARGIN)
        hi = min(z_max, center[2] + oh[2] + half[2] + geo.SWEEP_Z_MARGIN)
        if hi <= lo:
            continue
        blocked.append((lo, hi, center, oh))
    if not blocked:
        return np.zeros(n, dtype=bool)
    blocked.sort(key=lambda t: t[0])

    eps = 1e-9
    covered_hi = np.full(n, z_min)
    gap_found = np.zeros(n, dtype=bool)
    for lo, hi, center, oh in blocked:
        if np.all(gap_found):
            break
        sep_x = (sx_hi + geo.SAFETY_MARGIN_XY <= center[0] - oh[0]) | (center[0] + oh[0] + geo.SAFETY_MARGIN_XY <= sx_lo)
        sep_y = (sy_hi + geo.SAFETY_MARGIN_XY <= center[1] - oh[1]) | (center[1] + oh[1] + geo.SAFETY_MARGIN_XY <= sy_lo)
        active = ~(sep_x | sep_y)
        pending = active & ~gap_found
        would_gap = pending & (lo > covered_hi + eps)
        gap_found = gap_found | would_gap
        extend = pending & ~would_gap
        covered_hi = np.where(extend, np.maximum(covered_hi, hi), covered_hi)
    return (~gap_found) & (covered_hi >= z_max - eps)


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


def _corridor_excess(container, half, world_x, world_y, world_z, obstacles):
    """
    Phase14: 候補の天面が「同一Xレーンで自分より奥にある既配置物・棚の最低天面」をどれだけ
    超えるか[m]を返す(超えないなら0)。奥に何も無い/奥が既に天井付近まで埋まっている
    候補は0(守るべき搬入経路が存在しない)。

    z座標系は _evaluate_candidates と同じ「コンテナ底面=0」のローカル系
    (landing_top を thickness と直接比較しているのと同じ系)。
    """
    n = world_x.shape[0]
    height = container['height']
    thickness = container['thickness']
    buffer = container.get('buffer', 0.0)
    ceiling_limit = height - thickness - geo.START_MARGIN
    headroom_cap = ceiling_limit - CORRIDOR_MIN_HEADROOM
    resting_values = (thickness, height / 2.0 + thickness + buffer)

    cand_back_face = world_y + half[1]
    cand_x_lo = world_x - half[0]
    cand_x_hi = world_x + half[0]

    # 障害物ごとに「その天面の上の通路が生きている手前側の天面の上限」= top + 不感帯 を求め、
    # 奥にある障害物すべてについての最小値を取る。不感帯が一定(uniform)なら
    # min(top + DB) = min(top) + DB なので Phase23 までと数学的に同一。
    min_limit = np.full(n, np.inf)
    for center, oh in obstacles:
        top = center[2] + oh[2]
        # そこにはもう何も入らない天面は守らない(旧実装は最小天面に対してのみ判定していたが、
        # 最小天面が閾値以下ならその天面が採用されるので、不感帯が一定なら結果は同一)。
        if top > headroom_cap:
            continue
        # 障害物の手前面が候補の奥面より奥にある(=候補が後からこの障害物へ向かう経路を塞ぐ側)
        behind = (center[1] - oh[1]) >= (cand_back_face - CORRIDOR_Y_EPS)
        if not np.any(behind):
            continue
        overlap_x = (cand_x_lo < center[0] + oh[0]) & (cand_x_hi > center[0] - oh[0])
        mask = behind & overlap_x
        if not np.any(mask):
            continue
        if CORRIDOR_DB_MODE == 'surface':
            # この天面の上に着地する荷物の底面は top + REST_CLEARANCE。それが直置き判定に
            # 入るなら浮上しない(実効不感帯 ≒ 0)、入らないなら START_Z だけ浮上できる。
            b = top + geo.REST_CLEARANCE
            on_rest = any(0.0 <= (b - rv) <= 0.05 for rv in resting_values)
            db = CORRIDOR_DEADBAND_REST if on_rest else CORRIDOR_DEADBAND_LIFT
        else:
            db = CORRIDOR_DEADBAND
        min_limit = np.where(mask, np.minimum(min_limit, top + db), min_limit)

    excess = np.maximum(0.0, (world_z + half[2]) - min_limit)
    return np.where(np.isfinite(min_limit), excess, 0.0)


def _score(container, local_x, local_y, world_z, half, item, support_ratio, contact_bonus, slack,
           corridor_excess=None):
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
    # Phase14: 階段状スカイライン(奥の搬入経路を塞がない)選好。詳細は CORRIDOR_WEIGHT 参照。
    corridor_penalty = 0.0 if corridor_excess is None else corridor_excess * CORRIDOR_WEIGHT
    return (z_term + back_term + edge_term + support_term + contact_term + prio_term
            - stability_penalty + cog_term + boundary_term - corridor_penalty)


def _evaluate_candidates(container, item, half, obstacles, supports, candidate_xy, budget, stats=None,
                          strict_support=False, corridor_obstacles=None):
    """
    候補XY一覧について、乗せられる一番高い支持面(landing z)を求め、内包・搬入経路衝突を
    チェックしたうえで最良の1候補を返す。合法な候補が無ければ None。

    budget: SearchBudget。Phase17 で壁時計 deadline から置き換えた決定的な打ち切り予算。
    「予算切れなら評価を始めない」判定だけを行い、始めた評価は必ず最後まで行う
    (途中で壁時計を見て中断することはしない = 同一入力なら同一の結果になる)。

    stats: 診断用(tools/diagnose_stall.py 専用)。None以外を渡すと、Noneを返す直前に
    「どの段階で全滅したか」をカウントする。本番のonline呼び出し(agent.policy)では
    常にNoneのままなので、このオプションは通常経路の速度・挙動に影響しない。
    """
    if budget.exhausted():
        return None
    budget.spend(_eval_units(candidate_xy.shape[0], len(supports), len(obstacles)))
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
    #
    # Phase15(ターゲット2): 棚(is_shelf=True)は「常に最優先の最高支持面」として扱うと、
    # 棚の下の床が永久に候補から消える(棚の直下は必ず「棚の上面に乗る」候補に化けてしまい、
    # 棚の下という別コンパートメントの床置きが一度も生成されない)。実測: gen_shelf_patternAで
    # 棚下領域(0.987m^3、全体の最大区画)の利用率が0.0%だった(results/phase15_report.md参照)。
    # 棚を「床(下のコンパートメント)」と「別の床(棚上面、上のコンパートメント)」の2つに
    # 分離し、非棚支持体(既配置荷物)は自分の上面が棚の下面以下なら下コンパートメント、
    # 棚の上面以上なら上コンパートメントに帰属させる。候補の荷物が下コンパートメントの
    # 着地高さのまま棚の下面をクリアできるなら床置き(下)を採用し、収まらない場合のみ
    # 棚の上(または棚上の既配置荷物)へ強制する。棚がそもそも無い(XY重なりなし)候補は
    # 従来どおり単一コンパートメントのまま。
    item_area = max(4.0 * half[0] * half[1], 1e-12)
    shelf_bottom = np.full(n, np.inf)
    shelf_top = np.full(n, -np.inf)
    shelf_touch_any = np.zeros(n, dtype=bool)
    cache = []
    touch_list = []
    for center, oh, sup_prioritized, sup_soft, is_shelf in supports:
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
        forbidden = (sup_prioritized and not item_is_prioritized) or (sup_soft and not item_is_soft)
        cache.append((top, touch, x_lo, x_hi, y_lo, y_hi, ow, oh_, forbidden))
        if is_shelf:
            bottom = center[2] - oh[2]
            shelf_bottom = np.where(touch, np.minimum(shelf_bottom, bottom), shelf_bottom)
            shelf_top = np.where(touch, np.maximum(shelf_top, top), shelf_top)
            shelf_touch_any |= touch
        else:
            touch_list.append((top, touch))

    landing_below = np.full(n, thickness)
    landing_above = np.where(shelf_touch_any, shelf_top, thickness)
    for top, touch in touch_list:
        is_below_case = touch & (top <= shelf_bottom + 1e-6)
        is_above_case = touch & shelf_touch_any & (top >= shelf_top - 1e-6)
        landing_below = np.where(is_below_case, np.maximum(landing_below, top), landing_below)
        landing_above = np.where(is_above_case, np.maximum(landing_above, top), landing_above)

    item_top_if_below = landing_below + geo.REST_CLEARANCE + 2.0 * half[2]
    fits_below_shelf = ~shelf_touch_any | (item_top_if_below <= shelf_bottom - geo.OBSTACLE_Z_MARGIN)
    landing_top = np.where(fits_below_shelf, landing_below, landing_above)

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

    union_ratio = MIN_UNION_SUPPORT_RATIO_STRICT if strict_support else MIN_UNION_SUPPORT_RATIO
    span_ratio = MIN_SUPPORT_SPAN_RATIO_STRICT if strict_support else MIN_SUPPORT_SPAN_RATIO
    centroid_offset = MAX_SUPPORT_CENTROID_OFFSET_STRICT if strict_support else MAX_SUPPORT_CENTROID_OFFSET

    safe_area = np.maximum(sum_area, 1e-12)
    off_x = np.abs(cen_x / safe_area - world_x) / max(half[0], 1e-9)
    off_y = np.abs(cen_y / safe_area - world_y) / max(half[1], 1e-9)
    span_ok = (((span_x_hi - span_x_lo) >= span_ratio * 2.0 * half[0]) &
               ((span_y_hi - span_y_lo) >= span_ratio * 2.0 * half[1]))
    balanced = span_ok & (off_x <= centroid_offset) & (off_y <= centroid_offset)
    stacked_ok = (sum_ratio >= MIN_SUPPORT_RATIO) | ((sum_ratio >= union_ratio) & balanced)
    support_ok = on_floor | (stacked_ok & ~forbidden_hit)
    landing_ratio = np.where(on_floor, 1.0, np.minimum(sum_ratio, 1.0))

    world_z = landing_top + half[2] + geo.REST_CLEARANCE
    ceiling_limit = height - thickness - geo.START_MARGIN
    valid_h = (world_z + half[2]) <= ceiling_limit
    world_pos = np.stack([world_x, world_y, world_z], axis=1)

    slack = geo.inclusion_slack_batch(container, half, world_pos)
    incl = slack <= geo.INCLUSION_MARGIN
    base_legal = incl & valid_h & support_ok

    # NOTE(Phase11): 配置後の物理演算で荷物は目標点(支持面から geo.REST_CLEARANCE=16mm 浮かせた
    # 点)から支持面まで必ず落ちる。本ローカル基盤の Evaluator は validator と同じ
    # inclusion_margin=-0.005 で静止後の8角点を判定するため、床直置きの荷物は
    # 「底面 = 内床面 -> dot≈0 > -0.005」で必ず fill 集計から脱落する
    # (実測: 既積み6個だけの初期状態の fill_score = 0.00)。この事実に合わせて risk 評価を
    # 「沈降後の姿勢」で行うと床置きの価値が 0 になり、探索は積み上げ一辺倒になる。
    # ただし README「評価指標」は『内包判定は検証時よりも緩く設定されている』と明記しており、
    # 本番基盤の evaluator 側 margin が正値(Evaluator の既定は +0.01)である可能性が高い。
    # その場合「床置きの価値0」は本ローカル基盤だけの人工物になるため、Phase11 では
    # 目標点の slack のまま(=Phase10 と同じ評価)に留める。詳細と検証案は
    # results/phase11_report.md の「床直置きと fill_score」節を参照。

    if not item_is_prioritized:
        min_final = world_pos - half[None, :]
        max_final = world_pos + half[None, :]
        for center, oh, sup_prioritized, _, _shelf in supports:
            if not sup_prioritized:
                continue
            too_close = geo.box_overlap_batch(min_final, max_final, center, oh,
                                               margin_xy=PRIORITY_CLEARANCE_XY, margin_z=PRIORITY_CLEARANCE_Z)
            base_legal = base_legal & ~too_close

    if not np.any(base_legal):
        if stats is not None:
            if not np.any(support_ok):
                stats['fail_support'] = stats.get('fail_support', 0) + 1
                # Phase66(ステップ1-1): support_ok=False の内訳(閾値未達 vs forbidden_hit、
                # 下敷き禁止ハード制約)を分けて数える診断カウンタ。support_ok/stacked_ok/
                # forbidden_hit の判定式自体はどこも変更していない(既に計算済みの配列を
                # 事後にnp.sum()で内訳集計するだけ)。on_floorはこの分岐内では全件Falseの
                # はずだが(True なら support_ok も True になり this 分岐に入らない)、
                # 念のため明示的に除外する。
                need_support = ~on_floor
                forbidden_only = need_support & forbidden_hit & stacked_ok
                threshold_only = need_support & ~forbidden_hit & ~stacked_ok
                both = need_support & forbidden_hit & ~stacked_ok
                stats['fail_support_forbidden_only'] = (
                    stats.get('fail_support_forbidden_only', 0) + int(np.sum(forbidden_only)))
                stats['fail_support_threshold_only'] = (
                    stats.get('fail_support_threshold_only', 0) + int(np.sum(threshold_only)))
                stats['fail_support_both'] = (
                    stats.get('fail_support_both', 0) + int(np.sum(both)))
            elif not np.any(incl):
                stats['fail_inclusion'] = stats.get('fail_inclusion', 0) + 1
            elif not np.any(valid_h):
                stats['fail_ceiling'] = stats.get('fail_ceiling', 0) + 1
            else:
                stats['fail_inclusion_and_ceiling'] = stats.get('fail_inclusion_and_ceiling', 0) + 1
        return None

    # 直置き面(床 or 棚上面)なら浮上なし、それ以外(荷物の上)は搬入時に少し浮かせてから下ろす。
    # validator.check_transport_path の判定式と完全に一致させること:
    #     for r_z in resting_surfaces: if 0 <= (bottom_z - r_z) <= 0.05: effective_start_z = 0.0
    # Phase11: 旧実装は landing_top が直置き面と 1mm 以内で一致する場合しか「直置き」と
    # みなしていなかった。本家は「底面が直置き面の 0〜50mm 上」なら直置き扱いなので、
    # 例えば背の低い荷物の上に積む(landing_top が床から数cm)候補について、
    #   本家: 浮上なし(=最終高さのまま奥へ掃引)
    #   旧実装: 浮上あり(+START_Z=80mm の高い経路を検証)
    # と食い違い、「実際には通らない低い経路」を検証せずに合法と誤判定していた。
    # 背の低い荷物ほど起きやすく、union支持面の導入で低い段積みが増えたことで顕在化した
    # (実測: D04(flat) は 7手目で搬入失敗しエピソード即終了、配置 12→6 個・fill 15.20→3.16)。
    bottom_z = world_z - half[2]
    resting_values = [thickness, height / 2.0 + thickness + buffer]
    is_resting = np.zeros(n, dtype=bool)
    for rv in resting_values:
        d = bottom_z - rv
        is_resting |= (d >= 0.0) & (d <= 0.05)

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
    corridor = (_corridor_excess(container, half, world_x, world_y, world_z,
                                  corridor_obstacles if corridor_obstacles is not None else obstacles)
                if CORRIDOR_WEIGHT > 0.0 else None)
    scores = _score(container, local_x, local_y, world_z, half, item, landing_ratio, contact, slack,
                    corridor_excess=corridor)
    scores = np.where(legal, scores, -np.inf)
    best_i = int(np.argmax(scores))
    if not legal[best_i]:
        return None

    # Phase20(ターゲット2、既定では無効): 沈降後の静止姿勢での slack。
    # 目標zは支持面から geo.REST_CLEARANCE(16mm)浮かせた点だが、配置後の物理演算で
    # 荷物は必ず支持面まで落ちるため、本家 evaluator が8角点を判定するのは沈降後の姿勢。
    # 床直置きなら 目標点: 床面dot=-0.016 -> fill_risk_factor=0.55 / 沈降後: dot=0 -> 0.00。
    # 較正は良くなるが順位を変えないため不採用(USE_SETTLED_SLACK のコメント参照)。
    #
    # コスト: 全候補に対して inclusion_slack_batch をもう1回走らせると Phase19 で削った
    # ホットパスのコストを戻してしまうため、**argmaxで選ばれた1点だけ**に対して計算する
    # (O(面数)。候補数に依存しない)。既定(無効)では計算自体を行わないので、
    # Phase19 と計算量的にも完全に等価。
    settled_slack = None
    if USE_SETTLED_SLACK:
        settled_pos = world_pos[best_i].copy()
        settled_pos[2] -= geo.REST_CLEARANCE
        settled_slack = float(geo.inclusion_slack_batch(container, half, settled_pos[None, :])[0])

    return {
        'score': float(scores[best_i]),
        'local_pos': np.array([local_x[best_i], local_y[best_i], world_z[best_i]], dtype=np.float32),
        'slack': float(slack[best_i]),
        'settled_slack': settled_slack,
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


def _wall_slice_count(container, pool_list, n_pool):
    """Phase26: プールの荷物サイズ分布から壁(yスライス)の枚数を決める。

    詳細な根拠は WALL_MODE 定義部のコメント参照。要点は
      T = quantile_{WALL_QUANTILE}( min(l,w,h) over pool )   … 壁1枚の目標厚み
      n = clamp(floor(width / T), Y_SLICE_COUNT, WALL_MAX_SLICES)
    で、floor を使うことで実際の厚み width/n >= T を保証する(=プールの
    WALL_QUANTILE 割の荷物は最小辺をy方向に向ければ壁1枚に収まる)。

    プールは配置が進むと縮む(offline は残り全件、online は lookahead ウィンドウ)ため
    n は手番ごとに変わりうるが、pool_list の内容だけから決まる純粋関数なので
    同一入力に対しては常に同一の値を返す(決定性は保たれる)。
    """
    if n_pool <= 0:
        return Y_SLICE_COUNT
    mins = sorted(min(it['length'], it['width'], it['height']) for it in pool_list[:n_pool])
    idx = min(len(mins) - 1, int(WALL_QUANTILE * len(mins)))
    thickness_target = mins[idx]
    if thickness_target <= 1e-6:
        return Y_SLICE_COUNT
    n = int(container['width'] / thickness_target)
    return max(Y_SLICE_COUNT, min(WALL_MAX_SLICES, n))


def _apply_y_slice_filter(candidate_xy, half_y, y_active_lo):
    """候補のうち、手前側の端(local_y - half_y)がy_active_lo以上(=開放層内)のものだけを残す。"""
    if candidate_xy.shape[0] == 0:
        return candidate_xy
    keep = (candidate_xy[:, 1] - half_y) >= (y_active_lo - Y_SLICE_EPS)
    return candidate_xy[keep]


def _search_best(container_list, pool_list, n_pool, budget, enforce_priority_container,
                  has_prioritized_container, rng=None, score_noise=0.0, stats=None,
                  grid_density: int = BASE_GRID_DENSITY, n_y_slices: int = Y_SLICE_COUNT,
                  reserve_priority_container: bool = False, strict_support: bool = False,
                  prepacked_ids: dict | None = None, top_k: int = 1):
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
    by_item_overall: dict = {}
    for container in container_list:
        if budget.exhausted():
            break
        container_is_prioritized = container.get('is_prioritized', False)
        obstacles = _collect_obstacles(container)
        corridor_obstacles = _collect_corridor_obstacles(container, prepacked_ids)
        supports = _landing_supports(container)
        # Phase26(壁積み): 呼び出し元が既定(Y_SLICE_COUNT)のままの場合に限り、荷物サイズ
        # 分布から決めた壁枚数へ差し替える。plan() の最終リトライなど明示的に別値を渡す
        # 経路には介入しない。WALL_MODE=False なら分岐そのものが no-op。
        n_slices = n_y_slices
        if WALL_MODE and n_y_slices == Y_SLICE_COUNT:
            n_slices = _wall_slice_count(container, pool_list, n_pool)
        y_bounds = _y_slice_bounds(container, n_slices)
        # (pool_idx, orn_idx) -> (half, 全域候補xy)。層のlevelを上げてもgrid/extreme point自体は
        # 変わらないため、y絞り込みだけをlevelごとにやり直せるようキャッシュして再計算を避ける。
        candidate_cache: dict = {}

        container_best = None
        # Phase23(ビームサーチ): top_k>1 のときだけ「荷物ごとの最良手」も控える。
        # 探索そのもの(_evaluate_candidates の呼び出し回数・rngの消費回数)は一切変えないので、
        # top_k==1 の既定経路は従来と完全に同一のまま。
        container_by_item = None
        for level_idx, y_active_lo in enumerate(y_bounds):
            if budget.exhausted():
                break
            # 最終levelは全開放(従来の全域探索と同値)。y絞り込みのマスク生成・コピーは
            # 候補配列サイズ分のコストがかかるため、無駄なオーバーヘッドを避けるため省略する
            # (n_y_slices<=1の場合は常にここに該当し、実質従来のplanner.pyと同じ速度になる)。
            is_fully_open = level_idx == len(y_bounds) - 1
            level_best = None
            level_by_item = {} if top_k > 1 else None
            for pool_idx in range(n_pool):
                if budget.exhausted():
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
                    if budget.exhausted():
                        break
                    cache_key = (pool_idx, orn_idx)
                    if cache_key not in candidate_cache:
                        half = geo.half_extent(lwh, orn_idx)
                        full_xy = _candidate_xy(container, half, obstacles, grid_density=grid_density)
                        # Phase19(ターゲット1): 着地面評価より前に、どの着地高さを選んでも
                        # y搬入スイープが必ず失敗する候補を厳密に間引く(出力不変、§planner.py
                        # _y_sweep_unreachable_mask のコメント参照)。以降のy_slice絞り込み・
                        # _evaluate_candidates はこの縮小済み配列に対して行われる。
                        if full_xy.shape[0] > 0:
                            unreachable = _y_sweep_unreachable_mask(container, half, full_xy, obstacles)
                            if np.any(unreachable):
                                full_xy = full_xy[~unreachable]
                        candidate_cache[cache_key] = (half, full_xy)
                        # 候補XY集合の構築コストも同じユニット系で計上する(評価そのものでは
                        # ないが (pool_idx, orn) ごとに1回走る無視できない固定費)。
                        budget.spend(CANDIDATE_BUILD_COST * (31 * 23 * grid_density * grid_density
                                                              + 8 * len(obstacles)))
                    half, full_xy = candidate_cache[cache_key]
                    candidate_xy = full_xy if is_fully_open else _apply_y_slice_filter(full_xy, half[1], y_active_lo)
                    r = _evaluate_candidates(container, item, half, obstacles, supports, candidate_xy, budget,
                                              stats=stats, strict_support=strict_support,
                                              corridor_obstacles=corridor_obstacles)
                    if r is None:
                        continue

                    score = r['score']
                    if rng is not None and score_noise > 0.0:
                        score = score + float(rng.normal(0.0, score_noise))

                    rank = (-tier, score)
                    if level_by_item is not None:
                        prev = level_by_item.get(pool_idx)
                        if prev is None or rank > prev['rank']:
                            level_by_item[pool_idx] = {
                                'rank': rank, 'score': score, 'local_pos': r['local_pos'],
                                'item_idx': pool_idx, 'container_idx': container['index'],
                                'orientation': orn_idx, 'slack': r['slack'],
                                'settled_slack': r['settled_slack'],
                            }
                    if level_best is None or rank > level_best['rank']:
                        level_best = {
                            'rank': rank,
                            'score': score,
                            'local_pos': r['local_pos'],
                            'item_idx': pool_idx,
                            'container_idx': container['index'],
                            'orientation': orn_idx,
                            'slack': r['slack'],
                            'settled_slack': r['settled_slack'],
                        }
            if level_best is not None:
                container_best = level_best
                container_by_item = level_by_item
                break  # このコンテナは現在開放中の層内で置けるため、より手前の層は開放しない

        if container_best is not None:
            if best_overall is None or container_best['rank'] > best_overall['rank']:
                best_overall = container_best
            if top_k > 1 and container_by_item:
                for pi, ent in container_by_item.items():
                    prev = by_item_overall.get(pi)
                    if prev is None or ent['rank'] > prev['rank']:
                        by_item_overall[pi] = ent
    if top_k > 1:
        # 荷物ごとに1手だけ残し、rank降順の上位 top_k を返す(ビームの展開候補)。
        return sorted(by_item_overall.values(), key=lambda e: e['rank'], reverse=True)[:top_k]
    return best_overall


def plan_topk(container_list: list[dict], pool_list: list[dict], top_k: int,
              budget: 'SearchBudget', max_pool_items: int | None = None,
              rng=None, score_noise: float = 0.0, strict_support: bool = False,
              prepacked_ids: dict | None = None) -> list[dict]:
    """Phase23(ビームサーチ): 荷物ごとに最良の1手を集め、上位 top_k を返す offline 専用API。

    plan() と同じ探索(_search_best)を1回走らせるだけで上位k手が得られる点が要点である。
    「上位k個の荷物選択を展開する」ために plan() を k 回呼ぶ実装にすると探索コストが k 倍に
    なり、Phase22 §3.3 で実証済みの失敗(1手あたりコストを増やすと予算内で回れる組合せが
    減って逆効果)を繰り返すことになる。_search_best は元々 (荷物×向き×位置) を総当たりして
    argmax を取っているので、「荷物ごとの最良」を控えるだけならコストはほぼゼロで済む。

    戻り値は rank 降順の action 辞書のリスト(空リストは合法手なし)。
    ビームのスコアリング用に 'slack'/'settled_slack'/'score' も含めて返す
    (env に渡す action 形式とは別物であり、offline の探索内でのみ使う)。
    """
    n_pool = len(pool_list) if max_pool_items is None else min(len(pool_list), max_pool_items)
    has_prio_c = any(c.get('is_prioritized', False) for c in container_list)
    has_plain_c = any(not c.get('is_prioritized', False) for c in container_list)

    res = _search_best(container_list, pool_list, n_pool, budget, enforce_priority_container=True,
                       has_prioritized_container=has_prio_c, rng=rng, score_noise=score_noise,
                       reserve_priority_container=has_prio_c and has_plain_c,
                       strict_support=strict_support, prepacked_ids=prepacked_ids, top_k=top_k)
    if not res and has_prio_c and not budget.exhausted():
        res = _search_best(container_list, pool_list, n_pool, budget, enforce_priority_container=False,
                           has_prioritized_container=has_prio_c, rng=rng, score_noise=score_noise,
                           strict_support=strict_support, prepacked_ids=prepacked_ids, top_k=top_k)
    if not res and not budget.exhausted():
        res = _search_best(container_list, pool_list, n_pool, budget, enforce_priority_container=False,
                           has_prioritized_container=has_prio_c, rng=rng, score_noise=score_noise,
                           grid_density=RETRY_GRID_DENSITY, prepacked_ids=prepacked_ids, top_k=top_k)
    # top_k==1 のとき _search_best は単一のdict(またはNone)を返すのでリストへ正規化する
    if isinstance(res, dict):
        res = [res]
    out = []
    for e in res or []:
        out.append({
            'item_idx': e['item_idx'],
            'container_idx': e['container_idx'],
            'place_pos': e['local_pos'].astype(np.float32),
            'orientation': e['orientation'],
            'slack': e['slack'],
            'settled_slack': e['settled_slack'],
            'score': e['score'],
        })
    return out


def plan(container_list: list[dict], pool_list: list[dict], time_budget: float = 5.5,
         max_pool_items: int | None = MAX_POOL_ITEMS, rng=None, score_noise: float = 0.0,
         stats=None, info: dict | None = None, strict_support: bool = False,
         prepacked_ids: dict | None = None, budget: 'SearchBudget | None' = None,
         hard_deadline: float | None = None) -> dict | None:
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
    strict_support: Phase13(ターゲット2)。True の間、union支持の判定をより保守的な
    しきい値(MIN_UNION_SUPPORT_RATIO_STRICT等)に切り替える。agent.py が
    「offline optimize 無効(=事前の順序検証が無い)」シーンでのみ True を渡す。
    prepacked_ids: Phase15(ターゲット1)。{container_index: frozenset(item_index)}。
    エピソード開始時から存在していた既積み荷物のindex集合(geo.initial_prepacked_ids)。
    corridor_penalty の min_top_behind 計算だけをこの集合を除外した障害物一覧で行う
    (合法性判定には一切影響しない)。省略時Noneは「全障害物を対象にする」旧挙動と同じ。
    budget: Phase17。呼び出し元(offlineの順序探索)が管理する SearchBudget をそのまま使う。
    省略時は time_budget を UNITS_PER_SEC で決定的にユニット換算した専用予算を作る
    (online の agent.policy はこちら)。
    hard_deadline: 非常用の最終安全弁(絶対時刻)。budget を省略した場合にのみ使う。
    online(policy)は policy_timeout(8s)を絶対に踏まないため必ず指定すること。
    """
    if budget is None:
        budget = SearchBudget.from_seconds(time_budget, hard_deadline=hard_deadline)

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
    best_overall = _search_best(container_list, pool_list, n_pool, budget, enforce_priority_container=True,
                                 has_prioritized_container=has_prioritized_container, rng=rng,
                                 score_noise=score_noise, stats=stats,
                                 reserve_priority_container=has_prioritized_container and has_plain_container,
                                 strict_support=strict_support, prepacked_ids=prepacked_ids)
    if best_overall is None and has_prioritized_container and not budget.exhausted():
        # 優先コンテナ限定では合法手が全く無かった場合のみ、非優先コンテナも含めて再探索する
        # (それ以上待っても優先コンテナに入らない荷物を無駄に足止めしないため)。
        best_overall = _search_best(container_list, pool_list, n_pool, budget, enforce_priority_container=False,
                                     has_prioritized_container=has_prioritized_container, rng=rng, score_noise=score_noise,
                                     stats=stats, strict_support=strict_support, prepacked_ids=prepacked_ids)

    if best_overall is None and not budget.exhausted():
        # Phase7: 通常密度のグリッド+Extreme Pointで合法候補が1つも無かった場合の最終リトライ。
        # 「本当に空間的行き詰まり」なのか「粗いグリッドが偶然、荷物が収まる細い隙間を
        # 拾えなかっただけ」なのかを区別せず一律諦めると、agent.py側は合法性チェックを
        # 一切行わない無検証フォールバックに落ちてsudden death(即座にエピソード終了、
        # 残り全荷物を失う)になる。同じ margin(=real validatorに対して安全側)のまま
        # グリッド密度だけを上げ、残り時間予算内で「本当に置ける場所が無いか」をもう一段
        # 丁寧に探す。予算(budget)は呼び出し元が渡したものをそのまま使い、
        # 新たに延長はしない(policy_timeout=8sに対する安全マージンを保つため)。
        # NOTE: この最終リトライは strict_support を渡さない(=常に緩い方)。ここに来るのは
        # 合法手が完全に0件だった場合であり、真の行き詰まり回避(sudden death防止)を
        # stability の保守性より優先する。
        best_overall = _search_best(container_list, pool_list, n_pool, budget, enforce_priority_container=False,
                                     has_prioritized_container=has_prioritized_container, rng=rng, score_noise=score_noise,
                                     stats=stats, grid_density=RETRY_GRID_DENSITY, prepacked_ids=prepacked_ids)

    if best_overall is None:
        return None

    if info is not None:
        info['slack'] = best_overall['slack']
        # Phase20: 沈降後の姿勢での slack。simulate.simulate_order の risk調整済み体積
        # (=offline順序探索の目的関数)がこちらを使う。_score() は従来どおり目標点の
        # slack を使うため、**online の候補選択は一切変わらない**(B01-B04/P04 の
        # digest 一致で検証済み)。
        info['settled_slack'] = best_overall['settled_slack']

    return {
        'item_idx': best_overall['item_idx'],
        'container_idx': best_overall['container_idx'],
        'place_pos': best_overall['local_pos'].astype(np.float32),
        'orientation': best_overall['orientation'],
    }
