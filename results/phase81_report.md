# Phase81 報告: Deep Research(EP/EMS・precedence graph)の知見から軌道を変える3施策を実装・検証

## ステップ0: 既存実装との突合(必須・最優先)

Deep Research報告(`docs/NEDO 3D Bin Packing Contest_ EP_EMS and Precedence Graph
Implementation Design for Construction Algorithm Improvement.pdf`、7ページ、pypdfで
全文抽出して読了)の TL;DR は EP/EMS 全面置換(計算高速化→試行回数増→配置数増)を
第一優先としていたが、指示文が正しく指摘する通り**この主効果の仮説は我々の実測と矛盾する**
(Phase64: 予算2,300万倍でも0/27件、Phase25b: 探索予算飽和済み)。本フェーズはEP/EMS全面
置換を行わず、報告内で「軌道を広く変える」性質を持つ3施策(a)(b)(c)のみを対象にした。

### (a) DFTRC(Gonçalves & Resende 2013)

**既存実装なし。ただし指示文が想定する実装場所(`ordering.py`の5番目の初期順序戦略)は
誤り。** DFTRCは原典(Gonçalves & Resende 2013、Parreño et al. 2008)・報告書の
両方で「配置済み後のEMS(空きスペース)集合からどこを選ぶか」という**配置規則**であり、
荷物の処理順序を決める**sequencing rule ではない**(報告書自身も「sequence→placement
decoderという2段構成」と明記しており、DFTRCはdecoder=placement側)。`ordering.py`の
`_strategy_*`はitem_list単体をソートする関数で、荷物の座標(=配置位置)を一切持たない
ため、「コンテナの最大座標隅からの距離」を計算する材料がそもそも無い。これは
**Phase61のX方向掃引・Phase78のhint_soft/resolved_priorityと同型の「指示書の想定と
コード構造の不一致」**であり、Step0の趣旨(実装前に必ず確認)どおり、字義通りの実装を
やめ、DFTRCの本来の置き場所である`planner._score()`(候補位置 local_x/local_y/world_z を
既に持つ)へ、既存項に対する**追加のタイブレーク項**として実装した(詳細はステップ1)。

- `ordering.py`の4戦略・`planner._better`・risk_volの選好規則: 「隅からの距離」に相当する
  項は無い。`planner._score()`の`back_term`(y大=奥を優先、重み5.85、コンテナ隅への距離とは
  軸も方向も異なる目的=搬入経路を自ら塞がないための選好)、`edge_term`(|x|大を弱く優先、
  重み0.3)はいずれもDFTRCの代替にならない。**未実装と確認。**

### (b) moving extreme points(Heßler et al. 2024、cut_x/cut_y斜面対応)

**既存実装なし。実装の必要性を裏付ける既存コード上の証拠あり。** `geometry.py`の
実際のコンテナ形状は、`src/ground_handling/utils.py::write_open_cut_corner_cup_obj`が
生成する五角形断面(1端の床付近が斜めに切り欠かれた、公式ドキュメント記載どおりの
AKE/AKN型)であり、実測(`suite_A01`のcontainer dict)で`n_vecs`/`points`の面インデックス4に
`n=[-0.673, ~0, -0.740]`という非軸並行の法線を確認した(=斜め面は実在し、幅(y)方向には
一様に押し出されているためy成分はほぼ0)。**この斜め面自体は`check_inclusion_batch`の
内包判定では正しく使われている**(全面の中の1つとして扱われる)。しかし**候補点生成
(`_grid_point_frozenset`の軸並行グリッド、`_extreme_points`の軸並行障害物コーナー)は
斜め面を一切参照しない**——斜面ぎりぎりの位置は、粗いグリッド(≈30mm間隔)がたまたま
近傍の点を作らない限り候補にすら上がらない。`geometry.py`冒頭の既存コメント
(「-0.016付近から特定コンテナ形状(cut corner付近)で候補が急減する崖がある」)は、
本施策が狙う空間そのものが既に「崖」として記録されていたことを示す。**未実装と確認、
かつ既存コード内に間接的な裏付けあり。**

### (c) RCL/reactive GRASP(Parreño et al. 2008)

**部分的に既存実装あり(shuffle_ties)だが、性質は指示文の想定と異なる。**
`simulate.beam_construct_order`/`greedy_construct_order`の`shuffle_ties`は、
Phase71-72で確認済みのとおり「名前に反して`remaining`全体を`rng.shuffle()`で
一様ランダムに完全シャッフルする」処理であり、GRASPのRCL(上位候補からのランダム選択)
とは異なる一様分布である。この一様シャッフルが`ordering.build_order`の**フェーズ2
(残り予算でのランダム化リスタート)**で使われており、Phase72の実測では**28シーン中
0件しかフェーズ2由来のbest_orderが採用されなかった**(=このランダムリスタート予算は
実質的に無駄になっている可能性が高い、という指示文の指摘は正しい)。**「絞り込み
(RCL的な上位バイアス)は入っていない」という指示文の判定は正しく確認できた。
一様シャッフルをRCL方式に置き換える実装(ステップ1)を実施した。**

---

## ステップ1: 実装(3項目、いずれも env フラグ・既定無効・追加のみ)

### (1-a) `MYSOLVER_DFTRC_STRATEGY`(`agents/mysolver/planner.py`)

```python
DFTRC_STRATEGY = os.environ.get('MYSOLVER_DFTRC_STRATEGY', '0') == '1'
DFTRC_WEIGHT = float(os.environ.get('MYSOLVER_DFTRC_WEIGHT', '1.0'))
```

`_score()`の返り値に、コンテナのローカル座標系での最大隅(x=length/2, y=width/2,
z=height)からの正規化ユークリッド距離 × `DFTRC_WEIGHT`を**加算のみ**で追加した
(既定`DFTRC_STRATEGY=False`のとき`dftrc_term=0.0`、既存の重みは無変更)。
ステップ0で確認したとおり`back_term`(y大を優先)と一部軸で競合しうるが、字義通りの
規則をそのまま追加し、既存項に合わせて弱めることはしていない(本番A/Bで判定する対象)。

### (1-b) `MYSOLVER_CUTCORNER_CANDIDATES`(`agents/mysolver/geometry.py` + `planner.py`)

`geometry.py`に2関数を追加:
- `diagonal_face_indices(container)`: `n_vecs`のy成分がほぼ0・x/z成分がどちらも非軸並行
  (0でも±1でもない)面を機械的に検出する(特定のcut_x/cut_y値に依存しない、どの
  コンテナ構成でも同じ判定で拾える)。
- `cutcorner_boundary_x(container, half, world_z, face_idx, margin)`: `inclusion_slack_batch`
  と同じ内包式(`n·(pos−p)+Σ|n_i|half_i`)を、斜め面1枚についてworld_xについて解析的に
  解く(y成分に依存する特殊形状は安全弁でNoneを返す)。

`planner.py`に`_cutcorner_candidates(container, half)`を追加: 床置き
(`world_z = thickness + half[2] + REST_CLEARANCE`)を仮定し、`cutcorner_boundary_x`で
求めた斜面ぎりぎりのxと、コンテナ内の複数y位置(既定5点)を組にした候補点を返す。
`_candidate_xy`で`CUTCORNER_CANDIDATES`有効時のみ既存の`grid_pts | _extreme_points(...)`に
和で追加する(既存のgrid/extreme point生成は無変更)。

### (1-c) `MYSOLVER_RCL_SHUFFLE`(`agents/mysolver/simulate.py`)

```python
_RCL_SHUFFLE = os.environ.get('MYSOLVER_RCL_SHUFFLE', '0') == '1'
_RCL_FRACTION = float(os.environ.get('MYSOLVER_RCL_FRACTION', '0.3'))
```

`_rcl_shuffle_keys(remaining, rng)`: 体積降順でランクづけした残り荷物のうち、上位
`RCL_FRACTION`(既定30%、最低1件)からランダムに1件選んで確定させる、を残り0件まで
繰り返す(GRASP標準構成)。`beam_construct_order`・`greedy_construct_order`両方の
`shuffle_ties`分岐に同一の切り替えを実装した(指示文は`beam_construct_order`のみ
明記していたが、両関数は「`beam_width=1`かつ`top_k=1`は`greedy_construct_order`と
完全一致する」という設計上の等価性を持つため、片方だけ変えると既存のこの等価性検証が
壊れる。一貫性のため両方に適用)。

---

## ステップ2: 検証

### (2-1) 決定的8シーンのビット単位不変(8/8)

`scripts/bp_check.sh`(3フラグとも既定無効のまま):

```
[B01] ... OK(基準値と一致)
[B02] ... OK(基準値と一致)
[B03] ... OK(基準値と一致)
[B04] ... OK(基準値と一致)
[P04] ... OK(基準値と一致)
[A01] ... OK(基準値と一致)
[A02] ... OK(基準値と一致)
[A03] ... OK(基準値と一致)
```

**8/8確認。** `DFTRC_STRATEGY`は`dftrc_term=0.0`を加算するだけ(`_score()`の他の項は
無変更)、`CUTCORNER_CANDIDATES`は`if CUTCORNER_CANDIDATES:`で分岐自体に入らず
`_cutcorner_candidates`を一度も呼ばない、`RCL_SHUFFLE`は`rng.shuffle`の代わりの分岐に
入らないため、いずれも既定時は既存コードパスと完全に同一。

### (2-2) 各フラグ有効時の「壊れていないこと」確認

`tools/measure_regime.py`(`--optimize-budget 15`、緩2閾値を土台に14シーン、B01-04・P04・
A01-03の決定的8シーン+A05・C01・D01・D02・P01・P06)で baseline/dftrc/cutcorner/rcl の
4パスを実行した(`results/phase81_step2_*.json`)。

| パス | 完走 | 例外 | 配置数(14シーン合計) | 配置率 | policy_time max |
|---|---:|---:|---:|---:|---:|
| baseline(緩2) | 14/14 | 0 | 302/724 | 41.71% | 1.88s |
| dftrc | 14/14 | 0 | 295/724 | 40.75%(−0.96pp) | 1.90s |
| cutcorner | 14/14 | 0 | 297/724 | 41.02%(−0.69pp) | 2.16s |
| rcl | 14/14 | 0 | 302/724 | 41.71%(±0pp、baselineと完全一致) | 1.86s |

**全パスで14/14完走・例外0件。配置率の低下はいずれも−5%基準を大きく下回り(最大でも
−0.96pp)、壊れていない。**

`rcl`がbaselineと完全に一致した理由: `--optimize-budget 15`という短い予算では
`build_order`のフェーズ1(window幅を変えた決定的な貪欲構築、`WINDOW_CANDIDATES=[15,20,25,
30,None]`を順に試す)だけで予算を使い切り、フェーズ2(ランダム化リスタート、RCLが効く場所)
に一度も到達しない。これは新しい判定式ではなく単に「この短縮予算ではRCLが試される機会が
無かった」ことを意味する(RCL自体が壊れているかどうかの判定には影響しない――
フェーズ2に到達すること自体は本番の`DEFAULT_TIME_BUDGET`(120s)相当の予算で保証されている)。

#### (1-b)専用の追加検証: cutcorner候補追加によるpolicy時間への影響

指示書の懸念(「現状policy maxは本番実測で5.9〜6.0s、余裕は2s程度」)に対応するため、
より大きい・より要求の厳しい2シーン(B04: 2コンテナ80個、C03: 2コンテナ80個・優先荷物あり)
に絞り、`--optimize-budget 60`でbaseline/cutcornerを直接比較した:

| | baseline(緩2) | cutcorner |
|---|---:|---:|
| policy_time max | 1.75s | 1.69s |
| 配置数(B04+C03) | 45+28=73 | 45+28=73(完全一致) |

**この2シーンでは候補追加によるpolicy時間の悪化は観測されなかった(むしろ僅かに短い、
誤差範囲)。配置数もbaselineと完全一致(この2シーンでは斜め切り欠き候補が採用されな
かった)。** 8秒制限に対する余裕は本フォールで確保されている。ただし本測定はローカル
(実機・実データではない生成シーン)かつ2シーンのみであり、本番の「5.9〜6.0s」という
実測値に直接匹敵する条件(本番相当の壁時計圧迫・全シーン)を再現したものではない点は
留保する。

### (2-3) `(1-c)` RCLのphase2勝率の変化

`tools/phase72_winner_trace.py`(`MYSOLVER_HARD_WALL_LIMIT=3000`、既定閾値0.55/0.6/0.15、
Phase72と同一条件)を`MYSOLVER_RCL_SHUFFLE=1`のみ追加して26シーンで実行した
(`results/phase81_rcl_winner_trace.json`)。

| winner_source | 件数(Phase72、既定) | 件数(Phase81、RCL有効) |
|---|---:|---:|
| heuristic | 12 | 10 |
| phase1 | 16 | 16 |
| **phase2** | **0** | **0** |

**phase2由来のbest_orderは、RCL化後も0/26のまま変化しなかった。** Phase72の0/28
(sample_configの2タスクを含む28シーン中)から、本測定(sample_config抜きの26シーン)でも
0/26で不変。ランダムリスタート方式を一様シャッフルからRCL(上位30%からのランダム選択)に
変えても、フェーズ1の決定的な窓幅スイープに一度も勝てていない。

**解釈**: (2-2)の`--optimize-budget 15`での観測(フェーズ2に予算が全く回っていない)と
整合する形で、フェーズ2自体が「使われてすらいない」可能性と、「使われてはいるが
質が低く勝てない」可能性の両方が考えられるが、本測定(`MYSOLVER_HARD_WALL_LIMIT=3000`・
既定の`DEFAULT_TIME_BUDGET`)ではフェーズ2に十分な予算があるはずであり(Phase72が
同一条件で「フェーズ2は動いている」ことを前提に0/28を報告している)、後者(質の問題)が
主因である可能性が高い。**RCLへの変更だけでは、シャッフル方式に依存しない別の構造的な
理由(フェーズ1の決定的な窓幅スイープが単に強すぎる、あるいはフェーズ2のper_step_time_
budgetが不利など)がフェーズ2を勝たせていないと考えられる。** ローカルでの効果は
確認できなかったが、指示書のとおり採否はここで決めず、本番で判定する。

---

## ステップ3: zip化

すべて緩2の閾値(`MIN_UNION_SUPPORT_RATIO`=0.35 / `MIN_SUPPORT_SPAN_RATIO`=0.4 /
`MAX_SUPPORT_CENTROID_OFFSET`=0.25)+ 幾何定数は既定(`INCLUSION_MARGIN`=−0.012 /
`SAFETY_MARGIN_XY`=0.022 / `REST_CLEARANCE`=0.016)を土台に、該当フラグ1つだけを
`'1'`に固定して作成した(1 zip = 1フラグ、組み合わせzipは作成していない――
(2-3)の結果(RCLがローカルで無効)を踏まえ、単独3項目とも本番でまず単独の符号を
確認すべきと判断し、組み合わせは見送った)。

| zip | 有効フラグ | SHA256 |
|---|---|---|
| `mysolver_submit_dftrc.zip` | `MYSOLVER_DFTRC_STRATEGY=1` | `03eab3fa7ad1586d5952d42811ad41fec21be839d6d0afe99b5f8e6ae0f5a016` |
| `mysolver_submit_cutcorner.zip` | `MYSOLVER_CUTCORNER_CANDIDATES=1` | `c46bcd934c393a72db785b4530601dffa8c182ba8f64e3a96f8f526b2cd0f8ef` |
| `mysolver_submit_rcl.zip` | `MYSOLVER_RCL_SHUFFLE=1` | `eeddaa7bf1e9b76cb49e90d7640d58ea46d11c3f9a715a56479bdb85f5f71966` |

**アップロード時は必ず上記SHA256と照合すること。**

### 全定数grep(3zip共通の確認結果)

3zipとも以下16項目を`unzip`後のソースから直接grepして確認した(各zipのフラグ列は
該当するもの1つだけが`'1'`、残り15項目は指示どおりの既定値):

```
MYSOLVER_TELEMETRY                  = '0'    (共通)
MYSOLVER_HARD_WALL_LIMIT             = '165.0' (共通)
MYSOLVER_REPLICA_SELECT              = '0'    (共通)
MYSOLVER_REPLICA_METRIC              = 'fill' (共通)
MYSOLVER_FALLBACK_SAFE_POS           = '1'    (共通)
MYSOLVER_FALLBACK_AVOID_OBSTACLES    = '1'    (共通)
MYSOLVER_STRICT_SUPPORT_DISABLE      = '0'    (共通)
MYSOLVER_MIN_UNION_SUPPORT_RATIO     = '0.35' (共通、緩2)
MYSOLVER_MIN_SUPPORT_SPAN_RATIO      = '0.4'  (共通、緩2)
MYSOLVER_MAX_SUPPORT_CENTROID_OFFSET = '0.25' (共通、緩2)
MYSOLVER_INCLUSION_MARGIN            = '-0.012' (共通、既定)
MYSOLVER_SAFETY_MARGIN_XY            = '0.022'  (共通、既定)
MYSOLVER_REST_CLEARANCE              = '0.016'  (共通、既定)
MYSOLVER_DFTRC_STRATEGY              = dftrc.zipのみ '1'、他2zipは '0'
MYSOLVER_CUTCORNER_CANDIDATES        = cutcorner.zipのみ '1'、他2zipは '0'
MYSOLVER_RCL_SHUFFLE                 = rcl.zipのみ '1'、他2zipは '0'
```

加えて`MYSOLVER_BEAM_SOFT_LAST`(Phase78/80で不採用確定)・`MYSOLVER_ALNS`・
`MYSOLVER_REPAIR`・`MYSOLVER_WALL_MODE`が3zipとも既定のまま(それぞれ'0')であることも
確認した(意図しない機能の混入が無いことの追加確認)。

### zipとリポジトリの差分ファイル確認

`diff -rq`(zip展開先 vs `agents/mysolver/`)で、意図した1ファイルのみが差分:

- `mysolver_submit_dftrc.zip`: `planner.py`のみ差分(union/span/centroidの3行+
  `DFTRC_STRATEGY`の1行)。
- `mysolver_submit_cutcorner.zip`: `planner.py`のみ差分(同3行+`CUTCORNER_CANDIDATES`の
  1行)。`geometry.py`(新規関数を含む)はzipとリポジトリで完全一致(新関数自体は
  フラグに関わらず存在するコードであり、フラグが立つ`planner.py`側だけが違う)。
- `mysolver_submit_rcl.zip`: `planner.py`(union/span/centroidの3行)と`simulate.py`
  (`_RCL_SHUFFLE`の1行)の2ファイルが差分。

いずれも11エントリ、既存提出zipと同一構造(`mysolver/`直下に`agent.py`ほか10ファイル)。

### 決定的8シーンの対緩2差分(参考値)

`MYSOLVER_HARD_WALL_LIMIT=3000`(壁時計非拘束)で緩2単体の`build_order`と、
緩2+各フラグの`build_order`を比較した(参考値、28シーン全体の効果の大小を代表しない)。

| zip | 差分シーン数 | 差分シーン |
|---|---:|---|
| `mysolver_submit_dftrc.zip` | 2/8 | A01, A03 |
| `mysolver_submit_cutcorner.zip` | 0/8 | (差分なし) |
| `mysolver_submit_rcl.zip` | 0/8 | (差分なし) |

`cutcorner`が8シーンで無差分なのは、(2-2)の追加検証(B04/C03でも配置数完全一致)と
整合する——この決定的8シーン群では斜め切り欠き候補が採用されなかった。`rcl`の
無差分は(2-3)の「phase2が0/26のまま勝てない」ことと整合する(この8シーンの
best_orderはいずれも元々phase1/heuristic由来であり、RCLはphase2にしか影響しない)。

---

## 判定(本番結果待ち)

対照は緩2 = public 57.18(配置率64.34%)。3zip(`dftrc`/`cutcorner`/`rcl`)を提出し、
publicとnum_placed_itemsの両方で判定する。ローカルでの追加観測(判定には使わない、
参考情報):

- `dftrc`: ローカルで唯一「軌道が変わった」形跡(8シーン中2/8で差分、14シーン配置率
  −0.96pp)。back_termとの軸競合(ステップ0参照)により配置率がわずかに下がった
  可能性があるが、これは支持閾値のような「配置数を減らす」効果ではなく、単に
  スコアの優先順位が変わったことによる誤差範囲の変動である可能性が高く、ローカルでは
  判別できない。
- `cutcorner`: ローカルでは検証した範囲(14+2シーン)で挙動に変化が観測されなかった
  (候補は追加されているはずだが、選ばれなかった)。本番の異なるシーン分布では
  斜め切り欠きが実際に使われる可能性があり、ローカルの無反応をもって「効果なし」と
  断定はしない。
- `rcl`: ローカルでphase2の勝率が0/26のまま不変(既定のシャッフル方式を問わず
  phase2自体が勝てない)。3項目の中で最もローカルの支持材料が弱い。

**いずれもPhase80の教訓どおり、ローカルの弱い/無いシグナルだけで不採用を決めない。**
本番結果(public・num_placed_items)を見て判定する:
- 57.18を明確に超えた → 枠入れを検討、その軸を深掘り
- 57.18前後 → 効果なし、不採用
- 下がった → 不採用、理由を記録して閉じる

---

## やっていないこと

- EP/EMSの全面置換(本フェーズの範囲外、予算が律速でないことは実測済み)。
- 1つのzipに複数フラグを混ぜること(組み合わせzipは(2-3)の結果を踏まえ今回は作成せず)。
- 支持閾値・幾何定数を動かすこと(緩2+既定で固定)。
- ローカルA/Bの弱いシグナルで採否を決めること(上記「判定」参照、本番結果待ち)。
- 既存の4戦略・グリッド生成の削除/変更(いずれも追加のみ)。
- 本番の集計スコアから足切り閾値やシーン数を逆算すること。
- `.gitignore`の書き換え・force push。

## 生成物一覧

- `agents/mysolver/planner.py`(DFTRC項の追加、cut corner候補生成の追加、いずれも
  既定無効・加算のみ)
- `agents/mysolver/geometry.py`(`diagonal_face_indices`/`cutcorner_boundary_x`の追加)
- `agents/mysolver/simulate.py`(`MYSOLVER_RCL_SHUFFLE`、`_rcl_shuffle_keys`の追加)
- `results/phase81_step2_{baseline_loose2,dftrc,cutcorner,rcl}.json`(Step2スモークテスト)
- `results/phase81_rcl_winner_trace.json`(RCLのphase2勝率実測)
- `results/phase81_report.md`(本ファイル)
- `submissions/mysolver_submit_{dftrc,cutcorner,rcl}.zip`(新規、SHA256は上表参照)
- `README.md`(スコア推移表・グラフをPhase81時点まで更新、別途ユーザー指示による)

