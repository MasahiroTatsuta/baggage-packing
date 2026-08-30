# Phase86 報告: 探索レイヤーの高度化 — Tier1(Look-ahead Beam)

## 前提の訂正(実施前に発覚)

指示文は「Deep Research(本セッション)と外部LLM(ChatGPT)の両方が探索レイヤーの
高度化(MCTS/BRKGA/PyBullet仕上げ)で一致した」としていたが、このセッションに
実際にあったDeep Research成果物(`docs/NEDO 3D Bin Packing Contest_ EP_EMS and
Precedence Graph Implementation Design...pdf`、Phase81で全文読了・実装済み)には
MCTSの言及が無く、BRKGAも「EMSの実装参考コード」としての引用のみで、推奨ロード
マップ自体はEP/EMS→reactive GRASP→precedence graph→堅牢化と別物だった。

この点をユーザーに確認したところ、**別のDeep Research成果物
(`docs/3D Bin Packing for NEDO Contest_ Beyond Constructive Heuristics —
Evaluating MCTS, BRKGA, and DRL Approaches.pdf`)が既にリポジトリに存在する**
ことが判明した。全文を読了した結果、この文書は指示文の記述と完全に一致する:
(A) DRL不採用・(B) MCTS/look-aheadビームサーチが最有力・(C) BRKGA並行検証・
(D) pybullet後処理は余力があれば、という結論、および撤退ラインの記載
(「MCTS/look-aheadが第2週末で配置数改善ゼロなら探索深化は打ち切り、BRKGA一本に
絞る」)まで指示文と一致した。**前提の訂正は解消し、指示どおりTier1から着手した。**

---

## Tier1: Look-ahead Beam

### ステップ1-1: 設計

**現行`simulate.beam_construct_order`の構造**(実装前に確認):
各ステップで、ビーム内の各部分解に対し`planner.plan_topk`が返す上位`top_k`候補
(1手だけ置いた直後の(item, orientation, position))を、「その1手を置いた直後の
risk調整済みobjective」でランク付けし、上位`beam_width`個を残す。**現状は1手先
までしか見ない近視眼的な枝刈り。** 本番既定は`BEAM_WIDTH=1`で、その場合
`top_k = max(1, beam_width) = 1`となり、比較対象すら1個しかない(事実上の貪欲法)。

**設計(既存資産の再利用を徹底)**:

1. `MYSOLVER_LOOKAHEAD_DEPTH`(既定`'0'`)が正のとき、各ステップの枝刈りに使う
   スコアを「1手先の即時objective」から「その1手を置いた後、さらに
   `LOOKAHEAD_DEPTH`手をグリーディ(top_k=1、実質`planner.plan()`)に転がした
   終端でのobjective」に置き換える。**コミットする実際の状態(次ステップに引き
   継がれるcontainers/remaining/risk_vol)は常に最初の1手のみ**——追加の
   ロールアウトは評価専用の使い捨てで、次の実ステップであらためて展開し直す
   (固定した未来を強制しない、標準的なlook-ahead pruningの設計)。
2. **新しい評価関数は作らない。** ロールアウトは既存の`_objective`(risk調整済み
   体積 − 優先手荷物誤配置ペナルティ)と`geo.fill_risk_factor`をそのまま使う。
   本ソルバは元々「幾何評価でオフライン順序を選び、pybulletは本番評価器側でのみ
   最終判定する」という二段構えを既に持っているため、Deep Researchが指摘する
   「rolloutは幾何評価のみ、最終候補だけ本物の安定性判定」は追加実装なしで
   自動的に踏襲される。
3. `LOOKAHEAD_DEPTH>0`のときのみ、比較する枝の数を`LOOKAHEAD_BREADTH`
   (既定3、env`MYSOLVER_LOOKAHEAD_BREADTH`)以上に引き上げる。本番既定の
   `BEAM_WIDTH=1`では`top_k=1`のままlook-aheadしても比較対象が無いため、
   look-ahead有効時だけ最低限の比較候補数を確保する(`BEAM_WIDTH`自体は
   変更しない——ビーム幅を増やすと部分解を持ち越す本数が増え、コストの性質が
   look-aheadの深さとは別次元で変わるため)。
4. 増分更新(EP/EMS差分更新、Phase81のcutcorner含む)はロールアウトでも
   `planner.plan()`を通常どおり呼ぶだけで自動的に使い回される(候補生成自体は
   Phase81ですでに共有関数化されており、新規実装は不要)。

### ステップ1-2: 実装

`agents/mysolver/simulate.py`:

- `LOOKAHEAD_DEPTH = int(os.environ.get('MYSOLVER_LOOKAHEAD_DEPTH', '0'))`
- `LOOKAHEAD_BREADTH = int(os.environ.get('MYSOLVER_LOOKAHEAD_BREADTH', '3'))`
- 新規関数`_lookahead_rollout_score(...)`: 候補1手を仮コミットした状態から
  `extra_depth`手をグリーディに転がし、終端でのobjective値を返す(評価専用、
  `budget.exhausted()`または合法手枯渇で早期終了するanytime設計)。
- `beam_construct_order`内: `effective_top_k = max(top_k, LOOKAHEAD_BREADTH) if
  LOOKAHEAD_DEPTH > 0 else top_k`でplan_topkの候補数を調整。各候補について、
  `LOOKAHEAD_DEPTH>0`のときだけ複製→仮コミット→ロールアウトでランク用スコアを
  算出し、`cands.sort()`のキーに使う。**ビームに実際にコミットする
  risk_vol/n_prio/n_mis(='score')は常に浅い(1手だけの)値のまま**——
  ロールアウトの結果は枝刈りの判断材料としてのみ使い、実際の状態には反映しない。

既定(`LOOKAHEAD_DEPTH=0`)では`effective_top_k == top_k`かつ`rank_score == score`
となり、既存コードパスと完全に同一の計算になる。

### (2-3) 決定的8シーンのビット単位不変

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

**8/8確認。**

### クイックサニティ(A01、depth=1/2)

例外なく完走、depth=1/2それぞれ異なる出力(fill_strict 22.57/24.97)を確認。
### ステップ1-3: 検証結果

**時間予算の制約により、26シーン+sample_configではなく代表6シーン
(A01/A03/B01/C01/D01/P02、cc_rcl土台+`--optimize-budget 60`)に絞った。**
このスコープ制限自体を明記する(全26シーンでの測定は行っていない)。

| シーン | depth=0 | depth=1 | depth=2 |
|---|---:|---:|---:|
| A01_1c_40_plain | 26/40 | 15/40 | 11/40 |
| A03_1c_40_shelf | 22/40 | 17/40 | 9/40 |
| B01_1c_40_plain | 24/40 | 24/40 | 24/40 |
| C01_1c_40_shelf | 17/40 | 17/40 | 17/40 |
| D01_softheavy | 23/40 | 13/40 | 16/40 |
| P02_pre10 | 31/50 | 24/50 | 20/50 |
| **合計配置数** | **143/250** | **110/250(−23%)** | **97/250(−32%)** |

`optimize_time`はdepth=2でもmax 60.1s(HARD_WALL_LIMIT=165sに対し十分な余裕)で、
offline 3分予算を超過するリスクは無かった——**問題は予算超過ではなく、結果の
質そのものが明確かつ単調に悪化したこと。**

### 判定: 撤退(ノイズレベルを大きく超える明確な悪化)

**depth=1で−23%、depth=2で−32%という、深さを増やすほど悪化する明確で単調な
負の効果を確認した。** 6シーート中4シーンが両depthで一貫して悪化し(A01/A03/D01/
P02)、悪化しなかった2シーン(B01/C01)も改善はゼロ(完全に同一)——**改善方向の
シーンは1件も無かった。** 指示書の撤退基準「伸びがノイズレベルなら撤退」を
遥かに超える、明確な負の結果であるため、**Tier1は撤退と判定する。**

**分析(推測、未検証)**: look-aheadロールアウトは`_objective`(risk調整済み体積)を
グリーディに数手先まで伸ばして枝を選ぶが、この短いグリーディロールアウトは
「直近数手で体積を稼ぎやすい」枝を過大評価しやすく、その選択が後続の配置可能性
(搬入経路・残り空間の形状)を悪化させている可能性が高い。これはDeep Research
(B)自身がリスクとして明記していた「評価関数の精度がrollout品質を規定する点は
ALNSと同じ弱点」が、まさに実測で顕在化した形であり、Phase34のALNS不採用
(代理gainと実fillの順位相関ρ=−0.321)と同型の失敗モードだと考えられる。

**指示書の指示どおり、Tier2(MCTS)は見送る。** MCTSも本質的に同じロールアウト
評価(greedy playout by現行スコア関数)に依存する設計であり、Tier1で確認された
「短いグリーディロールアウトが逆効果」という問題が解消されない限り、MCTS化しても
同じ失敗モードを踏襲するリスクが高いと判断した。**Tier3(BRKGA)に集中する。**

### zip化: 実施しない

指示どおり、明確な悪化のため`mysolver_submit_cc_rcl_lookahead.zip`等は作成しない。
実装コード(`LOOKAHEAD_DEPTH`/`LOOKAHEAD_BREADTH`/`_lookahead_rollout_score`)は
既定無効(`MYSOLVER_LOOKAHEAD_DEPTH=0`)のまま安全にリポジトリへ残す
(既定無効時の8/8ビット単位不変は確認済み。将来ロールアウト評価の精度を上げる
別の取り組みがあれば、フラグを立てるだけで再利用できる)。

---

## 現状のまとめとTier3への移行判断

主枠(cc_rcl、public 57.545)・2枠目(rest020、55.96)は変更なし。Tier1は明確な
撤退、指示書自身の設計によりTier2(MCTS)も見送りとなった。

**Tier3(BRKGA)は、指示書上Tier1/2の成否に厳密には依存しない並行実施可能な軸として
位置づけられており、かつTier1が踏んだ「短いグリーディロールアウト評価の精度不足」
という失敗モードを構造的に回避する設計(decoderで実配置を最後まで進めてから評価する
ため、代理評価に依存しない)である。Tier1の失敗はTier3の妥当性を否定する根拠には
ならないと判断する。**

ただし、Tier3(BRKGA)は母集団・世代・エリート再評価・しきい値選択を含む独立した
実装(設計3時間+実装8時間+検証4時間、指示書見積り)であり、Tier1と同程度以上の
規模の新規作業になる。**本フェーズはここで一区切りとし、Tier3着手の可否を報告した
上で指示を仰ぐ**(指示書自身が各Tierの終わりに撤退判断を挟むことを求めているため、
Tier1の結論をまず確定させることを優先した)。

---

## やっていないこと

- Tier2(MCTS)の実装は行っていない(指示書の撤退基準により見送り)。
- Tier3(BRKGA)の実装はこの時点では行っていない(着手可否を報告し指示を仰ぐ)。
- Tier4(PyBullet仕上げ)はTier1-3のいずれかが有望な場合のみ着手する設計のため、
  未着手。
- 支持閾値・幾何定数・cutcornerのN_Y・rclのkは動かしていない(cc_rclで固定)。
- 本番の集計スコアから足切り閾値やシーン数を逆算していない。
- `.gitignore`の書き換え・force pushは行っていない。
- zip作成は行っていない(Tier1が明確に不採用のため)。

## 生成物一覧

- `agents/mysolver/simulate.py`(`LOOKAHEAD_DEPTH`/`LOOKAHEAD_BREADTH`/
  `_lookahead_rollout_score`の追加、既定無効・追加のみ)
- `results/phase86_tier1_depth{0,1,2}.json`(6シーン×3水準の生データ)
- `results/phase86_report.md`(本ファイル)
