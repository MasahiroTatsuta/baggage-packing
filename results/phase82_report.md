# Phase82 報告: cutcorner×rcl 組み合わせと各軸の深掘り

## Phase81本番結果(先出し、指示文より)

| | public | vs緩2 | 配置率 | 主な変化 |
|---|---:|---:|---:|---|
| cutcorner | **57.394** | **+0.209** | 63.49%(−0.84pp) | cog +0.77 / soft +1.20 |
| rcl | **57.334** | **+0.149** | **64.59%(+0.25pp)** | fill +0.19 / soft +1.00 |
| 緩2(対照) | 57.185 | — | 64.34% | — |
| dftrc | 54.567 | −2.618 | 61.75% | 全成分悪化 |

dftrcは不採用(既存risk_vol選好と競合)。cutcorner/rclは機序が独立(配置の質 vs 探索の
多様性)と推定され、本フェーズで組み合わせと各軸の深掘りを行う。

---

## ステップ0: RCLの効果を数字で確認する

### (0-1) phase2由来best_orderの勝率(RCL=0 vs RCL=1、26シーン)

`tools/phase72_winner_trace.py`を`MYSOLVER_RCL_SHUFFLE=0`/`=1`それぞれで実行
(`MYSOLVER_HARD_WALL_LIMIT=3000`、既定閾値0.55/0.6/0.15、Phase72と同一条件、
`configs/gen/suite_*.json`の26シーン)。

| winner_source | RCL=0(既定) | RCL=1 |
|---|---:|---:|
| heuristic | 10 | 10 |
| phase1 | 16 | 16 |
| **phase2** | **0** | **0** |

**phase2由来のbest_orderは、RCL=0/RCL=1のいずれでも0/26で完全に同一だった。**
Phase72の元の実測(0/28、sample_configの2タスクを含む)、Phase81での確認(RCL=1のみ
0/26)に続き、本フェーズでRCL=0との**同一条件でのペア比較**を行っても、内訳
(heuristic 10/phase1 16/phase2 0)が1件も動かなかった。

**この結果は、production結果(rcl: public 57.334、+0.149、配置率+0.25pp)と組み合わせて
読むと重要な意味を持つ**: RCLの本番での効果は、「phase2がこのローカル26シーンで勝つ
ようになる」という当初の仮説(Phase81のRCL実装の狙い)そのものでは説明できない
——ローカルでその経路は一度も開通していない。centroidがPhase79で示した構図
(「死亡局面の最後の1個を救う」という狭い経路ではなく「エピソード序盤からの軌道変化」
で効いていた)と同型で、**RCLの本番効果も、phase2の勝敗という狭い指標では捉えられない
別の経路(例えば、たとえ最終的にphase1が勝つとしても、phase2の試行がbest_scoreの
更新履歴やタイブレークに与える副次的な影響、あるいはこの26シーンでは再現されない
本番シーン特有の構成)で生じている可能性が高い**、という仮説を記録する(検証はしていない)。

### (0-2) RCLのk(現在値)

`agents/mysolver/simulate.py`の`_RCL_FRACTION = float(os.environ.get('MYSOLVER_RCL_FRACTION', '0.3'))`。
**現在k=30%。** ステップ1で15%(貪欲側)・50%(ランダム側)の2水準を追加で振る。

---

## ステップ1: zip 4本(cutcorner候補点2倍のsafety検証込み)

### (1-2)専用: cutcorner候補点2倍(N_Y 5→10)のpolicy時間実測(必須・zip化前)

指示書の警告(本番policy max=6.03s、8秒制限まで2秒)を受け、要求の厳しい4シーン
(A08: 2コンテナ140個、B04: 2コンテナ80個、C03: 2コンテナ80個・優先荷物、P05: 2コンテナ
既積み8個)に絞り、`--optimize-budget 60`でN_Y=5(現行)とN_Y=10(2倍)を比較した:

| N_Y | policy_time mean | policy_time max | 配置数(4シーン合計) |
|---|---:|---:|---:|
| 5(現行) | 1.018s | **1.69s** | 18+45+28+42=133 |
| 10(2倍) | 1.006s | **1.68s** | 18+45+28+42=133(完全一致) |

**候補点を2倍にしてもpolicy時間はほぼ無変化(誤差範囲、むしろ僅かに短い)。**
理由: 追加される候補は既存グリッド(31×23×density²点+extreme point数十点)に対して
数点(既定5→10点)を足すだけであり、割合として無視できる規模のため。**ローカルでは
安全マージンに問題は見られなかった。** ただし本番の6.03sという実測値をローカルで
再現できていない(ローカル最大は1.69s、本番より4倍以上小さい)ため、**候補追加が
生む「絶対的な追加コスト」がローカルで無視できるほど小さいことは確認できたが、
本番の未知の律速要因(遥かに大きい実シーン、壁時計圧迫等)に対して同じ比率で
安全と保証はできない**点は留保する。この限界を踏まえた上で、指示どおり2倍
(N_Y=10)で`cc_strong`をzip化した。

### zip一覧

すべて緩2の閾値(0.35/0.4/0.25)+ 幾何定数は既定を土台にする。

| # | zip | 有効フラグ | SHA256 |
|---|---|---|---|
| 1(最優先) | `mysolver_submit_cc_rcl.zip` | `MYSOLVER_CUTCORNER_CANDIDATES=1` + `MYSOLVER_RCL_SHUFFLE=1`(両方、既定パラメータ) | `b1a90fccfc1c00281f778dd7a5f10730dc8f58c7c66f975b16ebab3d53e46939` |
| 2 | `mysolver_submit_cc_strong.zip` | `MYSOLVER_CUTCORNER_CANDIDATES=1` + `MYSOLVER_CUTCORNER_N_Y=10`(現行5の2倍) | `b2cdc65639ed2b3a97b12578bccac33fb720a11b8e0759718df2d862aba8e70e` |
| 3 | `mysolver_submit_rcl_k15.zip` | `MYSOLVER_RCL_SHUFFLE=1` + `MYSOLVER_RCL_FRACTION=0.15`(貪欲側) | `54c274b14a15aee304c7dfb084ca6d778dd03dfbd68ff114c21abfe6eae31b77` |
| 4 | `mysolver_submit_rcl_k50.zip` | `MYSOLVER_RCL_SHUFFLE=1` + `MYSOLVER_RCL_FRACTION=0.50`(ランダム側) | `6456bd3e558272430040a3656abfab923dea9cb5ba0937f3dc3adda83c64c472` |

5本目(cc_strong+rcl等の組み合わせ)は見送った。理由: (0-1)でRCLのローカルでの
効果を再確認できなかった(phase2勝率は0/26のまま)ため、この時点でパラメータを
2軸同時に振る組み合わせを追加しても、本番結果が出るまでどちらの寄与か切り分ける
手段がなく、**1変更1zipの原則を破ってまで追加する情報的価値が無いと判断した**
(2h予算の中で4本の丁寧な検証を優先)。

**アップロード時は必ず上記SHA256と照合すること。**

### 全定数grep(4zip共通の確認結果)

`mysolver_submit_dftrc.zip`と同じ16項目チェックリストに、Phase82で新規に振った
`MYSOLVER_CUTCORNER_N_Y`・`MYSOLVER_RCL_FRACTION`を加えて確認した。4zipとも:

- 閾値3(union/span/centroid)= 緩2(0.35/0.4/0.25)で共通
- 幾何3(INCLUSION_MARGIN/SAFETY_MARGIN_XY/REST_CLEARANCE)= 既定で共通
- 他7(TELEMETRY/HARD_WALL_LIMIT/REPLICA_SELECT/REPLICA_METRIC/FALLBACK_SAFE_POS/
  FALLBACK_AVOID_OBSTACLES/STRICT_SUPPORT_DISABLE)= 既定で共通
- `DFTRC_STRATEGY`は4zipとも`'0'`(不採用のPhase81 dftrcは触っていない)
- `CUTCORNER_CANDIDATES`/`CUTCORNER_N_Y`/`RCL_SHUFFLE`/`RCL_FRACTION`は各zipの
  意図どおり(#1は両方有効・既定パラメータ、#2はcutcornerのみ・N_Y=10、
  #3/#4はrclのみ・k=0.15/0.50)
- `BEAM_SOFT_LAST`/`ALNS`/`REPAIR`/`WALL_MODE`は4zipとも既定`'0'`

### zipとリポジトリの差分ファイル確認

- `cc_rcl.zip`: `planner.py`(閾値3行+`CUTCORNER_CANDIDATES`)・`simulate.py`
  (`RCL_SHUFFLE`)の2ファイルが差分。
- `cc_strong.zip`: `planner.py`のみ差分(閾値3行+`CUTCORNER_CANDIDATES`+
  `CUTCORNER_N_Y`)。
- `rcl_k15.zip`/`rcl_k50.zip`: `planner.py`(閾値3行)・`simulate.py`
  (`RCL_SHUFFLE`+`RCL_FRACTION`)の2ファイルが差分。

いずれも11エントリ、既存提出zipと同一構造。

### 決定的8シーンの対緩2差分(参考値)

| zip | 差分シーン数 |
|---|---:|
| `cc_rcl` | 0/8 |
| `cc_strong` | 0/8 |
| `rcl_k15` | 0/8 |
| `rcl_k50` | 0/8 |

4本とも決定的8シーンでは差分なし(Phase81のcutcorner/rcl単体も0/8だったことと整合)。
この8シーン群はこれらのメカニズムに対して感度が低い母集団であり、本番の効果は
別のシーン群で生じていると考えられる((0-1)の考察と同様)。

### policy時間(参考、上記(1-2)のcc_strong実測を除き14シーンスモークは今回省略)

時間予算(2h)の制約上、cc_rcl/rcl_k15/rcl_k50については個別のpolicy時間スモークは
実施していない(いずれもRCLはオフライン`build_order`側のみに影響し、オンライン
`policy()`の候補評価コスト自体を増やす変更ではないため、cutcorner単体で確認した
安全性がそのまま持ち越される。cutcorner+rclの組み合わせ(#1)もcutcornerの候補数は
既定のN_Y=5のままであり、Phase81のcutcorner単体でのpolicy_time実測(max 1.69〜2.16s)
から大きく外れる理由はない)。

---

## 提出枠の更新

指示どおり、**主枠を`mysolver_submit_cutcorner.zip`(public 57.394)に変更した。**
2枠目は`rest020`(public 55.96、REST_CLEARANCEが崖から離れた独立の故障モードを持つ
保険)を維持(Phase77の判断を継続)。`docs/submission_policy.md` §1・§5に反映した。

## 判定(本番結果待ち)

対照は緩2=57.185 / cutcorner=57.394(現主枠) / rcl=57.334。4zip(`cc_rcl`/`cc_strong`/
`rcl_k15`/`rcl_k50`)を提出し、publicとnum_placed_itemsの両方で判定する。ローカルでの
追加観測(判定には使わない、参考情報):

- 4zipとも決定的8シーンで無差分、cc_strongの追加候補もこの4シーン(A08/B04/C03/P05)
  では選ばれなかった(配置数完全一致)。**ローカルはこれらの施策全てに対して
  低感度な母集団であり、無反応は不採用の根拠にならない**(Phase81のcutcorner/rcl
  単体も同じくローカル無反応だったが本番は+0.15〜+0.21だった)。
- RCLのphase2勝率はRCL=0/1で完全に同一(0/26)。**k(RCL_FRACTION)を15%/50%に
  振っても、そもそもphase2が勝つ経路自体がローカルで開通していないため、
  この2水準の違いをローカルで判別する材料は無い。** 本番のみが判定材料になる。
- cc_strong(N_Y 2倍)はローカルpolicy時間で安全マージンを確認できたが、
  効果の有無・方向はローカルでは何も分からない(4シーンで配置数完全一致)。

**Phase80の教訓どおり、ローカルの無反応・弱いシグナルだけで不採用を決めない。**
本番結果を見て判定する:
- 57.394(現ベスト)を明確に超えた → 枠入れを検討
- 57.2〜57.4 → 誤差範囲、枠はcutcornerのまま
- 下がった → その方向は不採用

---

## やっていないこと

- dftrcの再挑戦(明確に不採用、指示どおり実施していない)。
- 支持閾値・幾何定数を動かすこと(緩2+既定で固定)。
- policyが8秒制限を超えるリスクを未検証のままzip化すること((1-2)で実測してから
  cc_strongをzip化した)。
- ローカルA/Bの弱いシグナルで採否を決めること(上記「判定」参照、本番結果待ち)。
- 本番の集計スコアから足切り閾値やシーン数を逆算すること。
- `.gitignore`の書き換え・force push。

## 生成物一覧

- `results/phase82_wintrace_rcl0.json` / `phase82_wintrace_rcl1.json`(ステップ0、
  RCL=0/1のphase2勝率ペア比較)
- `results/phase82_report.md`(本ファイル)
- `submissions/mysolver_submit_{cc_rcl,cc_strong,rcl_k15,rcl_k50}.zip`(新規、
  SHA256は上表参照)
- `docs/submission_policy.md`(§1・§5にPhase82追記、主枠をcutcornerへ更新)

コードファイル(`agents/mysolver/*.py`)は無変更(Phase81で実装済みの3フラグを
異なるパラメータ・組み合わせで使っているだけ)。
