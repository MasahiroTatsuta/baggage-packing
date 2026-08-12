# Phase 31 報告：選択則(risk_vol)の取りこぼしを実測 → 「床面限定」案を実装したが t=1.18 で不採用、原因はD01の反例1件

作成日: 2026-08-12
対象コミット: (最終コミットで確定、直前 67d30a7)

**本フェーズは ALNS を実施していない。** ステップ0で選択則側の取りこぼしが閾値(+1.5)を
大きく超えたため、指示どおりALNS着手前に一度立ち止まり、選択則の修正(本フェーズの主題)
を先に検証した。結果は不採用(t=1.18)。詳細と次の選択肢を本報告にまとめる。

変更ファイル:
`agents/mysolver/geometry.py`(`inclusion_slack_batch` に `floor_only` 引数を追加。
既定 False で無改変)、`agents/mysolver/simulate.py`(`RISK_SLACK_FACES` 環境変数フックを追加、
既定 `'all'` で無改変)、`tools/phase31_selection_eval.py`(新規、選択則差し替えの再評価)。

**測定条件**: ローカルA/Bは `MYSOLVER_UNITS_PER_SEC=1.55e7`。既定 `MYSOLVER_RISK_SLACK_FACES=all`。
提出は行っていない(ローカルで有意な改善が出なかったため、指示のルール5により提出しない)。

---

## 0. エグゼクティブ・サマリ

- **【ステップ0】実 `fill_strict` での選択の取りこぼしは 26シーン換算 +2.638**
  (21シーン合計68.58、σ=4.07、gap>0が12/21シーン、最大 P01 +12.95)。
  指示の閾値(+1.5)を大きく超え、ALNS着手前の一時停止が確定した。

- **【診断】原因は risk_vol の「全面の最悪値」が、実際にはほぼ起きない壁際接近を
  過剰に罰していること(仮説どおり)。** Phase21 の全数監査(床99.7%/背面0.3%、
  側壁・天井・切り欠きの脱落は実測0件)と整合する。素の `placed_volume`(risk調整なし)
  への単純置換だけでも 26シーン換算 +0.351 まで縮む(取りこぼしの13%を回収)。

- **【本命実装: 床面限定】** `geo.inclusion_slack_batch` に `floor_only` を追加し、
  `simulate.py` の risk_vol 集計だけに適用(hard legality・`_score()` のboundary_termは
  完全に不変。**構築[どこに何を置くか]は一切変わらない**、変わるのは「作り終えた
  複数の候補順序のどれを勝者に選ぶか」だけ)。**既定は現行(`all`)のままビット単位一致**
  (決定的5シーン+A01-A03、8/8完全一致)。

- **【26シーンA/B結果】不採用。** 5/21シーンで勝者が変わり(4勝1敗)、
  26シーン換算 **mean +0.948 / σ=4.109 / SE=0.806 / t=1.177**。
  採否基準(t>2、事前に「mean≥+1.60必要」と明示していた水準)に届かず。

- **【なぜ効かなかったか】D01(softheavy)で -10.77 の反例が出た。** 事前の仮説
  (「床の割引はD01で本物の仕事をしているので壊れないはず」)は**支持されなかった**。
  実際には D01 で「壁際接近の罰則」が、shadow simulator が過大評価した候補
  (最大の生placed_volumeを持つ候補)を正しく格下げする**副次的な正則化**として
  機能しており、それを外すと過大評価候補が繰り上がった。詳細は §4。

- **【判定】不採用(既定 `MYSOLVER_RISK_SLACK_FACES=all` のまま)。** 実装は残す
  (既定無効・再利用可能な部品)。public提出は行っていない(ローカルで有意でないため)。

- **【次の選択肢】** 本報告の末尾(§6)にまとめ、ユーザーの判断を仰ぐ。

---

## 1. ステップ0: 実 fill での選択の取りこぼし

`results/phase30_cand_eval.json`(21シーン・130候補・**本物の評価器**による実測、
新規ロールアウト不要)を再利用し、各シーンで
`max(候補の実fill_strict) − 勝者の実fill_strict` を計算した
(勝者は現行の選択則 `risk_vol − PLACEMENT_PENALTY_WEIGHT×総容積×違反率` の argmax、
`placement_score=100`かつ`soft_item_score=100`を満たす候補のみを候補集合とした)。

| 指標 | 値 |
|---|---:|
| 21シーン合計gap | 68.58 |
| **26シーン換算**(optimize無効5シーンは0) | **+2.638** |
| σ(21シーン) | 4.07 |
| gap>0のシーン数 | 12/21 |
| 最大のgap | P01 +12.95 / A03 +9.10 / C01 +9.03 / D04 +8.87 / A01 +8.73 |

判定基準(+1.5以上で報告)を大きく超えたため、ここでALNS着手前に立ち止まった。

補助診断として、選択則を `risk_vol`(risk調整済み)から素の `placed_volume`(risk調整前)の
argmaxへ単純置換した場合も測定した: 26シーン換算 **+0.351**(σ=3.60)で、取りこぼしの
13%しか回収できず、しかもD01で **-10.77** の大きな悪化が既にこの時点で見えていた
(=risk_vol の割引がD01で何らかの実質的な仕事をしているという最初の兆候)。

---

## 2. 実装: risk_vol の床面限定(既定無効)

### 2.1 設計

`agents/mysolver/geometry.py::inclusion_slack_batch` に `floor_only: bool = False` を追加。
`True` のとき、全7面(壁5・天井1・床1)の中から**法線が最も下向きの面**
(`argmin(n_vecs[:,2])`、全シーン・全コンテナで実測 index=0 と確認済み)だけの
dot値を返す。**既存の呼び出し(`floor_only` を渡さない箇所)は完全に無改変。**

`agents/mysolver/simulate.py` に環境変数 `MYSOLVER_RISK_SLACK_FACES`(既定 `'all'`)を追加。
`'floor'` のとき、`simulate_order` の risk_vol 集計(1行のみ)が
`geo.inclusion_slack_batch(container, half, placed['pos'], floor_only=True)` を使う。

**この変更が触れるのは risk_vol(候補順序を選ぶための集計値)だけ**であり、
以下は一切変更していない:

- `planner._evaluate_candidates` の `slack`(hard legality `incl` 判定、既存のまま)
- `planner._score` の `boundary_term`(online policy() / offline構築の両方が使う
  配置スコア、既存のまま)
- `simulate.beam_construct_order` 内部の risk_vol(構築中のビーム剪定用。
  `MYSOLVER_BEAM_WIDTH` 既定1では剪定が事実上no-opなので触れる必要が無い、§2.2参照)

つまり**「どこに何を置くか」(=各候補順序の実際の配置内容)は既定のまま完全に不変**で、
「作り終えた複数の候補順序のどれを勝者に選ぶか」だけが変わる。この設計により
**候補生成の再ロールアウトが一切不要**になった(既存の `order` を
`simulate_order` に1回流すだけで新しい risk_vol が求まる)。

### 2.2 「構築は変わらない」ことの確認根拠

`ordering.BEAM_WIDTH` の既定は `1`(Phase25aで確定)。このとき
`simulate.beam_construct_order` の `plan_topk(..., top_k=max(1,beam_width)=1, ...)` は
各ステップで候補を1件しか返さないため、risk_vol によるビーム内比較(`cands.sort`)は
**比較対象が常に1件しかない no-op**になる(実際に選ばれる手は `_score()`(boundary_term
込み、無改変)の argmax のみで決まる)。したがって候補順序の**生成**(構築)は
risk_vol の定義に一切依存せず、`order`(item indexの並び)は既定のまま完全に同一になる。
これは今回**新たに導出した**性質ではなく、`ordering.py` の既存コード
(`build_order` が `beam_construct_order` で `order` を作り、**別途** `simulate_order(order)`
を呼んで選定用の risk_vol を得る、という二段構成)を読んで確認したものである。

### 2.3 無変更の確認(ビット単位一致)

既定 `MYSOLVER_RISK_SLACK_FACES=all` のまま、決定的5シーン(B01-B04, P04)+
optimize有効の先頭3シーン(A01-A03)を測定し、`results/phase29_noleak.json` と突き合わせた。

```
完全一致(ビット単位)のシーン: 8/8
  fill_strict/fill_loose/cog_score/stability_score/placement_score/soft_item_score
  すべて差分 0.000
```

---

## 3. 26シーンA/B: 床面限定 risk_vol

### 3.1 方法

`results/phase29_cand_g1/g2.json` の全130候補(21シーン)について、記録済みの `order` を
`simulate_order(..., RISK_SLACK_FACES='floor')` で**再生**(構築のやり直しではなく、
既に確定した順序を1回流すだけ)し、新しい risk_vol を得た。`RISK_SLACK_FACES='all'` でも
同様に再生し、記録済みの risk_vol と**完全一致**(最大差 0.00e+00)することを確認した
(再生パイプラインの妥当性検証)。

新しい risk_vol で `ordering._better` と同じ選択則(`(risk_vol - pw*総容積*違反率, 配置数)`
の辞書式比較、`_better` の実装をそのまま踏襲)により各シーンの勝者を選び直し、その勝者の
**本物の** fill_strict/fill_loose/placement_score/soft_item_score/cog_score を
`results/phase30_cand_eval.json`(既存、新規ロールアウト不要)から引いた。
decisive 5シーン(B01-B04, P04)は build_order 自体が走らないため gap=0(§2.3で無変更確認済み)。

### 3.2 到達したシーン数(足切りルール §1)

**5/21シーンで勝者が変わった。** k=5 は t の上限 √5≈2.24 が原理的に t>2 に届きうる
下限ラインなので、26シーン統計に進んだ。

| scene | 旧勝者 | 新勝者(floor) | 実fill: 旧→新 | Δfill |
|---|---|---|---|---:|
| A01_1c_40_plain | c4 | c0 | 18.49 → 27.22 | **+8.73** |
| C02_2c_55_shelfprio | c3 | c4 | 25.19 → 30.08 | **+4.88** |
| D01_A_1c_40_softheavy | c5 | c3 | 31.53 → 20.76 | **−10.77** |
| D04_A_1c_40_flat | c1 | c2 | 18.08 → 26.95 | **+8.87** |
| P01_A_1c_pre6 | c3 | c5 | 11.86 → 24.81 | **+12.95** |
| (残り16シーン) | — | — | 不変 | 0.00 |

新勝者の `placement_score`/`soft_item_score` は5シーンとも **100.0**(制約維持)、
`stability_score` も 98.1〜98.4 の範囲(既存レンジ内、悪化なし)。

### 3.3 26シーン統計

| 指標 | 26シーン平均 | σ | SE(=σ/√26) | **t** |
|---|---:|---:|---:|---:|
| **fill_strict** | **+0.948** | 4.109 | 0.806 | **1.177** |
| fill_loose | +1.402 | 5.384 | 1.056 | 1.328 |

**採否基準 t>2 に届かず、不採用。** 事前に明示していた通過水準(σ=4.07想定でmean≥+1.60)
に対し、実測はσがさらに悪化(4.11)し、mean(+0.95)も届いていない。

### 3.4 比較: 素の `placed_volume` 置換(§1で先出しした値の正式集計)

| 指標 | 26シーン平均 | σ | SE | t |
|---|---:|---:|---:|---:|
| fill_strict | +0.351 | 3.60 | 0.706 | 0.497 |

床面限定案(t=1.177)のほうが素の置換(t=0.497)より2倍以上良いが、どちらも不採用ライン。

### 3.5 offline限定 vs online拡大の切り分け

**online側は変更していない**(指示どおり、まずoffline限定でA/Bしてから判断する設計)。
offline限定の時点で t=1.177 と基準に届かなかったため、**online側への拡大は見送った**。
online拡大にはさらにコード変更(`planner._score` の boundary_term にも同じ切り替えを
通す必要があり、hard legality と混線しないよう `_evaluate_candidates`/`plan`/`plan_topk`
のシグネチャ変更が要る、§本文冒頭の設計検討)が要る。offline側で有意性が出ていない以上、
その追加実装コストを正当化できないと判断し、実装しなかった。

---

## 4. なぜ効かなかったか: D01の反例

### 4.1 事前仮説との食い違い

指示書の仮説は「D01(softheavy)はPhase21が特定した床沈み込みの機序があるので、
床面限定でも床の情報は残り、壊れないはず」だった。**実測はこれを支持しなかった**
(-10.77の悪化)。

### 4.2 実際に何が起きていたか

D01の6候補を全faces(旧)と実測realで比較すると:

| cand | phase | risk_vol(all) | shadow_placed_vol | shadow_n | 実fill | 実n |
|---|---|---:|---:|---:|---:|---:|
| c0 | heuristic | 0.767 | 1.213 | 8 | 12.80 | 9 |
| c1 | window | 1.262 | 1.587 | 16 | 28.45 | 16 |
| c2 | window | 1.166 | 1.479 | 19 | 26.49 | 19 |
| **c3** | window | 1.403 | **1.690**(最大) | **23**(最大) | 20.76 | 19 |
| c4 | window | 0.949 | 1.252 | 19 | 19.25 | 18 |
| **c5**(旧勝者) | window | **1.415**(最大) | 1.615 | 21 | **31.53**(最大) | 21 |

c3 は shadow 上「最も多く・最も多くの体積を置けた」候補(shadow_n=23、
shadow_placed_vol=1.690、いずれも6候補中最大)だが、**実機では19個しか数えられず、
fillはc1・c2にも劣る**。これはPhase20が特定した「shadow simulatorの予測と実機の乖離」
(native Spearman 0.71)そのものである。

旧(all-faces)formulaでは、c5(risk_vol=1.415)がc3(risk_vol=1.403)をわずかに上回って
勝者になっていた。**この僅差の逆転が、たまたまc3の「shadowの過大評価」を弾く役割を
果たしていた。** 床面限定にすると、c3は(床置き部分の評価は変わらないが、壁際に置かれた
一部の荷物への罰則が消えて)スコアが相対的に上がり、逆転してc5を追い越して勝者になった。

### 4.3 結論

「床面限定」という設計の前提(壁際への罰則は実際にはほぼ意味を持たない、
Phase21の99.7%/0.3%監査)は**大枠としては正しい**(4/5の改善シーンはこれで説明できる)。
しかしD01では、壁際への罰則が**副次的に**「shadow simulatorが過大評価しやすい高volume
候補を格下げする正則化」として機能しており、それを取り除くと過大評価候補が繰り上がった。
つまり **risk_vol の「全面の最悪値」は、床/壁の物理的な意味とは別に、
shadow-real乖離に対する偶発的なノイズ耐性も兼ねていた**、というのが実測から言える結論である。

---

## 5. 判定: 不採用

- 26シーンA/Bで t=1.177(基準t>2に届かず)。
- ルール5(ローカルで有意な改善が出た場合のみpublic提出)に従い、**public提出は行っていない**。
- 既定 `MYSOLVER_RISK_SLACK_FACES=all`(現行のまま)。
- 実装(`floor_only`引数・`RISK_SLACK_FACES`フック)は既定無効のまま残す
  (再利用可能な部品。§6の追試候補で使う)。

---

## 6. 次の選択肢(ユーザーの判断を仰ぐ)

3つの道を提示する。いずれも本フェーズでは実施していない。

1. **床面+背面限定への改良を試す**: Phase21監査は脱落原因を floor 99.7% / **back 0.3%**
   / 側壁・天井・切り欠き 0.0% と分解していた(合計100%)。「床面のみ」ではなく
   「床面と背面(奥壁)の最悪値」に広げれば、D01のように壁際情報が実質的に効いていた
   ケースの一部を救える可能性がある。実装は `floor_only` を `faces='floor'|'floor_back'|'all'`
   へ一般化する程度の小さな拡張で、既存の再生パイプライン(`tools/phase31_selection_eval.py`)
   がそのまま使い回せるため、コストは低い(数十分規模)。ただし新しい仮説であり、
   効く保証はない。
2. **選択則の改良は打ち切り、予定通りALNSへ進む(Opus)**: 選択則側のオラクル上限
   (+2.638)のうち、単純な指標置換ではまだ届いていない(最良でも+0.95、t未達)。
   これは「候補生成側(ALNS)の伸びしろがまだ閉じていない」ことも意味する——
   選択だけで全部は取れないなら、生成の質そのものを上げる方向に価値が残る。
3. **両方は保留し、シーン単位で何が違うのかもう少し診断する**: 5/21シーンしか
   動いていない母数の小ささ自体が問題である可能性もある。「なぜ21シーン中16シーンでは
   risk_vol の顔ぶれを変えても勝者が変わらないのか」(候補間のrisk_vol差がそもそも
   大きいのか、床面限定でも大小関係が保たれるのか)を先に診断する道。

---

## 7. 再現手順

```bash
# ステップ0: 実fillでの選択取りこぼし(既存データの再集計、新規ロールアウト不要)
# (対話ログのアドホック集計。tools/phase30_pareto.json 等の既存データから再構成可能)

# 床面限定案: 記録済み候補順序を新risk_vol formulaで再生し、実評価器の結果と突き合わせ
MYSOLVER_UNITS_PER_SEC=1.55e7 PYTHONPATH=. .venv/bin/python tools/phase31_selection_eval.py \
  --cand results/phase29_cand_g1.json results/phase29_cand_g2.json \
  --real results/phase30_cand_eval.json \
  --out results/phase31_selection_eval.json

# 無変更の確認(既定 MYSOLVER_RISK_SLACK_FACES=all)
MYSOLVER_UNITS_PER_SEC=1.55e7 PYTHONPATH=. .venv/bin/python tools/measure_regime.py \
  --config-path configs/gen/suite_B0*.json configs/gen/suite_P04*.json \
                configs/gen/suite_A01*.json configs/gen/suite_A02*.json configs/gen/suite_A03*.json \
  --module-path agents/mysolver/ --repeats 1 --out results/phase31_noleak.json --label phase31_default
PYTHONPATH=. .venv/bin/python tools/phase29_cmp.py \
  --before results/phase29_noleak.json --after results/phase31_noleak.json
```
