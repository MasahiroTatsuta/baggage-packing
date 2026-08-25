# Phase65 報告: plan()がNoneを返す局面で、合法解を落としているフィルタを機械的に特定

## 結論(先出し)

**犯人は単一、かつ極めて一貫している。** Phase64 が見つけた18シーン・345件の合法解サンプルの
**345/345(100%)が、`planner._evaluate_candidates` 内の支持品質判定(`support_ok`、
`MIN_UNION_SUPPORT_RATIO`系)で落ちている。** 内包判定・搬入経路(legal1/legal2)・
候補生成(グリッド/Extreme Point)・向きの列挙・優先コンテナのtier分離——このいずれでもない。

しかもこの結論は「解の実座標そのもの」だけでなく「plannerが実際に生成する最近傍候補点」
(中央値約12mm、最小1.14mm離れた実在候補)の両方で**完全に一致**しており(0件の食い違い)、
候補生成側の粗さによる見かけ上の結果ではない。

**このフィルタは Phase60 の `SAFETY_MARGIN_XY` のような「実測に基づかない安全側の推測」
ではなく、Phase11/Phase13 で実測(3水準スイープ・stability回収実験)によって現在値
(0.55 / strict 0.75)が決まった値である。** 緩める根拠は「実測がない」ではなく
「実測の結果、現在値より緩い側は既に検証済みで悪化する」という逆方向の実測が存在する。
緩めることの安全性は現時点では未検証であり、Phase11/13の実測パターンから見て
楽観できない(§4)。

---

## 実装(1-1)

`tools/phase65_filter_trace.py`(新規、読み取り専用)。

- 入力: `results/phase64_exhaustive_26.json` / `results/phase64_exhaustive_sampleconfig.json`
  の `exhaustive_findings_sample`(18シーン、サンプル計345件。各シーン最大20件、
  P02=15件・P03=10件のみそれ未満)。
- 死亡直前の観測(`obs_before`)自体はこれらのJSONに保存されていないため、
  `tools.phase64_exhaustive._reach_death` を**変更せずそのままimportして呼び**、
  Phase61-64と全く同一の手順(同一シード・同一エージェント)で同じ死亡直前局面を
  再現した(読み取り専用、`agents/mysolver/`配下は一切変更していない)。
- 各解(item・orientation・container・座標)について、`planner.py`の実行順序に沿って
  以下を**新しい判定式を書かず**、既存関数をそのまま呼んで通過/落選を記録した:

  | 段階 | 呼び出した関数 | 判定内容 |
  |---|---|---|
  | 1 | `planner._candidate_xy` | 解の座標に対する、plannerが実際に生成する候補点集合(グリッド∪Extreme Point)との最短距離(base密度・retry密度の両方) |
  | 2 | `planner._unique_orientations` | 解のorientation番号が列挙対象に含まれるか(含まれなくても同一half-extentの向きが残っていれば幾何的には無害、というところまで判定) |
  | 3 | `planner._apply_y_slice_filter` | y層規律(level0、`Y_SLICE_COUNT=2`)を通過するか |
  | 4 | (情報記録のみ) | 優先コンテナの`enforce_priority_container`/`reserve_priority_container`(tier)条件——コンテナ・アイテムの`is_prioritized`実値を記録 |
  | 5 | `planner._evaluate_candidates`(本体) | 支持品質(`MIN_UNION_SUPPORT_RATIO`/`_STRICT`込みの`support_ok`)・内包・搬入経路(legal1/legal2)を一括で含む本物の合法性判定。`stats`診断カウンタ(`fail_support`/`fail_inclusion`/`fail_ceiling`/`fail_transport_y`/`fail_transport_x`、いずれも`tools/diagnose_stall.py`が使うのと同じ既存フック)をそのまま読む |

  段階5は**(a) 解の実座標そのもの**と**(b) plannerが実際に生成する最近傍候補点**の
  両方に対して呼び、結果が一致するかも記録した。

---

## 1-2: 集計

### どの段階で何件落ちたか(345件ベース)

| 段階 | 結果 |
|---|---:|
| 1) 候補生成(`_candidate_xy`) | 落選0件(常に有限距離で生成される。分布は下記) |
| 2) `_unique_orientations` | 落選0件(345件全て、解のorientationがそのまま列挙対象に含まれていた) |
| 3) y層規律(level0) | **345/345件が level0 では不通過**(§1-4で解釈) |
| 4) 優先コンテナtier | ハード排除なし(§1-4で解釈)。`priority_clearance_could_apply`(非優先荷物の近くに優先荷物が存在する)は305/345件でTrueだが、コンテナ単位の`is_prioritized`はscene設計通りの分布(90/345が優先コンテナ) |
| 5) `_evaluate_candidates`(exact point) | **345/345件が`fail_support`で落選(100%)** |
| 5') `_evaluate_candidates`(plannerの最近傍実候補点) | **345/345件が同じく`fail_support`で落選(exactとの食い違い0件)** |

候補生成の距離分布(base密度、`_candidate_xy`との最短距離):

| | 距離(mm) |
|---|---:|
| 最小 | 1.14 |
| 中央値 | 11.96 |
| 最大 | 21.18 |

### 最初に落とした段階の内訳(最頻の犯人)

```
fail_support: 345 / 345 (100%)
```

他の分類(`fail_inclusion` / `fail_ceiling` / `fail_inclusion_and_ceiling` /
`fail_transport_y` / `fail_transport_x`)は**1件も出現しなかった**。

### シーンごとに犯人が違うのか、共通なのか

**18シーン全てで犯人は完全に同一(`fail_support`)。** シーンごとの内訳を分けても
例外は1件もない(下表、件数はサンプル数=そのシーンで記録されていた解の数):

| シーン | 件数 | 犯人 |
|---|---:|---|
| suite_A01_1c_40_plain | 20 | fail_support (20/20) |
| suite_A02_1c_80_plain | 20 | fail_support (20/20) |
| suite_A03_1c_40_shelf | 20 | fail_support (20/20) |
| suite_A05_2c_80_prio | 20 | fail_support (20/20) |
| suite_B01_1c_40_plain | 20 | fail_support (20/20) |
| suite_B02_1c_40_shelf | 20 | fail_support (20/20) |
| suite_B03_2c_80_prio | 20 | fail_support (20/20) |
| suite_C01_1c_40_shelf | 20 | fail_support (20/20) |
| suite_C03_2c_80_prio | 20 | fail_support (20/20) |
| suite_D02_A_1c_40_prioheavy_nocont | 20 | fail_support (20/20) |
| suite_D04_A_1c_40_flat | 20 | fail_support (20/20) |
| suite_P01_A_1c_pre6 | 20 | fail_support (20/20) |
| suite_P02_A_1c_pre10 | 15 | fail_support (15/15) |
| suite_P03_A_2c_pre8_prio | 10 | fail_support (10/10) |
| suite_P04_B_1c_pre8_shelf | 20 | fail_support (20/20) |
| suite_P05_C_2c_pre8_shelfprio | 20 | fail_support (20/20) |
| sample_config::000 | 20 | fail_support (20/20) |
| sample_config::001 | 20 | fail_support (20/20) |

`strict_support`(`MIN_UNION_SUPPORT_RATIO_STRICT`=0.75、B01-B04・P04・sample_config::001の
5シーンが該当)・通常(0.55、他13シーン)のどちらの閾値のもとでも結果は同じ
`fail_support`一色であり、**閾値の緩急に関わらずこの経路で落ちている。**

---

## 1-4: 段階1〜4の解釈(非ブロッキング要因であることの確認)

集計だけを見ると「y層規律(段階3)で345/345件落ちている」ようにも見えるが、これは
**実際の探索を止める原因ではない。** `planner._search_best`のlevelループは
「現在開放中の層で1件も合法手が見つからない場合のみ」次の層を開放し、
`Y_SLICE_COUNT=2`(既定・`WALL_MODE`は既定False)では**level1が必ず全開放
(`_y_slice_bounds`の最終要素)になる**構造になっている。Phase64で確認済みの通り
`planner.plan()`は極端に大きい予算(1e15、実消費の最大2,300万倍)でも27件中0件しか
見つけられておらず、**予算不足で全開放levelに到達できなかった、という可能性はPhase64で
既に否定されている。** したがって全開放levelでも本解析と同じ`_evaluate_candidates`が
同じ`fail_support`を返したはずであり、段階3の「level0不通過」はこの局面での
最終的な不採用理由ではない(§段階5'で最近傍実候補点でも同じ`fail_support`が出ている
ことが、この解釈と整合する)。

段階2(`_unique_orientations`)・段階4(優先コンテナtier)は345件中1件も
ブロッキング要因にならなかった(orientationの列挙漏れ0件、tierはランキングの
優先順位を変えるだけでハード排除ではない)。段階1(候補生成)も、最遠でも21.18mmと
plannerのグリッド間隔(BASE_GRID_DENSITY≈30mm間隔)の範囲内に収まっており、
Phase64の結論(「探索していない座標ではなく、探索範囲のすぐ隣か同一点」)を
本フェーズでも再確認した。

**したがって345件全てにおいて、実際にplannerを止めている単一の条件は
`_evaluate_candidates`内の`support_ok`(支持品質判定)である。**

---

## 2. 判定(報告のみ・実装なし)

### 2-1: この条件が何のために存在するか、緩めた場合に何が壊れうるか

`support_ok`は以下の式で決まる(`planner.py` L940-951、抜粋):

```python
union_ratio = MIN_UNION_SUPPORT_RATIO_STRICT if strict_support else MIN_UNION_SUPPORT_RATIO
...
stacked_ok = (sum_ratio >= MIN_SUPPORT_RATIO) | ((sum_ratio >= union_ratio) & balanced)
support_ok = on_floor | (stacked_ok & ~forbidden_hit)
```

- `MIN_UNION_SUPPORT_RATIO`(0.55)・`MIN_SUPPORT_SPAN_RATIO`(0.6)・
  `MAX_SUPPORT_CENTROID_OFFSET`(0.15)は**Phase11で導入**(commit `9d65292`)。
  「複数の支持面にまたがって乗る」着地を許可する条件で、根拠は
  「支持点が荷物の底面を広く・偏りなく囲んでいるか」という物理的安定性の代理指標。
- `MIN_UNION_SUPPORT_RATIO_STRICT`(0.75)・span/centroidのSTRICT版は**Phase13で導入**
  (commit `379ebc4`)。offline optimize無効なシーン(事前の順序検証が無い)限定で
  より保守的な閾値に切り替える。

**どちらも「実測に基づかない安全側の推測」(Phase60の`SAFETY_MARGIN_XY`のケース)ではない。**
Phase11 §5.2 は union/span/centroid を **0.55/0.6/0.15 → 0.70/0.7/0.10 → 0.80/0.8/0.08**
の3水準で実測しており、結果は次の通り(4シーンサンプル):

| 設定 | B01 fill / stability | A01 fill | P06 fill | C03 fill |
|---|---|---:|---:|---:|
| **s0 = 0.55/0.6/0.15(現行の緩い方)** | 28.00 / **94.20 ✗** | 24.05 | 20.08 | 28.97 |
| s1 = 0.70/0.7/0.10 | 21.61 / 96.92 ✗ | 24.05 | 14.42 | 28.97 |
| s2 = 0.80/0.8/0.08(締める方向) | 17.49 / **98.44 ✓** | 24.48 | 13.97 | 27.00 |

**現行値(0.55)は3水準のうち最も緩い側であり、その状態で既にB01のstabilityが
97制約に抵触している(94.20)。** 締める方向へ動かすとstabilityは改善するがfillを失う
(s2でB01 -10.51 / P06 -6.11)という実測済みのトレードオフが存在する。
Phase13はこの反対方向ではなく、「optimize無効シーンに限定してだけ締める」
(strict_support、0.75)ことでB01のstabilityを94.20→**97.16**まで回収した
(コストはB01自身のfill -4.50のみ)。

**この実測パターンから素直に読める含意は、「緩めればfillは伸びるが、stabilityは
悪化する方向へ動く」という逆向きの実測関係が既に存在するということである。**
Phase65が特定した345件は、まさにこの`support_ok`によって「積みたいが支持が
不十分」と判定されて弾かれている候補であり、**この条件を緩めることは
Phase11 s0→s1→s2のグラフを逆方向にたどることに相当する**可能性が高い
(確度100%の断定ではない——345件がPhase11実測時とは異なるシーン・異なる局面
であるため、実際の効き方は改めて計測が要る。ただし「緩めれば安全側に効く」と
期待できる実測的根拠は無く、むしろ逆方向の実測結果がある)。

加えて`support_ok`には`forbidden_hit`(非優先/非ソフト荷物が優先/ソフト荷物の上に
乗るのを禁止する下敷き防止ハード制約、Phase9由来)も含まれている。345件の
`fail_support`が「閾値未達」由来か「下敷き禁止」由来かは、`_evaluate_candidates`が
現状この2つを区別して返さない(統合された`support_ok`の可否のみを返す)ため、
本フェーズの計測だけでは切り分けられていない。**これは仮説ではなく「現状の
診断フックの解像度の限界」として明記する。** 切り分けるにはplanner.py自体に
一時的な診断出力を追加する必要があり、それは「この段階でplannerを変更しない」
という本フェーズの制約に反するため実施していない。

### 2-2: 犯人は複数か、シーンごとにばらばらか

**該当しない。** 犯人は単一(`support_ok`)で、18シーン全てに共通していた
(§1-2参照)。「単一の修正では解けない」という懸念は本件には当てはまらない
——ただし当てはまらない代わりに、**この単一条件が本当に安全に緩められるかどうかに
懸賞が全て懸かっている**ことを意味する(§2-1のトレードオフ参照)。

### 2-3: 修正の規模と期待できる上限(楽観的な数字を出さない)

- **理論上の上限は Phase64 の 18/27(67%)のまま、上振れしない。** 本フェーズは
  「どこで落ちるか」を特定しただけで、9件の genuinely 詰みのケースを新たに
  救う材料は何も得ていない。
- **18/27全てで効果が出る保証もない。** `support_ok`を緩めた場合、
  (a) その候補が本当に選ばれるとは限らない(同じターンで他の候補のスコアが
  上回れば選ばれない)、(b) 選ばれて1個置けたとしても、次の手で
  またすぐ`support_ok`(または他の条件)に引っかかって詰む可能性がある
  (指示にある「1手伸びたあと、次の手でまた詰まる」割引)。Phase54が
  「あと1手の価値 +0.14pt/シーン(下限値)」を計測した前例があり、
  同水準かそれ以下を想定するのが妥当な出発点である。
- **下方リスクが上方期待より具体的に見えている点が本件の特徴。** Phase11/13の
  実測は「この閾値を動かすとstabilityとfillがシーソーになる」ことを既に
  示しており、緩めることによるfill/placementの増分よりも先に、
  stability制約(97以上)への抵触再発リスクを実測で確認する必要がある。
- **修正規模の見積り:** 実装自体は小さい(閾値定数の変更、または
  `strict_support`と同じパターンで新しい条件分岐を1本足す程度)。しかし
  検証コストは大きい——Phase11 §5.2 と同水準(3水準以上 × 複数シーン ×
  fill/stability両計測)のスイープが要る。**「緩めてみて壊れなければ勝ち」
  という一発実装では判断できない**、というのがPhase11/13の教訓そのものである。

---

## 生成物一覧

- `tools/phase65_filter_trace.py` — 新規・読み取り専用。Phase64の合法解345件を
  plannerの実パイプラインへ順に通し、落選段階を記録する。
- `results/phase65_filter_trace.json` — 18シーン・345件ぶんの段階別トレース生データ。
- `results/phase65_report.md` — 本報告。

**`agents/mysolver/`配下は本フェーズ中一切変更していない**(指示通り、実装は行わず
報告のみ)。
