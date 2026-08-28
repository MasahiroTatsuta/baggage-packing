# Phase80 報告: `BEAM_SOFT_LAST` 不採用、配置数を増やす軸の探索終了、今後の方針

## 本番結果(先出し)

`mysolver_submit_loose2_softlast.zip`(緩2 + `MYSOLVER_BEAM_SOFT_LAST=1`):

| | 緩2(主枠) | softlast | Δ |
|---|---:|---:|---:|
| public | 57.18 | 55.68 | **−1.51** |
| num_placed_items | 64.34% | 63.37% | **−0.97pp**(ローカル予測 +0.89pp と逆方向) |
| soft_item | — | — | **−9.90**(換算寄与 −1.41、崩壊の94%) |

**不採用。** window内でハードを先に積み切る制約が探索の柔軟性を奪い、配置数
そのものを減らす方向に効いた。

---

## ステップ1: 後片付け

- (1-1) `MYSOLVER_BEAM_SOFT_LAST` の既定は `agents/mysolver/simulate.py:68`
  で `os.environ.get('MYSOLVER_BEAM_SOFT_LAST', '0') == '1'` のまま。**変更不要
  であることを確認した(実際に変更していない)。**
- (1-2) `submissions/mysolver_submit_loose2_softlast.zip` は削除せず保管
  (存在確認済み、122103 bytes)。`docs/submission_policy.md` の Phase78行
  および Phase1節に、本番結果と不採用の判定を追記した。
- (1-3) 主枠(緩2, public 57.18)・2枠目(`rest020`, public 55.96)は変更なし
  (`docs/submission_policy.md` §5、変更していないことを確認)。

---

## ステップ2: 現状の総括

Phase73〜80で試した以下がすべて「打ち止め」と判定された:

| 軸 | 結論 | 根拠フェーズ |
|---|---|---|
| 支持閾値3定数(union/span/centroid) | centroidのみ有効。頂上確認済み(0.225→57.06/0.25→57.18/0.275→56.94) | Phase73〜75 |
| 幾何定数3つ(INCLUSION_MARGIN/SAFETY_MARGIN_XY/REST_CLEARANCE) | 現行値が最適点。REST_CLEARANCEは片側−48.0の非対称な崖 | Phase75〜77 |
| ビームwindowのis_soft順序制御 | 本番で逆効果(−1.51)、不採用 | Phase78/80 |
| オフライン順序の過去4フェーズ再考(Phase24/28/29/34) | 独立理由(t値の構造的上限・代理精度)で不採用、cutoffとは無関係と再確認 | Phase79 |
| ALNS | 代理関数の精度不足(ρ=−0.321、本物の評価器で測定済み)、cutoffとは無関係 | Phase34/Phase79 |
| ρ-test(REPLICA_SELECT) | 規約上これ以上の調査手段が尽きている(診断ビルドは規約違反) | Phase44〜61/Phase79 |
| 死亡局面345件のcentroid再解析 | 緩2閾値でも0/345が救済。centroidの効果は「エピソード序盤からの軌道変化」であって「最後の1個の救済」ではない | Phase79 |

**配置数(num_placed_items)を増やす経路は、Phase73〜80の棚卸しにより実質的に
尽きたと判断する。** 現在のベストは緩2(public 57.18)。53.64からの純増+3.54は
すべてcentroid一軸に由来する(Phase74の分離実験で確定)。

進捗まとめは `README.md` §6(「学んだこと/今後の課題」)に1節として追記した。
特に以下の教訓を明記した:

> **ローカルA/Bの弱いシグナルは、本番で符号ごと反転しうる。** 緩2はPhase67の
> ローカル測定で`stability t=−1.93・悪化13件`と不採用寄りの判定だったにも
> かかわらず、本番では緩1を上回る57.18を記録した。逆にPhase78のsoftlastは、
> ローカルA/Bで`num_placed_items +0.89pp`という弱いが方向は正のシグナルだった
> にもかかわらず、本番では−0.97ptと符号が反転した。t値が小さく(t<2相当)、
> per-sceneの挙動がカオス的(改善と悪化が拮抗)なローカル結果は、本番投入の
> 可否を決める根拠として弱い。

---

## ステップ3: 残された可能性の再確認

### (3-1) 死亡局面以外(エピソード全体)でのフィルタ発火頻度計測の価値判定

**一言判定: 価値なし。実施しない。**

#### 規模の見積もり

実装: `agents/mysolver/agent.py`の`policy()`から`planner.plan()`へ、シーン単位
(または全体)で共有する`stats`辞書を通し番号なしで貫通させ、`_evaluate_candidates`
の全呼び出し(1ステップあたり container×item×orientation の全組合せ)で
集計する改修。既存の診断カウンタ(Phase65/66で実装済み)をそのまま使い回せる
ため、コード自体は数十行規模(半日以内)。

実行: 意味のある計測には、本番相当の時間予算(120s前後)で26〜30シーンを
最低1パス通す必要がある(死亡局面だけでなく全ステップの内訳を見るのが目的
のため、短い予算に切り詰めると本番と乖離した挙動を測ってしまう)。
**26シーン×120s ≈ 52分**が最低ライン、ノイズ除去に複数パスが必要なら
数時間規模になる。

#### 「価値なし」と判定する構造的な理由(単なるコスト論ではない)

`plan()`が実際に`None`を返す(=そのステップで一切配置できない)のは、
`_search_best`が(container×item×orientation×xy)の**全組合せ**を尽くして
1件も合法手が無かった場合に限られる。逆に言えば、ある(item, orientation, xy)
の組合せが`_evaluate_candidates`で`None`(fail_support等)を返しても、
**他の組合せのどれか1つが合法であれば、探索はその手を採用してそのまま
配置数に貢献する**——個々の`_evaluate_candidates`の失敗は、`plan()`全体が
`None`を返すという「配置数を減らす事象」の**必要条件でしかなく、
十分条件ではない**。

したがって、num_placed_itemsという指標に影響するのは「ある(item, orientation,
xy)の失敗」の総数ではなく、**「そのステップの全組合せが同時に失敗した」
という事象(=plan()がNoneを返した回数)** だけである。そして、Phase62の
実測によれば、`plan()`がNoneを返すと`agent.py`は無検証の`_fallback_place_pos`
に落ち、観測された27件全てで即座にis_valid違反(sudden death、エピソード
終了)を引き起こしている。つまり**`plan()`がNoneを返す事象は、実質的に
「そのシーンの配置数がそこで確定する」事象と同一である**(全27件で例外なく
観測された関係であり、理論上これが常に成り立つ保証まではないが、これまでの
実測範囲では反例が無い)。

**この「意味のある」事象(plan()がNoneを返す瞬間)は、Phase64〜66・79が
既に網羅的に測定済みである**(27シーン全件、345件の候補サンプル、既定閾値・
緩2閾値の両方)。エピソード全体の全ステップで`_evaluate_candidates`の
内訳を集計しても、その大部分は「他の組合せが成功したので配置数には
影響しなかった失敗」というノイズであり、**num_placed_itemsという主KPIの
説明力という観点では、死亡局面だけを見た今回の測定が既に十分**である。

#### 推測「エピソード全体でもsupport以外が主要因である可能性は低い」の妥当性

指示文が示した推測(死亡局面でsupportが支配的=345/345だったので、全体でも
support以外が主要因になる可能性は低い)は、**上記の構造的理由により、
そもそも問う意味のある推測ではない**——「死亡局面以外」の失敗は、定義上
`plan()`全体をNoneにしないので、どの段階が主要因であっても
num_placed_itemsには影響しない。推測自体の真偽を判定する意味が無いことが
分かった、というのが正確な結論である(「おそらく正しい」という弱い肯定
ではなく、「問い自体が指標との関連を持たない」という強い形の否定)。

### (3-2) 配置数の軸の宣言

**(3-1)が「価値なし」であるため、配置数(num_placed_items)を増やす軸は
完全に閉じる。** Phase73〜80で支持閾値・幾何定数・順序制御(オンライン/
オフライン双方)・ρ-testのすべてを尽くし、残る唯一の未測定項目
(死亡局面以外でのフィルタ発火)も構造的に無意味と確定したため、これ以上
このリポジトリ内で配置数を増やす新しい実験を行う理由は無い。

---

## 残された可能性が見当たらない場合のdeep researchの進め方(提案・未実施)

配置数の軸が閉じた以上、次に投資すべきは**public score全体**であり、
Phase70の内訳分解(loose1の利得: stability 36.0%・soft_item 30.8%・
cog 26.3%・placement 5.0%・fill 1.8%)が示すとおり、配置数の副産物として
動いた成分(stability/cog)を**直接の設計目標にする**余地がまだ検証されて
いない。以下、実施はしていない提案として記録する。

### 提案1: cog/stabilityを「副作用」から「直接の目的関数」へ

現状の`agents/mysolver`は、支持面の閾値(union/span/**centroid**)という
**静的な幾何条件**でのみ安定性を扱っている。今回の簡易な文献調査
(WebSearch、下記Sources参照)によると、近年のロボットビンパッキング研究は
これをさらに一歩進め、**力・トルクのつり合い解析**(force/torque balance
analysis)や、**多層スタックの重力分布推定**(gravity distribution
estimation for multilayered packing)によって安定性を直接評価する手法へ
移行している。また、「center-of-gravity polygon support」(重心が支持
多角形の内部にあることを要求する)という制約が、単純な full-base/
partial-base support よりも一貫して性能が良いと報告されている——これは
`MAX_SUPPORT_CENTROID_OFFSET`(重心オフセット制約)と設計思想が一致して
おり、**現状の設計方針自体は文献的に妥当**なことの傍証になった。

一方、現状は「支持面(1つ下の層)との関係」しか見ておらず、**スタック
全体の力の伝播**(2段下・3段下への荷重集中、通路封鎖と同様に「その後の
積み上げで初めて生じる」動的な効果)は評価に入っていない。これは
Phase24の`corridor_penalty`が抱えていた「時間方向のmyopia」と同型の
限界であり、影シミュレータ(`simulate.py`)側でスタック単位の簡易な
力学近似(例: 各荷物の重量を下の支持面へ再帰的に分配し、コンテナ床面での
反力分布のばらつきを罰則にする)を追加する余地がある。

### 提案2: soft_item/placementのローカル再現性を、別角度から取り直す

Phase44〜47で「配布sample_configですら満点」という形でローカル再現に
完全失敗した経緯があるが、これは`_find_stacking_pairs`のpybullet接触
判定に依存した測定だった。Phase80のsoftlast失敗(soft_item −9.90)は、
**soft_item_scoreが「下敷きの有無」だけでなく、もっと広い条件(例えば
配置順序そのもの、コンテナ内の分布)に反応している可能性**を示唆する。
規約(内部パラメータの逆算禁止)に触れない範囲で、**符号だけを見る
小さな本番プローブ**(例えば、ソフト貨物の配置順序だけを変え他は緩2の
まま固定した1本を提出し、soft_item_scoreの符号だけを見る)を、
public score全体ではなく**成分ごとの感度分析**として計画的に行う価値は
まだ残っている(Phase70以降、成分ごとの感度は基本的に「主レバーの副産物」
としてしか観測しておらず、単一成分を狙った実験は未実施)。

### 提案3: 本番投入の統計的設計を見直す

Phase67→70→71→73の経緯(ローカルで境界的だった緩1・緩2が本番で当たり、
緩3は外れ、softlastも外れ)から、**本番フィードバックは実質的に唯一
信頼できる評価チャネルであり、かつ1回の投入コストが高い**(結果を見るのに
日単位の遅延があると推測される)。今後は「1軸1回の投入」ではなく、
**事前に判定基準(閾値・方向)を固定した上で、直交する複数の小さな変更を
1回の投入にまとめてバッチテストする**(例: cog向けの重み1つ + soft_item
順序1つ、を独立成分として1本にまとめ、本番のnum_placed_items/各成分の
両方をログして事後にどちらが効いたかを近似的に切り分ける)ことで、
限られた投入回数からより多くの情報を得られる可能性がある。厳密な
factorial designにはならない(交互作用は分離できない)が、**「打ち止め」
と「文献的に妥当な新しい方向」の両方が出揃った今の局面では、次の1手を
慎重に設計する価値がある**。

### Sources(提案1の裏付けに使用したWeb検索)

- [Collaborate sim and real: Robot Bin Packing Learning in Real-world and Physical Engine](https://arxiv.org/html/2511.19932)
- [A greedy search for the three-dimensional bin packing problem: the packing static stability case](https://www.academia.edu/26020453/A_greedy_search_for_the_three_dimensional_bin_packing_problem_the_packing_static_stability_case)
- [Static stability versus packing efficiency in online three-dimensional packing problems](https://www.sciencedirect.com/science/article/abs/pii/S0305054825000334)
- [A three-stage layer-based heuristic to solve the 3D bin-packing problem under balancing constraint](https://www.sciencedirect.com/science/article/pii/S1319157821001749)
- [Stable Bin Packing of Non-convex 3D Objects with a Robot Manipulator](https://motion.cs.illinois.edu/papers/ICRA2019-Wang-BinPacking.pdf)

**上記提案はいずれも未実施(報告・提案のみ)。** 実装するかどうかは
次フェーズの指示を仰ぐ。

---

## やっていないこと

- `agents/mysolver`のコード変更は一切行っていない(既定の確認のみ)。
- `MYSOLVER_BEAM_SOFT_LAST`の既定を有効にしていない。
- 本番の集計スコアから足切り閾値やシーン数を逆算していない。
- `.gitignore`の書き換え・force pushは行っていない。
- 提案1〜3はいずれも実装していない(提案のみ)。

## 変更ファイル

- `docs/submission_policy.md`(Phase80追記、§5の該当行更新)
- `README.md`(§6にPhase69〜80のまとめを追記)
- `results/phase80_report.md`(本ファイル)
