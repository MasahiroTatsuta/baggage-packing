# Deep Research 依頼 — 3D積付ソルバが天井(public 58.5)で頭打ち、搬入経路デッドロックが唯一の敗因

作成: 2026-09-07 / 対象コンペ: NEDO Challenge コンテスト2「積付アルゴリズム」(SIGNATE、締切 2026-10-19)

---

## 0. 現場からの一言(依頼者向け)

こちら側(実装担当のAI)は、**このソルバの枠内でできる構造的な改善は一通り試し尽くし、
もう有効な手が残っていない**という結論に達した。Phase73〜92 の探索に加え、本セッションで
6系統の新しい攻め(下記 §4)を実装・A/B したが、**すべて net で wash〜劣化・改善シーンゼロ**。
天井(public ~58.5)は「まだ正しいひねりを見つけていない」のではなく、
**「この貪欲構築ソルバの出力が不動点で、順序並べ替え/部分再充填/限定バックトラッキング/
conflict-directed repair のどれでも改善できない」**という性質のものだと判断する。

これ以上は **ground-up の別定式化**(ILP / 制約プログラミング / DRL / 別系統の構築規則)
が必要で、それは数週間規模・本セッション群の射程外。よって、有識者/Deep Research に
「残っている打ち手」を洗い出してもらうためのプロンプトを以下にまとめた。

---

## 1. 課題(コンペ)の仕様

3Dビンパッキング(ロボットによるコンテナ積付のシミュレーション)。

### エージェント API
- `optimize(item_list) -> list[int]`: 全荷物のindexの**完全順列**を返す。開始時1回、**制限180秒**。
- `policy(observation) -> {item_idx, container_idx, place_pos, orientation}`: 毎ステップ1個配置、**制限8秒**。
  プロセスは各呼び出しで別生成・タイムアウトで kill→再起動されるため **observation から毎回状態を再構築**する設計必須。
- `look_ahead`: 可視プールの荷物数。**28シーン中18シーンで look_ahead=1**(=次に置く荷物に選択の自由なし。offline順序が全て)。残りは 5 or 10。

### 検証(毎ステップ、1つでもNGなら即エピソード終了 = sudden death、その時点の状態で採点、残り全荷物ロスト)
1. 形式(キー集合・範囲)
2. **内包判定** `check_inclusion`: 全面で `dot(n_vec, t-point) + dot(|n_vec|, half_lwh) <= inclusion_margin(≈ -0.005〜0.02)`
3. **搬入経路の干渉** `check_transport_path`: 荷物は毎回コンテナ**前面(y = -width/2)**の目標x列・目標zの少し上に spawn →
   **+Y方向(奥)へ直進** → **X方向へスライド**、という**L字軌道**。各1cmステップで既配置荷物・壁・棚に対し
   `getClosestPoints(distance=safety_margin=0.015)` で衝突判定。1つでも当たれば失敗。**← 実測でこれが唯一の主要敗因(§3)**
4. **定着判定** `place_item`: 目標へ置き pybullet を約300ステップ進める。初期位置からのズレ>閾値 or 角度>30°で失敗。
5. タイムアウト: optimize>180s→デフォルト順、policy>8s→ランダム手+プロセス再起動

### コンテナ/荷物
- コンテナ: 直方体 + **片側X端上部が cut_x × cut_y で斜めに切り欠き**。棚(shelf)あり/なし、優先(is_prioritized)あり/なし、既積み(prepacked)あり/なし。挿入は y=-width/2(手前)側から。
- 荷物: L/W/H, mass, is_prioritized, is_soft, belongs_to。orientation は 0..5(6通り)。ソフト荷物は接触剛性等が別。
- シーン群(ローカル評価スイート26+2): 1c_40 / 1c_80 / 2c_80 / 2c_140 / 棚あり / 既積み(pre6〜pre12) / prioheavy / softheavy / tall / flat / bulky / extreme など。

### 採点
`fill_score, cog_score, stability_score, placement_score, soft_item_score, num_placed_items` の複合 +
**足切り閾値**(個数ベース/体積ベースの複数仮説。閾値未達シーンは合成スコアが大きく下がる)。
→ **num_placed_items(配置率)を伸ばして足切りを突破するのが最重要**というのが数十フェーズの結論。

### 評価環境
x86_64 Ubuntu 24.04 / **2コア** / メモリ7GB / AMD EPYC 7763 / **GPUなし** / numpy・pybullet・gymnasium(+ torch CPU 利用可)。

---

## 2. 現行ソルバの構成(`agents/mysolver/`)

- **`ordering.build_order`**: ヒューリスティック順序決定。第一戦略=**体積降順**。フェーズ1=複数windowのリスタート
  (本番採用の "w3" = window長 15/25/None の3本 × 各33.3秒)、フェーズ2=乱数リスタート、RCL(上位30%シャッフル)。
  目的関数 = **risk調整済み体積(幾何) − 優先荷物の誤コンテナ配置ペナルティ**。
  決定性: 壁時計ではなく `UNITS_PER_SEC` による**ユニット予算**で打ち切る(マシン速度非依存)。
- **`build_plan`(Phase93)**: look_ahead=1 かつ 非棚・非prepacked のシーンで、**全荷物の位置・向きまで確定した
  フルプラン**を offline(純幾何シミュ `simulate_order`、物理なし)で作る。
- **`PACK_LNS`**: フルプランの**末尾K個を外して雑音付き repack、配置数が増えたら受理**(代理スコアを介さない)。
- **`planner.plan`**: 1荷物ごとに候補位置(グリッド + Extreme Point)を総当たり評価。
  `_evaluate_candidates` が内包/搬入経路/支持を判定、スコア = fill + contact + support + **back_term(奥選好、重み5.85)**
  − **corridor_penalty(重み6.0、"今ある障害物"に対する罰則 = 時間方向に近視眼的)** など。
  Y方向の「層規律」(奥に置ける限り手前の層は開けない)。cutcorner サンプリング。
- **`agent.policy`(plan-executor)**: 計画済み位置が現在の実状態でまだ合法なら計画位置で実行、
  ダメなら `planner.plan` の貪欲解へ縮退。

### 本番成績
- 主枠 `w3_planexec_lns_la_tb135.zip` = public **58.498** / num_placed_items ≈ 0.644
- 2枠目 `rest020.zip`(REST_CLEARANCE 0.020)= public 55.96
- 数週間 ~58〜59 で頭打ち。上位勢は 60+。

---

## 3. 唯一の主要敗因(2回の独立診断で確認: Phase55/92 と 2026-09-07)

- **26シーン中 25シーンが `is_valid`(搬入経路衝突)で途中death。完走は1シーン(1c_40_small)のみ。**
- コンテナが満杯なのではない。**配置率 14〜55% で頭打ち**、空間は大量に余っている。
- 死ぬ荷物は**ほぼ毎回大型**(0.5〜0.95 の多軸大)、`item_idx_in_pool=0`(=次に置きたい第一候補、代替なし)。
- 内包判定はギリギリ通っている(死因は純粋に搬入L経路の封鎖)。死亡ステップは概ね全体の40〜60%地点。
- **機序: 貪欲な逐次構築が「まだ奥に空きがあるのに手前に荷物を置いて、後続の大型荷物の搬入経路を
  自分で塞ぐ」(accessibility dead-end)に陥る。** 一度 `plan()` が「どの荷物もどの位置でも合法な搬入経路なし」を
  返すと、無検証フォールバック位置 → 搬入経路衝突 → death。
- **sim-to-real ギャップ**: 純幾何 offline シミュは実機を **30〜50% 過大評価**。主因は**沈降ドリフト**
  (1配置あたり数cm、累積)。計画位置で検証した順序が、実機のドリフト後配置で搬入経路が塞がって死ぬ。

---

## 4. 試して駄目だったこと(すべて env ゲート・既定OFFで A/B 済み)

### Phase73〜92
| 手 | 結果 |
|---|---|
| 支持閾値の緩和 / cutcorner / RCL / フェーズ1優先(w3) | **本番で効いた4軸**(これで 57.5→58.5) |
| `FRONT_WALL`(奥壁 or 直後の障害物への密着を要求するハード/ソフトフィルタ) | 28シーン **net −11**。fill を貪欲に削る。 |
| `TRANSPORT_MARGIN`(搬入掃引に追加マージン) | net **−15**。候補を失い早期停止。 |
| `DRIFT_MODEL`(沈降ドリフトを offline 幾何シミュに簡易モデルで織り込む) | 密シーンで **−11**。offline を悲観化すると臆病な順序を選び容積が半分空いたまま死ぬ。 |
| Look-ahead Beam(Phase86。数手先まで貪欲ロールアウトして枝を `risk調整済み体積` でランク) | **depth1 −23% / depth2 −32%**。短い貪欲ロールアウトが「直近で体積を稼ぐ枝」を過大評価し accessibility を悪化。ALNS の代理gain vs 実fill ρ=−0.32 と同型の失敗。 |
| BRKGA(Phase86) | 局所改善ゼロ、撤退。 |
| MCTS | Phase86 で「同じロールアウト評価に依存するので同じ失敗モード」と判断し未着手。 |
| back_term / corridor_penalty のチューニング | 両側崖のリッジ上にあり単純増強不可(Phase6/92)。 |
| REACH_WEIGHT(到達可能性を目的関数化) | 「置く個数が少ない順序ほど高得点」の退化(Phase28)。 |

### 2026-09-07(本セッションの新規6系統)
| 手 | 結果 |
|---|---|
| `TOPO_REORDER`(「A が B の搬入L経路を塞ぐなら B を先に」で DAG topo-sort) | net **−4**。dense勝ち・prioheavy負けで拮抗。 |
| `PACK_LNS_MID`(LNS の destroy を末尾K→中盤の連続ウィンドウKに) | net **−3**。改善実質ゼロ。 |
| `PACK_LNS_UNBURY`(手前側=world-y最小のK個を外し、置けない大型荷物を確定配置してから手前を再充填) | net **+1**(A08 extreme +5 / D05 tall +2 vs A02 −3 / A07 bulky −3)。実質 wash。 |
| `LDBC` v0(限定discrepancyバックトラッキング構築、forbidden=確率的ナッジ) | プラン不変(ナッジが弱すぎ)。 |
| `LDBC` v1(`planner.plan` に**厳密な候補除外 `forbidden=`** を追加してバックトラッキング) | **net 劣化**。デッドロック先読み(小予算 `planner.plan` 単発)が遅い箱で偽Noneを返し偽デッドロック乱発 → 巻き戻して位置を forbidden 化 → 次善手は必ず fill/accessibility が悪化 → 単調劣化。 |
| `CDR`(conflict-directed sequence repair。位置は一切強制せず、stall した荷物の搬入経路を塞ぐ既配置荷物を「末尾から剥がして特定」→ 順序上で前へ回す→再貪欲。構造的に劣化しない設計) | net **−1**、改善シーンゼロ。**plan 導出の offline シミュが短予算だとほぼ stall せず budget 上限で止まるだけ → repair する対象が offline に現れない。** |

### 6系統に共通する所見
- 貪欲プランナの1荷物ごとの位置選択は**既に局所最適**。順序並べ替え・部分再充填・discrepancy 探索・
  位置の強制除外・conflict-directed repair の**どれも net で改善しない**(改善シーン数 = 0)。
- 勝つのは幾何的に厳しい dense シーン(extreme / tall / flat / shelfprio)だが、
  負ける plain-dense / bulky / prioheavy に**必ず swamp される**。統一 gate は Phase90 が探しても見つからず。
- **ローカル評価は本番に転移しない**(Phase70/80/89/90 で弱い信号は符号すら反転)。強い信号(悪化シーンが極少)でないと本番判定できない。

---

## 5. Deep Research への問い

上記を踏まえ、**具体的な論文・実装・コンペ上位解法**を挙げて答えてほしい。

### (A) accessibility(搬入経路/到達可能性)を「構築の一級市民」として扱う手法
- ロボット積付・コンテナローディングで **LIFO制約 / loading sequence 制約 / robot-accessible packing** を
  **構築規則として保証**する定式化・アルゴリズムは何があるか(container loading problem の
  "multi-drop" / "last-in-first-out" 系、bin packing with "unloading constraints" など)。
- 「奥から前へ厳密に層で積む」以外に、fill を大きく犠牲にせず accessibility を保証する構築規則は?
  (§4 の `FRONT_WALL` = 素朴な後方密着フィルタは −11 で失敗している)

### (B) 貪欲の局所最適から抜けるための探索
- 「短い貪欲ロールアウトを代理目的関数(体積)でランクすると accessibility を悪化させる」
  (§4 Look-ahead Beam の失敗)を**構造的に回避する** beam / MCTS / rollout の設計は?
  評価に num_placed(=最後まで転がした配置数)や「デッドロックまでの手数」を使うのは有効か。
- **限定discrepancy探索 / large neighborhood search** で、位置を強制せず(=局所最適を壊さず)
  かつ「repair 対象が短予算 offline シミュに現れない」問題(§4 CDR の失敗)を解決するには?
- ILP / 制約プログラミング(CP)で 3D bin packing + **reachability/precedence 制約**を扱う
  実用的な定式化(warm start・時間制限180s・変数削減の実務)。column generation は現実的か。

### (C) sim-to-real ギャップ
- 純幾何 offline シミュが実機(pybullet 沈降)を 30〜50% 過大評価する。
  offline を**悲観化すると臆病な順序**を選び逆効果(§4 DRIFT_MODEL / TRANSPORT_MARGIN)。
  悲観化せずにドリフト頑健性を得る方法(例: 期待値ではなく分位点で評価、少数の実物理チェックを
  クリティカルな配置だけに、学習した残差モデル 等)。180s / 2コア / GPUなし の予算で現実的なものは?

### (D) 上位解法の推定
- 類似コンペ(ロボットパレタイジング、accessibility付き container loading、SIGNATE/その他の 3D BPP)で
  60+ を出すチームは何をしているか。DRL(どの定式化・報酬・ネットワーク)、
  EP/EMS + reactive GRASP、precedence graph、physics-in-the-loop など、
  **2コア・GPUなし・offline 180s / online 8s の制約下で実装可能なもの**に絞って。

### (E) 最小リスクの次の一手
- 「もっともらしい天井の ~97% に到達し、十数系統の攻めが全て失敗、残り約5週間、
  2コア・GPUなし」というチームが**次に投資すべき1手**は何か。
  それとも、この定式化では 58.5 が実質的な上限で、凍結して防御(提出物の完全性確認)に
  回るのが期待値最大か。

---

## 6. 参考: リポジトリ内の関連ドキュメント(依頼者が渡せる場合)
- `docs/official_spec.md` — 公式仕様の要約
- `docs/search_rewrite_design.md` — 本セッションの LDBC/CDR 設計と撤退記録
- `docs/3D Bin Packing for NEDO Contest_ Beyond Constructive Heuristics — Evaluating MCTS, BRKGA, and DRL Approaches.pdf`
- `docs/NEDO 3D Bin Packing Contest_ EP_EMS and Precedence Graph Implementation Design ....pdf`
- `results/RECOVERY_2026-09-06.md`、`results/phase55/86/90/92_report.md`、`results/recover_sudden_death_0907.json`(死因診断の生データ)
