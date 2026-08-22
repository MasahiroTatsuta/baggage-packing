# Phase61 報告

## 0. 方針の変更(禁止事項の確定)

公式チュートリアルセミナー(2026年、SIGNATE/三菱総合研究所)44:35付近での運営発言:

> 「フィードバック結果から、評価基盤で実装されている評価関数の中に設定されている
>   パラメーターとか、手荷物の具体的な内容とかを解析したりするのは基本的にNGなので
>   やめていただきたい」

これを受けて:

- `docs/official_spec.md`(新規)に公式確定値をまとめた。
- `docs/submission_policy.md` §4 に追記し、Phase40までの「未確認」状態を
  「確認済み・禁止」に確定。ρ-test診断ビルド(実行時間への内部状態符号化)は
  完全に断念、`MYSOLVER_TELEMETRY`系は既定無効のまま凍結、セミナー口頭説明を
  一次資料に含める旨を明記。
- `README.md`「開発運用ルール」に恒久ルールとして追記。

**この確定を踏まえ、本フェーズの§1・§2は、依頼された作業内容の一部を実施していない。**
理由は次節で説明する。

---

## 1. 足切り(cutoff)— 実施内容と実施しなかったこと

### 実施しなかったこと

依頼の1-2「7候補のうち本番値に最も近い閾値はどれかを報告する」、および1-4「最も近い
組み合わせを今後の主KPIに採用する」は実施していない。これは非公開の評価関数パラメーター
(足切り閾値)を本番の集計スコアから逆算する行為であり、0節で確定した禁止事項に該当する
と判断したため。1節と2節は、この一点において依頼と異なる結果になっている。

### 実施したこと

`tools/scorer.py` は変更せず、`tools/rescore_with_cutoff.py`(新規)を作成。既存の
5指標フル計測(`results/phase60_diag_xy022.json` 26シーン、`results/phase47_sample_config_diag.json`
sample_config)と、`src/ground_handling/containers.py` の公式volume計算をそのまま
呼び出して抽出したコンテナ有効体積(`results/phase61_container_volumes.json`)を使い、
新規ロールアウトなしで事後再計算した。

**26シーン平均(合成スコア=5指標単純平均)**:

| 条件 | 合成スコア |
|---|---|
| baseline(足切りなし・定義A) | 77.46 |
| 定義Bのみ(未配置も違反に算入) | **53.64** |
| 足切り[count>=10]のみ(26/26 clear) | 77.46 |
| 足切り[count>=15]のみ(24/26 clear) | 72.04 |
| 足切り[count>=20]のみ(21/26 clear) | 63.63 |
| 足切り[count>=30%total]のみ(24/26 clear) | 71.96 |
| 足切り[count>=50%total]のみ(16/26 clear) | 49.81 |
| 足切り[volume>=30%container]のみ(21/26 clear) | 63.13 |
| 足切り[volume>=50%container]のみ(1/26 clear) | 7.60 |
| 足切り[count>=20]+定義B | 46.07 |
| 足切り[count>=50%total]+定義B | 38.13 |
| 足切り[volume>=30%container]+定義B | 42.73 |

sample_config(2タスク)も同様の傾向(baseline 75.91、定義Bのみ 54.77、
volume>=50%container のみ 0/2 clear → 5.08)。詳細は
`results/phase61_cutoff_rescore.json`。

### 所見

「定義Bのみ」(53.64)が既に本番相当の乖離を大きく縮めており、Phase59の結論(空白の
主因は配置率そのもの)を追認する形になっている。足切り候補の中では count 系・volume 系
とも閾値次第で結果が大きく振れ、**閾値を当てずに一般的な感度としてこれ以上絞り込むことは
できない**(絞り込もうとすると、まさに禁止されている「本番値への適合」になる)。
このため、足切りを主KPIに組み込むかどうかの判断はこのデータだけでは行わない。

---

## 2. 揺らしテスト(shake test)— 実施内容と実施しなかったこと

### 実施しなかったこと

依頼の2-2「本番の70.44に最も近い水準はどれかを報告する」は実施していない。理由は
1節と同じ(非公開パラメータの逆算に該当するため)。

### 実施したこと

`tools/scorer.py` は変更せず、`tools/diagnose_stability.py` の amplitude を
`MYSOLVER_DIAG_SHAKE_AMPLITUDE`(既定 '6.0')でenv化。公式開示レンジ全体
(1G=9.8 / 2G=19.6 / 3G=29.4 m/s²)で26シーン+sample_config(2タスク)の
stability_scoreを測定した(**production値との適合を狙った選定はしていない**)。

副次的に、既存の`diagnose_stability.py::main()`が複数タスクconfig(sample_config.json)で
最初のタスクしか処理しない既存バグを発見し修正した(26シーンのsuite_*.jsonは1ファイル
1タスクなので、この修正によるsuite側の結果への影響はない)。

**26シーン平均・分布**:

| 振幅 | 平均 | 最小 | 最大 |
|---|---|---|---|
| 6.0 (現行既定, Phase52実測) | 97.4〜98.8に密集 | — | — |
| 9.8 (1G) | 95.67 | 94.97 | 96.94 |
| 19.6 (2G) | 85.46 | 71.02 | 91.59 |
| 29.4 (3G) | 70.76 | 50.85 | 85.32 |

sample_config(2タスク平均):

| 振幅 | task 000 | task 001 |
|---|---|---|
| 9.8 (1G) | 94.94 | 95.32 |
| 19.6 (2G) | 83.91 | 87.20 |
| 29.4 (3G) | 76.15 | 71.73 |

### 所見

**Phase52の結論(「stabilityは物理エンジンの沈静化残差で上限+0.39」)は、揺らしが弱すぎて
差が出ていなかっただけだったという見立てを裏付ける結果になった。** 振幅を上げるほど
シーン間の分散が明確に開き(9.8では26シーン中1.3ptしかない幅が、29.4では34.5ptまで
拡大)、**攻略対象として扱いうる勾配が生まれている**。ただし「29.4が本番の70.44に近い
から本番の振幅は29.4だ」という結論は導いていない——それは非公開パラメータの逆算になる
ため。**言えるのは「公式開示レンジの上限付近で、ローカルでも本番オーダーの平均・分散が
再現できる」ということのみ**であり、これは今後stabilityをKPIとして扱う根拠として
十分と判断する(具体的な振幅値の断定は不要)。

揺らしの波形(2-4): 現行実装 `gx=A·sin(2πt/30), gy=A·cos(2πt·0.7/30)` は、周波数比が
無理数的でない(0.7倍の有理比)ため周期的ではあるが、単純な単一軸の往復よりは複雑な
リサージュ的軌道になっている。「単純な縦横ではなく複雑な動き」という公式説明とは
方向性として矛盾しないが、これ以上の一致度(位相・周波数の具体値)を追求すること自体が
非公開パラメータの推測に当たるため行わない。振幅を公式開示レンジに合わせるに留めた。

詳細データ: `results/phase61_stability_1G.json` / `_2G.json` / `_3G.json` /
`_sample_1G/2G/3G.json`。

---

## 3. 搬入経路のX方向掃引

### 3-1. コード調査

**依頼文の前提(「X方向の掃引が実装されていない」)は事実と異なる。** X方向掃引は
実装されている。

- **Y方向掃引**: `agents/mysolver/planner.py::_evaluate_candidates` 内の `legal1`
  (`phase1`)。搬入時のx(`start_x_world`、コンテナ入口付近にクリップ)を固定し、
  y方向に `y_entry`(コンテナ手前端)→目標yまで掃引する範囲でAABB重なりを判定する。
  `src/ground_handling/validator.py::check_transport_path` の「Y軸方向に動かす
  (target_0 = (start_pos[0], target_pos[1], start_pos[2]))」と対応。
- **X方向掃引**: 同関数内の `legal2`(`phase2`)。**実装されている。** y=目標yに固定し、
  start_x_world→目標xまで掃引する範囲で判定する。validatorの「X軸方向に動かす
  (target_1 = (target_pos[0], target_pos[1], start_pos[2]))」と対応。
  `legal = base_legal & legal1 & legal2` で両方を要求している。
  `_evaluate_candidates` の統計にも `fail_transport_x` が独立して存在する
  (Y失敗と区別して集計する設計になっている)。
- 一方、`_y_sweep_unreachable_mask`(事前枝刈り用のヒューリスティック、
  legal1相当のみを近似的に先読みして候補を間引く)はY方向専用で、X方向を扱わない。
  ただしこれは**最終判定ではなく高速化のための早期棄却**であり、生き残った候補は
  `_evaluate_candidates` で legal1・legal2 とも正規に評価される。関数名がY専用に
  見えるため誤解を招きやすいが、判定の抜け穴ではない。
- **掃引の高さ**: `sweep_z = min(ceiling_sweep, world_z + effective_start)`。
  `effective_start` は非直置き時 `geo.START_Z = 0.08`(= 8cm、`configs/`配布物の
  `start_z: 0.08` と一致)で、`validator.check_transport_path`の
  `effective_start_z`クリップ式(直置き判定・天井余裕クリップとも)をコメントで
  明記の上、同一式で複製している。
- **判定に使う形状・離散化**: planner側はAABB(軸並行境界箱、`box_overlap_batch`)による
  **解析的な区間重なり判定**(区間 [lo,hi] 同士の比較)であり、離散化ステップという
  概念自体がない(連続区間として厳密に判定)。一方、本物の`check_transport_path`は
  `step_len=0.01`(既定1cm)刻みで物理ステップを実行し、pybulletの接触判定に委ねる
  シミュレーションベースの判定である。マージンは `geo.SAFETY_MARGIN_XY = 0.022`
  (コメントに「実際は0.015程度」とあり、配布`sample_config.json`の
  `safety_margin: 0.015`と符合、余裕を持たせて大きめに設定)。

### 3-2. sudden death 26シーンの衝突位置集計

`tools/phase61_transport_phase.py`(新規、読み取り専用)で、公式配布の実バリデータ
(`src/ground_handling`)に対し `_move_item` の呼び出しをフックし、
`check_transport_path`のどちらのphaseで失敗したかを直接判定した(推測ではなく
呼び出し回数・結果からの厳密な分類)。

| シーン集合 | is_valid死亡数 | うちY区間(phase1) | うちX区間(phase2) |
|---|---|---|---|
| 26シーン | 25/26 | **25/25** | 0/25 |
| sample_config(2タスク) | 2/2 | **2/2** | 0/2 |

**27件すべてがY区間で失敗しており、X区間まで到達した死亡例は1件もなかった。**

### 3-3. 判定と修正規模の見積もり(実装はしていない)

依頼文が想定していた「X区間に集中 → plannerにX方向掃引を追加」という仮説は**成立しない**。
X方向掃引は既に実装されており、かつ実測でも一度もX区間で失敗していない。したがって:

- **plannerへのX方向掃引追加という修正は不要**(既にある上、原因でもない)。
- 原因は完全にY区間(phase1)の判定精度に集中している。25/26のsudden death全てが
  同一の失敗モードである可能性が高く、Phase54(同一の決定論的バグ:agent.pyフォールバック
  のz座標)の系譜に近い、**単一の構造的原因**である可能性が高い。次に調べるべきは
  legal1(Y掃引)側の判定式・マージン・sweep_zの計算が、実際にどの障害物との
  どんな幾何関係で本物のcheck_transport_pathと食い違っているか、であり、
  X方向とは別の切り口になる。本フェーズではここまでの切り分けに留め、実装はしない。

詳細データ: `results/phase61_transport_phase_26.json` /
`results/phase61_transport_phase_sampleconfig.json`。

---

## 変更ファイル

- `docs/official_spec.md`(新規)
- `docs/submission_policy.md`(§4追記)
- `README.md`(開発運用ルールに恒久ルール追記)
- `tools/rescore_with_cutoff.py`(新規)
- `tools/phase61_transport_phase.py`(新規)
- `tools/diagnose_stability.py`(amplitude env化、複数タスクconfigバグ修正)
- `results/phase61_*.json`(計測結果一式)
- `results/phase61_report.md`(本ファイル)
