# Phase84 報告: online policy() の候補生成調査 → 既に共通関数経由でcutcorner適用済みと判明、実装なしで終了

## 結論(先出し)

**ステップ0で(0-2)の終了条件に該当した。** `agent.py::policy()` は offline側と
**完全に同一の`planner.plan()`(→`_search_best`→`_candidate_xy`)を呼んでおり**、
`_candidate_xy`はリポジトリ全体で**1箇所しか定義されていない共通関数**である。
Phase81で実装したcutcornerの斜面下候補点は、`MYSOLVER_CUTCORNER_CANDIDATES`が
有効な限り**online/offlineを問わず自動的に適用される**。現在の主枠
(`mysolver_submit_cc_rcl.zip`)は`CUTCORNER_CANDIDATES`の既定を`'1'`に固定して
いるため、**online側でも既にcutcornerは効いている**。指示書が想定していた
「online側だけ古いロジックのまま」というシナリオはコード構造上存在しなかった
(Phase61のX方向掃引・Phase78のhint_soft/resolved_priorityと同型の的中パターン
——今回は逆に「実装済みと確認できたので何もしない」が正しい結論)。

ステップ1(3倍見積もり)・ステップ2(実装)・ステップ3(zip化)は(0-2)の指示どおり
実施していない。ステップ4(総括)を報告する。

---

## ステップ0: 既存実装の確認

### (0-1) 精読結果

`agents/mysolver/agent.py`:

```python
def policy(self, observation: dict) -> dict:
    ...
    action = planner.plan(container_list, pool_list, time_budget=POLICY_TIME_BUDGET,
                           hard_deadline=time.perf_counter() + POLICY_HARD_WALL,
                           strict_support=not self._optimize,
                           prepacked_ids=self._prepacked_ids)
```

`POLICY_TIME_BUDGET = 5.5`(秒相当のユニット予算、決定的)、
`POLICY_HARD_WALL = float(os.environ.get('MYSOLVER_POLICY_HARD_WALL', '6.0'))`
(壁時計の非常用安全弁)。

呼び出し先`planner.plan()`は`_search_best()`を呼び、`_search_best()`は
`_candidate_xy(container, half, obstacles, grid_density=grid_density)`を呼ぶ
(`planner.py:1324`、1箇所のみ)。`_candidate_xy`自体:

```python
def _candidate_xy(container, half, obstacles, grid_density: int = 1):
    grid_pts = _grid_point_frozenset(container['length'], container['width'], grid_density)
    pts = grid_pts | _extreme_points(container, half, obstacles)
    if CUTCORNER_CANDIDATES:
        pts = pts | _cutcorner_candidates(container, half)
    ...
```

**質問への回答:**

- **候補点はofflineと同じものか、別ロジックか** → **完全に同一。**
  `_candidate_xy`はリポジトリ全体で定義が1箇所しかなく(`planner.py:626`)、
  呼び出し箇所も`_search_best`内の1箇所のみ(`planner.py:1324`)。online専用の
  簡略版候補生成ロジックは存在しない。`agent.py::policy()`→`planner.plan()`
  →`_search_best()`→`_candidate_xy()`という、offlineの`beam_construct_order`
  →`planner.plan_topk()`→`_search_best()`→`_candidate_xy()`と**全く同じ関数を
  末端まで共有している**(`_search_best`自体も1つしか定義がない)。
- **cutcornerの斜面下候補点はonline側にも既に反映されているか** → **反映済み。**
  `CUTCORNER_CANDIDATES`はモジュールレベルの単一フラグであり、`_candidate_xy`が
  呼ばれる文脈(online/offline)を区別しない。現在の主枠zip
  (`mysolver_submit_cc_rcl.zip`)は`MYSOLVER_CUTCORNER_CANDIDATES`の既定を`'1'`に
  固定しているため、**online policy()は既に斜面下候補点込みで動いている。**
- **policy呼び出し1回あたりの候補評価数** → プールは`MAX_POOL_ITEMS=20`
  (`planner.py:29`、online固定)に切り詰められる。候補xy数はコンテナ寸法・
  grid_density(既定`BASE_GRID_DENSITY`)・extreme point数・(cutcorner有効なら)
  斜面候補5点(`CUTCORNER_N_Y_SAMPLES`既定5)で決まり、(item×orientation×container)
  の組合せごとにこの候補集合を評価する。offlineの`beam_construct_order`との違いは
  プールサイズの上限(online=20、offlineは`max_pool_items=None`で全件)だけであり、
  **候補生成ロジック自体に違いは無い**。
- **現在のpolicy実行時間に対し、候補を増やす理論上の余地** → 後述(ステップ1相当の
  分析)。

### (0-2) 終了条件に該当

**共通関数経由でcutcornerは既にonlineに効いている。** 指示どおり、このフェーズは
ここで実装なしで終了する。

---

## 参考: なぜ本番policy時間が6.02〜6.07sに集中するのか(副産物)

`POLICY_HARD_WALL`(既定6.0s)は`planner.plan()`に`hard_deadline`として渡され、
`SearchBudget.exhausted()`が`HARD_CHECK_INTERVAL`回に1回、壁時計を実際にチェックする
安全弁として働く(`planner.py:409-415`)。主予算は`POLICY_TIME_BUDGET=5.5`
(決定的ユニット換算)だが、**本番の実マシン速度がユニット較正(`UNITS_PER_SEC`)より
遅い場合、決定的予算が壁時計5.5s相当を使い切る前に実時間としては6.0sの安全弁へ先に
到達しうる。** 観測値(6.02〜6.07s)が一貫して6.0sのわずか上に集中しているのは、
**探索が自然完了しているのではなく、この安全弁に毎回引っかかって打ち切られている**
ことを強く示唆する(チェック間隔`HARD_CHECK_INTERVAL`ぶんのオーバーシュートが
0.02〜0.07s)。これはcutcorner候補追加が本番で効いているかどうかとは独立の一般的な
観測事実で、8秒制限までの実質的な余裕(8.0−6.07≈1.93s)がなぜこの値になっているかの
説明になる。

**この安全弁は`SearchBudget`が候補配列のベクトル化評価1回を「不可分な単位」として
扱う設計(Phase17)のため、評価の途中では発火しない。** 1回の`_evaluate_candidates`
呼び出し中に候補数が増えても、そのバッチが完了するまで安全弁はチェックされない。
cutcornerが追加する候補はこの意味で「既存バッチへの数点の追加」であり、単独の
新しい不可分単位を作るわけではないため、追加コストは相対的に小さく吸収されている
と考えられる(これも既にonlineへ反映済みの効果として本番実測値に織り込み済み)。

---

## ステップ4: 総括

### (4-1) Phase73〜84で試した全軸の一覧

| Phase | 軸 | 結論 |
|---|---|---|
| 73〜75 | 支持閾値3定数(union/span/centroid) | **centroidのみ有効(+3.54)**、頂上確定(0.25)、union/spanはほぼ無寄与 |
| 75〜77 | 幾何定数3つ(INCLUSION_MARGIN/SAFETY_MARGIN_XY/REST_CLEARANCE) | 現行値が最適点(両側実測)。REST_CLEARANCEは40倍非対称の崖のすぐ上 |
| 78/80 | ビームwindowのis_soft順序制御 | 本番で逆効果(−1.51)、**不採用** |
| 79 | 死亡局面345件の再解析(centroidの機序特定) | centroidは「最後の1個の救済」ではなく「軌道変化」と判明。過去の不採用軸(ALNS/REPAIR旧版/ρ-test)を再確認、いずれもcutoff盲目性とは独立の理由で正当と確認 |
| 81 | DFTRC(配置スコアへのタイブレーク項) | 既存`back_term`と競合、本番−2.618で**不採用** |
| 81 | moving extreme points(cutcorner) | **本番+0.209で採用**。online/offline両方に自動反映(本フェーズで確認) |
| 81 | RCL(restricted candidate list) | 本番+0.149で暫定採用。ローカルでのphase2勝率は0/26のまま(機序は未解明) |
| 82 | cc_rcl(cutcorner+rcl併用) | **加法性が実測で成立(+0.360≈+0.209+0.149)。現主枠(57.545)** |
| 82 | cc_strong(cutcorner候補点2倍) | +0.013で誤差範囲、**飽和**。N_Yは既定のまま |
| 82/83 | rcl_k15/k50(RCLのk水準) | 緩2と13桁完全一致という不具合。**原因を特定できず、探索打ち切り**(未検証の仮説: phase2勝利という低確率事象の乱数依存) |
| 83 | REPAIR(Phase29のブロッカー駆動リスタート再挑戦) | +0.003で誤差範囲、**不採用**。cutcorner/rclと機序が近すぎた(いずれも順序側の生成・後処理ロジック) |
| 84 | online policy()側の候補生成 | **独立軸として存在しなかった**——online/offlineは同一関数を共有しており、cutcornerは既に両方に効いている。追加実装の余地なし |
| (未実施) | ALNS(Phase34)の再挑戦 | Phase82/83で再検討し**見送り**(代理関数=risk_adjusted_volumeの精度問題、ρ=−0.321は本物の評価器で測定済みで、cutcorner/rcl/REPAIRのいずれとも無関係な受理判定側の欠陥) |
| (未実施) | WALL_MODE(Y_SLICE構造)の再挑戦 | Phase82/83で再検討し**見送り**(Phase9/13/14/26で4連敗、Phase26は統計的に有意な悪化で機序も特定済み、新しい土台と無関係) |
| (未実施) | precedence/blocking graph(Deep Research第2優先) | 未実装。REPAIR(この系統の軽量版)がPhase29・Phase83の2回とも弱い結果に終わったこと、原論文自身が「online適用は原理的に近似」「オフライン3分側で本領を発揮する」としていることから、期待値は低いと評価(詳細は(4-2)参照) |

### (4-2) 「もう試すべき独立軸が見当たらない」状態かどうかの判定

**見当たらない、と判定する。** 根拠:

1. **既知の全レバー(閾値・幾何定数・順序制御・候補生成・リスタート方式・
   後処理・online/offline分離)を実際に触った。** 支持閾値(centroid)・候補生成
   (cutcorner)・リスタート(RCL)の3系統だけが実際に前進し、残りは全て
   統計的に有意な悪化・誤差範囲・実装対象なしのいずれかで終わった。
2. **Deep Research(外部の学術文献)という新しい情報源も使い切った。** 論文が
   優先度付きで挙げた3項目(DFTRC・moving extreme points・RCL)のうち2つを採用、
   1つを正しい理由で不採用にした。残る「precedence/blocking graph」は論文自身が
   online適用の限界を明言しており、その軽量版(REPAIR)を2回(Phase29・Phase83)
   試して2回とも弱い結果だったため、フルスケール実装への投資は正当化しにくい
   (実装コストは本フェーズまでの各軸より明らかに大きく、期待値はREPAIRの実測
   +0.003が最も近い代理指標になる)。
3. **online側は今回の調査で「offlineと不可分」と判明した。** online/offlineが
   別々の独立した攻め筋だという前提自体が誤りだった——online専用の改善余地は
   構造的に存在しない(候補生成を触るとoffline側にも同時に効き、逆も同じ)。
4. **加法性の上限もほぼ見えている。** cc_rcl(+0.360)にREPAIRを重ねても+0.003
   だったことから、「独立な機序を探して重ねれば足し算になる」というPhase82の
   方針そのものが、少なくとも今すぐ使える形の追加候補を使い果たしたことを示す。

**したがって、攻めを終了し防御フェーズへの移行を推奨する。**

### 防御フェーズの内容案

- **主枠(cc_rcl)・2枠目(rest020)が極端なシーン構成でも壊れないかの確認**:
  Phase46で行った構成比stress検証(ソフト比率80%等)の枠組みを、緩2ではなく
  cc_rcl構成で再実行し、違反0件が維持されるかを確認する。
- **policy/optimizeのタイムアウト安全マージンの最終確認**: 本フェーズで判明した
  「本番policy時間は`POLICY_HARD_WALL`(6.0s)の安全弁で頭打ちになっている」
  という構造を踏まえ、この安全弁自体が正しく機能し続けているか(タイムアウトに
  よる強制ランダム行動が発生していないか、statusの推移から間接的に確認できる
  範囲で)を最終チェックする。
- **提出物の最終選択期限(2026-10-12)に向けたチェックリスト作成**: 主枠・2枠目
  それぞれのzipのSHA256・全定数grep結果を1つの表にまとめ、期限までに再確認すべき
  項目(SIGNATE上の選択操作、提出物の取り違え防止)を明文化する。

---

## やっていないこと

- ステップ1(3倍見積もり)・ステップ2(実装)・ステップ3(zip化)は(0-2)の
  終了条件により実施していない。
- 支持閾値・幾何定数・cutcornerのN_Y・rclのkは動かしていない。
- 本番の集計スコアから足切り閾値やシーン数を逆算していない。
- `.gitignore`の書き換え・force pushは行っていない。

## 生成物一覧

- `results/phase84_report.md`(本ファイル)

コードファイルの変更は無い(調査のみ、既存コードの読解と1箇所の確認で
(0-2)の終了条件に該当したため)。
