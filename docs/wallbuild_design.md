# 第1手: Wall-Building Constructor (WBC) — 設計

**着手 2026-09-07。** Deep Research(`docs/3D_BinPacking_Accessibility_First-Class_Constraint_DeepResearch_2026-09-07.pdf`)
の第1手。planner の候補生成を **Extreme Point 貪欲 → maximal-space + 層(壁)構築** へ差し替え、
コンテナを奥→手前に層で埋めて **front-to-back をハード保証** する。
下敷き: Parreño et al. 2008(maximal-space)/ Jiao et al. 2023 CLP-RLS(robot-loader wall-building)。

**ストップロス(必須)**: 2週間で、ローカル評価スイート 26+2 のうち **悪化シーン ≤ 2 かつ net で明確にプラス**
の強い信号が出なければ撤退し、58.5 提出物(`w3_planexec_lns_la_tb135.zip` / `rest020.zip`)を防御確定する。

---

## 1. なぜ効くはず(FRONT_WALL との違い)

- `FRONT_WALL`(−11): 各荷物に「奥壁 or 直後の障害物への**密着**」を強制 → 貪欲に fill を削る。
  文献(Junqueira 2012「reachability を厳しくすると充填効率が落ちる」)通りの失敗。
- WBC: 「密着」ではなく **「まだ開いていない手前空間の温存」** を構築規則で保証する。
  - 奥の層から埋め、層を閉じてから次の手前層を開く。
  - 層 k に置いた荷物は、手前の層 k+1..front(**まだ空**)を通って搬入される。
  - L経路(前面→+Y で奥へ→X スライド)は「空の手前空間」を必ず通れる = **搬入経路衝突が構造的に起きない**。
  - fill は削らない(手前空間は "温存" するだけで、そこは後で層として埋める)。
- 目的関数 = **num_placed_items**(足切り突破。体積ではない。Look-ahead Beam が体積でランクして −32% した轍を踏まない)。

---

## 2. アルゴリズム(WBC v1、offline、決定的)

### 2.1 データ構造

- **maximal space**: 空き領域を表す極大直方体。`(x_lo, y_lo, z_lo, x_hi, y_hi, z_hi)`(いずれもコンテナ local)。
  集合 `S` は互いに重なりうる(Parreño 流)。
- コンテナ内部の初期 `S`:
  - 素の直方体内部を1個。
  - **cutcorner**: `geometry.diagonal_face_indices` で斜め面を識別し、切り欠きの矩形近似分を除いた
    2〜3個の直方体でカバー(斜め面は保守側に矩形で内接)。
  - **shelf**: 床面の空間 + 棚上面の空間(`small/big_shelf_aabb` の上)を別々の初期空間として持つ。
  - **prepacked**: 既積み荷物の AABB を初期 `S` から subtract 済みの状態で開始。
- `remaining`: 未配置荷物。**体積降順**で保持(層に対して大きい面から詰める)。同値は index 昇順(決定的)。

### 2.2 メインループ

```
conts = clone_containers(container_list)          # geo 状態(_place で沈降)
S = init_maximal_spaces(conts)                     # コンテナごと
plan = []
while remaining and budget not exhausted and wall not deadline:
    # (1) 参照空間 = 「最も奥(y_lo 最大)、次に最も低い(z_lo 最小)、次に最も左(x_lo 最小)」
    #     → これが front-to-back 保証の核。常に奥・下・左から埋める。
    ref = argmin over S of (-y_lo, z_lo, x_lo)     # 奥優先 → 低優先 → 左優先
    if ref is None: break

    # (2) ref に収まり、層 fill スコア最大の (item, orn) を選ぶ
    best = None
    for item in remaining (体積降順、上位 CAND_ITEMS 個まで):
        for orn in unique_orientations(item):
            half = half_extent(item.lwh, orn)
            for corner in {ref の (x_lo,y_lo,z_lo) を基準に 8隅のうち "奥下左" 固定}:
                pos = corner + half                # local 中心
                if not fits_in(half, pos, ref): continue
                if not check_inclusion_batch(cont, half, world(pos)): continue
                if not transport_legal_batch(cont, half, world(pos), obstacles=placed_in_this_cont):
                    continue                       # 安全網(back-to-front なら通常 True)
                sc = layer_score(item, half, pos, ref, cont)
                best = max(best, (sc, item, orn, pos))
    if best is None:
        S.remove(ref)                              # この空間には何も入らない → 層クローズの一部
        continue

    # (3) 配置(geo で沈降させる)
    _, item, orn, pos = best
    act = {'place_pos': pos, 'orientation': orn, 'container_idx': ci}
    placed = simulate._place(cont, item, act)
    cont['packed_items'].append(placed)
    plan.append({'index': item.index, 'container_idx': ci,
                 'place_pos': tuple(pos), 'orientation': orn})
    remaining.remove(item)

    # (4) maximal space 更新(Parreño): placed の world AABB と重なる各 s in S を
    #     「placed の 6方向スラブ」に分割 → S から s を除き、生成スラブのうち非退化のものを追加
    #     → S 内で他に完全包含される空間を prune。
    S = update_maximal_spaces(S, placed_aabb, ci)
return plan
```

### 2.3 `layer_score`(目的 = 層を密に埋めつつ手前を温存)

```
gap_behind = pos.y - half.y - ref.y_lo            # ref の奥面からのずれ(小さいほど良い = 奥壁積み)
depth_overrun = max(0, (pos.y + half.y) - (ref.y_lo + WALL_BAND))   # 層の厚み帯からのはみ出し
xz_footprint = (2*half.x) * (2*half.z)            # X-Z 面の占有(大きいほど良い)
support_ok = 直下に十分な支持(床/棚/既配置)。無ければ大減点。
score =  W_FOOT * xz_footprint
       - W_BEHIND * gap_behind
       - W_OVERRUN * depth_overrun
       - (0 if support_ok else BIG)
       + W_VOL * item.volume * 1e-3                # 同点は大きい荷物優先
```

- `WALL_BAND`: 層の厚み。**その層の最初に置いた荷物の depth** で決める(初回は ref の depth 全体)。
  以降その層の荷物は `depth <= WALL_BAND + tol` を強く選好(はみ出しは `W_OVERRUN` で罰)。
- 層が閉じる契機: ref(奥の空間)が「何も入らない」で S から消え、次の ref の y_lo が明確に手前 = 次の層。

### 2.4 online(policy)

**無改造。** plan-executor が WBC のプランを実機で再現、`validate_planned_xy` で弾かれたら
`planner.plan` 貪欲へ縮退(現行どおり)。第3手で「大型荷物の直前だけ選択的物理検証」を足す。

### 2.5 マルチコンテナ(2c シーン)

Parreño は単一コンテナ。2c は現行の優先コンテナ割当ロジックで荷物→コンテナを先に決め、
各コンテナで独立に WBC。割当は暫定的に「優先荷物→優先コンテナ、あふれは非優先へ」+ 体積バランス。

---

## 3. 実装

- 新規 `agents/mysolver/wallbuild.py`:
  - `MaximalSpace`(namedtuple or dataclass、frozen)/ `_split_space` / `_prune_contained` / `init_maximal_spaces`
  - `wbc_plan(container_list, items_by_index, item_list, lookahead_k, budget, wall_deadline)` → `list[plan_entry]`
  - 定数は全て `MYSOLVER_WBC_*` env(WALL_BAND 係数 / W_FOOT / W_BEHIND / W_OVERRUN / CAND_ITEMS 等)
- `ordering.build_plan`: `MYSOLVER_WBC='1'` のとき `simulate_order` の代わりに `wallbuild.wbc_plan` を呼ぶ。
  既定 `'0'` で従来経路・ビット単位不変(`LDBC`/`CDR` と同じ分岐点)。
- 再利用: `geometry.half_extent` / `check_inclusion_batch` / `local_to_world` / `static_obstacles` /
  `transport_x_bounds` / shelf AABB、`planner.transport_legal_batch`、`simulate._place` / `clone_containers`。
- 新規評価関数は作らない(Phase86 の教訓)。fill は幾何占有、支持は既存の判定を流用。

## 4. 検証計画

1. **単体テスト**(`tools/test_maximal_space.py`、host py3.12 不要な純ロジック): `_split_space` の
   6分割・`_prune_contained`・`init_maximal_spaces`(cutcorner/shelf/prepacked)を既知ケースで。
2. **決定的8シーン**: `MYSOLVER_WBC=0` で `tools/recover_check.py` が現状と不変(必須)。
3. **単シーン sanity**(A08 extreme / A01 plain / A03 shelf / P02 prepacked): 例外なく完走、
   プランが生成される、`num_placed` が baseline と比較可能。
4. **10代表シーン A/B**(`tools/local_eval.py`、budget45 / `PLAN_WALL_FACTOR=2.6`): base vs WBC。
5. 3-4 で悪化が目立たなければ **全26+2シーン**。
6. **ストップロス判定**: 悪化シーン ≤ 2 かつ net 明確プラス → 本番 zip 1枠。出なければ撤退。

## 5. リスク

- maximal-space 更新(6分割 + 包含 prune)の正しさ。単体テスト必須。
- cutcorner の矩形近似が保守すぎると deep 空間を取り逃す。斜め面 moving-EP(`geometry.cutcorner_boundary_x`)で緩められる。
- geo `_place` の沈降で層帯からずれる → `WALL_BAND` の tol とスコアの `W_OVERRUN` で吸収。実機ドリフトは第3手。
- 2c の荷物→コンテナ割当が雑だと 2c シーンで悪化。割当は現行ロジックを流用し、まず 1c で効果検証。
- コスト: 1配置あたり O(|S| * CAND_ITEMS * orn * corner)。|S| を prune で ~100 以下に保ち、
  CAND_ITEMS を 12 程度に制限。offline 予算(削り後 ~30s ユニット)で N=140 まで回るか要実測。

## 6. 進捗

- [x] 2026-09-07: 設計(本ドキュメント)。
- [x] 2026-09-07: `wallbuild.py` v1 実装(maximal-space コア + `wbc_plan`)+ `planner.plan` に
      `region=` 制約 + `build_plan` 統合(`MYSOLVER_WBC`、既定OFF)。**例外なく動く。**
- [x] 2026-09-07: 3シーン smoke(A01/A03/A08) → **v1 は大幅な回帰**:
      A01 23→13 / A03 22→17 / A08 19→11(net −23)。num_placed 37.4%→27.6%。
      実エピソードで多数の "not inside (hit boundary plane)" = WBC が提案する位置が内包判定を
      落としている。原因候補(要デバッグ、次セッション):
      - `init_maximal_spaces` の z 上限 = `height`。実際の使用可能上限が `height - thickness` 等なら
        天井側の空間が過大 → planner が region 内で天井貫通位置を拾い RETRY_GRID で妥協。
      - cutcorner の矩形近似が甘い/位置ずれ → 楔の外の deep 空間を過大評価。
      - `region` 絞りが厳しすぎて planner が region 内で合法候補を失い、最終 RETRY_GRID の
        粗いグリッドで縁ぎりぎりの位置を返している(→ region を「中心XYが入る」でなく
        「荷物AABBが概ね入る」の緩い判定に、または region に z 上限も渡す)。
      - `_pick_ref` が「最も奥」を選ぶが、奥に薄いスリット空間が残ると毎回そこを選んで
        小物を詰めようとし失敗 → S.remove するが数が減らない可能性。ref 選択に「体積下限」も。
      - 沈降後 `simulate._place` の pos ズレを maximal-space 更新に反映していない(planned pos で
        AABB を引いている)→ 実際より空きを狭く見積もる/広く見積もる。
- [ ] 上記のデバッグ(まず `init_maximal_spaces` の bounds を1シーンでダンプして目視)。
- [ ] `wbc_plan` が baseline に匹敵する配置数を出すまで v1 を調整。
- [ ] 決定的8シーン不変確認(`MYSOLVER_WBC=0`)→ 10シーン A/B → 全26+2 → ストップロス判定。

**現状: WBC のスキャフォールドは組めて end-to-end で動くが、v1 は未調整で baseline に −23。
2週間の第1手の初日としては想定内(初版の maximal-space 構築が調整済み貪欲に即勝つことは稀)。
次は "hit boundary plane" の原因特定から。**
