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

- [x] 2026-09-07 夜: **v2 デバッグ反復**(A01 のダンプで3バグ特定・修正):
  - z 上限の座標系ミス(`height` → `center[2]+height/2`)、x/y も壁厚ぶん内側に。
  - **壁帯が即崩壊**(「最も奥の空間」だけだと大型を数個置いた後に前面へ飛ぶ)→ 壁カーソル
    `wall_back` を導入。現在の壁帯 `[wall_back - band, wall_back]` に届く空間だけ ref 候補、
    帯が埋まって初めて前進。`_layer_score` を背面(y_hi)密着・手前非張り出しに修正。
  - `region` を渡す際 y を壁帯にクリップ(前面まで伸びる ref 空間でも荷物中心を壁帯内に限定)。
    z 帯逸脱・実内包マージン(-0.005)超えの候補も事前除外。
  - **結果(A01/A08/B01)**: A01 13→**18**(baseline 23)、A08 **11**(baseline 19、変化なし)、
    B01 **24**(baseline 24、WBC が有効に効かず貪欲へ縮退)。boundary エラーは A01 序盤で
    解消したが全体ではまだ 21 件(B01 7 / A01 8 / A08 6、episode 中)。
  - 残る課題: **各壁が 2-3 個で次へ進み X-Z 面が密に埋まらない**(配置済み荷物周りの
    スリット空間が `min_item_dim` 未満で捨てられている疑い)。A08(140 個の小物)は maximal-space
    のスリット処理が特に弱い。

**現状: WBC は end-to-end で動き、v1(−23)→ v2(A01 −5 / A08 −8 / B01 ±0)まで回復。
まだ baseline 未満。密な壁充填ができていないのが最大の残課題。2週間の第1手として順調に反復中。**

### v3〜v5 の反復(2026-09-07 夜、続き)
- v3: 適応 `wall_depth`(最初の荷物が壁厚を決める)。→ 初期 band が `_max_dim`(0.89)で
  最初の荷物が深く貫通、壁1が 0.66m(半分)に。wall_back も飛びすぎ。
- v4: 初期 band = `WBC_INIT_BAND_MULT(1.8) × 最小荷物辺`(薄く)+ wall_back を「壁1枚ぶんずつ」
  前進(contiguous)。→ 壁1が wd=0.35 と薄くなり背面密着で複数入るように。ただし usable を
  top-K 大型だけで判定 → 小物スリットを捨て A01 が 4 手で停止。
- v5: `usable` = 残り荷物の**どれか**が入る空間、per-ref 候補も「その ref に3辺で収まる荷物」に。
  → **WBC が正しい wall-building 挙動に到達**(ダンプ確認: wd≈0.25 の薄い壁を背面密着で
  #0-#3、次の壁 #4-#5、その次 #6-#9 と contiguous に前進、offline プラン 14+ 手)。
  **しかし episode 結果はまだ baseline 以下**: A01 21(base 23)/ B01 24(=)/ A04 27(=)/
  D04 **18**(base 27、flat 系で回帰)。num_placed 47.8%。boundary errs 27(episode 中)。

**現状: WBC は構造的には正しく動く(薄い壁を奥から密に積む)ことをダンプで確認。
だが plan-executor の replay が実機ドリフトで崩れ、episode 配置数が baseline を超えない。
D04(flat)は WBC と相性が悪く回帰。**

### 次セッションの WBC 作業(優先順)
1. **plan-executor replay のドリフト耐性**(第3手を前倒し): WBC の計画位置が実機の沈降後
   状態でも合法であるよう、計画時に背面/床から数 mm の余裕を確保(`REST_CLEARANCE` 相当)、
   または `validate_planned_xy` で弾かれた時に「同じ壁帯内で貪欲」に縮退(現状は全域貪欲へ縮退
   → 壁が壊れる)。
2. **壁内 X-Z の密度**: `_layer_score` に「X-Z 占有(footprint)」の重みを上げ、
   スリット併合(隣接する薄い空間の merge)。
3. **D04(flat 系)の回帰**を1シーンダンプで特定。flat 荷物は薄い壁に不利かもしれない
   → band 下限を荷物の中央値辺で。
4. baseline に匹敵したら決定的8シーン不変確認 → 10シーン A/B → ストップロス判定。

### v6(2026-09-07、続き): plan-executor 縮退の region 制約 → 効果薄
`agent.policy` で validate_planned_xy が弾いた荷物を、全域貪欲でなく計画位置 ±0.20 の
region 貪欲へ先に縮退(`MYSOLVER_WBC_REPLAY_R`、`MYSOLVER_WBC=1` のときだけ)。
6シーン A/B: A01 21 / B01 24 / A04 27 / D04 17 / A05 20 / D05 18(baseline 23/24/27/27/24/14、
**net −12**)。D05(tall)+4 のみ好転、D04(flat)−10・A05(prio)−4。boundary errs 41(v5 は 27)。
→ **領域制約の縮退では不十分。根本は WBC の計画位置自体の内包スラックが薄い**
(ダンプの slk −0.013〜−0.024、実閾値 −0.005 に近い)ため実機ドリフトで落ちる。

### v7/v8(2026-09-07 深夜): スラック余裕を入れたら constructor が崩壊
`WBC_SLACK_MARGIN`(-0.03)+ `WBC_AABB_INFLATE`(0.008)を追加 + `WBC_1C_ONLY`。
→ **offline plan が A01=4 / A02=0 / A07=6 で崩壊、`|S|`(maximal space 数)が数手で 0 に。**
= replay ではなく **maximal-space コア(`_split_space`/`_prune`/`init_maximal_spaces`)自体の
バグ**。大きな前面空間を失っている。単体テストを書かずに速く組んだツケ。
v5(14手プラン)の方が良かった。knobs は -0.008 / 0.0 に戻した。

### v9/v10(2026-09-07 深夜、続き): `_overlap` の index バグを発見・修正 → まだ密に詰まらない
- **★ `_overlap(s, box)` がタプル `(ci, x0..z1)` の ci オフセットを無視して `s[1] < box[3]`
  (x0 と z0 を比較)していた。正: `s[1] < box[4]`。** → `_split_space` がほぼ発火せず
  maximal space が更新されず、数手で `|S|→0`、plan 崩壊。**v1〜v8 の全 WBC 不振の主因。**
  `tools/test_maximal_space.py`(コンテナ内で実行)で特定・回帰防止。
- v9(`_overlap` 修正): `|S|` 崩壊は止まった(17〜240)。だが offline plan はまだ 5〜17 手。
- v10(region の y-下限を大きく緩める): plan が 13〜16 手に。だが **episode はまだ baseline 以下**:
  A01 15(23)/ A02 16(24)/ D01 14(23)/ D04 19(27)/ B01 24(=)/ **D05 20(14、+6)**/ A07 9(=)。
  **D05(tall)だけ一貫して baseline 超え。**

### 現状の総括(v1→v10、10ラウンド)
`_overlap` バグ修正で maximal-space は正しく動くようになったが、**WBC の構築が 40個中
~14-16個で止まり、それ以上密に詰められない**。1.5 壁ぶんくらい埋めて stall する。
`planner.plan` + region で1荷物ずつ充填する方式が、壁内 X-Z を密に埋めきれていない。
D05(tall、縦壁が有利)以外は baseline に勝てない。

### v11(2026-09-07 深夜、続き): stall 原因を全手トレースで特定
- A01 全手ダンプ + stall カウンタ: **stall=13 全て `no_best_z`**(planner が返した候補を
  z-band チェックで全 reject)。`_pick_ref` が `(-y_hi, z0, x0)` = 「最も奥」優先で、数手後に
  **高 z のスリット slab(z[1.38,1.61] 等)を ref に選び**、planner が床に着地する候補を
  全て弾いていた。`|S|` は 154 あるのに使えず。
- v11: `_pick_ref` を `(z0, -y_hi, x0)` = **床優先**に。→ z-reject は減った(13→1〜9)が、
  **今度は `no_best_planNone` が支配的**(A01: 21、D04: 31)。planner.plan が region 内で
  合法候補ゼロを返す。プランサイズは 9〜16 で不変、episode も baseline 以下
  (A01 15/D01 13/D04 18/D05 12)。

### ★ 現状の構造的結論(v1→v11、11ラウンド)
`_overlap` 修正で maximal-space は正常化。`_pick_ref` も床優先に。だが **WBC が ~13-16/40 で
止まるのは、maximal-space(幾何・planned pos)が「空き」と言う ref に対し、
`planner.plan`(実 packed_items の障害物 + 支持 + 搬入経路 + 内包)が「置けない」を返す
divergence**。特に配置済み荷物の間の細いギャップ空間を `_space_fits_any`(3辺比較のみ)が
「入る」と誤判定し、`_pick_ref` がそこを ref に選び続ける。
**= `planner.plan` を1荷物ずつの充填器に使う方式では密に詰まらない。**

### 次の作業(優先順、更新)
1. **壁内充填を `planner.plan` 依存から外す**(設計 §「第4手」の前倒し・本命):
   確定した1つの壁帯(y 固定)の中で、その帯に入る荷物集合を **2D(X-Z)shelf/skyline packing**
   で自前に詰める(Parreño の層内アルゴリズム、または CP-SAT `AddNoOverlap2D` を層あたり
   180s のごく一部で)。着地 z は skyline、支持は「下の shelf に載る」で保証、搬入は
   帯より手前が空なので自動。planner.plan は「最終検証」だけに使う。
2. これで密度が出て baseline を超えたら決定的8シーン不変 → 10シーン A/B → ストップロス判定。
3. 出なければ WBC(この定式化)も打ち止め → A3(58.5 凍結・防御)。

### WBC 反復まとめ(v1→v11)
バグ修正: 座標系(v2)/ 薄い壁・contiguous(v4)/ usable 判定(v5)/ **`_overlap` index(v9、最重要)**/
region y 緩和(v10)/ `_pick_ref` 床優先(v11)。
**構造(back-to-front 薄壁)は正しくなったが、`planner.plan` 充填の密度不足が最後の壁。
壁内 2D packing を自前で持つのが次の本命。**
2. **シーン別**: D04(flat)/A05(prio)で回帰。flat は薄い壁に不利 → band 下限を上げる。
   prio は優先コンテナ割当(2c)の WBC 対応が未実装。まず 1c・非優先で baseline 超えを狙う。
3. 壁内 X-Z 密度(`_layer_score` の footprint 重み↑、スリット併合)。
4. baseline 匹敵 → 決定的8シーン不変(`MYSOLVER_WBC=0`)→ 10シーン A/B → 2週間ストップロス判定。

### WBC 反復のまとめ(v1→v6、2026-09-07 全体)
v1(未調整、−23)→ 座標系/壁カーソル/region クリップ(v2)→ 薄い壁/contiguous 前進(v4)
→ usable 判定(v5)→ replay region 縮退(v6、効果薄)。
**6ラウンドで「構造的に正しい wall-building」に到達(ダンプ確認)。だが 6シーン net −12。
baseline 超えには計画スラック余裕 + シーン別調整 + 壁内密度。2週間の第1手、week1 相当で
「動く・正しい」まで。week2 で「baseline 超え」を狙う段階。ストップロス(悪化≤2 かつ
net明確+)は維持。**
