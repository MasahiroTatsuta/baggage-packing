# 探索の全面書き換え設計 — LDBC (Limited-Discrepancy Backtracking Construction)

**開始: 2026-09-07。** ユーザー選択 A1(数日規模・本番保証なしの全面書き換えに着手)。
本ドキュメントは複数セッションにまたがる作業の設計基盤。today = 設計 + スケルトン。

## 0. なぜ書き換えるのか(確定した事実)

- 復旧コード(plan-executor + PACK_LNS、本番 public 58.498)を全26シーンで診断
  (`results/recover_sudden_death_0907.json`): **25/26 が cause=is_valid(搬入経路衝突)、
  完走は A06 のみ、合計配置率 44.6% @budget40**。死ぬ荷物はほぼ毎回大型・
  item_idx_in_pool=0(=次に置きたい第一候補、代替なし)。
- 局所 repair を3系統実装・A/B(全て既定OFF、`exp/lns-mid-destroy`):
  `TOPO_REORDER` net −4 / `PACK_LNS_MID`(中盤ウィンドウ除去)net −3 /
  `PACK_LNS_UNBURY`(手前K個を外し大型荷物を確定配置)net +1。
  **3つとも wash。** 勝つのは extreme/tall/flat/shelfprio(幾何的に厳しい dense)、
  負けるのは plain-dense/bulky/prioheavy。統一 gate なし ＝ Phase90 の結論の再現。
- **貪欲構築 + 局所 repair の系統は打ち止め。**

## 1. Phase86 の失敗から得た制約(最重要)

Phase86 の Look-ahead Beam は **depth=1 で −23%、depth=2 で −32%** と単調悪化して撤退。
分析: 「短いグリーディロールアウトを `_objective`(risk調整済み体積)でランクすると、
直近数手で体積を稼ぐ枝を過大評価し、その選択が後続の配置可能性(搬入経路・残り空間の
形状)を悪化させる」。Phase34 の ALNS 不採用(代理gain vs 実fill ρ=−0.32)と同型。

**→ 本設計の鉄則: 枝を「体積(代理目的関数)」でランクしない。** 一次シグナルは
**「デッドロックに到達したか / 到達までに何個置けたか(num_placed)」**。体積は同点時の
深いタイブレークにのみ使う。num_placed は足切り/本番スコアを直接駆動する量であり、
代理ではない。

## 2. LDBC のアーキテクチャ

`build_order` が返す**アイテム順序はそのまま使う**(体積降順+精緻化、良い)。
置き換えるのは**位置決定と、デッドロック時の巻き戻し**。

### 2.1 状態と遷移

- 状態 S = (各コンテナの packed_items(位置確定), 残りアイテム列 R)
- 遷移: R の先頭アイテム `it` について、候補位置を `planner` の候補列挙で得てランク、
  最良を選んで `simulate._place` で確定 → S'。
- **決定点スタック**: 各要素 = {step, item, tried_positions:set, ranked_candidates, choice_idx}

### 2.2 デッドロック検出

`it` を置いた直後、**次の1〜2アイテム**が「合法な搬入位置を1つ以上持つか」を
`planner` の候補列挙 + `transport_legal_batch` で確認する
(全残りアイテムのチェックは高コストなので先頭数個に限る近似)。
- `it` 自身が合法位置ゼロ → 即デッドロック
- 次アイテムが合法位置ゼロ → デッドロック(先読み1手)

### 2.3 巻き戻し(限定discrepancy)

デッドロック時:
1. 直近の決定点のうち「まだ試していない候補が残っている」ものへ戻る
   (chronological backtracking。戻り幅の上限 D、既定 D=6 手)。
2. その決定点で **選んだ位置を forbidden に追加** し、次順位の候補を選ぶ。
3. そこから前方construction再開。
4. 総 discrepancy 数 ≤ K(既定 K=8)。K 到達で打ち切り。
5. **anytime**: 常に num_placed 最大のプランを保持し、ユニット/壁時計予算切れで返す。

### 2.4 forbidden の実装(v0 → v1)

- **v0(今セッションのスケルトン)**: 決定点で「box を除去 → その step 以降を破棄 →
  `planner.plan` を `score_noise` 引き上げ + rng seed オフセットで再実行」。同じ選択を
  避ける確率的ナッジ。実装が軽く既存 planner に無改造。
- **v1(次段)**: `planner.plan(..., forbidden=set[(item_index, xy_quantized, orn)])` を
  追加し、`_search_best` の候補列挙(`candidate_xy` を `_evaluate_candidates` に渡す前)で
  マッチ候補を除外。既定 None で完全にビット単位不変。厳密な「その手を封じて次善手」を
  実現。`_search_best` は hot かつ nested なので慎重に。

### 2.5 統合点

- `MYSOLVER_LDBC='0'` 既定。`'1'` のとき `ordering.build_plan` の
  `simulate.simulate_order(..., plan_out=plan)` 呼び出しを `search.ldbc_plan(...)` に
  差し替え(順序 `order` は従来どおり `build_order` から)。
- plan-executor(agent.policy)側は無改造 — LDBC が返す plan も同じ
  `{index, container_idx, place_pos, orientation}` 列。
- gate: 現行の `_lns_risky`(lookahead_k>1)/ shelf・prepacked は当面 LDBC も従来経路に
  委任(まず lookahead_k==1・非shelf・非prepacked で効果検証)。

## 3. 検証計画

1. 決定的8シーン: `MYSOLVER_LDBC=0` で `bp_baseline` 相当がビット単位不変(必須)。
2. スケルトンの単シーン sanity(A08 extreme — UNBURY で +5 だった。LDBC が最も効くはずの形)。
3. 10代表シーンで base vs LDBC(`tools/local_eval.py`、budget45 / PLAN_WALL_FACTOR=2.6)。
   **判定基準: 悪化シーンが少なく net 正**(Phase90 が満たせなかった水準)。
4. 通れば全26シーン → 本番1枠。

## 4. リスク

- コスト: 巻き戻しは placements の再生成。スナップショットスタックで緩和(各決定点で
  `clone_containers` を積む。メモリ ~ D 個)。
- Phase86 と同じ失敗モードの再来: num_placed を一次シグナルにすることで構造的に回避する
  設計だが、「次アイテムの合法性チェック」の近似(先頭数個)が甘いと見逃す。
- 本番の隠しテストで効く保証なし(ローカル≠本番はプロジェクト既知)。

## 5. 進捗

- [x] 2026-09-07: 設計(本ドキュメント)+ `agents/mysolver/search.py` スケルトン
      (LDBC 制御フロー、既定OFF、compile OK)。
- [ ] v0 forbidden(確率的ナッジ)で 10シーン A/B。
- [ ] 効果があれば v1 forbidden(planner に厳密除外)。
- [ ] 全26シーン → 本番。
