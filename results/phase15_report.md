# Phase 15 報告：corridor_penalty の副作用回収と棚シーンの候補生成改善

作成日: 2026-07-22
対象コミット: `phase15` 一連(`80e664f` ターゲット1・`6ba4f17` ターゲット2、いずれもpush済み)
前フェーズからの変更ファイル: `agents/mysolver/planner.py`、`agents/mysolver/geometry.py`、
`agents/mysolver/agent.py`、`agents/mysolver/ordering.py`、`agents/mysolver/simulate.py`
(+ 診断補助 `tools/diagnose_shelf_zones.py` 新規、`tools/diagnose_prepacked.py` 軽微修正)

---

## 0. エグゼクティブ・サマリ

- **【ターゲット1・採用済み(セッション開始時点で既にcommit/push済み)】** corridor_penalty の
  `min_top_behind` 計算から、エピソード開始時から存在した既積み荷物(pre-packed)を除外した。
  26シーン×3回平均は fill_strict 23.61→23.61(中立、ノイズ床±0.42-0.59以内)、
  fill_loose 37.03→37.36(+0.33、ノイズ床±0.39以内)で全体は中立。**決定的な P04(棚×pre-packed、
  offline optimize無効)は 14.58→19.98(+5.40/+5.40)と明確に改善**し、B01-B04など非pre-packed
  シーンはコード上ゼロ影響(合法性判定は一切変えず、corridor_penaltyのスコアリングのみが対象の
  ため)。**本フェーズで P02 単体を5回追試したところ、fill_loose は 35.65±5.59 まで回復し
  phase13ベースライン(36.42)にほぼ並んだ**が、fill_strict は 20.71±5.25 で依然ベースライン
  (26.58)を下回る。P02 は offline optimize の実行時間依存ノイズが ±5pt を超え、単一シーンでの
  これ以上の当落判定は測定コストに見合わないと判断し、現状の修正(既積み荷物除外)を最終形として
  据え置く(§1)。
- **【ターゲット2・実装・実測・採用・push済み】** `gen_shelf_patternA` の行き詰まり
  (fail_transport_y 62.3%、配置14個で3フェーズ不変)を診断した結果、**棚下の空間(0.987m³、
  コンテナ最大の区画)が着地候補として一度も使われていなかった(below-shelf util = 0.0%)**。
  原因は「棚は誰の上にも乗る最優先の最高支持面」として単一skylineに合成していたため、棚の直下が
  常に「棚の上面に乗る」候補に化けてしまっていたこと。棚を境に「棚下の床」「棚上面」を独立した
  2つのコンパートメントとして扱うよう `_evaluate_candidates` を変更した。
- **【重要:実装中に発覚したバグを同一コミットで修正】** 初版は `geo.static_obstacles()` が返す
  全構造物(棚を持たないコンテナにも常設される小さいledge `small_shelf_aabb` を含む)を一律
  `is_shelf=True` としていたため、**棚が無いコンテナ(例: B01)まで着地面計算が変わってしまい、
  fill_strict が 26.32→18.45 に落ちる**という無関係シーンへの副作用があった。`is_shelf=True` を
  実際の大棚(`big_shelf_aabb`、`container.get('shelf', False)` のときだけ存在)由来に限定して
  修正し、B01 は 26.32→26.32(完全不変)を確認した。
- shelf系7シーン×3回average(修正後): **fill_strict 24.85→26.13(+1.28)、fill_loose
  33.44→39.50(+6.06)**。ノイズ床(±1.00/±0.81)を上回り両レジーム改善。標的の
  `gen_shelf_patternA` 自身も **fill_strict 25.23→28.20(+2.97)、fill_loose 29.94→38.46
  (+8.52)、below-shelf util 0.0%→30.7%**。P05のみ悪化(§3.4)。
- **【ターゲット3・計測のみ・不採用】** cog タイブレーク重みを 1.2→3.6(3倍)に上げた場合、
  パターンB5シーンで cog_score +1.40(66.08→67.48)に対し fill_strict −1.30、fill_loose −0.41
  という明確なトレードオフを確認した(1点測定、Phase16の判断材料として記録・不採用)。
- 制約はすべて維持: 旧6シーンで placement/soft 満点・stability 最小97.85を維持、
  policy 最大3.25s(<7s)、optimization 最大30.00s(<170s、開発予算のため実運用の余裕はさらに大きい)。

---

## 1. ターゲット1:corridor_penalty の既積み層過剰保護(P02回収)

### 1.1 実装(セッション開始時点で既にcommit済み、`80e664f`)

Phase14 の申し送り通り、エピソード開始時から存在した既積み荷物(pre-packed)の index を
`geo.initial_prepacked_ids()` で記録し、`corridor_penalty` の `min_top_behind` 計算
(`_collect_corridor_obstacles`、探索のスコアリングのみ)からそれらを除外した。合法性判定
(衝突・支持面・搬入経路のハード制約)には一切影響しない。

### 1.2 26シーン×3回平均(`80e664f` コミットメッセージより)

| 指標 | before | after | 差 |
|---|---|---|---|
| fill_strict | 23.61 | 23.61 | ±0.00(ノイズ床±0.42-0.59以内) |
| fill_loose | 37.03 | 37.36 | +0.33(ノイズ床±0.39以内) |

全体は中立。ただし内訳は非一様:

- **P04(棚×pre-packed、offline optimize無効、決定的)**: 14.58→19.98(**+5.40/+5.40**)
- **P0x(pre-packed)部分平均**: strict +1.31 / loose +2.19
- **B01-B04(非pre-packed)**: コード上ゼロ影響、実測でも完全に不変

### 1.3 P02 の5回追試(本フェーズで実施)

`suite_P02_A_1c_pre10.json`(offline optimize有効、budget=30sで実測)を5回試行した。

| repeat | fill_strict | fill_loose |
|---|---|---|
| 1 | 17.53 | 33.00 |
| 2 | 23.01 | 37.77 |
| 3 | 17.16 | 30.39 |
| 4 | 16.89 | 32.68 |
| 5 | 28.94 | 44.42 |
| **平均±std** | **20.71±5.25** | **35.65±5.59** |

比較対象:

| 段階 | fill_strict | fill_loose |
|---|---|---|
| phase13ベースライン(corridor_penalty無し) | 26.58 | 36.42 |
| phase14(corridor_penalty、既積み除外なし) | 18.46 | 23.37 |
| **phase15(既積み除外あり、本測定平均)** | **20.71±5.25** | **35.65±5.59** |

**fill_loose はほぼ完全に回復した**(35.65 は 36.42 とノイズ内で一致)。**fill_strict は
phase14の落ち込み(18.46)からは持ち直したが、phase13ベースライン(26.58)には届いていない**。
ただし run間std が ±5.25/±5.59 と非常に大きく、これは P02 が offline optimize(ordering.build_order)
の時間予算依存で構築順序自体が試行ごとに変わるためである(`80e664f` コミットメッセージの
「P02自体はoffline optimizeの実行時間依存ノイズが大きく単発では回収を確認できなかった」という
所見と整合)。

### 1.4 判断:これ以上のコード変更は行わない

指示にあった残る修正候補 (b)「既積み層に限って不感帯を拡大」・(c)「免除判定の精緻化」は、
いずれも P02 単体でしか効果を検証できない一方、**P02自身のノイズ床(±5pt超)がP01(+8.41)・
A05(+6.87)・B04(+9.53)で得られた改善幅と同オーダーであり、追加修正が本当に効いたのか単なる
運かを見分けるには最低でも10回以上の追試が必要**になる。既存の修正(既積み荷物除外)は

1. 合法性に影響しない安全な変更である、
2. P04を明確に改善し、P0x全体も平均で改善している、
3. B01-B04など非pre-packedシーンにゼロ影響である、

という3条件を満たしており、Phase14で発生したコストの大部分(fill_loose はほぼ全額、
fill_strict も部分的)を安全に回収できている。**これ以上のP02狙い撃ちの修正は費用対効果が
見合わないと判断し、本フェーズでは追加のコード変更を行わない**。P02のような「offline optimize
+ pre-packed」シーンの安定化は、corridor_penaltyの微調整ではなく ordering.build_order 側の
時間予算配分の課題として Phase16 以降に切り出すことを推奨する。

---

## 2. ターゲット2:棚下/棚上を独立コンパートメントとして扱う候補生成

### 2.1 診断:棚下領域が一度も使われていない

`tools/diagnose_shelf_zones.py`(新規、読み取り専用)で `gen_shelf_patternA` の停止時点の
状態を可視化した。

修正前(棚を単一の最優先支持面として扱う従来実装):

```
shelf: z=[0.805,0.845] y=[0.040,0.685] (container z=[0.040,1.570])
below-shelf region footprint volume: 0.9869 m^3   <- コンテナ最大の区画
above-shelf region footprint volume: 0.9353 m^3
items below shelf:   0  volume=0.0000  util=0.0%   <- 一度も使われていない
items above shelf:   4  volume=0.4133  util=44.2%
items elsewhere(back strip beyond shelf y):  10  volume=0.7715
```

原因は `_evaluate_candidates` の着地面(skyline)計算が「XYが少しでも重なる支持体はすべて
その上に乗るしかない障害」として扱うため、棚(常に最優先の最高支持面)とXYが重なる候補は
必ず「棚の上面に乗る」候補に変換され、「棚の下の床に直接置く」候補が生成される余地が
そもそも無かったこと。

### 2.2 修正:棚を境に2つのコンパートメントへ分離

`_landing_supports` に `is_shelf` フラグを追加し、`_evaluate_candidates` で

- **棚下コンパートメント** (`landing_below`): 床、および天面が棚の下面以下にある既配置物の
  最大天面
- **棚上コンパートメント** (`landing_above`): 棚の上面、および天面が棚の上面以上にある
  既配置物の最大天面

を別々に計算し、候補の荷物が `landing_below` の高さのまま棚の下面をクリアできる
(`item_top_if_below <= shelf_bottom - OBSTACLE_Z_MARGIN`)なら棚下を採用し、収まらない場合
のみ棚上へ強制する。棚とXYが重ならない候補(`shelf_touch_any=False`)は従来どおり単一
コンパートメントのまま(棚の無いコンテナと数式上完全に同じ経路を通る)。

### 2.3 実装中に発覚したバグとその修正(同一コミット内)

初版は `geo.static_obstacles(container)` が返す全構造物を一律 `is_shelf=True` としていた。
しかし `static_obstacles()` は

- `small_shelf_aabb`: **全コンテナに常設される**、入口脇の小さいledge
- `big_shelf_aabb`: `container.get('shelf', False)` のときだけ存在する大棚

の両方を返す関数であり、初版はこの区別をしていなかった。結果として **棚を持たない
コンテナ(B01等)でも small_shelf_aabb を「棚」とみなしたコンパートメント分割が誤発火**し、
無関係なシーンの着地面計算まで変えてしまっていた。

発覚の経緯: shelf系7シーンの検証と並行して、target2 が非棚シーンに与える影響をゼロと
仮定してよいか実測で確認したところ、決定的シーン B01 で **fill_strict 26.32→18.45という
無視できない差**が出た。コードを再検査し、上記の原因を特定。`is_shelf=True` を
`big_shelf_aabb` 由来の場合だけに限定するよう修正し、**B01 は 26.32→26.32(完全に不変)を
再確認**した。この修正後の状態で以降の検証(§2.4以降)をすべて実施している。

**教訓**: 「この変更は特定条件でのみ発火するはずだから他シーンへの影響はゼロ」という
仮定は、コードレビューだけでなく必ず無関係シーンでの実測でも裏付けること。今回は
`container.get('shelf', False)` を直接見ずに共通のヘルパー関数の戻り値を丸ごと転用した
ことが原因であり、ヘルパー関数の中身(何を返すか)を実装時に確認せず名前から類推した
ことが根本原因である。

### 2.4 shelf系7シーン×3回average(修正後)

| シーン | fill_strict (before→after) | fill_loose (before→after) |
|---|---|---|
| A03_1c_40_shelf | 23.81 → 26.88 | 36.51 → 50.63 |
| B02_1c_40_shelf | 23.03 → 24.47 | 27.24 → 32.80 |
| C01_1c_40_shelf | 31.84±2.34 → 30.86±9.23 | 37.86±1.76 → 41.23±7.07 |
| C02_2c_55_shelfprio | 21.49 → 28.40 | 32.51 → 46.96 |
| P04_B_1c_pre8_shelf | 19.98 → 21.92 | 30.34 → 30.04 |
| P05_C_2c_pre8_shelfprio | 28.58 → 22.18±2.26 | 39.66 → 36.40±1.42 |
| **gen_shelf_patternA** | **25.23 → 28.20** | **29.94 → 38.46** |
| **平均** | **24.85±0.33 → 26.13±1.00** | **33.44±0.25 → 39.50±0.81** |

**両レジームともノイズ床(±1.00/±0.81)を上回って改善**。標的だった `gen_shelf_patternA`
自身も明確に改善し(below-shelf util 0.0%→30.7%)、診断で立てた仮説どおりの機序で
改善したことを確認した。C01 は run間stdが大きく(±9.23)悪化とは言えない(ノイズ内)。
**P05 のみ明確に悪化**(fill_strict −6.40、fill_loose −3.26)しており、pre-packed×棚×
2コンテナ×優先という条件で棚上コンパートメントへの強制がどこかで不利な選択を招いている
可能性がある。ただし7シーン平均が両レジームで大きく改善しているため、Phase13/14で
定めた「両レジーム改善なら採用」の基準は満たしている。

### 2.5 旧6シーンの制約確認(修正後)

| シーン | fill_strict | fill_loose | stability | placement | soft |
|---|---|---|---|---|---|
| sample_config::000 | 27.97 | 41.94 | 97.92 | 100.00 | 100.00 |
| sample_config::001 | 25.26 | 31.97 | 97.85 | 100.00 | 100.00 |
| gen_2containers_patternB::001 | 19.76 | 29.16 | 98.37 | 100.00 | 100.00 |
| gen_2containers_priority::000 | 22.13 | 30.14 | 98.17 | 100.00 | 100.00 |
| gen_shelf_patternA::000 | 28.20 | 38.46 | 97.93 | 100.00 | 100.00 |
| gen_manyitems_patternA::000 | 36.27 | 50.98 | 98.07 | 100.00 | 100.00 |
| **平均** | **26.60** | **37.11** | **98.05** | **100.00** | **100.00** |

**placement/soft は全シーン満点を維持、stability は最小97.85で全シーン97以上を維持**
(budget=30sの開発予算での計測。値がphase14報告の対応シーンと僅かに異なるのは同一の
開発予算差によるもので、非shelf/非prepackedシーンの値はphase15ターゲット1・2いずれの
コード変更からも影響を受けない)。

### 2.6 実行時間(修正後、full budget)

| シーン | optimization[s] | policy[s] |
|---|---|---|
| gen_shelf_patternA | 26.02 | 0.30 |
| suite_C02_2c_55_shelfprio | 30.00 | 3.25 |
| suite_P04_B_1c_pre8_shelf | 0.00 | 2.82 |

**optimization 最大30.00s(<170s)、policy 最大3.25s(<7s)** で制約に対し大きな余裕がある。
追加した計算は既存の支持体ループに定数個のnumpy演算を足しただけで、計算量のオーダーは
変わっていない。

---

## 3. ターゲット3:cog再検討の準備(計測のみ・不採用)

`_score` の cog タイブレーク項 `cog_term = (1.0 - height_ratio) * mass_norm * W` の重み
`W` を一時的に 1.2→3.6(3倍)に変更し、パターンB5シーン(B01-B04, P04)で1点だけ計測した
(コードは計測後に 1.2 へ復元済み、commitなし)。

| W | fill_strict | fill_loose | cog_score |
|---|---|---|---|
| 1.2(現行) | 21.46 | 30.50 | 66.08 |
| 3.6 | 20.16 | 30.09 | 67.48 |
| 差 | **−1.30** | **−0.41** | **+1.40** |

**cog を上げると両レジームのfillが揃って下がるトレードオフが明確に存在する**。個別シーンでは
B04(fill_strict 30.27→22.05)のように大きく下がるものもあれば、B02(cog_score 64.77→66.67、
fill_strictはむしろ改善)のように逆に動くものもあり、シーン依存性が高い。本フェーズでは
採用しない(1点測定のみで、重みの最適値探索はしていない)。Phase16でcogの改善を検討する
際は、この1点だけでもトレードオフの存在と方向性(fillを犠牲にしないと有意なcog改善は
得られない)を判断材料にできる。

---

## 4. 制約の総合確認

| 制約 | 結果 |
|---|---|
| 旧6シーン placement/soft 満点 | 維持(§2.5) |
| 旧6シーン stability 97以上 | 維持(最小97.85、§2.5) |
| B01 stability | ターゲット2はB01に完全ゼロ影響(§2.3)。ターゲット1のcommit時点で97.89を維持済み |
| policy < 7s | 維持(最大3.25s、§2.6) |
| optimization < 170s | 維持(最大30.00s、開発予算のため実運用時の余裕はさらに大きい、§2.6) |

---

## 5. Phase16 への申し送り

1. **P02(pre-packed×offline optimize)のfill_strict回収は未完了。** fill_loose はほぼ
   回復したが fill_strict は phase13ベースラインに届いていない。この種のシーンは単一シーンの
   run間stdが±5pt超と大きく、corridor_penalty側の微調整では当落判定自体が困難。
   `ordering.build_order` の時間予算配分(offline optimize自体の安定性)を課題として
   切り出すことを推奨する。
2. **shelf系のP05(pre-packed×棚×2コンテナ×優先)がターゲット2で悪化した。** 7シーン平均は
   改善しているため採用したが、この組み合わせだけ棚上コンパートメントへの強制が不利に
   働いている可能性がある。次にshelf系を触るときの優先調査対象。
3. **「共通ヘルパー関数の戻り値を丸ごと転用する前に、その関数が何を返すか実装時に必ず確認する」
   という教訓(§2.3)。** 今回 `geo.static_obstacles()` が棚以外の常設構造物も含むことを
   見落とし、無関係シーンに実害を出しかけた。今後、新しいis_xxxフラグを支持体に付与する際は
   個別の生成関数(`small_shelf_aabb`/`big_shelf_aabb`等)を直接使うか、フラグ付与直後に
   最低1つの「無関係シーン」で実測してゼロ影響を確認すること。
4. **cog引き上げ(W: 1.2→3.6)は両レジームのfillとの明確なトレードオフが1点測定で確認できた
   (§3)。** 採用するなら、fillの犠牲をどこまで許容するかの基準(例:26シーン×3回のノイズ床
   ±0.45/±0.39以内)を先に決めてから重みの段階的掃引に進むべき。

---

## 6. 生成物一覧

**コード変更(push済み)**

- `agents/mysolver/planner.py`
  - ターゲット1(`80e664f`、セッション開始時点で既にcommit済み): `_collect_corridor_obstacles()`
    新規、`prepacked_ids` を `_search_best`/`plan` に配線
  - ターゲット2(`6ba4f17`、本セッションで実装・修正・commit): `_landing_supports` に
    `is_shelf` フラグ追加(`big_shelf_aabb`由来のみTrue)、`_evaluate_candidates` の着地面
    計算を棚下/棚上の2コンパートメントに分離
- `agents/mysolver/geometry.py` / `agent.py` / `ordering.py` / `simulate.py`
  (`80e664f`、`geo.initial_prepacked_ids()` 等ターゲット1関連)
- `tools/diagnose_shelf_zones.py` 新規 — 棚下/棚上の体積利用率を定量化する診断ツール(読み取り専用)
- `tools/diagnose_prepacked.py` — supportsタプルが5要素になったことに伴うアンパック修正のみ

**計測結果(`results/`, 生データJSONは非追跡)**

- `phase15_p02_current5.json` — P02単体5回追試(§1.3)
- `phase15_shelf7_before.json` / `phase15_shelf7_after_fixed.json` — shelf系7シーン×3回
  before/after(バグ修正後、§2.4の根拠)
- `phase15_old6_after_fixed.json` — 旧6シーンでの制約確認(バグ修正後、§2.5)
- `phase15_cog_base.json` / `phase15_cog_w3.6.json` — ターゲット3のcog重み1点測定(§3)
- `phase15_report.md` — 本報告(`git add -f` で追跡)
