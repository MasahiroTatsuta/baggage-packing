# Phase 17 報告：探索打ち切りの壁時計依存を全階層で排除する(決定性リファクタ)

作成日: 2026-07-26
対象コミット: (計測完了後に確定)
変更ファイル: `agents/mysolver/planner.py`, `agents/mysolver/simulate.py`,
`agents/mysolver/ordering.py`, `agents/mysolver/agent.py`,
`tools/phase17_probe.py`(新規), `tools/phase17_dump.py`(新規),
`tools/phase17_analyze.py`(新規), `tools/phase17_validate.sh`(新規)

*** 本ファイルは計測進行中のドラフトです。最終版は全計測完了後に差し替えます。 ***

---

## 1. 背景と方針

Phase16 §2.3 で、反復間ノイズ(P02 で fill_strict ±5.14)とシーン単位の budget 非単調性
(スプレッド最大13.4pt)の共通根本原因が
`planner._search_best` / `_evaluate_candidates` の多重な `time.perf_counter()` チェックに
あることを特定した。リスタート層だけの決定化では解決しないことも実証済み。

本フェーズでは、打ち切り条件の**表現**を「壁時計」から「消費した評価コスト(ユニット)」へ
全階層で置き換える。挙動そのものを変えるのが目的ではないが、打ち切り位置は必然的に
変わるため、出力の完全一致は「optimize無効かつ時間内に自然完了するシーン」でのみ要求する。

壁時計に残す役割は2つだけ:

1. **較正**: 呼び出し元が秒で表現した予算を、決定的な定数 `UNITS_PER_SEC` でユニット数へ換算する。
2. **非常用の最終安全弁**: 本番タイムアウト(policy 8s / optimization 180s)を絶対に踏まない
   ための保険。通常は発火しない。発火した場合のみ決定性が失われるが、その場合でも
   制約遵守を優先する(方針3)。

---

## 2. 実装

### 2.1 コストモデルの実測較正(`tools/phase17_probe.py`)

`_evaluate_candidates` の全呼び出しについて (候補XY数 n_xy, supports数, obstacles数, 実時間) を
収集し、`_candidate_xy` と `plan()` の実時間も併せて計測した(6シーン、`plan()` 総時間155.1s、
`_evaluate_candidates` 29994回、`_candidate_xy` 26420回)。

| 内訳 | 総時間 | 1回あたり | 呼び出し回数 | plan時間比 |
|---|---|---|---|---|
| `_evaluate_candidates` | 62.6s | 2.09ms | 29994 | 40.4% |
| `_candidate_xy` | 90.2s | 3.41ms | 26420 | 58.1% |

**重要な実測結果: 候補集合の構築(`_candidate_xy`)のほうが評価(`_evaluate_candidates`)より重い。**
グリッド生成 + Extreme Point 列挙 + `set`/`sorted` が Python レベルで走るためで、しかも
呼び出し回数もほぼ同数である(候補キャッシュは `(pool_idx, orn_idx)` 単位で、y層のlevel 0 で
確定することが多いため再利用がほとんど効かない)。評価コストだけをユニット化すると探索作業の
半分以上を計量し損ねるため、候補構築も同じユニット系で計上する。

採用したコストモデル:

```
units(評価)     = n_xy * (n_supports + n_obstacles + COST_CONST)      # COST_CONST = 8.0
units(候補構築) = CANDIDATE_BUILD_COST * (グリッド点数 + 8 * n_obstacles)  # = 18.2
```

`CANDIDATE_BUILD_COST` は「両者が同じ units/sec になる」ように決めた
(評価 1.63e7 units/s ÷ 候補構築 8.94e5 生units/s = 18.2)。

較正定数は集計 1.60e7 units/s(シーン別 1.31e7〜2.01e7)に対し、名目秒あたりの実所要時間が
名目を超えにくいよう **`UNITS_PER_SEC = 1.55e7`** を採用した。

### 2.2 `planner.SearchBudget`

打ち切りを「消費ユニット」で判定する予算オブジェクト。親子の入れ子をサポートし、
`spend` は親へ伝播、`exhausted` は親も見る(1手の予算 ⊂ 1リスタートの予算 ⊂ optimize全体の予算)。
`exhausted()` は「これ以上新しい評価を**始めない**」判定であり、始めた評価は必ず最後まで行う。
超過量は1回の `_evaluate_candidates` 分(高々数ms相当)で有界。

### 2.3 各階層の置き換え

| 階層 | 旧: 壁時計 | 新: ユニット |
|---|---|---|
| `_evaluate_candidates` | 入口で `perf_counter() > deadline` | `budget.exhausted()` → `budget.spend(units)` |
| `_search_best` | container/level/pool/orientation の4箇所で `perf_counter()` | 同4箇所を `budget.exhausted()` |
| `plan` | `deadline = start + time_budget`、再探索の可否も壁時計 | `SearchBudget.from_seconds(time_budget)`、再探索可否は `not budget.exhausted()` |
| `simulate.simulate_order` | 毎ステップ `deadline - now` から予算算出 | 親予算の `child_seconds(per_step)` |
| `simulate.greedy_construct_order` | 同上 | 同上 |
| `ordering.build_order` | リスタート配分・打ち切りを「残り時間」で決定 | 「残りユニット」で決定 |

### 2.4 build_order が単調になる理由

残り予算がユニットで決まると、**リスタート回数 N も各リスタートへの配分も決定的**になる。
さらにリスタート i の乱数列はリスタート 0..i-1 の消費だけで決まり、それらも決定的なので、
**リスタート i の結果は N に依存しない**。したがって `build_order` は
「決定的な系列の先頭 N 個の argmax」という構造になり、予算(=N)を増やすと目的関数
(risk調整済み体積 − placementペナルティ)が**単調に**改善する。
Phase16 §1.5 で観測した「予算を変えると乱数列がまるごとシフトして、シーンごとに改善するか
悪化するかが事実上ランダムになる」構造がここで解消される。

### 2.5 非常用安全弁(方針3)

| 経路 | 名目予算 | 安全弁 | 根拠 |
|---|---|---|---|
| `agent.policy` | 5.5s 相当のユニット | `POLICY_HARD_WALL = 6.0s` | policy_timeout 8s / 制約 policy<7s。決定性より制約遵守を優先 |
| `agent.optimize` | `DEFAULT_TIME_BUDGET = 120s` 相当 | `min(165s, 名目 × 1.4)` | optimization_timeout 180s / 制約 <170s |

安全弁の壁時計チェックは `exhausted()` 64回に1回に間引いている(発火しない限り結果に関与しない)。
本環境より**遅い**マシンでは同じユニット数の消化に時間がかかるが、安全弁が発火するまでは
最後まで決定的に探索しきる(=マシン速度に依らず同じ出力)。本環境より**速い**マシンでは
同じ探索を短時間で終える(安全側)。

---

## 3. 検証(進行中)

### 3.1 リファクタ前後の完全一致(optimize無効シーン)

`tools/phase17_dump.py` で optimize が返した順序・毎ステップの action・最終配置を
JSON に落とし、sha256 で比較した。

| シーン | 配置数 | digest | 判定 |
|---|---|---|---|
| B01_1c_40_plain | 22 | `e61510eaa6d015e2` | **IDENTICAL** |
| B02_1c_40_shelf | 20 | `ba096f649111f22a` | **IDENTICAL** |
| B03_2c_80_prio | 27 | `4fb88042d6452548` | **IDENTICAL** |
| B04_2c_80_noprio | 45 | `646f6bc1aa516694` | **IDENTICAL** |
| P04_B_1c_pre8_shelf | 23 | `f505ad7355e7ef43` | **IDENTICAL** |

5/5 シーンでリファクタ前後の出力が完全一致。これらは optimize 無効(=`build_order` を
通らない)かつ policy が名目予算内に自然完了するシーンであり、旧実装でも打ち切りが
一度も発火していなかったことの裏返しでもある。

### 3.2 検証(a): P02 の反復間ノイズ — std 5.25 → **0.0000**

`tools/measure_regime.py --repeats 5` を budget 30(旧ベースラインと同条件)と
budget 120(現デフォルト)の両方で実施した。

| budget | fill_strict | fill_loose | cog | stability | placement | soft |
|---|---|---|---|---|---|---|
| 30(ベースライン, Phase15/16) | 20.71±5.25 | 35.65±5.59 | — | — | — | — |
| **30(Phase17)** | **19.8895±0.0000** | **31.7930±0.0000** | 75.1866±0 | 98.5675±0 | 100±0 | 100±0 |
| **120(Phase17)** | **25.1655±0.0000** | **34.2442±0.0000** | 71.5644±0 | 98.5304±0 | 100±0 | 100±0 |

**全6指標で反復間 std = 0.0000(n=5)**。目標(±1以下)を大幅に達成した。

決定的に重要なのは、これが「マシンが偶然安定していたから」ではないことである。
同じ5回のrunの実所要時間は

* budget 30: 41.5 / 34.4 / 36.6 / 29.5 / 37.4 秒(最大最小比 **1.41倍**)
* budget 120: 141.5 / 111.3 / 107.1 / 102.5 / 101.9 秒(最大最小比 **1.39倍**)

と約4割ばらついているにもかかわらず、出力は完全に同一である。**壁時計のゆらぎが
出力に到達しなくなった**ことの直接的な証拠であり、Phase16 で「リスタート層だけを
決定化しても std が 5.25→5.14 と動かなかった」のと対照的な結果になった。

副次的な観察として、P02 自身が 30→120 で strict 19.89→25.17 / loose 31.79→34.24 と
**単調に**改善している。Phase16 の同シーン(22.96 / 23.14 / 26.89 / 22.04)が
予算に対してカオス的に振れていたのと対照的で、§2.4 の「予算に対する単調性」が
実際に効いていることを示唆する(全シーンでの確認は §3.3)。

### 3.3 検証(b): budget 再掃引(30/60/120/165)

(計測中)

### 3.4 検証(c): 26シーンの回帰

(計測中)

### 3.5 制約確認(旧6シーン)

(計測中)

---

## 4. 計測プロトコルの再評価

(計測完了後に記載)
