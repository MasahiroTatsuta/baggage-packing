# Phase78 報告: ビームウィンドウの is_soft 問題(境界またぎ → ソフト先取り)

## 結論(先出し)

- **ステップ1で Phase72 の推測を確定させた。** `beam_construct_order` の window が
  ハード/ソフト境界をまたぐと、window 内で最高スコアのソフト荷物が(ハードが
  残っていても)先に選ばれ、「ハード先積み」が崩れる。
- **崩れは広範**: 本番相当の time_budget=150s で全26シーンを測り、採用順序での
  ソフト位置が理想から **|差| > 0.05 のシーンが 12/26**(判定基準は「5シーン以上」)。
- 判定基準に従い**ステップ2へ**: `MYSOLVER_BEAM_SOFT_LAST`(既定 '0'、無効)を実装。
  window 内にハードが残る限りソフトを候補から外す。既定無効で決定的8シーン 8/8
  ビット単位不変を確認。
- **A/B(緩2 構成・150s・26シーン)の結果は弱いプラスだが一貫しない**:
  num_placed_items +0.89pp(+6実数)、しかし per-scene は 5改善/3〜5悪化のカオス的再配置。
  local `soft_item_score` は OFF/ON とも全シーン 100.00 で無反応(ローカル採点器は
  足切り/soft を実装しておらず本命効果を測れない ── 本フェーズの前提)。
- **提出候補 `mysolver_submit_loose2_softlast.zip` を作成**(枠には入れない)。機序は
  確定しており本番で `num_placed_items` と `soft_item_score` を直接見る価値がある。
  SHA256 `04a6a1b88f7fcc7f57b32c164744f3abf59d910062f0bcaab2db8887811701b6`。

---

## ステップ1-1: 採用順序でのソフト位置の崩れ(time_budget=150s、全26シーン)

`tools/phase78_soft_positions.py`(読み取り専用。`build_order` の戻り順を観測するだけ)。
`mean` = 採用順序でのソフト荷物の平均位置比率、`ideal` = 全ソフトが末尾に固まった
理想順序での同比率、`diff = mean − ideal`(負ほど「ソフトが前に出て崩れている」)。
`hard_after` = 最初のソフトより後ろにあるハード荷物の数。

### |diff| > 0.05 のシーン(12/26、崩れの大きい順)

| シーン | diff | 最初のソフト位置(比率) | その後のハード残数 | n_soft/n_items |
|---|---:|---:|---:|---:|
| P01_A_1c_pre6            | **−0.667** | 5 (0.152) | 26 / 31 | 3/34 |
| P04_B_1c_pre8_shelf      | **−0.433** | 3 (0.091) | 24 / 27 | 7/34 |
| A01_1c_40_plain          | **−0.427** | 10 (0.256) | 24 / 34 | 6/40 |
| D04_A_1c_40_flat         | **−0.392** | 5 (0.128) | 25 / 30 | 10/40 |
| B01_1c_40_plain          | **−0.379** | 0 (0.000) | 30 / 30 | 10/40 |
| P02_A_1c_pre10           | **−0.374** | 15 (0.385) | 18 / 33 | 7/40 |
| A06_1c_40_small          | **−0.301** | 18 (0.462) | 14 / 32 | 8/40 |
| C01_1c_40_shelf          | **−0.244** | 16 (0.410) | 16 / 32 | 8/40 |
| A07_1c_40_bulky          | **−0.240** | 9 (0.231) | 23 / 32 | 8/40 |
| D01_A_1c_40_softheavy    | **−0.228** | 0 (0.000) | 16 / 16 | 24/40 |
| P06_A_1c_pre12_dense     | **−0.217** | 12 (0.414) | 11 / 23 | 7/30 |
| A03_1c_40_shelf          | **−0.090** | 22 (0.564) | 10 / 32 | 8/40 |

- **`D01` は Phase72 が指摘した当該シーン**: mean 0.478 / ideal 0.705 / diff −0.228。
  Phase72 の「150s で 0.478 まで崩れる」と完全一致。
- **崩れは `1c_40`(単一コンテナ・40個)と `pre*`(既積み)に集中**。B01 は最初のソフトが
  順序の**先頭(位置0)**に出て、その後ろに30個すべてのハードが並んでいる。
- **崩れていないシーン(diff ≈ 0.000、14/26)**: A02/A04/A05/A08/B02/B03/B04/C02/C03/
  D02/D03/D05/P03/P05 ── ほぼ全て 2コンテナ or 80個以上の大規模シーン。

生データ: `results/phase78_soft_positions_b150.json` / `.txt`

---

## ステップ1-2: window がハード/ソフト境界をまたぐ頻度(機序の確定)

`tools/phase78_window_trace.py` + `simulate.py` に追加した読み取り専用トレース
(`LAST_BEAM_TRACE`、環境変数 `MYSOLVER_BEAM_TRACE=1` のときだけ1ステップ1行記録。
既定では分岐に入らず 8/8 ビット単位不変)。

### window 幅の決まり方(コード確認)

`ordering.WINDOW_CANDIDATES = [15, 20, 25, 30, None]`。フェーズ1は体積優先シード
(is_soft 第1キー)で全 window を決定的に試す。`beam_construct_order` は各ステップで
`pool = list(remaining.values())[:window]` を作り、`planner.plan_topk` が **pool 内の
スコア最大手を返す**。pool 内にハード/ソフトが混在していても選択はスコアのみで決まり、
**「ハードが残っている間はソフトを選ばない」というガードは存在しない**。

### 実測(全26シーン合算、フェーズ1相当の決定的構築)

| window | 境界またぎ step 合計 | **ソフト先取り step 合計**(pool にハードが残るのにソフトを選択) | 該当シーン数 |
|---:|---:|---:|---:|
| 15   | 95  | **36** | 9/26 |
| 20   | 165 | **71** | 13/26 |
| 25   | 217 | **83** | 15/26 |
| 30   | 287 | **77** | 16/26 |
| None | 411 | **97** | 24/26 |

- **window が大きいほど境界またぎもソフト先取りも単調に増える。** `None`(= 残り全件、
  WINDOW_CANDIDATES に含まれる)では 24/26 シーンでソフト先取りが発生。
- 小さい window でも `1c_40` 系(A03/A06/B01/B02/D01/D04/P06 等)は w15 から崩れる
  ── これらはハード荷物が少なく、序盤から pool が境界をまたぐため。
- **Phase72 の推測(境界またぎで最高スコアのソフトが先に選ばれる)は確定。**

生データ: `results/phase78_window_trace.json` / `.txt`

---

## ステップ1-3: 判定

- 崩れが広範: **|diff| > 0.05 が 12 シーン ≥ 5**。
- 原因の確定: window 境界またぎ → ソフト先取りをステップ全数で実測。
- → **ステップ2(修正して A/B)へ進む。**

---

## ステップ2: 修正と A/B

### 2-1: 実装(`agents/mysolver/simulate.py`)

`beam_construct_order` の1ステップで、`pool`(window 適用後)にハード荷物が1つでも
残っている間はソフト荷物を pool から除外する。全部ソフトになったら通常どおり。

```python
if _BEAM_SOFT_LAST:                       # MYSOLVER_BEAM_SOFT_LAST=1 のときだけ
    _hard_pool = [it for it in pool if not it.get('is_soft', False)]
    if _hard_pool:
        pool = _hard_pool
```

- `_BEAM_SOFT_LAST = os.environ.get('MYSOLVER_BEAM_SOFT_LAST', '0') == '1'`。**既定 '0'
  で無効**(従来経路とビット単位で同一)。
- あわせて読み取り専用トレース `LAST_BEAM_TRACE`(`MYSOLVER_BEAM_TRACE=1` 時のみ)を
  追加。既定ではどちらの分岐にも入らない。

### 2-2: 決定的8シーンのビット単位不変

`MYSOLVER_BEAM_SOFT_LAST` 未設定(既定)で B01–B04 / P04 / A01–A03 の build_order を
`scripts/bp_baseline_8scenes.json` と照合 → **8/8 一致**。

### 2-3: A/B(緩2 構成 0.35/0.4/0.25、time_budget=150s、全26シーン、shake=19.6)

`tools/phase67_suite_metrics.py` を BEAM_SOFT_LAST=0/1 で各1回(1シーン1ロールアウト)。

#### 平均(26シーン)

| 指標 | OFF | ON | Δ |
|---|---:|---:|---:|
| **num_placed_items** | 98.82% | **99.70%** | **+0.89pp** |
| placed 実数(26シーン合計) | 668 | **674** | **+6** / 1475 |
| **soft 実数(実際に置けた)** | 75 | **47** | **−28** / 295 |
| fill_score | 27.08 | 27.37 | +0.29 |
| cog_score | 63.03 | 63.74 | +0.71 |
| stability_score | 82.89 | 82.44 | −0.45 |
| placement_A (local) | 100.00 | 100.00 | 0.00 |
| **soft_A (local)** | **100.00** | **100.00** | **0.00** |
| composite_A | 67.58 | 67.72 | +0.14 |
| composite_B | 51.39 | 49.58 | −1.81 |

#### 効いてはいるが方向が一貫しない(per-scene)

| 改善 | placed / fill | 悪化 | placed / fill |
|---|---|---|---|
| D05_1c_40_tall | +13 / +8.8 | A03_1c_40_shelf | −6 / −14.8 |
| A01_1c_40_plain | +5 / +9.9 | D01_1c_40_softheavy | −5 / +1.6 |
| D04_1c_40_flat | +4 / +6.7 | P05_2c_pre8 | −3 / −8.9 |
| C01_1c_40_shelf | +2 / +7.8 | P01_1c_pre6 / P06_dense | −2 / −4〜−5 |

18/26 シーンは完全に不変。動いた8シーンは 5改善 / 3〜5悪化で、Phase60・71 で見た
「カオス的な再配置」と同じ。

#### 解釈

- **主KPI(num_placed_items)は弱いプラス**: +6実数 / +0.89pp。ただし per-scene は
  改善と悪化が拮抗しており、明確なレバーとは言えない。
- **`soft_A`(local soft_item_score)は OFF/ON とも全シーン 100.00 で完全に無反応。**
  ローカル採点器は soft_item / 足切りを実装しておらず(Phase70)、この軸の
  本命効果はローカルでは原理的に測れない ── **本フェーズを立てた前提そのもの**。
- **実際に置けたソフト荷物は 75→47 と 28個(−37%)減った。** 修正が意図どおり
  「ハードを先に積み切る」ため、途中で打ち切られるシーンではソフトに到達しない。
  これが本番の `soft_item_score` にプラスに出るか(足切り通過シーンが増える)
  マイナスに出るか(ソフト配置数そのものが減る)は、**ローカルでは判定不能**。

### 2-4: zip 化(提出候補、枠には入れない)

主KPI が弱いながらプラスで、機序も確定している(26シーン中 24 で window=None の
ソフト先取りが発生)ため、**本番で直接確かめる価値はある**と判断し提出候補を作成:

- `submissions/mysolver_submit_loose2_softlast.zip`
  (緩2 閾値 0.35/0.4/0.25 + `MYSOLVER_BEAM_SOFT_LAST` 既定 '1')
- **SHA256**: `04a6a1b88f7fcc7f57b32c164744f3abf59d910062f0bcaab2db8887811701b6`
  (**アップロード時に必ず照合**)
- zip と作業ツリーの差分は `planner.py`(緩2 の3行)+ `simulate.py`(BEAM_SOFT_LAST
  既定 '0'→'1' の1行)のみ。他8ファイルはビット単位一致。11エントリ、既存zipと同一構造。
- 全13定数 grep 済み(閾値3=0.35/0.4/0.25、幾何3=既定 −0.012/0.022/0.016、
  他7=既定)。
- 決定的8シーンの対緩2 差分: A/B の per-scene から B01–B04・P04・A01–A03 は
  A01(placed 19→24)・A03(22→16)の2シーンで挙動が変わる(他6は不変)。

**枠には入れない**(主枠 緩2 / 2枠目 rest020 は Phase77 のまま)。本番結果で
`num_placed_items` と `soft_item_score` の両方が緩2(57.18)を上回れば枠入れを検討。
下回る/横ばいなら BEAM_SOFT_LAST は不採用とし、`is_soft` 問題は「ローカルでは
崩れが見えるが最終スコアには効かない」で決着。

---

## やっていないこと

- 幾何定数・支持閾値は緩2 で固定(いじっていない)。
- 本番の集計スコアから足切り閾値やシーン数の逆算はしていない。

## 生成物一覧

- `agents/mysolver/simulate.py`(`MYSOLVER_BEAM_SOFT_LAST` スイッチ + 読み取り専用
  `LAST_BEAM_TRACE`。既定はどちらも無効、8/8 不変)
- `tools/phase78_soft_positions.py` / `tools/phase78_window_trace.py`(新規、計測用)
- `results/phase78_soft_positions_b150.json` / `.txt`
- `results/phase78_window_trace.json` / `.txt`
- `results/phase78_report.md`(本ファイル)
- `results/phase78_ab_soft_last_OFF.json` / `_ON.json`(A/B 結果)
- `submissions/mysolver_submit_loose2_softlast.zip`(提出候補、未追跡、SHA256
  `04a6a1b88f7fcc7f57b32c164744f3abf59d910062f0bcaab2db8887811701b6`)
