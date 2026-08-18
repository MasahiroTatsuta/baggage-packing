# Phase45 報告: placement/soft_item乖離の原因調査(「途中終了」仮説の検証)

## 0. 背景

Phase44は原因候補として (1) 壁時計の圧力 (2) scorer.pyの近似ずれ を挙げたが、
より単純で確実性の高い候補として「**sudden death を起こした異常な配置状態が、
そのまま placement_score/soft_item_score の違反としても数えられている**」という
仮説を検証する。仮説(2)は本番の判定式が非公開で比較対象が無いため検証手段が無く、
追わない(指示どおり)。

---

## ステップ1: ローカルで sudden death を再現する

### (1-1) 26シーンの status 集計

既存データ(`results/bp_ab_phase42_latch_fix_off.json`、壁時計非拘束・
`MYSOLVER_REPLICA_SELECT=0`・`UNITS_PER_SEC=2.00e7`、Phase42で測定済み)の
`statuses` フィールドと、Phase44の診断結果(`n_placed`)を突き合わせた。

**この時点で「完走シーンしか無い」という前提には該当しなかった**
(**25/26シーンが既に "Stopped in the middle" で終了している**)。
したがって、指示の分岐条件(1-2「完走シーンしか無い場合、人工的に途中終了を起こす」)の
**前提が成立せず、人工的な再現ステップは実施しなかった**。25件という多数の実例が
既に存在し、既存データで(1-3)の判定に直接進めるため。

| シーン | 配置数/総数 | 配置率 | placement | soft | status |
|---|---:|---:|---:|---:|---|
| A01_1c_40_plain | 20/40 | 50.0% | 100.00 | 100.00 | STOPPED |
| A02_1c_80_plain | 17/80 | 21.2% | 100.00 | 100.00 | STOPPED |
| A03_1c_40_shelf | 19/40 | 47.5% | 100.00 | 100.00 | STOPPED |
| A04_2c_80_noprio | 32/80 | 40.0% | 100.00 | 100.00 | STOPPED |
| A05_2c_80_prio | 24/80 | 30.0% | 100.00 | 100.00 | STOPPED |
| A06_1c_40_small | 40/40 | 100.0% | 100.00 | 100.00 | **completed** |
| A07_1c_40_bulky | 12/40 | 30.0% | 100.00 | 100.00 | STOPPED |
| A08_2c_140_extreme | 20/140 | **14.3%** | 100.00 | 100.00 | STOPPED |
| B01_1c_40_plain | 22/40 | 55.0% | 100.00 | 100.00 | STOPPED |
| B02_1c_40_shelf | 20/40 | 50.0% | 100.00 | 100.00 | STOPPED |
| B03_2c_80_prio | 27/80 | 33.8% | 100.00 | 100.00 | STOPPED |
| B04_2c_80_noprio | 45/80 | 56.2% | 100.00 | 100.00 | STOPPED |
| C01_1c_40_shelf | 20/40 | 50.0% | 100.00 | 100.00 | STOPPED |
| C02_2c_55_shelfprio | 36/55 | 65.5% | 100.00 | 100.00 | STOPPED |
| C03_2c_80_prio | 27/80 | 33.8% | 100.00 | 100.00 | STOPPED |
| D01_A_1c_40_softheavy | 21/40 | 52.5% | 100.00 | 100.00 | STOPPED |
| D02_A_1c_40_prioheavy_nocont | 21/40 | 52.5% | 100.00 | 100.00 | STOPPED |
| D03_A_2c_60_prioheavy_cont | 40/60 | 66.7% | 100.00 | 100.00 | STOPPED |
| D04_A_1c_40_flat | 20/40 | 50.0% | 100.00 | 100.00 | STOPPED |
| D05_A_1c_40_tall | 12/40 | 30.0% | 100.00 | 100.00 | STOPPED |
| P01_A_1c_pre6 | 19/34 | 55.9% | 100.00 | 100.00 | STOPPED |
| P02_A_1c_pre10 | 31/40 | 77.5% | 100.00 | 100.00 | STOPPED |
| P03_A_2c_pre8_prio | 38/60 | 63.3% | 100.00 | 100.00 | STOPPED |
| P04_B_1c_pre8_shelf | 23/34 | 67.6% | 100.00 | 100.00 | STOPPED |
| P05_C_2c_pre8_shelfprio | 36/50 | 72.0% | 100.00 | 100.00 | STOPPED |
| P06_A_1c_pre12_dense | 28/30 | 93.3% | 100.00 | 100.00 | STOPPED |
| **平均配置率** | | **52.3%** | | | **25/26 STOPPED** |

本番の `num_placed_items=61.4%` と極めて近い平均配置率(52.3%)で、かつ本番と全く同じ
失敗検査シグネチャ(`Did not satisfy ['is_included', 'is_valid', 'is_placed_safe']`)で
途中終了しているにもかかわらず、**placement_score・soft_item_scoreは25シーン全件で
100.00のまま**(14.3%しか配置できなかったA08含む)。

### (1-3) 判定

**100を下回るシーンは0件 → 仮説は否定。** 「途中終了そのもの」がplacement/soft_item
違反を引き起こしているわけではない。既に25件という多様な実例(配置率14.3%〜93.3%の
範囲)があり、人工的な再現を追加する必要は無いと判断した。

---

## ステップ2: 壁時計の圧力を検証する

ステップ1が否定されたため実施した。`MYSOLVER_HARD_WALL_LIMIT=165`(本番既定)で
26シーンを実行(`tools/diagnose_stacking.py --out results/phase45_stacking_diag_wallbound.json`)。

### (2-1)(2-2) 結果

**26/26シーンで `n_placed` が壁時計非拘束時と完全に同一**(1件の差分も無し)、
25/26が"Stopped in the middle"、**placement_score・soft_item_scoreは26シーン全件で
100.00のまま**(違反0/26)。

全26シーンの中で最も総所要時間(optimize+policy)が長かったのは
`suite_A06_1c_40_small.json`の170.5秒(2番目は`C02_2c_55_shelfprio`の146.2秒)。
A06はそれでも`episode_status='The packing has been completed successfully.'`
(=途中終了していない)であり、170.5秒のうち超過分はpolicy loop(HARD_WALL_LIMITの
対象外)の時間であって、`HARD_WALL_LIMIT`が制約する構築(optimize)フェーズ自体が
165秒に到達した形跡は無い。

**結論**: **このMac環境では、本番既定値(165秒)に壁時計を拘束しても、26シーン中
1件も挙動が変わらなかった。** ローカルのunit予算システム(`UNITS_PER_SEC`較正済み)は
そもそも165秒よりずっと短い時間で構築を終える設計になっており、**このシーン規模・
このハードウェアでは壁時計165秒の天井に一度も到達しない**。したがって
「壁時計の圧力がplacement/soft_item違反を引き起こす」という仮説も、**ローカルでは
再現も反証もできなかった**(=本番の`optimization=165.109`(天井に張り付いている)
という状況自体をローカルで再現できていない)。

---

## まとめ・見立て

Phase45で検証した2つの仮説は、いずれも実データにより**明確に否定・不確定**となった:

- **仮説A(途中終了そのものが原因)**: 25件の実例(配置率14.3%〜93.3%)で
  一貫して違反0 → **否定**。
- **仮説B(壁時計の圧力)**: ローカル環境では本番既定の165秒制限を課しても
  1件も挙動が変わらず、本番のような「天井張り付き」状態自体を再現できない
  → **ローカルでは検証不能(反証というより測定不能)**。

仮説C(scorer.pyの近似ずれ)は指示により追わない。

**残る手がかり**: 本番とローカルで一致しているのは「同じ失敗検査シグネチャで
途中終了する」ことだけであり、**その後の配置内容(どのアイテムがどこに置かれたか)は
本番とローカルで別物である可能性が高い**——ローカルのplanner.pyの意思決定は
Phase44で確認した通り優先/ソフトの下敷きを避ける設計だが、本番で42%/53%もの
違反率が出るということは、本番の**構築順序(optimize)またはpolicy判断そのもの**が
ローカルと異なる結果を出している可能性が濃厚(仮説Bで示唆された「本番はより遅い
ハードウェアで壁時計に本当に追われている」ことと矛盾しない——ただしローカルでは
その状況そのものを作れないため、直接の確認はできなかった)。次の一手について
指示を仰ぐ。

---

## ステップ3: 診断スクリプトのリポジトリ化

Phase41〜44で3回、同種のスクリプトがセッションのスクラッチパッドに残るだけで
リポジトリ化されていない、という指摘があった。今回は単純な「ファイル移動」ではなく、
`argparse`によるCLI化・出力先の一般化を行った上で `tools/` 配下に置いた
(ハードコードされたパスのままでは実質的に「再実行できる形」とは言えないため)。

- **`tools/diagnose_stacking.py`**(Phase44の`phase44_stacking_diag.py`を一般化):
  26シーン(または `--config-path` で絞った任意のシーン集合)で
  is_prioritized/is_soft配置数・積み重なりペア数・下敷き数・episode_status(sudden death
  検出用)を測定しJSON出力する。

  ```
  MYSOLVER_HARD_WALL_LIMIT=3000 PYTHONPATH=. python tools/diagnose_stacking.py \
      --out results/phase45_stacking_diag_off.json     # 壁時計非拘束
  MYSOLVER_HARD_WALL_LIMIT=165 PYTHONPATH=. python tools/diagnose_stacking.py \
      --out results/phase45_stacking_diag_wallbound.json  # 本番既定の壁時計拘束
  ```

- **`tools/diagnose_stacking_geocheck.py`**(Phase44の`phase44_geo_crosscheck.py`を
  一般化): `diagnose_stacking.py`の出力(`--no-geo`を付けない実行)に対して、
  pybullet接触判定とは独立な幾何的クロスチェックを行う。`--scene`省略時は
  最も荷物が密なシーンを自動選択する。

  ```
  python tools/diagnose_stacking_geocheck.py results/phase45_stacking_diag_wallbound.json
  ```

いずれも読み取り専用(`tools/scorer.py`・`configs/`は変更しない)。

---

## 変更ファイル

- `tools/diagnose_stacking.py`(新規、Phase44スクリプトの一般化)
- `tools/diagnose_stacking_geocheck.py`(新規、同上)
- `results/phase45_stacking_diag_wallbound.json`(ステップ2の実測データ)
- `results/phase45_report.md`(本ファイル)

`tools/scorer.py`・`configs/gen`は変更していない(指示どおり)。
