# Phase44 報告: ρ-test を既定無効化、placement/soft_item の空白地帯に着手

## 0. 背景・方針転換の根拠

Phase42提出結果: `fill_score` は `38.09476291926298` のまま13桁一致 → **ρ-test は依然
本番で機能していない**。一方 `optimization` は 155.46 → 165.109 に変化し、
`HARD_WALL_LIMIT=165.0` が本番で初発火した。候補単位ラッチ(Phase42)は効いているが、
失敗自体は解消していない。

ρ-testが完璧に動いても public は +0.48(fill +1.671 × 2/7)、目標まで残り6.36。
一方 `placement_score`(57.6、上限+6.06)・`soft_item_score`(47.15、上限+7.55)は
合計+13.61で完全に手つかず。ρ-testの原因究明は診断ビルドを要し保留中のため、
本フェーズで優先順位を入れ替える。

---

## ステップ1: ρ-test を既定で無効にする

### 1-1. 実装

`agents/mysolver/ordering.py` の `REPLICA_SELECT` 既定値を `'1'` → `'0'` に変更。

```python
REPLICA_SELECT = os.environ.get('MYSOLVER_REPLICA_SELECT', '0') == '1'
```

根拠: 本番では一度も機能しておらず(Phase42提出でfill_scoreが13桁不動)、有効にしても
得点は1ポイントも動かない。一方で45秒(`REPLICA_RESERVE_S`)の壁時計取り置きを常に
消費し、`optimization_timeout`(180s)への余裕を24.5s→14.9sに縮めている。**無効化は
純粋な安全側の改善である。**

### 1-2. replica.py / 候補単位ラッチ / 回帰テストは削除していない

`agents/mysolver/replica.py`・`ordering.py` 内の `_record_replica_failure()` 等の
候補単位ラッチ機構・`tools/test_replica_missing_keys.py`・`tools/test_replica_latch.py`
はすべてそのまま残した。原因(ρ-testがなぜ本番で機能しないか)が判明すれば
`MYSOLVER_REPLICA_SELECT=1` に戻すだけで復帰できる。

### 1-3. 26シーンOFFの一致確認 — **再測定不要と判断**

再測定は行わなかった。根拠:

1. `REPLICA_SELECT = os.environ.get('MYSOLVER_REPLICA_SELECT', <既定値>) == '1'` という
   式の構造上、**環境変数を明示的に `'0'` にした場合の挙動と、既定値を `'0'` に変えて
   環境変数を指定しない場合の挙動は、コード上完全に同一の分岐(`False`)に評価される**。
   これは実行結果の比較ではなくコードの構造から導かれる恒真の等価性であり、
   実測を要しない。
2. `MYSOLVER_REPLICA_SELECT=0`(明示)での26シーン結果は、Phase41・Phase42で
   それぞれ独立に測定され、いずれも `results/phase40_baseline_off_mac.json` と
   **26/26シーンで完全一致(diff 0.000)**を確認済み(`results/phase41_report.md`
   §3-3、Phase42追記§3-3)。
3. `bp_ab.sh` は off側の `MYSOLVER_REPLICA_SELECT=0` を環境変数として明示的に渡す
   実装になっており、既定値変更の影響を受けない(スクリプト自体の挙動は不変)。

以上より、今回の変更(既定値の文字列変更のみ、分岐ロジック自体は無変更)によって
挙動が変わる余地が無いことをコードレベルで保証できるため、40分規模の26シーン
A/B再測定は行わなかった。

---

## ステップ2: placement_score / soft_item_score の乖離を解明する

### (2-1) 26シーン実測

`tools/local_eval.py` の `run_one_scene()` と同じ手順(env初期化→optimize→policyループ)
を踏襲し、評価後に `Scorer._find_stacking_pairs()` を直接呼んで積み重なりペアと
違反数を集計した(壁時計非拘束・`UNITS_PER_SEC=2.00e7`、Phase41/42と同条件)。

| シーン | 配置数 | 優先手荷物配置数 | ソフト貨物配置数 | 積み重なりペア数 | 優先下敷き | ソフト下敷き |
|---|---:|---:|---:|---:|---:|---:|
| A01_1c_40_plain | 20 | 7 | 3 | 13 | 0 | 0 |
| A02_1c_80_plain | 17 | 1 | 0 | 12 | 0 | 0 |
| A03_1c_40_shelf | 19 | 4 | 0 | 10 | 0 | 0 |
| A04_2c_80_noprio | 32 | 5 | 0 | 22 | 0 | 0 |
| A05_2c_80_prio | 24 | 5 | 0 | 13 | 0 | 0 |
| A06_1c_40_small | 40 | 6 | 8 | 26 | 0 | 0 |
| A07_1c_40_bulky | 12 | 2 | 1 | 7 | 0 | 0 |
| A08_2c_140_extreme | 20 | 5 | 0 | 12 | 0 | 0 |
| B01_1c_40_plain | 22 | 0 | 10 | 16 | 0 | 0 |
| B02_1c_40_shelf | 20 | 4 | 4 | 10 | 0 | 0 |
| B03_2c_80_prio | 27 | 6 | 5 | 16 | 0 | 0 |
| B04_2c_80_noprio | **45** | 4 | 6 | **32** | 0 | 0 |
| C01_1c_40_shelf | 20 | 3 | 5 | 10 | 0 | 0 |
| C02_2c_55_shelfprio | 36 | 9 | 0 | 22 | 0 | 0 |
| C03_2c_80_prio | 27 | 6 | 0 | 18 | 0 | 0 |
| D01_A_1c_40_softheavy | 21 | 2 | 16 | 13 | 0 | 0 |
| D02_A_1c_40_prioheavy_nocont | 21 | 8 | 0 | 17 | 0 | 0 |
| D03_A_2c_60_prioheavy_cont | 40 | 19 | 0 | 30 | 0 | 0 |
| D04_A_1c_40_flat | 20 | 5 | 0 | 12 | 0 | 0 |
| D05_A_1c_40_tall | 12 | 0 | 0 | 8 | 0 | 0 |
| P01_A_1c_pre6 | 19 | 2 | 1 | 10 | 0 | 0 |
| P02_A_1c_pre10 | 31 | 1 | 4 | 24 | 0 | 0 |
| P03_A_2c_pre8_prio | 38 | 5 | 0 | 17 | 0 | 0 |
| P04_B_1c_pre8_shelf | 23 | 3 | 7 | 11 | 0 | 0 |
| P05_C_2c_pre8_shelfprio | 36 | 9 | 2 | 12 | 0 | 0 |
| P06_A_1c_pre12_dense | 28 | 2 | 7 | 18 | 0 | 0 |
| **合計(26シーン)** | **670** | **123** | **79** | **411** | **0** | **0** |

**結論(区別)**: 「対象荷物が0個だから満点」ではなく、**「対象は多数存在し(優先手荷物123個・
ソフト貨物79個)、積み重なり自体も広範囲で発生している(411ペア)が、違反が真に0件
だから満点」**である。処方はこちらの区分(検出ロジック or シーン難易度の精査)。

### (2-2) `_find_stacking_pairs` の判定ロジック

```python
def _find_stacking_pairs(self, containers):
    self.client.performCollisionDetection()
    for container in containers:
        for i, j in 全ペア(i<j):
            contacts = self.client.getContactPoints(bodyA=item_a.pybullet_id, bodyB=item_b.pybullet_id)
            if not contacts:
                continue
            # contactNormalOnB(index7)のZ成分の絶対値が0.7超の接触のみ「積み重なり」
            if not any(abs(c[7][2]) > 0.7 for c in contacts):
                continue
            # z座標が低い方をbottom、高い方をtopとして (bottom, top) を記録
            ...
```

検出条件:
- **接触判定**: `performCollisionDetection()` 直後の `getContactPoints` が返す実接触点のみ
  (pybulletのナローフェーズ、既定のcontact margin依存)。
- **上方向の定義**: 接触法線(`contactNormalOnB`)のZ成分の絶対値が `0.7` 超
  (鉛直から約45.6°以内)であること。水平方向の接触(横並び)は除外される。
- **許容誤差**: 明示的な距離閾値は無い(pybulletの接触判定に完全依存。既定の
  `contactBreakingThreshold` 相当のマージン)。

呼び出しタイミングは `Scorer.evaluate()` 内で `placement_score`/`soft_item_score` が
`stability_score`(破壊的な揺らし計算)より**前**に呼ばれており、配置直後の状態で
判定している(揺らし後の乱れた状態を見ているわけではない)。

### (2-2追記) 実データでの突き合わせ(検出漏れの有無)

**最も荷物が密なシーン**(`suite_B04_2c_80_noprio.json::000`、45個配置、
`_find_stacking_pairs` 検出32ペア)を選び、pybulletの接触判定とは完全に独立な
**幾何的クロスチェック**を行った: 各荷物の `pos`/`orn`/寸法から回転後の8頂点を計算し
world AABB を求め、同一コンテナ内の全ペアについて (a) XY足跡が重なるか、
(b) 下側候補の天面zと上側候補の底面zの隙間(gap)、を独立に算出した。

- XY重なりありかつ上下関係が矛盾しない幾何的候補: **79ペア**(pybullet検出32ペアより多いのは、
  側面接触や非常にわずかな重なりも候補に数える緩い条件のため)。
- **gap(隙間)が実質ゼロ(-0.8mm〜0.0mm、pybulletの静止接触でよく見る微小めり込み量)の
  「本当に接触している」ペアは20件**。うち **底面が優先手荷物・ソフト貨物であるものは
  1件も無い**(優先手荷物は逆にitem52/item10/item36/item31のように**top側**として
  複数回登場しており、非優先荷物の上に安全に置かれている)。
- XY重なりのみを条件にした(gap不問の)「優先/ソフトが下敷きになりうる」候補は8件
  見つかったが、**全てgap=77.9mm〜1126.4mmと大きく離れており、物理的に接触していない**
  (同じXY列の別高さに他の荷物やギャップが存在するだけの、幾何的偶然の重なり)。

**結論**: pybullet接触判定(32ペア検出)と幾何的クロスチェック(20件の真の接触ペア)は
整合しており、いずれの方法でも**優先手荷物・ソフト貨物が実際に下敷きになっている
実例は1件も見つからなかった**。したがって**このシーンにおいて `_find_stacking_pairs`
に検出漏れは無い**——「ロジックを読んだだけの妥当に見える判断」ではなく、密シーン1件を
実データで突き合わせた上での結論である。

### 見立て: 検出漏れではなく、方式の違いか実行条件の違い

26シーン全件で(a)対象荷物は十分に存在し、(b)積み重なりも広範囲に発生し、
(c)`_find_stacking_pairs`はそれを正しく検出しているが、(d)違反が一貫して0件——
これは`_find_stacking_pairs`自体の検出漏れではなく、**ローカルのplanner.pyが
「優先手荷物は下段/優先コンテナへ、ソフトは上段/上に非ソフトを載せない」という
設計方針(CLAUDE_CODE_指示書.md §5)どおりに一貫して違反を回避できている**ことを
示している。

本番の42%/53%という高い違反率との乖離は、以下のいずれか(または両方)が濃厚:

1. **ローカル評価の実行条件が本番と異なる**: ローカル計測は`MYSOLVER_HARD_WALL_LIMIT`を
   常に非拘束にしている(docs/migration_to_mac.md §3)一方、本番は`optimization`が
   Phase42提出で155.46→165.109とHARD_WALL_LIMIT付近まで来ている
   (=**壁時計に追われている**)。ローカルの「余裕のある」実行では計画的に優先/ソフトの
   下敷きを避けられていても、本番の時間圧力下では配置判断が粗くなり、結果的に
   下敷きが発生している可能性がある。
2. **本番の`placement_score`/`soft_item_score`の判定基準がローカル近似(tools/scorer.py)と
   異なる**: tools/scorer.pyは README「評価指標」節に基づく近似であり
   (docstring「重みや正規化定数は本番と厳密には一致しない」)、**基準そのもの**
   (接触閾値・上方向の定義・許容誤差)が本番と食い違っている可能性を完全には排除できない。

(2-3)の指示どおり、この段階では `scorer.py` / `configs/gen` の修正は行っていない。
上記2点のどちらを次に検証すべきか、指示を仰ぐ。

---

## 変更ファイル

- `agents/mysolver/ordering.py`(`REPLICA_SELECT` 既定値 `'1'`→`'0'`)
- `results/phase44_report.md`(本ファイル)

診断スクリプト(`phase44_stacking_diag.py`/`phase44_geo_crosscheck.py`)はセッションの
スクラッチパッドに置かれており、副作用のない読み取り専用の一回限りの調査用のため
リポジトリ化していない(必要であれば`tools/`へ移す)。
