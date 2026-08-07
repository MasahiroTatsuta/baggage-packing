# Phase 19 報告：候補生成の高速化(EP増分更新 + 到達不能候補の早期枝刈り)

作成日: 2026-08-07
対象コミット: (計測完了後に確定)
変更ファイル: `agents/mysolver/planner.py`(ターゲット1)、`agents/mysolver/geometry.py`,
`agents/mysolver/simulate.py`(ターゲット2)、`agents/mysolver/ordering.py`(ターゲット3)、
`tools/phase19_*`(新規診断スクリプト、コミット対象外の一時ツールは省く)

---

## 0. エグゼクティブ・サマリ

(T2/T3完了後に確定させる。以下は現時点の暫定サマリ。)

- **【ターゲット1・実装/採用】** `_candidate_xy` 直後・着地面評価より前に、「どの着地高さ
  (sweep_z)を選んでも y方向搬入スイープが必ず失敗する」候補を厳密に判定して間引く
  (`planner._y_sweep_unreachable_mask`)。sweep_z は候補の (x,y) に依存しない決定的な範囲
  `[SWEEP_Z_MIN, SWEEP_Z_MAX]` に必ず収まることを利用し、その範囲全体が既存障害物の
  z遮断区間(`_apply_obstacle_filters` の phase1 と同一の margin_xy/margin_z)で覆われて
  いる候補だけを削る、近似を含まない厳密な必要条件判定である。**B01-B04・P04の5シーン全部で
  digest完全一致**を確認し(§1.2)、24000件のランダム合成ケースでの直接比較でも不一致0件
  (§1.1)。同一出力の B01-B04/P04 ワークロードで **plan()時間 -21.2%(84.64s→66.72s)**、
  `_evaluate_candidates` の実処理時間 **-58.6%**、候補XY数(n_xy)中央値 **-73%** を確認
  (§1.3)。`gen_shelf_patternA` の行き詰まり診断では fail_transport_y が 272件(59.6%)→
  2件(0.4%)に激減し、出力(配置数・行き詰まり点)は完全に不変であることも確認した(§1.4)。

---

## 1. ターゲット1: 到達不能候補の早期枝刈り

### 1.1 実装(`agents/mysolver/planner.py`)

`_evaluate_candidates` の legal1(y方向搬入経路の掃引衝突判定、`_apply_obstacle_filters` の
phase1)は候補の着地高さ `sweep_z`(landing_top から決まる)に依存するため、一見「着地面評価
より前」には判定できないように見える。しかし以下が成り立つ:

```
sweep_z = min(ceiling_sweep, world_z + effective_start)
world_z = landing_top + half[2] + REST_CLEARANCE   (landing_top は常に thickness 以上)
effective_start >= 0                                (常にクリップされ負にならない)
ceiling_sweep = height + buffer - thickness - half[2] - START_MARGIN
```

このため、候補の (x,y) に一切依存しない決定的な範囲

```
SWEEP_Z_MIN = thickness + half[2] + REST_CLEARANCE   (床置きが is_resting になるため達成可能な真の下限)
SWEEP_Z_MAX = ceiling_sweep                            (min() が必ず頭打ちにする上限)
```

に、実際の sweep_z は**必ず**収まる(`planner._y_sweep_range`)。したがって、候補の
XYスイープ足跡(transport_x_bounds でクリップした x 固定列 × y_entry〜local_y の範囲)が、
`[SWEEP_Z_MIN, SWEEP_Z_MAX]` の**全域**を、XY方向で重なる既存障害物の z遮断区間
(`box_overlap_batch` と同一の `margin_xy=SAFETY_MARGIN_XY`・`margin_z=SWEEP_Z_MARGIN`)で
切れ目なく覆われている場合、実際に選ばれる着地高さが範囲内のどこであっても legal1=False が
確定する。これは近似ではなく厳密な必要条件の判定であり、後段の判定結果を一切変えない
(`planner._y_sweep_unreachable_mask`)。

計算量: 素朴に「候補ごとに障害物の合併区間を計算」すると `_extreme_points` の生成コストを
上回りかねない(O(候補数 × 障害物数 × セグメント数)の行列積になる)。障害物を z遮断区間の
下端でソートし、**候補ごとに同じ処理順序を共有**しながら「区間の合併が範囲を覆うか」を
古典的な線分被覆走査でベクトル化することで、O(候補数 × 障害物数) に抑えている(障害物ごとに
1回、全候補に対する numpy 演算を行う、既存の `_apply_obstacle_filters` と同じ形のループ)。

`_search_best` の候補キャッシュ構築(`candidate_cache[cache_key] = (half, full_xy)`)の
直後にこの枝刈りを適用し、以降の y_slice 絞り込み・`_evaluate_candidates` は縮小済み配列に
対して行われる。

### 1.2 正当性検証

**(a) 合成ランダムケース(`/tmp/.../test_prune.py`、コミット対象外の一時スクリプト)**:
乱数のコンテナ・障害物・半寸法で400試行 × 候補60件 = 24000件について、`_y_sweep_unreachable_mask`
が True(不可能と判定)を返した候補について、sweep_zの到達可能範囲を61点でサンプリングし
実際に legal1 が一度でも真になるケースが無いかを全数チェックした。**不一致0件**。
(合成ケースでの枝刈り率は平均6.3%、実シーンより低いのは障害物密度が低いランダム配置のため。)

**(b) `tools/phase17_dump.py` によるエピソード全体のdigest一致**: B01-B04・P04(決定的な
optimize=False シーン)で、順序・毎ステップaction・最終配置のsha256 digestを比較。

| シーン | digest判定 |
|---|---|
| suite_B01_1c_40_plain | IDENTICAL |
| suite_B02_1c_40_shelf | IDENTICAL |
| suite_B03_2c_80_prio | IDENTICAL |
| suite_B04_2c_80_noprio | IDENTICAL |
| suite_P04_B_1c_pre8_shelf | IDENTICAL |

**5/5 シーンで完全一致。**

### 1.3 KPI測定(`tools/phase17_probe.py`、B01-B04・P04・digest一致ワークロード)

optimize=Falseのこの5シーンは build_order を通らず、online policy() の呼び出し列も
digest一致で保証されているため、「同一の呼び出し列に対して枝刈りがどれだけ軽くしたか」を
交絡なく直接測れる(offline optimize が絡む6シーン計測は探索深度が予算適応的に変わるため
参考値に留める。§1.5)。

| 指標 | before | after | 差 |
|---|---|---|---|
| plan()時間合計 | 84.64s | 66.72s | **-21.2%** |
| `_evaluate_candidates` 処理時間合計 | 44.16s | 18.28s | **-58.6%** |
| `_candidate_xy` 処理時間合計 | 39.39s | 40.57s | +3.0%(コード自体は不変、呼び出し回数+1.3%分) |
| `_evaluate_candidates` 呼び出し回数 | 19414 | 19755 | +1.8%(同一出力、リトライパスの僅かな差) |
| 候補XY数(n_xy) 中央値 | 1070 | 290 | **-73%** |
| 候補XY数(n_xy) p90 | 2902 | 1287 | **-56%** |
| 候補XY数(n_xy) 最大 | 11507 | 5936 | -48% |

`_evaluate_candidates` 自体の処理時間が半分以下になったことが plan() 全体の -21.2% の主因。
候補構築(`_candidate_xy`)のコード自体は変更していないため時間はほぼ不変だが、実処理時間に
占める**割合**は相対的に上昇する(eval対candの内訳比: eval 52.2%→27.4%、cand 46.5%→60.8%、
両者の和に対する比率)。これは母数(eval側)が縮んだことの裏返しであり、**残る最大のコスト
要因が候補構築(`_candidate_xy`)であることを直接裏付ける**(ターゲット2の根拠)。

### 1.4 行き詰まり診断での確認(`tools/diagnose_stall.py`、`gen_shelf_patternA`、optimize予算15s)

指示にあった fail_transport_y の支配(旧計測62.3%)を再現・検証するため、同一シーン・同一
最適化予算で行き詰まり診断を before/after で実行した。

| | before | after |
|---|---|---|
| 行き詰まり検出時の配置数 | 20個/1コンテナ | 20個(**不変**) |
| 残り未配置(順序を変えれば置けた) | 2個 | 2個(**不変**) |
| 残り未配置(真の空間的行き詰まり) | 19個 | 19個(**不変**) |
| 全試行の内訳: 搬入経路Y掃引で全滅 | 272件(59.6%) | 2件(0.4%) |
| 全試行の内訳: 候補位置が1つも生成されない | 0件 | 352件(77.2%) |

fail_transport_y として棄却されていた候補の大半が、evaluate に到達する前の時点で「候補が
1つも残っていない(no_xy)」に置き換わっている(=枝刈りで先に消えた)。**行き詰まり検出時の
配置数・置ける/置けない荷物の内訳は完全に不変**であり、これも出力不変性の追加確認になる。

### 1.5 参考: offline optimize を含む6シーンでの計測(交絡あり、次節の伏線)

旧6シーン(`sample_config` ×2, `gen_2containers_patternB`, `gen_2containers_priority`,
`gen_shelf_patternA`, `gen_manyitems_patternA`、既定 optimize 予算120s)で同じ before/after
比較を行うと、`_evaluate_candidates` 1回あたりのコストは同様に下がる(n_xy p50 1071→900、
dt p50 1.60ms→1.11ms)一方、**plan()時間の合計は 407.7s→426.3s とむしろ増加**した
(`_evaluate_candidates` 呼び出し回数 84501→103964、+22.9%)。

これは回帰ではない。`ordering.build_order` は「秒」ではなく**固定ユニット予算**
(`CONSTRUCT_SLICE=20.0秒相当`など)でリスタートを打ち切る設計(Phase17)であり、1回の
`_evaluate_candidates` が消費するユニットは候補数 n_xy に比例する。n_xy が下がったことで
**同じユニット予算でより多くの手を先まで探索できる**ようになった結果、探索がより深く進み、
使用する実時間(壁時計)も増えた、というのが観測された増加の内実である(§3のターゲット3で
詳述)。**「同一ユニット予算に対する実時間」の較正(`UNITS_PER_SEC`)が古くなったことを示す
明確なシグナルであり、ターゲット3でこれを測り直す。**

---

## 2. ターゲット2: 候補構築の増分キャッシュ(Extreme Point法の増分更新)

### 2.1 方針: 「グリッド縮小+EMSで密度回復」ではなく、数学的に完全に同一の増分キャッシュ

指示にあった Crainic, Perboli & Tadei (2008) の Extreme Point 則(荷物を1つ置くたびに
その射影から新しいEPを増分生成し、無効化されたEPを除去する)を、本実装のEP表現
(障害物の角を、配置しようとしている荷物自身の半寸法 `half` だけオフセットした**中心座標**
として直接列挙する方式。古典的な「最小コーナー座標」表現ではなく、荷物サイズごとに
オフセットが変わる)にそのまま適用しようとすると、EPアンカー自体が荷物サイズに依存するため
「1つの汎用EP集合を全荷物形状で共有する」ことができない。

一方で §1.5 の計測から、`_candidate_xy` の実処理時間の大半は **EPの列挙そのものではなく
グリッド生成(`np.meshgrid`+丸め+`set()`化、31×23×density^2 ≈ 2852点、density=2)** と、
**既配置荷物ごとの障害物AABB算出(`geo.item_world_aabb`、pybulletのクォータニオン→行列変換
`quat_abs_rotmat` を含む)** で占められていることが分かった。いずれも「荷物・障害物には
依存しない/前回の呼び出しから増分しか変わらない」計算を**呼び出しのたびにフルスキャンで
再計算している**、という共通の無駄がある。この無駄を消すことは「候補集合をどう作るか」を
変えずに、Crainic et al. と同じ発想(前回の集合+差分で構築し、キャッシュを自然に効かせる)を
実現できる。**候補は既存コードと1点残らず数学的に同一になる**ため、グリッド縮小やEMS角点の
追加による密度回復は不要と判断した(指示にあった「fillが落ちるリスク」への対策そのものが
不要になる設計)。

### 2.2 実装

**(a) グリッド点集合の値ベースキャッシュ(`planner._grid_point_frozenset`)**: `_grid_xy` が
毎回組み立てていた「meshgrid生成→丸め→`set()`化」は `(length, width, density)` だけで
決まる純粋関数(荷物・障害物に一切依存しない)。`functools.lru_cache` で値ベースにメモ化し、
`_candidate_xy` は都度の `set()` 再構築ではなくキャッシュ済み frozenset との `|` 演算に置き換えた。
値ベース(dict identityではなく寸法の値でキー化)のため、online(agent.py が毎ステップ
container dict を再構築する)でもコンテナ寸法が同じであれば安全にヒットする。

**(b) 既配置荷物AABBのメモ化(`geo.item_world_aabb`)**: pos/orn は配置時に一度だけ設定され
以降変更されない(grep で確認済み: 再代入箇所は simulate.py の配置処理1箇所のみ)ことを
利用し、計算結果を item dict に記憶化する。pybulletのクォータニオン行列変換
(`quat_abs_rotmat`)を、同一の既配置荷物に対して毎ステップ再実行していたコストを避ける。

**(c) 障害物・着地面リストの増分キャッシュ(`geo.packed_obstacles`, `planner._landing_supports`)**:
container dict に「前回どこまで処理したか(`packed_items` の長さ)」を記録する増分キャッシュを
持たせ、新規に追加された荷物の分だけAABBを計算して末尾に追加する。offline探索
(`simulate.py`)は `clone_containers` で1回だけ複製した後は `packed_items.append()` で単調に
成長させるだけ(リストを丸ごと差し替える経路は無いことを確認済み)なので、
「同じリストオブジェクト(`is`)かつ長さが単調増加」という条件で安全に増分できる。
online(agent.py)は毎ステップ observation から container dict を丸ごと再構築するため
リストオブジェクトの identity が一致せず、この条件チェックは必ず不成立となって安全に
フルスキャンへフォールバックする(誤ったキャッシュヒットは構造的に起こり得ない)。

いずれも(a)(b)(c)とも**近似を一切含まない厳密なキャッシュ**であり、候補集合・スコア・
`_eval_units`/`CANDIDATE_BUILD_COST` が参照する候補数/障害物数などのカウントは一切変えない
(ターゲット1と異なり、探索の「ユニット予算」消費のされ方にも影響しない設計)。

### 2.3 検証: digest完全一致(旧6シーン全部、optimize有効シーンも含む)

`tools/phase17_dump.py` で、ターゲット1のみのコードとターゲット1+2のコードを比較した。

| シーン | optimize | digest判定(T1のみ vs T1+T2) |
|---|---|---|
| suite_B01_1c_40_plain | 無効 | IDENTICAL |
| suite_B02_1c_40_shelf | 無効 | IDENTICAL |
| suite_B03_2c_80_prio | 無効 | IDENTICAL |
| suite_B04_2c_80_noprio | 無効 | IDENTICAL |
| suite_P04_B_1c_pre8_shelf | 無効 | IDENTICAL |
| sample_config::000 | 有効 | IDENTICAL |
| sample_config::001 | 有効 | IDENTICAL |
| gen_2containers_patternB::001 | 有効 | IDENTICAL |
| gen_2containers_priority::000 | 有効 | IDENTICAL |
| gen_shelf_patternA::000 | 有効 | IDENTICAL |
| gen_manyitems_patternA::000 | 有効 | IDENTICAL |

**11/11 シーンで完全一致**。§1.5 で述べた通り、ターゲット1は候補数(n_xy)を減らすことで
`_eval_units` が下がり、`build_order` の固定ユニット予算内でより多くの手を探索できるように
なる(=optimize有効シーンでは出力が変わりうる)副作用があったが、**ターゲット2はいかなる
カウントも変えないため、build_orderを通るシーンを含め出力が完全不変**であることが直接
確認できた(`_evaluate_candidates`/`_candidate_xy` の呼び出し回数も前後で1回も違わないことを
§2.4で確認)。

### 2.4 KPI測定(`tools/phase17_probe.py`)

**(a) 清浄なワークロード(B01-B04・P04、optimize無効、呼び出し列がdigestで保証された同一列)**:

| 指標 | 変更前(Phase18時点) | T1のみ | T1+T2 |
|---|---|---|---|
| plan()時間合計 | 84.64s | 66.72s (-21.2%) | **60.08s (-29.0%)** |
| `_candidate_xy`処理時間合計 | 39.39s | 40.57s | **31.42s (T1比-22.6%)** |
| `_evaluate_candidates`呼び出し回数 | 19414 | 19755 | 19755(**T1と1回も違わず一致**) |
| `_candidate_xy`呼び出し回数 | 12832 | 12996 | 12996(**T1と1回も違わず一致**) |

呼び出し回数が T1のみ と T1+T2 で完全に一致していることは、§2.3 の digest 一致と整合する
(全く同じ手順・同じ回数の呼び出しを、より軽い実装で行っているだけ)。

**(b) 旧6シーン(optimize有効シーン込み、探索深度が予算適応的に変わりうる参考値)**:

| 指標 | T1のみ | T1+T2 |
|---|---|---|
| plan()時間合計 | 426.27s | **403.38s (-5.4%)** |
| `_candidate_xy`処理時間合計 | 216.96s | **172.14s (-20.7%)** |
| `_evaluate_candidates`呼び出し回数 | 103964 | 103964(**完全一致**) |
| `_candidate_xy`呼び出し回数 | 78164 | 78164(**完全一致**) |

こちらも呼び出し回数が完全一致しており、§2.3のdigest一致(build_orderを通るシーンも含め
出力不変)と矛盾しない。plan()時間の減少幅が(a)より小さいのは、offline探索
(`ordering.build_order`)の全体時間が主に「固定のnominal秒予算をどこまで使うか」という
ordering.py側の別要因(§3で扱う)に律速されているためで、`_candidate_xy` 自体の処理時間の
削減幅(-20.7%)は(a)と同水準で一貫している。

## 3. ターゲット3: UNITS_PER_SEC再較正+26シーン最終確認

### 3.1 再較正の必要性

§1.5 で見た通り、ターゲット1は候補数(n_xy)を減らすことで評価1回あたりの名目ユニット数
(`_eval_units`)も下げる。`ordering.build_order` は壁時計ではなく**固定ユニット予算**で
リスタートを打ち切るため、1回あたりのユニットが下がると同じ予算内でより多くの評価回数を
消化できるようになり、offline探索(旧6シーンの計測で `_evaluate_candidates` 呼び出し回数
84501→103964、+22.9%)は**むしろ実時間を長く使う**方向に動いた(§1.5)。これは
`UNITS_PER_SEC`(名目秒→ユニットへの較正定数)が、ターゲット1・2後の実コストに対して
古くなった(=同じユニット数を消化するのに要する実時間の想定が変わった)ことを意味する。
再較正しないままだと、名目秒(例: `DEFAULT_TIME_BUDGET=120`)に対する実所要時間が想定より
延び、非常用安全弁(壁時計、`HARD_WALL_LIMIT=165s`)に頼る頻度が増えて決定性を損なう
リスクが上がる。

### 3.2 再較正の実施

Phase17 §2.1 と同一の手法(`tools/phase17_probe.py`、旧6シーン、`COST_CONST=8` 固定)で
測り直した。

| 定数 | Phase17時点 | Phase19(ターゲット1+2適用後) |
|---|---|---|
| `CANDIDATE_BUILD_COST`(評価side rate ÷ 候補構築side生rate) | 1.63e7 ÷ 8.94e5 = **18.2** | 1.211e7 ÷ 1.401e6 = **8.64** |
| `UNITS_PER_SEC`(両者を costed unitsで揃えた後の集計 units/s) | 集計1.60e7(シーン別1.31e7〜2.01e7) | 集計1.088e7(シーン別9.93e6〜1.145e7) |

Phase17と同じ「集計値よりわずかに小さい値を採用する」方針で、`CANDIDATE_BUILD_COST=8.64`,
`UNITS_PER_SEC=1.05e7` を採用した(`agents/mysolver/planner.py`)。

`CANDIDATE_BUILD_COST` が 18.2→8.64 に下がったのはターゲット2で候補構築の**生コスト**
(grid/AABBキャッシュにより実処理時間)が下がったことの直接的な反映。`UNITS_PER_SEC` が
1.60e7→1.088e7 に下がっているのは一見「較正が悪化した」ように見えるが実際は逆で、
ターゲット1により評価1回あたりの**名目ユニット数**も下がった(n_xyが縮んだため)ため、
「名目ユニット数の低下」と「実処理速度の向上」が両方効いた結果の比率であり、**この2定数を
セットで下げることで、名目秒(例: 120s)に対応する実所要時間をPhase17と同水準に保つ**、
という較正の意味になる。

### 3.3 決定性への影響確認

再較正後も B01-B04・P04(online policy() のみ、offline探索を経由しない5シーン)で
`tools/phase17_dump.py` の digest が変更前(Phase18終了時点)と**完全一致**することを確認した
(§1.2の表と同一の5シーン、5/5 IDENTICAL)。`UNITS_PER_SEC` を下げると `POLICY_TIME_BUDGET`
(5.5秒相当)から得られるユニット予算も下がるが、これらのシーンでは元々予算が律速していな
かった(=探索が全候補を使い切って自然終了していた)ため、影響が出なかった。

### 3.4 26シーンスイートでの最終確認

(計測中)

## 4. 制約確認

(T2/T3完了後に追記)
