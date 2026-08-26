# Phase67 報告: 支持閾値スイープ(2G揺らし、threshold_onlyシーンに絞った再検証)

## 結論(先出し)

**明確な採用基準クリアには至らなかった。緩1(0.45/0.5/0.20)が主KPI(全28シーンの
composite_B)でt=2.056と境界的に基準を上回ったが、悪化シーンが8/28(28.6%)あり、
「悪化シーン少数」の基準に照らすと採用可否は判断が割れる水準だった。** 緩2
(0.35/0.4/0.25)は明確に基準未達(stability悪化、悪化シーン13/28)、strict_support
無効化はほぼ無効果(全指標が誤差範囲)。

**重要な所見**: 効果は狙い通り `threshold_only` シーン(A02/P01/P02/sample_config×2、
forbidden_hitが一切絡まない5シーン)に集中していた——緩1ではこの5シーンで
composite_B t=+2.369・num_placed_items t=+2.331 と明確に改善。一方
`forbidden_hit` シーン(B03/C03/D02/P03/P04)は緩めても有意な改善なし(Phase66の
構造的予測どおり)。残り18シーン(`rest`)は平均でほぼゼロ(緩1: composite_B平均
+0.127)だが、個別には±5点級の振れ(カオス的な再配置、Phase60 §結論と同型)があり、
これが「悪化シーン」件数を押し上げている。

判断は次節の通り指示を仰ぐ。緩2・strict無効化については不採用が妥当と考える。

---

## ステップ0: 2G条件での対照

`MYSOLVER_DIAG_SHAKE_AMPLITUDE=19.6`(2G、公式開示レンジ1〜3Gの中央。**本番値への
適合を狙った選定ではない**)で26シーン+sample_config(2タスク、計28シーン)を
現行設定のまま測定し、`results/phase67_baseline_2G.json` /
`results/phase67_baseline_2G_sample.json` として保存した。

### 28シーン平均(現行設定・2G)

| 指標 | 全28シーン | threshold_only(n=5) | forbidden_hit(n=5) |
|---|---:|---:|---:|
| fill_score | 24.42 | 23.01 | 24.76 |
| cog_score | 64.15 | 64.76 | 67.63 |
| stability_score | 85.47 | 86.55 | 85.99 |
| composite_A(定義A) | 67.61 | 67.57 | 68.57 |
| composite_B(定義B) | 50.73 | 48.46 | 50.24 |
| num_placed_items | 25.64 | 23.00 | 27.20 |

**stabilityは0.6G相当(現行既定amplitude=6.0)から大きく低下している。** Phase61が
同一ロジック(`tools/diagnose_stability.py`、本フェーズはこれをそのまま呼び出している)
で測定した際の値は「amplitude=6.0で26シーン97.4〜98.8に密集」であり、今回の2G平均
85.47(28シーン、うち26シーンのみでも同水準)と比べて**約12〜13ポイントの低下**。
これはPhase61で既に報告済みの傾向(1G→2G→3Gで分散が拡大)の追認であり、
`results/phase40_baseline_off_mac.json`(0.6G測定、fill中心の別ファイル)とは
比較していない——**禁止事項の通り、本番値に近い振幅を探す目的ではなく、
「公式開示レンジの中央」という理由でamplitude=19.6を選んだのみである。**

同一ツール(`tools/diagnose_stability.py::stability_with_item_detail`)を呼び出して
いるため、Phase61の2G実測値(26シーン平均85.46)と今回の28シーン平均85.47が
ほぼ完全に一致しており、**測定方法の一貫性を確認できた**(新規ツール
`tools/phase67_suite_metrics.py`の実装が正しいことの傍証)。

---

## ステップ1: 支持閾値のスイープ

### (1-1) env化・8/8確認

`agents/mysolver/planner.py`に以下を追加した(Phase66で一度実装して差分から
除去したものの復活、既定値は変更なし):

```python
MIN_UNION_SUPPORT_RATIO = float(os.environ.get('MYSOLVER_MIN_UNION_SUPPORT_RATIO', '0.55'))
MIN_SUPPORT_SPAN_RATIO = float(os.environ.get('MYSOLVER_MIN_SUPPORT_SPAN_RATIO', '0.6'))
MAX_SUPPORT_CENTROID_OFFSET = float(os.environ.get('MYSOLVER_MAX_SUPPORT_CENTROID_OFFSET', '0.15'))
```

および `plan()` 冒頭に、strict_support(Phase13導入、0.75)を丸ごと無効化する診断
フックを追加(既定`'0'`で無効、`agent.py`が渡す値をそのまま使うため既定挙動は不変):

```python
if os.environ.get('MYSOLVER_STRICT_SUPPORT_DISABLE', '0') == '1':
    strict_support = False
```

```
$ bash scripts/bp_check.sh   # 全env未設定=既定値
[B01] ... OK(基準値と一致)
[B02] ... OK(基準値と一致)
[B03] ... OK(基準値と一致)
[B04] ... OK(基準値と一致)
[P04] ... OK(基準値と一致)
[A01] ... OK(基準値と一致)
[A02] ... OK(基準値と一致)
[A03] ... OK(基準値と一致)
```

**8/8ビット単位不変を確認。**

### (1-2)(1-3) 4水準 × 2G × 28シーン全件

| 水準 | MIN_UNION_SUPPORT_RATIO | MIN_SUPPORT_SPAN_RATIO | MAX_SUPPORT_CENTROID_OFFSET | strict_support |
|---|---:|---:|---:|---|
| 対照(現行) | 0.55 | 0.6 | 0.15 | 有効(Phase13どおり) |
| 緩1 | 0.45 | 0.5 | 0.20 | 有効 |
| 緩2 | 0.35 | 0.4 | 0.25 | 有効 |
| strict無効化 | 0.55(既定) | 0.6(既定) | 0.15(既定) | **無効化**(常にFalse) |

いずれも2G(amplitude=19.6)・26シーン+sample_config全件(計28シーン)を実測
(`results/phase67_{loose1,loose2,strictoff}_2G[_sample].json`)。
`tools/phase67_analyze.py`でbaseline(対照)との対応のある差分(paired diff)を
シーン単位で算出し、mean/σ/SE/tを求めた
(`results/phase67_analysis_{loose1,loose2,strictoff}.json`)。

#### 全28シーンの差分(vs 対照)

| 水準 | Δfill | Δstability | Δcomposite_A | Δcomposite_B | Δnum_placed_items |
|---|---:|---:|---:|---:|---:|
| 緩1 | +1.19 (t=1.10) | +0.01 (t=0.02) | +0.12 (t=0.50) | **+2.28 (t=2.06)** | −0.18 (t=−0.23) |
| 緩2 | +2.51 (t=2.45) | −2.87 (t=−1.93) | −0.23 (t=−0.58) | +0.71 (t=0.71) | +0.14 (t=0.17) |
| strict無効化 | −0.22 (t=−0.45) | +0.36 (t=0.61) | +0.05 (t=0.30) | +0.03 (t=0.14) | −0.18 (t=−0.46) |

#### threshold_only(A02/P01/P02/sample×2、n=5)の差分 —— 狙った対象への効果

| 水準 | Δfill | Δstability | Δcomposite_A | Δcomposite_B | Δnum_placed_items |
|---|---:|---:|---:|---:|---:|
| 緩1 | +3.83 (t=1.29) | +0.35 (t=0.25) | +0.77 (t=1.00) | **+7.98 (t=2.37)** | **+2.40 (t=2.33)** |
| 緩2 | +6.17 (t=1.87) | −0.48 (t=−0.26) | +1.03 (t=1.16) | +3.41 (t=0.92) | +0.80 (t=0.36) |
| strict無効化 | +0.03 | +0.28 | +0.13 | +0.39 | +0.00 |

**緩1は狙った5シーン(threshold_only)でcomposite_B・num_placed_itemsともt>2を
明確にクリア。** 緩2はfillが伸びる一方でstabilityが相殺し、composite_Bはt=0.92に
留まる(緩めすぎ)。strict無効化はこの群でもほぼ無効果(この5シーンは
optimize=Trueのパターンで、そもそもstrict_supportの対象外のため当然の結果)。

#### forbidden_hit(B03/C03/D02/P03/P04、n=5)の差分 —— 構造的に緩めても救えない群

| 水準 | Δfill | Δstability | Δcomposite_A | Δcomposite_B | Δnum_placed_items |
|---|---:|---:|---:|---:|---:|
| 緩1 | +0.52 (t=0.70) | −0.64 (t=−0.92) | −0.32 (t=−0.96) | +4.31 (t=1.22) | +1.20 (t=1.63) |
| 緩2 | +4.22 (t=2.74) | −1.60 (t=−1.62) | +0.38 (t=1.01) | +2.37 (t=2.43) | +2.20 (t=2.06) |
| strict無効化 | +0.16 | −0.69 | −0.17 | −0.17 | +0.00 |

forbidden_hitシーンでも一部有意な動きが見えるが(緩2でnum_placed_items/composite_Bが
t>2)、これはPhase66で確定した式構造(`forbidden_hit=True`なら`support_ok`は
恒久的にFalse)からは**直接には説明できない**——閾値を緩めたことで探索が別の
配置列へ分岐し、その副次効果として同じ荷物数がforbidden面に触れずに置けた、と
考えるのが妥当(下敷き禁止そのものを回避したわけではない)。この群を主目的の
根拠にはしない。

#### 残り18シーン(rest)—— 悪化していないかの確認

| 水準 | Δcomposite_B(mean, t) | 悪化シーン数(composite_B, -0.01超) |
|---|---|---:|
| 緩1 | +0.13 (t=0.16) | 8/18(rest由来。全体の悪化8件は全てrest群) |
| 緩2 | −0.50 (t=−0.45) | 13/18 |
| strict無効化 | −0.01 (t=−0.03) | 3/18 |

**緩1の悪化シーン8件は全て`rest`群(threshold_only・forbidden_hitのどちらでもない
シーン)に集中している。** 平均は小さい(+0.13)が、個別には−5.5〜−0.1ptの幅があり
プラス方向の振れと相殺して平均が小さくなっている(下表参照)。

### 悪化シーン(composite_B、-0.01超の悪化)

**緩1(8件、全てrest群)**:

| シーン | Δcomposite_B |
|---|---:|
| suite_D03_A_2c_60_prioheavy_cont | −5.52 |
| suite_A01_1c_40_plain | −5.00 |
| suite_A06_1c_40_small | −2.76 |
| suite_D01_A_1c_40_softheavy | −2.22 |
| suite_D05_A_1c_40_tall | −1.12 |
| suite_A07_1c_40_bulky | −0.68 |
| suite_B04_2c_80_noprio | −0.23 |
| suite_P06_A_1c_pre12_dense | −0.11 |

**緩2(13件)**: suite_C01(−10.37), suite_B01(−8.45), suite_D03(−5.03),
suite_P02(−4.98、**threshold_onlyシーンが緩2では逆に悪化**), suite_A04(−3.31),
suite_P06(−1.80), suite_D04(−1.55), sample_config::000(−1.30), suite_D05(−1.12),
suite_C02(−0.48), suite_A01(−0.45), suite_A03(−0.27), suite_A06(−0.16)。

**strict無効化(4件、いずれも僅少)**: suite_B04(−4.10)、suite_P04(−0.71)、
suite_B01(−0.47)、suite_B03(−0.14)。

これらの悪化は、Phase60(SAFETY_MARGIN_XYスイープ)の結論
「マージンを変えるとどの候補がぎりぎり合法/違法になるかが変わり、後続の探索状態が
連鎖的に変わる(カオス的な再配列)」と同型の現象と考えられる——支持しきい値を
緩めると、それまで「不合格」だった候補が新たに合格になり、探索の優先順位
(スコア比較)が変わることで、閾値とは無関係な後続の配置列全体が変わりうる。

### (1-3) strict_support無効化 —— 全指標が誤差範囲

真の採点式換算でのPhase13の価値(net −0.65、指示書記載どおり)は0.6Gでの数字
だったが、**2Gで測り直しても無効化の効果はほぼゼロ**(全28シーンでcomposite_B
mean+0.03・t=0.14、num_placed_items mean−0.18・t=−0.46)。B01-B04・P04のみが
strict_support対象だが、B04で−4.10ptの悪化が出た以外は軒並み無風。**Phase13の
判断(B01のstability回収)を2Gでも覆す根拠は得られなかった。**

### fill/stabilityのトレードオフ比(全28シーン平均)

| 水準 | Δfill | Δstability | net=(2Δfill+1.5Δstability)/7 | 判定 |
|---|---:|---:|---:|---|
| 緩1 | +1.19 | +0.01 | **+0.343** | プラス(stabilityがほぼ無傷なので素直にプラス) |
| 緩2 | +2.51 | −2.87 | +0.103(境界: 0.75×|Δstab|=2.15 < Δfill=2.51) | 僅かにプラスだが、悪化シーン数・stability有意な悪化を踏まえると割に合わない |
| strict無効化 | −0.22 | +0.36 | +0.014 | ほぼゼロ |

---

## ステップ2: 判定

**主KPI(全28シーンのcomposite_B)で緩1がt=2.056と基準(t>2)を形式上クリアした。**
しかし「悪化シーン少数」の基準に照らすと、8/28(28.6%)というシーン数、かつ
最大−5.5ptという振れ幅は「少数」と言い切れるか判断が分かれる。効果の中身を
分解すると:

- **threshold_only(狙った5シーン)は明確にプラス**(composite_B t=2.37、
  num_placed_items t=2.33)。ここだけを見れば採用に値する。
- **悪化8件は全てrest群**(狙っていない18シーンの一部)で発生しており、
  rest群平均はほぼゼロ(+0.13)——体系的な悪化ではなくカオス的な再配置による
  シーン単位の振れと考えられるが、個別シーンで実害が出ている事実は残る。
- 緩2は fill は伸びるが stability の有意な悪化(t=−1.93)と悪化13件を伴い、
  **明確に不採用が妥当。**
- strict_support無効化は**全指標が誤差範囲でありPhase13判断を覆す根拠なし。
  不採用が妥当。**

以上により、**緩2・strict無効化は不採用と判断する。緩1については、
「対象シーンには明確な効果があるが、無関係な18シーンの一部に−5点級の副作用が
出る」という中身を踏まえたうえで、採用するか(effectを狙ったシーンだけに絞る
実装に変えるか、全シーン一律の変更として許容するか)の判断を仰ぎたい。**
提出はまだ行っていない。

---

## 生成物一覧

- `agents/mysolver/planner.py` — 支持閾値3定数のenv化・strict_support無効化フックを追加
  (Phase66で一度実装して除去したものの復活、既定値は無変更・8/8ビット単位不変確認済み)。
- `tools/phase67_suite_metrics.py`(新規) — 5成分+num_placed_items+確定採点式合成スコア
  (定義A/定義B)を1ロールアウトで測定する統合診断ツール。`tools/diagnose_stacking.py`
  (fill/placement_A/soft_A/cog)と`tools/diagnose_stability.py`(envified stability)を統合し、
  Phase59定義Bをその場で計算する。
- `tools/phase67_analyze.py`(新規) — baselineと各水準の対応のある差分(シーン単位paired
  diff)からmean/σ/SE/tを算出し、threshold_only/forbidden_hit/restで層別集計する。
- `results/phase67_baseline_2G.json` / `_sample.json` — 対照(現行設定・2G・28シーン)。
- `results/phase67_loose1_2G.json` / `_sample.json` — 緩1(0.45/0.5/0.20)。
- `results/phase67_loose2_2G.json` / `_sample.json` — 緩2(0.35/0.4/0.25)。
- `results/phase67_strictoff_2G.json` / `_sample.json` — strict_support無効化。
- `results/phase67_analysis_{loose1,loose2,strictoff}.json` — 各水準の統計サマリ。
- `results/phase67_report.md` — 本報告。

`tools/scorer.py`・既存26シーンのconfig・`.gitignore`はいずれも変更していない。
振幅・閾値とも「本番の集計スコアに合わせて選ぶ」ことはしていない(2Gは公式開示
レンジの中央、緩1/緩2はPhase11の実測範囲の外側へ機械的に2段階広げただけ)。
