# Phase66 報告: fail_support の内訳を切り分け、forbidden_hit が過半のためステップ2は中止

## 結論(先出し)

**ステップ1の判定で停止条件に該当したため、ステップ2(揺らし2Gでの再スイープ)は実施していない。**

Phase65が特定した345件の`fail_support`の内訳は:

| 内訳 | 件数 | 比率 |
|---|---:|---:|
| 閾値未達のみ(`threshold_only`) | 153 | 44.3% |
| **forbidden_hit(下敷き禁止)を伴う(`both`)** | **192** | **55.7%** |
| forbidden_hitのみ(閾値は満たしていた) | 0 | 0.0% |

**forbidden_hitが関与する件数(192件、55.7%)が過半に達した。** 指示書ステップ1-3の
判定基準通り、ステップ2は実施せず、本報告のみで停止する。

さらに、これは統計的な傾向ではなく**コードのブール式から導かれる確定的な事実**である
(§1-3参照): `support_ok = on_floor | (stacked_ok & ~forbidden_hit)` という式の構造上、
`forbidden_hit=True` の候補は `stacked_ok` の値に関わらず**恒久的に**
`support_ok=False`になる。したがって `MIN_UNION_SUPPORT_RATIO` 系をどれだけ緩めても、
この192件(55.7%)は**1件たりとも救えない**——指示書が事前に警告していた
「もし大半がforbidden_hitなら、閾値を緩めても1件も救えない」がそのまま実測で確認された。

---

## ステップ1: fail_support の内訳を切り分ける

### 1-1: 実装

`agents/mysolver/planner.py`の`_evaluate_candidates`に、既存の`fail_support`カウンタの
直下へ3行の内訳カウンタを追加した(判定ロジックは一切変更していない。`support_ok`/
`stacked_ok`/`forbidden_hit`は既存の配列そのままで、事後に`np.sum()`で内訳を数えるだけ):

```python
stats['fail_support'] = stats.get('fail_support', 0) + 1
need_support = ~on_floor
forbidden_only = need_support & forbidden_hit & stacked_ok
threshold_only = need_support & ~forbidden_hit & ~stacked_ok
both = need_support & forbidden_hit & ~stacked_ok
stats['fail_support_forbidden_only'] = stats.get('fail_support_forbidden_only', 0) + int(np.sum(forbidden_only))
stats['fail_support_threshold_only'] = stats.get('fail_support_threshold_only', 0) + int(np.sum(threshold_only))
stats['fail_support_both'] = stats.get('fail_support_both', 0) + int(np.sum(both))
```

指示通り**環境変数フックは追加していない**(統計の追加のみ)。この分岐は
`stats is not None`かつ`not np.any(support_ok)`のときにしか実行されないため、
本番のonline呼び出し(常に`stats=None`)には一切影響しない。

**決定的8シーン(B01-B04, P04, A01-A03)でビット単位不変を確認: 8/8。**
(`scripts/bp_check.sh`、変更前後とも全シーン`OK(基準値と一致)`)

### 1-2: 345件の再トレースと内訳

`tools/phase65_filter_trace.py`は`_evaluate_candidates`の`stats`辞書をそのまま
返しているため、コード変更は不要で再実行するだけで新しい内訳カウンタが得られた
(`results/phase66_fail_support_breakdown.json`、18シーン・345件)。

**全体(345件、解の実座標での判定):**

| 内訳 | 件数 | 比率 |
|---|---:|---:|
| `threshold_only`(forbidden_hit無し、閾値/span/centroidのみで不合格) | 153 | 44.3% |
| `both`(forbidden_hit あり、かつ閾値も不合格) | 192 | 55.7% |
| `forbidden_only`(forbidden_hit あり、閾値は合格していた) | 0 | 0.0% |

**plannerが実際に生成する最近傍候補点での判定でも同様(頑健性確認):**

| 内訳 | 件数 | 比率 |
|---|---:|---:|
| `threshold_only` | 141 | 40.9% |
| `both` | 204 | 59.1% |
| `forbidden_only` | 0 | 0.0% |

どちらの測り方でも forbidden_hit 関与件数(`both`+`forbidden_only`)は過半(55.7%/59.1%)。

**シーン別の内訳(解の実座標での判定、345件ベース):**

| シーン | threshold_only | both | forbidden_only |
|---|---:|---:|---:|
| suite_A01_1c_40_plain | 5 | 15 | 0 |
| suite_A02_1c_80_plain | **20** | 0 | 0 |
| suite_A03_1c_40_shelf | 10 | 10 | 0 |
| suite_A05_2c_80_prio | 5 | 15 | 0 |
| suite_B01_1c_40_plain | 2 | 18 | 0 |
| suite_B02_1c_40_shelf | 10 | 10 | 0 |
| suite_B03_2c_80_prio | 0 | **20** | 0 |
| suite_C01_1c_40_shelf | 10 | 10 | 0 |
| suite_C03_2c_80_prio | 0 | **20** | 0 |
| suite_D02_A_1c_40_prioheavy_nocont | 0 | **20** | 0 |
| suite_D04_A_1c_40_flat | 11 | 9 | 0 |
| suite_P01_A_1c_pre6 | **20** | 0 | 0 |
| suite_P02_A_1c_pre10 | **15**(全件) | 0 | 0 |
| suite_P03_A_2c_pre8_prio | 0 | **10**(全件) | 0 |
| suite_P04_B_1c_pre8_shelf | 0 | **20** | 0 |
| suite_P05_C_2c_pre8_shelfprio | 5 | 15 | 0 |
| sample_config::000 | **20** | 0 | 0 |
| sample_config::001 | **20** | 0 | 0 |

**シーンによって偏りが大きい。** 優先コンテナ・優先/ソフト荷物が絡むシーン
(B03/C03/D02/P03/P04、いずれも`container_is_prioritized`もしくは近傍に優先荷物がある
構成)は**forbidden_hitが100%**。逆に優先要素の薄いシーン(A02/P01/P02、sample_config
の両タスク)は**threshold_onlyが100%**で、forbidden_hitは1件も出現していない。
Phase65で記録していた`priority_clearance_could_apply`(305/345件でTrue)よりも
今回の`forbidden_hit`実測(192/345件)の方が狭く、かつシーン単位でよりくっきり分離しており、
「優先/ソフト貨物の下敷き禁止」が原因の候補として具体的に絞り込めた。

### 1-3: 判定 —— なぜ「両方に該当」でも threshold を緩める意味がないのか

`planner.py`の該当箇所(変更していない既存コード、`agents/mysolver/planner.py` L949-951相当):

```python
stacked_ok = (sum_ratio >= MIN_SUPPORT_RATIO) | ((sum_ratio >= union_ratio) & balanced)
support_ok = on_floor | (stacked_ok & ~forbidden_hit)
```

`forbidden_hit=True`の候補は`stacked_ok & ~forbidden_hit`が**恒等的にFalse**になる
(`stacked_ok`の値に関係なく`~forbidden_hit`がFalseになるため)。したがって
`MIN_UNION_SUPPORT_RATIO`/`MIN_SUPPORT_SPAN_RATIO`/`MAX_SUPPORT_CENTROID_OFFSET`を
どれだけ緩めて`stacked_ok`をTrueにしても、`forbidden_hit=True`である限り
`support_ok`はFalseのままである。

さらに、`forbidden_only`が0件だったこと自体もこの式の構造から説明できる:
`forbidden_hit`が立つ支持面(優先/ソフト荷物)に触れている面積は、
`_evaluate_candidates`の集計ループで`sum_area`/`sum_ratio`に**そもそも加算されない**
(`if forbidden: forbidden_hit |= at_level; continue`でスキップされる)。つまり
forbidden面への接触面積は、閾値判定の分子から丸ごと除外される。触れている支持面の
大部分がforbidden面である候補ほど、非forbidden面だけで作れる`sum_ratio`が必然的に
小さくなり、`stacked_ok`も同時に不合格になりやすい——`both`が`forbidden_only`より
多いのは偶然ではなく、この計算構造の帰結である。

**判定: forbidden_hit関与件数(192/345、55.7%)が過半に達したため、指示書
ステップ1-3の基準に従い、ステップ2(揺らし2Gでの閾値再スイープ)は実施しない。**
下敷き禁止(`forbidden_hit`)は`placement_score`/`soft_item_score`のペナルティを
防ぐハード制約であり、緩める対象ではないことも指示書の通り。

---

## ステップ2について

**実施していない。** 上記の停止条件に該当したため、指示書の指示通り測定は行わなかった。
そのため以下は測定していない・報告できない:

- 4構成(現行2G対照/緩1/緩2/strict無効化)の5成分・合成スコア(定義A/B)
- 2Gでの対照の取り直し(`phase40_baseline_off_mac.json`は0.6Gなので対照に使えないという
  指摘は正しいが、対照自体を取得する測定は行っていない)
- stabilityが2Gでどう動くか、fillとのトレードオフ実測

ステップ2向けに準備していたコード(`MIN_UNION_SUPPORT_RATIO`系のenv化、
`strict_support`無効化フック、26シーン一括計測ツール)は、実施しないと決まった時点で
**全て元に戻した**(未使用のまま`agents/mysolver/planner.py`に残す理由がないため)。
現在の`agents/mysolver/planner.py`の差分はステップ1の診断カウンタ追加のみである
(§1-1参照、8/8ビット単位不変を確認済み)。

**もしこの筋を追うなら、次の一手は「全体を緩める」ことではなく、`threshold_only`の
153件(44.3%、forbidden_hitが一切絡まないシーン=A02/P01/P02/sample_configの両タスクに
集中)に対象を絞ったスイープになるはずである。** ただしこれは新しい提案であり、
本フェーズの指示範囲(ステップ1で停止)を超えるため、実施の要否は次フェーズの指示を待つ。

---

## 生成物一覧

- `agents/mysolver/planner.py` — `_evaluate_candidates`に診断カウンタ3種を追加
  (`fail_support_forbidden_only`/`fail_support_threshold_only`/`fail_support_both`)。
  判定ロジックは無変更、既定(`stats=None`のオンライン経路)はビット単位で無変更
  (8/8で確認)。
- `results/phase66_fail_support_breakdown.json` — 18シーン・345件の内訳生データ
  (`tools/phase65_filter_trace.py`をコード変更なしで再実行したもの)。
- `results/phase66_report.md` — 本報告。

`tools/scorer.py`・既存26シーンの生成物・`.gitignore`はいずれも変更していない。
