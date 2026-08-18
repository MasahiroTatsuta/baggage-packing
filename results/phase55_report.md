# Phase55 報告: sudden death根本原因の修正 — is_includedは解消、is_validが新たな律速に

## 結論(先出し)

**`agent.py::policy()`フォールバックのz座標を修正し、Phase54で特定した
「100%決定論的にis_includedで死ぬ」バグを構造的に解消した。** V-1で確認した
とおり、修正後は**30/30件全てでis_included違反がゼロになった**(is_valid判定を
確実に満たす形にはみ出し量が反転)。**ただし同じ手数(n_placed_at_death)で
別の検査(is_valid=衝突を伴う搬送経路判定)に即座に引っかかり、エピソードの
継続手数自体は1件も伸びなかった。** 26シーンA/Bのcomposite_strict差分は
t=-0.34(有意でない、ノイズレベル)。**修正自体は正しく、既知のバグを確実に
除去した一方、今回計測した31シーンの範囲では公開スコアへの実効果は測定できなかった**
——「フォールバック発火の時点で(is_includedとは別の意味で)既に詰まっている」
ことが判明した。既定は有効(`MYSOLVER_FALLBACK_SAFE_POS=1`)のまま据え置く。
`scripts/bp_baseline_8scenes.json`の更新は不要(理由は後述、V-3で確認済み)。

---

## `inclusion_slack_batch`の符号規約(実装から引用)

```python
def inclusion_slack_batch(container, half, world_pos, floor_only=False):
    """
    world_pos: shape (N,3)。各候補について、全面のうち最も厳しい(壁に最も近い)
    dot値(validator.check_inclusion と同式)を返す。小さい(より負)ほど壁からの
    余裕が大きく、real evaluatorの厳しいinclusion_margin(-0.005程度)や配置後の
    沈降ドリフトに対して安全であることを意味する。
    """
    ...
    dots = np.einsum('nfc,fc->nf', diff, n_vecs) + bonus[None, :]
    ...
    return np.max(dots, axis=1)
```
(`agents/mysolver/geometry.py`より引用)

**戻り値は「候補ごとの、全平面中でdotsが最大(=最も外側に近い)値」であり、
`check_inclusion_batch`は`inclusion_slack_batch(...) <= margin`で合法判定する
(`dots<=margin`がvalidator.check_inclusionと同式)。値が小さい(より負)ほど
安全。** ステップ1設計の「最悪値が最小の候補を選ぶ」は符号として正しく、
実装は`int(np.argmin(slack))`で選択した(引用したdocstringの記述と一致)。

---

## 実装

`agents/mysolver/agent.py`の`if action is None:`ブロック内のみを変更(diffの要点):

```python
FALLBACK_INCLUSION_MARGIN = float(os.environ.get('MYSOLVER_FALLBACK_INCLUSION_MARGIN', '-0.005'))
FALLBACK_CLEARANCE_EPS = float(os.environ.get('MYSOLVER_FALLBACK_CLEARANCE_EPS', '1e-4'))
FALLBACK_SAFE_POS = os.environ.get('MYSOLVER_FALLBACK_SAFE_POS', '1') == '1'

def _fallback_place_pos(container, item):
    if not FALLBACK_SAFE_POS:
        return np.array([0.0, 0.0, thickness + height / 2.0], dtype=np.float32)  # 旧挙動(V-3用)
    half = geo.half_extent([item['length'], item['width'], item['height']], 0)
    clearance = -FALLBACK_INCLUSION_MARGIN + FALLBACK_CLEARANCE_EPS   # 修正2: 符号は「+eps」
    z = min(thickness + half[2] + clearance, 天井側clip)
    # xy: 内側AABBの3x3グリッド(cut_xのみ考慮、cut_yはinclusion_slack_batch側で自動考慮)
    local_candidates = [[x, y, z] for x in (x_lo, x_mid, x_hi) for y in (y_lo, y_mid, y_hi)]
    world_candidates = [geo.local_to_world(container, c) for c in local_candidates]
    slack = geo.inclusion_slack_batch(container, half, world_candidates)
    return local_candidates[argmin(slack)]
```

- `item_idx: 0`固定は維持(指示どおり変更していない)。
- `planner.plan()`本体は無変更。
- 3値とも環境変数化(`MYSOLVER_FALLBACK_INCLUSION_MARGIN`/`_CLEARANCE_EPS`/`_SAFE_POS`)、
  margin値はハードコードしていない。
- 実装後、単体テストで`suite_A01`のitem19(死因アイテム)に対し
  `_fallback_place_pos`を呼んだところ`slack=-0.0051`(margin=-0.005を満たす、
  ちょうどeps=1e-4分だけ内側)を確認した。

---

## V-1: sudden death件数・死因内訳・n_placed_at_death

`tools/diagnose_sudden_death.py`を31シーン(26+scalestress3+sample_config2)で
再実行(既定`MYSOLVER_FALLBACK_SAFE_POS=1`、修正後):

| 項目 | 修正前(Phase54) | 修正後(Phase55) |
|---|---|---|
| sudden death件数 | 30/31 | **30/31(件数は不変)** |
| 死因 | `is_included`が30/30(100%) | **`is_valid`が30/30(100%)——is_includedは0/30** |
| はみ出し量(inclusion判定) | 平均+0.0050000m(境界外) | **平均-0.0001000m(境界内、margin内側にeps分の余裕。全30件で符号が反転)** |
| n_placed_at_death | 平均22.6(最小12〜最大45) | **全30件で修正前と1手も違わず完全に同一**(例: A01は21手目のまま、B04は46手目のまま) |

**is_includedは100%解消された(狙いどおり修正は機能している)。** しかし
**同じ手番で`is_valid`(`check_transport_path`——搬送経路上の衝突を実際に
pybulletで確認する判定)が代わりに落ち、継続手数は1件も伸びなかった。**

### なぜ手数が伸びなかったか

`check_transport_path`は候補位置へ実際にアイテムをスポーンし、Y軸→X軸の順に
動かして経路上の衝突を検知する(既存荷物との物理的な干渉チェック)。
本フォールバックは**コンテナの壁からのクリアランスしか見ておらず、既に
配置済みの荷物との衝突は一切考慮していない**。`planner.plan()`が`None`を
返す時点は「壁の内側だが荷物で埋まっていて置き場がない」状態である可能性が高く、
その場合はis_included問題を解消しても、次はis_valid(衝突)で確実に落ちる。
**「フォールバック発火の時点で既に詰んでいる」というPhase55指示の懸念どおりの
結果であり、件数の増減だけで判断してはならないという注意書きが的中した形。**

(注: 荷物衝突回避まで踏み込むには`item_idx`選択やより広い探索が必要になり、
「フォールバックのみに閉じる」「item_idx選択に手を入れない」という本フェーズの
scope外——指示どおり手を付けていない。)

---

## V-2: 26シーンA/B(対照 `phase40_baseline_off_mac.json`)

`tools/measure_regime.py`を26シーン全件・修正後のagent(既定有効)で実行
(`results/phase55_ab_fix_on.json`、壁時計非拘束・`REPLICA_SELECT=0`・
`UNITS_PER_SEC=2.00e7`、`phase40_baseline_off_mac.json`と同一条件)。

| 成分 | 差分の性質 |
|---|---|
| **fill_strict** | **26シーン全件で完全に不変(diff=0.000000)** |
| **placement_score** | **26シーン全件で完全に不変(diff=0.000000)** |
| **soft_item_score** | **26シーン全件で完全に不変(diff=0.000000)** |
| cog_score | 全件で±0.001未満のごく小さな変化(数値ノイズ水準) |
| stability_score | 全件で±0.04未満の変化(両方向、系統的な偏りなし) |

**fill/placement/soft_itemが完全不変なのは、V-1で確認したとおり
「配置に成功したアイテムの集合と手順が完全に同一」であることの直接証拠**
——修正はis_includedを解消したが、is_validで即座に置き換わって死ぬため、
**実際に配置されるアイテムの集合(num_placed_items含む)は1件も変化していない。**
cog/stabilityの微小な変化は、フォールバックの失敗する候補位置が変わった
ことで(`(0,0,z)`→グリッド探索後の位置)、**失敗する直前に一瞬スポーンされ
除去される荷物の残留物理状態がわずかに異なる**ことに起因すると考えられる
(stabilityは揺らし試験で物理エンジンの残留状態に敏感——Phase52参照)。

### composite_strict: mean / σ / SE / t

| 集計 | n | mean(Δcomposite) | σ | SE | t |
|---|---:|---:|---:|---:|---:|
| **全26シーン** | 26 | **-0.000295** | 0.004439 | 0.000871 | **-0.339** |
| 既積みあり6シーン(P01-P06) | 6 | -0.000756 | 0.005996 | 0.002448 | -0.309 |
| 非既積み20シーン | 20 | -0.000157 | 0.004047 | 0.000905 | -0.173 |

**|t|<1、有意な変化ではない。** 改善12シーン・悪化13シーン・不変1シーン
(A06、そもそもsudden death非該当)——**方向性に一貫した偏りはなく、
ノイズレベルの上下動にすぎない。**

### 悪化シーン(Δcomposite<0)一覧

| シーン | Δcomposite |
|---|---:|
| P02_A_1c_pre10 | -0.008475 |
| A05_2c_80_prio | -0.007977 |
| B04_2c_80_noprio | -0.005893 |
| P03_A_2c_pre8_prio | -0.005079 |
| D02_A_1c_40_prioheavy_nocont | -0.004321 |
| A01_1c_40_plain | -0.003916 |
| C03_2c_80_prio | -0.003403 |
| B02_1c_40_shelf | -0.003151 |
| P05_C_2c_pre8_shelfprio | -0.002854 |
| A02_1c_80_plain | -0.002260 |
| A04_2c_80_noprio | -0.002125 |
| A08_2c_140_extreme | -0.000858 |
| P06_A_1c_pre12_dense | -0.000218 |

最大でも-0.0085pt——**いずれもstability_scoreの数値ノイズ由来であり
(fill/placement/soft_itemはこれらのシーンも含め全て0.000000)、実質的な
悪化ではない。**

---

## V-3: 決定的8シーンのビット単位不変(`MYSOLVER_FALLBACK_SAFE_POS=0`)

```
$ MYSOLVER_FALLBACK_SAFE_POS=0 bash scripts/bp_check.sh
[B01] ... OK(基準値と一致)
[B02] ... OK(基準値と一致)
[B03] ... OK(基準値と一致)
[B04] ... OK(基準値と一致)
[P04] ... OK(基準値と一致)
[A01] ... OK(基準値と一致)
[A02] ... OK(基準値と一致)
[A03] ... OK(基準値と一致)
EXIT_CODE=0
```

**8/8完全一致。** ただし注記が必要: `bp_check.sh`は`agents/mysolver/ordering.py::build_order()`
(オフラインの候補順序探索)のみを検証しており、これは`agent.py::policy()`の
フォールバックを一切呼ばない独立した経路である。**したがってこの8/8確認は
「今回の修正が意図せずオフライン順序探索に影響していないこと」の確認であり、
修正そのもの(オンラインpolicyのフォールバック)を機能テストしたものではない。**
(`MYSOLVER_FALLBACK_SAFE_POS=1`側でも同じ理由で8/8になることは自明——今回は
指示どおり無効側のみ確認した。)

---

## V-4: 既定値の判断とbaseline更新方針

**既定を有効(`MYSOLVER_FALLBACK_SAFE_POS=1`)のまま据え置く。**

判断根拠:
1. **既知の100%決定論的バグ(is_included違反)を構造的に解消した**(V-1で確認)。
2. **26シーンA/Bで統計的に有意な悪化はない**(t=-0.34、V-2)。fill/placement/
   soft_itemは完全不変、cog/stabilityの微小変動もノイズレベル。
3. 今回計測した31シーンでは公開スコアへの直接効果は測定できなかったが
   (is_validが新たな律速のため)、これは「今回試した31シーンがたまたま
   フォールバック発火時点で衝突必至の状態だった」ことを示すに過ぎず、
   **is_includedバグ自体を残す理由にはならない。** 別のシーン構成・本番の
   実データでは、フォールバック発火時点でなお空間に余裕が残っている
   ケースがあり得る(その場合はis_validも通り、実際に1手多く置ける)。

### `scripts/bp_baseline_8scenes.json`の更新方針

**更新不要。** 理由(`docs/migration_to_mac.md`§5.2の恒久ルール——「既定値を
変更したら、既定値に依存するすべての検証経路を再実行する」——に従い実際に
検証した):

- `bp_check.sh`が照合する`build_order()`の出力は`ordering.py`のみに依存し、
  `agent.py::policy()`(本フェーズの変更対象)を一切呼ばない、独立した経路である。
- V-3で`MYSOLVER_FALLBACK_SAFE_POS=0`(無効)時に8/8一致を実測済み。既定の`=1`側でも
  同一の理由(経路が独立)により影響を受けないことをコード上でも確認済み
  (`_fallback_place_pos`は`agent.py::Agent.policy()`内でのみ呼ばれ、
  `ordering.build_order()`からは到達不可能)。
- Phase44のREPLICA_SELECT既定値変更のように「bp_check.shが暗黙にその変数へ
  依存していた」という構造は今回存在しない——新設の`MYSOLVER_FALLBACK_*`系
  環境変数は`agent.py`にのみ存在し、`ordering.py`/`bp_check.sh`のどちらからも
  参照されない。

---

## 変更ファイル

- `agents/mysolver/agent.py`(修正本体。`_fallback_place_pos()`新設、
  `if action is None:`ブロックのz座標計算を置き換え。`planner.plan()`本体・
  `item_idx`選択ロジックは無変更)
- `results/phase55_sudden_death_26_after.json` / `_scalestress_after.json` /
  `_sampleconfig_after.json`(新規、V-1の修正後診断結果)
- `results/phase55_ab_fix_on.json`(新規、V-2の26シーン測定結果)
- `results/phase55_report.md`(本ファイル)

`tools/scorer.py`・既存26シーン・`configs/sample_config.json`・`planner.py`・
`scripts/bp_baseline_8scenes.json`・`phase40_baseline_off_mac.json`はいずれも
無変更(指示どおり)。`.gitignore`の書き換え・force pushもなし。
