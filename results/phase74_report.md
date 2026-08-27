# Phase74 報告: centroid一点に絞った3水準提出

## 結論(先出し)

**分離実験の結果、これまでの利得(緩1〜3で観測した+3.54)はほぼ全て
`MAX_SUPPORT_CENTROID_OFFSET`単独が生んでいたことが判明した。**
`MIN_UNION_SUPPORT_RATIO`/`MIN_SUPPORT_SPAN_RATIO`はほとんど何もしていない。
探索空間はcentroid一本に縮んだため、union/spanを緩2と同じ0.35/0.4に固定し、
centroidを0.225/0.275/0.300の3水準で提出した(判定は本番結果待ち)。

---

## 分離実験の結果(再掲・分析)

| zip | union/span | centroid | public |
|---|---|---|---:|
| (A) `mysolver_submit_loose2_tightC.zip` | 0.35/0.4 | 0.15(既定) | 53.76 |
| (B) `mysolver_submit_loose4_tightC.zip` | 0.25/0.3 | 0.15(既定) | 53.87 |

| 固定 | 動かす | 効果 |
|---|---|---:|
| centroid=0.15 | union/span 0.55→0.35→0.25 | +0.12 → +0.23(ほぼゼロ) |
| union/span=0.35/0.4 | centroid 0.15→0.25(緩2) | **+3.43** |

**仮説は完全に逆だった。** Phase73では「centroidを緩めると転倒しやすい配置が
通る」と読んだが、実際はcentroidが配置そのものを最も強く制約しており、緩める
ことで置けるようになる荷物が増え、足切りを越えるシーンで複数指標が同時に跳ねる
(Phase70以来の構造)。union/spanを動かしても4指標はほぼ動かない。

---

## ステップ1: 提出枠(変更なし)

- **主枠**: `submissions/mysolver_submit_loose2.zip`(public 57.18)
- **2枠目**: `submissions/mysolver_submit_phase67_loose1.zip`(public 56.05)
- (A)(B)は枠に入れない。
- `docs/submission_policy.md`§1にPhase74追記(分離実験の結論)を記録した。

---

## ステップ2: centroidを3水準で提出

union/spanは緩2と同じ0.35/0.4に固定し、centroidだけを動かした。

### 全10項目のgrep(3zip共通)

3zipとも他7項目は指示どおりの既定値(TELEMETRY=0 / HARD_WALL_LIMIT=165.0 /
REPLICA_SELECT=0 / REPLICA_METRIC=fill / FALLBACK_SAFE_POS=1 /
FALLBACK_AVOID_OBSTACLES=1 / STRICT_SUPPORT_DISABLE=0)を確認済み。
union/span/centroidの3項目もzip内コードから直接grepし、以下のとおり確認した:

```
(C) mysolver_submit_c0225.zip:
  MIN_UNION_SUPPORT_RATIO      = '0.35'
  MIN_SUPPORT_SPAN_RATIO       = '0.4'
  MAX_SUPPORT_CENTROID_OFFSET  = '0.225'

(D) mysolver_submit_c0275.zip:
  MIN_UNION_SUPPORT_RATIO      = '0.35'
  MIN_SUPPORT_SPAN_RATIO       = '0.4'
  MAX_SUPPORT_CENTROID_OFFSET  = '0.275'

(E) mysolver_submit_c0300.zip:
  MIN_UNION_SUPPORT_RATIO      = '0.35'
  MIN_SUPPORT_SPAN_RATIO       = '0.4'
  MAX_SUPPORT_CENTROID_OFFSET  = '0.300'
```

`git diff --stat agents/mysolver/`は空(リポジトリ追跡ファイルは無変更)。
各zipビルド用コピーとリポジトリの`planner.py`の差分も、この3行のみ
(union/spanの2行 + centroidの1行)。

### SHA256とビルド情報

| zip | SHA256 |
|---|---|
| `mysolver_submit_c0225.zip` | `d80c16c51da147c89de3fffc7e6988212756a7f0280906d3177c9e3de3ba14c7` |
| `mysolver_submit_c0275.zip` | `950e07fd1d34d6f27ffd8507f61fee3c82ff4626d5e0ca2536f19cb540ee5798` |
| `mysolver_submit_c0300.zip` | `e3aa3097e06f9b41563d3aee031fc7a0da8b6ab857484772213d6240dc8a2e55` |

いずれも11ファイル、既存提出zipと同一構造。**アップロード時は必ず上記SHA256と
照合すること。**

### 決定的8シーンの差分(参考値)

| zip (centroid) | 差分シーン数 | 差分シーン |
|---|---:|---|
| (C) 0.225 | 3/8 | B02, A01, A03 |
| (D) 0.275 | 2/8 | A01, A03 |
| (E) 0.300 | 2/8 | B02, A03 |

差分シーンの構成が水準ごとに非単調に変化する(Phase60・Phase71で確認した
「カオス的な再配置」と同型の現象)。28シーン全体の効果の大小を代表するもの
ではなく、あくまで参考値。

### 判定基準(先出し、現在のカーブ)

0.15→53.76 / 0.20(緩1相当)→56.05 / 0.25(緩2)→**57.18** / 0.30(union/span別)→54.80

- (D) 0.275が57.18を超えた → ピークは0.25〜0.30。さらに0.26/0.29を詰める。
- (C) 0.225が57.18を超えた → ピークは0.20〜0.25側。
- どちらも下回った → **centroid 0.25が最適点。閾値軸は終了。**
- (E) 0.30が54.80より明確に高い → 緩3の崩壊はunion/span 0.25/0.3が寄与して
  いたことになる(その場合はunion/spanも再検討)。

現時点では提出・判定は未実施。zip作成とローカル確認のみ完了。

---

## 生成物一覧

- `submissions/mysolver_submit_c0225.zip`(新規、SHA256:
  `d80c16c51da147c89de3fffc7e6988212756a7f0280906d3177c9e3de3ba14c7`)
- `submissions/mysolver_submit_c0275.zip`(新規、SHA256:
  `950e07fd1d34d6f27ffd8507f61fee3c82ff4626d5e0ca2536f19cb540ee5798`)
- `submissions/mysolver_submit_c0300.zip`(新規、SHA256:
  `e3aa3097e06f9b41563d3aee031fc7a0da8b6ab857484772213d6240dc8a2e55`)
- `results/phase74_report.md`(本ファイル)
- `docs/submission_policy.md`(§1にPhase74追記)

`agents/mysolver/`(リポジトリ追跡ファイル)・`tools/scorer.py`・既存configは
いずれも無変更。本番の集計スコアから足切り閾値やシーン数を逆算する分析は
行っていない。
