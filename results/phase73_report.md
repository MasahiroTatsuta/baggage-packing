# Phase73 報告: 3パラメータを分離して2本提出

## 結論(先出し)

**緩3の本番結果(public 57.18→54.80、−2.38)を受け、支持閾値3パラメータ
(union/span/centroid)を分離した2本の提出用zipを作成した**(判定は本番結果待ち)。
主枠(緩2、public 57.18)・2枠目(緩1、public 56.05)は変更しない。

---

## 緩3の本番結果(再掲・分析)

public 57.18 → **54.80(−2.38)。緩めすぎ。**

閾値カーブ: 0.55→53.64 / 0.45→56.05 / **0.35→57.18(ピーク)** / 0.25→54.80

崩壊の内訳(緩2→緩3):

| 成分 | 変化 | 換算寄与 |
|---|---:|---:|
| stability | −4.00 | −0.857 |
| cog | −3.77 | −0.807 |
| placement | −5.25 | −0.750 |
| fill | +0.09 | — |
| soft_item | +0.05 | — |

配置率は64.34%→64.08%と微減。「もっと置ける」効果は緩2で止まり、置いたものが
崩れるコストだけが残った——cog低下(重心上昇)・stability低下・placement低下は
不安定な高積みの典型的症状であり、`MAX_SUPPORT_CENTROID_OFFSET`(接触面積重心の
底面中心からのずれ許容量)を緩めすぎたことが疑われる。3パラメータを同時に
動かしてきたため犯人を切り分けていなかった。

---

## ステップ1: 提出枠の更新

- **主枠**: `submissions/mysolver_submit_loose2.zip`(public 57.18、変更なし)
- **2枠目**: `submissions/mysolver_submit_phase67_loose1.zip`(public 56.05、変更なし)
- 緩3は枠に入れない。
- `docs/submission_policy.md`§1にPhase73追記を記録した(§5は変更なし)。

---

## ステップ2: 2本のzip

### 全10項目のgrep(共通7項目 + 支持閾値3項目)

両zipとも他7項目は指示どおりの既定値(TELEMETRY=0 / HARD_WALL_LIMIT=165.0 /
REPLICA_SELECT=0 / REPLICA_METRIC=fill / FALLBACK_SAFE_POS=1 /
FALLBACK_AVOID_OBSTACLES=1 / STRICT_SUPPORT_DISABLE=0)を確認済み。

### (A) `mysolver_submit_loose2_tightC.zip` — centroidを既定に戻す版

```diff
-MIN_UNION_SUPPORT_RATIO = float(os.environ.get('MYSOLVER_MIN_UNION_SUPPORT_RATIO', '0.55'))
+MIN_UNION_SUPPORT_RATIO = float(os.environ.get('MYSOLVER_MIN_UNION_SUPPORT_RATIO', '0.35'))
-MIN_SUPPORT_SPAN_RATIO = float(os.environ.get('MYSOLVER_MIN_SUPPORT_SPAN_RATIO', '0.6'))
+MIN_SUPPORT_SPAN_RATIO = float(os.environ.get('MYSOLVER_MIN_SUPPORT_SPAN_RATIO', '0.4'))
 MAX_SUPPORT_CENTROID_OFFSET = float(os.environ.get('MYSOLVER_MAX_SUPPORT_CENTROID_OFFSET', '0.15'))  # 既定のまま(緩めない)
```

`git diff --stat agents/mysolver/`は空(リポジトリ追跡ファイルは無変更)。差分は
union/spanの2行のみで、centroidの行はリポジトリ既定値のまま変更していない。

- **出力先**: `submissions/mysolver_submit_loose2_tightC.zip`(11ファイル)
- **SHA256**: `f7a18ac5fe1189673798bb38ad96fe3cc6547db9bc51c293d08943a647ed0f56`
- 決定的8シーンの差分: **1/8(A03)**

### (B) `mysolver_submit_loose4_tightC.zip` — union/spanだけさらに緩める版

```diff
-MIN_UNION_SUPPORT_RATIO = float(os.environ.get('MYSOLVER_MIN_UNION_SUPPORT_RATIO', '0.55'))
+MIN_UNION_SUPPORT_RATIO = float(os.environ.get('MYSOLVER_MIN_UNION_SUPPORT_RATIO', '0.25'))
-MIN_SUPPORT_SPAN_RATIO = float(os.environ.get('MYSOLVER_MIN_SUPPORT_SPAN_RATIO', '0.6'))
+MIN_SUPPORT_SPAN_RATIO = float(os.environ.get('MYSOLVER_MIN_SUPPORT_SPAN_RATIO', '0.3'))
 MAX_SUPPORT_CENTROID_OFFSET = float(os.environ.get('MYSOLVER_MAX_SUPPORT_CENTROID_OFFSET', '0.15'))  # 既定のまま(緩めない)
```

`git diff --stat agents/mysolver/`は空。差分はunion/spanの2行のみ。

- **出力先**: `submissions/mysolver_submit_loose4_tightC.zip`(11ファイル)
- **SHA256**: `9c484c7d6ab23fca816945a58bfd20e73d5a538f4cda43df6415c54c2b524271`
- 決定的8シーンの差分: **1/8(A03)**

### 決定的8シーンの比較結果(両zip共通)

```
[B01] 一致  [B02] 一致  [B03] 一致  [B04] 一致  [P04] 一致
[A01] 一致  [A02] 一致  [A03] ★差分あり

差分ありシーン数: 1/8(A03、両zipで同一)
```

(A)・(B)ともcentroidは既定(0.15)のまま共通なので、A01/B02で緩1〜3に見られた
差分がここでは消え、centroidを動かしていた緩1〜3とは異なるシーン集合になった
(参考値。28シーン全体の効果の大小を代表するものではない)。

### アップロード時の照合

**アップロード時は必ずSHA256と照合すること**
((A): `f7a18ac5fe1189673798bb38ad96fe3cc6547db9bc51c293d08943a647ed0f56`、
(B): `9c484c7d6ab23fca816945a58bfd20e73d5a538f4cda43df6415c54c2b524271`)。

### 判定基準(先出し)

- (A) > 57.18 → centroidが犯人と確定。主枠を(A)に。(B)の結果次第でさらに緩める。
- (A) ≈ 57.18 → 3つは分離できない。緩2が最適点。閾値軸は終了。
- (A) < 57.18 → centroidの緩和も配置数に寄与していた。緩2が最適点。
- (B)は(A)が上回った場合のみ意味を持つ。(A)が駄目なら(B)の結果は参考値。

現時点では提出・判定は未実施。zip作成とローカル確認のみ完了。

---

## 生成物一覧

- `submissions/mysolver_submit_loose2_tightC.zip`(新規、SHA256:
  `f7a18ac5fe1189673798bb38ad96fe3cc6547db9bc51c293d08943a647ed0f56`)
- `submissions/mysolver_submit_loose4_tightC.zip`(新規、SHA256:
  `9c484c7d6ab23fca816945a58bfd20e73d5a538f4cda43df6415c54c2b524271`)
- `results/phase73_report.md`(本ファイル)
- `docs/submission_policy.md`(§1にPhase73追記)

`agents/mysolver/`(リポジトリ追跡ファイル)・`tools/scorer.py`・既存configは
いずれも無変更。本番の集計スコアから足切り閾値やシーン数を逆算する分析は
行っていない。
