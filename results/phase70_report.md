# Phase70 報告: 緩2の提出と主枠の入れ替え

## 結論(先出し)

**主枠を緩1(`mysolver_submit_phase67_loose1.zip`、public 56.05)に入れ替えた。**
2枠目はPhase55(public 53.64)を一旦維持。**緩2の提出用zip
(`submissions/mysolver_submit_loose2.zip`)を作成した**(判定は本番結果待ち)。

---

## 緩1の本番結果(再掲・分析)

public 53.64 → **56.05(+2.41)**。11提出ぶりの前進。

| 成分 | 変化 | 換算寄与 | 割合 |
|---|---:|---:|---:|
| stability | +4.05 | +0.868 | 36.0% |
| soft_item | +5.20 | +0.743 | 30.8% |
| cog | +2.95 | +0.633 | 26.3% |
| placement | +0.85 | +0.121 | 5.0% |
| **fill** | **+0.15** | **+0.044** | **1.8%** |

`num_placed_items`: 61.37% → 62.25%(Phase35以来はじめて動いた)。

**利得の98%はfill以外。** Phase67のローカル2G実測(Δfill +1.19・Δstability +0.01)
とは方向が逆であり、原因は`tools/scorer.py`が足切りを実装していないこと
(Phase61で確認済み)。**ローカルの合成スコアは、配置数がわずかに増えて
足切りを越えたシーンで複数指標が同時に跳ねるという、本番で判明した最大の
効果を構造的に検出できない。**

---

## ステップ1: 提出枠の入れ替え

- **主枠**: `submissions/mysolver_submit_phase67_loose1.zip`(public 56.05)
- **2枠目**: `submissions/mysolver_submit_phase55.zip`(public 53.64、一旦維持)
- `docs/submission_policy.md`§1にPhase70追記、§5の主枠・2枠目まとめ表を更新した。

---

## ステップ2: 緩2の提出用zip

### (2-1) 閾値3定数をzip内でのみ既定値へ固定

`agents/mysolver/planner.py`のzipビルド用コピーのみ書き換え(Phase68と同一手順、
リポジトリ追跡ファイルは無変更):

```diff
-MIN_UNION_SUPPORT_RATIO = float(os.environ.get('MYSOLVER_MIN_UNION_SUPPORT_RATIO', '0.55'))
+MIN_UNION_SUPPORT_RATIO = float(os.environ.get('MYSOLVER_MIN_UNION_SUPPORT_RATIO', '0.35'))
-MIN_SUPPORT_SPAN_RATIO = float(os.environ.get('MYSOLVER_MIN_SUPPORT_SPAN_RATIO', '0.6'))
+MIN_SUPPORT_SPAN_RATIO = float(os.environ.get('MYSOLVER_MIN_SUPPORT_SPAN_RATIO', '0.4'))
-MAX_SUPPORT_CENTROID_OFFSET = float(os.environ.get('MYSOLVER_MAX_SUPPORT_CENTROID_OFFSET', '0.15'))
+MAX_SUPPORT_CENTROID_OFFSET = float(os.environ.get('MYSOLVER_MAX_SUPPORT_CENTROID_OFFSET', '0.25'))
```

`git diff --stat agents/mysolver/`は空(リポジトリ追跡ファイルは無変更)。
zipビルド用コピーとリポジトリの`planner.py`の差分もこの3行のみ。

他7項目をzip内コードから直接grep:

```
planner.py:1394: MYSOLVER_STRICT_SUPPORT_DISABLE', '0'
agent.py:39:     MYSOLVER_FALLBACK_SAFE_POS', '1'
agent.py:47:     MYSOLVER_FALLBACK_AVOID_OBSTACLES', '1'
ordering.py:108: MYSOLVER_HARD_WALL_LIMIT', '165.0'
ordering.py:475: MYSOLVER_REPLICA_SELECT', '0'
ordering.py:505: MYSOLVER_TELEMETRY', '0'
ordering.py:526: MYSOLVER_REPLICA_METRIC', 'fill'
```

**7項目すべて指示どおりの既定値。**

### (2-2) zip生成・決定的8シーンでの差分確認

- **出力先**: `submissions/mysolver_submit_loose2.zip`(11ファイル、既存の提出zipと同じ構造)
- **SHA256**: `c26eba72b99a037b7e1f08c200bf2b0f390e1241f8cfd1f78890eb6a2a9ac62a`

決定的8シーン(B01-B04, P04, A01-A03)でzip版とリポジトリ版の`build_order`を比較:

```
[B01] repo_n=40 zip_n=40 一致
[B02] repo_n=40 zip_n=40 一致
[B03] repo_n=80 zip_n=80 一致
[B04] repo_n=80 zip_n=80 一致
[P04] repo_n=34 zip_n=34 一致
[A01] repo_n=40 zip_n=40 ★差分あり
[A02] repo_n=80 zip_n=80 一致
[A03] repo_n=40 zip_n=40 ★差分あり

差分ありシーン数: 2/8(A01, A03)
```

**緩1と同じ2/8(A01, A03)だった。** 「緩2はより攻めた閾値なので緩1(2/8)以上になる
はず」という予想は外れた——決定的8シーンのうち、この閾値変更で候補の合否が
変わるのはA01/A03の2シーンのみで、緩1(0.45/0.5/0.20)から緩2(0.35/0.4/0.25)へ
さらに緩めても、この8シーンの範囲では新たに動くシーンは増えなかった。
(28シーンでの効果はPhase67のローカル2G実測を参照。決定的8シーンはあくまで
ビット単位一致確認用のサブセットであり、28シーン全体の効果の大小を代表する
ものではない。)

### (2-3) アップロード時の照合

**アップロード時は必ずSHA256(`c26eba72b99a037b7e1f08c200bf2b0f390e1241f8cfd1f78890eb6a2a9ac62a`)
と照合すること。**

### 判定基準(先出し)

- **56.05を超えた** → 緩2を主枠に。さらに緩める方向(緩3)を検討。
- **56.05前後** → 2枠目として確保(緩1と挙動が異なるので枠の価値がある)。
- **明確に下がった** → 緩めすぎ。緩1と緩2の中間(0.40/0.45/0.225)を試す。

現時点では提出・判定は未実施。zip作成とローカル確認のみ完了。

---

## ステップ3: 進捗まとめの更新

`docs/submission_policy.md`§1に以下を記録した:

- public 56.05の成分内訳と、fillの寄与が1.8%しかなかったこと
- ローカルの合成スコア(定義A・定義Bとも)は足切りを見ておらず、本番の主効果を
  構造的に検出できないという制約
- 今後のA/Bは`num_placed_items`(配置数)を第一の主KPIとする(合成スコアは補助指標に降格)

---

## 生成物一覧

- `submissions/mysolver_submit_loose2.zip`(新規、SHA256:
  `c26eba72b99a037b7e1f08c200bf2b0f390e1241f8cfd1f78890eb6a2a9ac62a`)
- `results/phase70_report.md`(本ファイル)
- `docs/submission_policy.md`(§1・§5を更新)

`agents/mysolver/`(リポジトリ追跡ファイル)・`tools/scorer.py`・既存configは
いずれも無変更。本番の集計スコアから足切り閾値やシーン数を逆算する分析は行っていない。
