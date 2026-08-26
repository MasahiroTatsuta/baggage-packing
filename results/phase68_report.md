# Phase68 報告: 緩1を2枠目として確保、is_validの調査を打ち切り

## 結論(先出し)

**緩1(0.45/0.5/0.20)の提出用zipを作成した**(`submissions/mysolver_submit_phase67_loose1.zip`、
SHA256: `8f35132f3ad076956173cf9aab7e141b252fcf99d92a439a4900b0d55d2aa05e`)。
リポジトリ追跡ファイル(`agents/mysolver/planner.py`)の既定値は無変更。
判定は本番結果が出た時点で行う(先出し基準は下記)。

あわせて、Phase60〜67の8フェーズにわたる`is_valid`(搬送経路衝突)調査を打ち切り、
何が否定されたかをREADME進捗まとめに記録した。

---

## ステップ1: 緩1の提出用zipの作成

### (1-1) 閾値3定数をzip内でのみ既定値へ固定

`agents/mysolver/planner.py`のコピー(zipビルド用ディレクトリ、リポジトリ本体は
一切変更していない)で、支持閾値3定数の既定値のみ書き換えた:

```diff
-MIN_UNION_SUPPORT_RATIO = float(os.environ.get('MYSOLVER_MIN_UNION_SUPPORT_RATIO', '0.55'))
+MIN_UNION_SUPPORT_RATIO = float(os.environ.get('MYSOLVER_MIN_UNION_SUPPORT_RATIO', '0.45'))
-MIN_SUPPORT_SPAN_RATIO = float(os.environ.get('MYSOLVER_MIN_SUPPORT_SPAN_RATIO', '0.6'))
+MIN_SUPPORT_SPAN_RATIO = float(os.environ.get('MYSOLVER_MIN_SUPPORT_SPAN_RATIO', '0.5'))
-MAX_SUPPORT_CENTROID_OFFSET = float(os.environ.get('MYSOLVER_MAX_SUPPORT_CENTROID_OFFSET', '0.15'))
+MAX_SUPPORT_CENTROID_OFFSET = float(os.environ.get('MYSOLVER_MAX_SUPPORT_CENTROID_OFFSET', '0.20'))
```

`git diff --stat agents/mysolver/`は空(リポジトリ追跡ファイルは無変更)。
zipビルド用コピーとリポジトリの`planner.py`を`diff`した結果もこの3行のみで、
それ以外の差分はない。

### (1-2) 他の既定値(zip内コードを直接grep、全7項目)

```
$ grep -n "MYSOLVER_TELEMETRY'\|MYSOLVER_HARD_WALL_LIMIT'\|MYSOLVER_REPLICA_SELECT'\|MYSOLVER_REPLICA_METRIC'\|MYSOLVER_FALLBACK_SAFE_POS'\|MYSOLVER_FALLBACK_AVOID_OBSTACLES'\|MYSOLVER_STRICT_SUPPORT_DISABLE'" mysolver/*.py

planner.py:1394:    if os.environ.get('MYSOLVER_STRICT_SUPPORT_DISABLE', '0') == '1':
agent.py:39:FALLBACK_SAFE_POS = os.environ.get('MYSOLVER_FALLBACK_SAFE_POS', '1') == '1'
agent.py:47:FALLBACK_AVOID_OBSTACLES = os.environ.get('MYSOLVER_FALLBACK_AVOID_OBSTACLES', '1') == '1'
ordering.py:108:HARD_WALL_LIMIT = float(os.environ.get('MYSOLVER_HARD_WALL_LIMIT', '165.0'))
ordering.py:475:REPLICA_SELECT = os.environ.get('MYSOLVER_REPLICA_SELECT', '0') == '1'
ordering.py:505:MYSOLVER_TELEMETRY = os.environ.get('MYSOLVER_TELEMETRY', '0') == '1'
ordering.py:526:REPLICA_METRIC = os.environ.get('MYSOLVER_REPLICA_METRIC', 'fill')
```

**7項目すべて指示どおりの既定値**(TELEMETRY='0'、HARD_WALL_LIMIT='165.0'、
REPLICA_SELECT='0'、REPLICA_METRIC='fill'、FALLBACK_SAFE_POS='1'、
FALLBACK_AVOID_OBSTACLES='1'、STRICT_SUPPORT_DISABLE='0'(=strict_supportは
Phase13どおり有効))。

### (1-3) zip生成・決定的8シーンでの差分確認

```
cd <build_dir> && zip -r submissions/mysolver_submit_phase67_loose1.zip ./mysolver \
  -x '*__pycache__*' -x '*.pyc'
```

- **出力先**: `submissions/mysolver_submit_phase67_loose1.zip`(11ファイル、
  `mysolver/`直下に.pyファイル10個——既存の提出zipと同じ構造)
- **SHA256**: `8f35132f3ad076956173cf9aab7e141b252fcf99d92a439a4900b0d55d2aa05e`

zipを別ディレクトリへ展開し、`mysolver`(zip版、別名でimport)と`agents.mysolver`
(リポジトリ版)を同一プロセスにimportして、決定的8シーン(B01-B04, P04, A01-A03)の
`build_order()`出力を比較した:

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

**閾値の差により意図どおり2/8シーンで差分が出た**(今回は一致しないのが正しい)。
残り6シーンは閾値変更後も候補の合否が変わらず、同一の`build_order`結果になった。

### (1-4) アップロード時の照合

**アップロード時は必ずSHA256(`8f35132f3ad076956173cf9aab7e141b252fcf99d92a439a4900b0d55d2aa05e`)
と照合すること。**

### 提出後の判定基準(先出し)

- **53.64を超えた** → 2枠目として確定。主枠との入れ替えも検討。
- **53.64前後** → 2枠目として確保(挙動が主枠と異なるので枠の価値がある)。
- **明確に下がった** → 2枠目からも外す。Phase55と過去の53.64構成に戻す。

現時点では提出・判定は未実施。zip作成とローカル確認のみ完了。

---

## ステップ2: is_validの調査(Phase60〜67)を打ち切り

Phase60〜67の8フェーズで`is_valid`(搬送経路衝突によるsudden death)のレバーを
調べ尽くした。**何が否定されたかを、次に誰かが同じ道を辿らないよう記録する**
(README §6進捗まとめに追記、詳細は下記および各phaseの報告書を参照):

- **Phase60 SAFETY_MARGIN_XY**: 0.022→0.019/0.017へ緩めても、候補の合否
  (`stats['success']`)自体が3水準で完全に不変(23件のまま)。マージンは
  律速要因ではなかった。
- **Phase61 X方向掃引**: 「未実装」という前提が誤りで、実装済みだった
  (`legal2`/phase2)。sudden death 27件は全てY区間(phase1)で発生しており、
  X区間まで到達した死亡例は1件もなかった。
- **Phase62 legal1の判定精度**: 判定自体が実行されていなかった。
  `planner.plan()`が全27件でNoneを返し、legal1/legal2を一切通らない
  `agent.py::_fallback_place_pos`が使われ、既配置荷物へ最大−360mm食い込む
  位置に着地していた。
- **Phase63 fallbackへの衝突回避**: `_fallback_place_pos`に搬入経路の衝突回避を
  追加したが、9候補中0件が合法(27/27件全てで)。効果ゼロ、is_valid死亡数・
  衝突距離とも1件も変化しなかった。
- **Phase64 総当たり**: `planner.plan()`がNoneを返す27件に対し、内包判定×legal1×
  legal2のみで総当たり探索した結果、**18/27で合法解が実在**した。予算を
  桁違いに増やして(実消費の最大でも1e15分の4300万=0.000004%)再実行しても
  1件も見つからず、**予算不足が原因ではないことを確定した。**
- **Phase65 犯人特定**: Phase64の18シーン345件の合法解サンプルを実パイプラインへ
  順に通した結果、**345/345(100%)が`_evaluate_candidates`の支持品質判定
  (`support_ok`、`MIN_UNION_SUPPORT_RATIO`系)一本で落選**。他の条件
  (内包/legal1/legal2/候補生成/向き/tier)は0件。単一・全シーン共通の原因だった。
- **Phase66 内訳**: 345件の内訳は閾値未達のみ153件(44.3%)、forbidden_hit
  (下敷き禁止)併発192件(55.7%)、forbidden_hitのみ0件。**forbidden_hit=True
  は`support_ok`の式構造上、閾値をどれだけ緩めても恒久的にFalse**——55.7%は
  構造的に緩められない。
- **Phase67 スイープ**: 2G揺らしで支持閾値を再スイープ。緩1(0.45/0.5/0.20)は
  `threshold_only`シーン(forbidden_hitが絡まない5シーン)には明確に効く
  (composite_B t=2.37、num_placed_items t=2.33)が、全28シーンではt=2.06止まりで、
  副作用(悪化8件、最大−5.5pt)が`rest`群(狙っていない18シーン)に無差別に出る。
  緩2・strict_support無効化はいずれも不採用。

**結論: is_validのレバーは調べ尽くして閉じた。** 上限は18/27(67%、Phase64)、
そのうち実効があるのはthreshold_only分(345件中153件、44.3%)のみで、残り
forbidden_hit分(55.7%)は式構造上どれだけ閾値を緩めても救えない。緩1はこの
threshold_only分への対応として2枠目に確保するが、主枠には採用しない
(判断理由はPhase67報告および本フェーズ冒頭の指示を参照)。

---

## 次フェーズの予告(本フェーズでは着手しない)

**stability + cog**(重み1.5/7ずつ、合計3/7でfillの2/7より大きい)。Phase61/67で
2G測定の土台(`tools/phase67_suite_metrics.py`)が揃い、2Gで26シーンのstability
分散が1.3pt→34.5ptに開くことを確認済み。公式セミナーで運営が「重心を低くすれば
揺らしテストの点数も良くなる」と明言しており、cogとstabilityは同じ施策で
同時に動く可能性がある。完全に手つかずの領域。

---

## 生成物一覧

- `submissions/mysolver_submit_phase67_loose1.zip`(新規、2枠目候補の提出用zip、
  SHA256: `8f35132f3ad076956173cf9aab7e141b252fcf99d92a439a4900b0d55d2aa05e`)。
- `results/phase68_report.md`(本ファイル)。
- `README.md`(§6進捗まとめにPhase60〜67の否定リストを追記)。

`agents/mysolver/`(リポジトリ追跡ファイル)・`tools/scorer.py`・既存configは
いずれも無変更。本番の集計スコアから非公開パラメーターを逆算する分析は行っていない。
