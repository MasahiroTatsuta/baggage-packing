# Phase38 報告: n=4(evaluate()内の実行時例外)の中身を自己申告させる

## 0. 背景の要約

Phase37で本番の`optimization`実測値(160.0091s)がn=4(`ReplicaEvaluator.evaluate()`実行中の
実行時例外)に一致することを確定した。H1(preflight失敗)/H2(import失敗)/H3(is_applicable
False)はいずれも否定されており、**本番でpybulletは動き、`open()`まで成功している**。
本フェーズの目的は、n=4の中身(例外クラス・発生タイミング)を特定すること。

---

## ステップ0: メモリ仮説(RLIMIT_AS枯渇)の検証 — 否定

### 0.1 仮説

`src/ground_handling/runner.py:14-15` は子プロセスに `resource.setrlimit(RLIMIT_AS, max_mem GB)`
(既定4GB、本フェーズ対象configは12GB)を課す。`open()`は成功し`evaluate()`で落ちる、という段階は
「アイテムを1個ずつ置くたびに衝突形状が増えてVAが伸び、途中で確保に失敗する」という挙動と
一致する。Phase36が測ったpeak RSS(102→103MB)はRLIMIT_AS(仮想アドレス空間)とは別物であり、
安全性の根拠にならない。

### 0.2 実装

`agents/mysolver/replica.py`に`MYSOLVER_REPLICA_VMLOG=1`で有効化するVAログを追加(既定無効)。
`/proc/self/status`のVmPeak/VmSize/VmHWM/VmRSSを`open()`直後・`run_order`の10配置ごと・
終了時に記録する。

### 0.3 実測

ローカル最大級の2シーン(A02: 1コンテナ80アイテム、A08: 2コンテナ140アイテム=ローカル最大)で、
12GB(config既定)・4GB(明示的制限)の両方を課して実行した。

| シーン | 制限 | open()直後VmPeak | 4候補評価後VmPeak(頭打ち) | 結果 |
|---|---:|---:|---:|---|
| A02(80アイテム) | 12GB | 229MB | 265MB | 正常完了、runtime_error 0件 |
| A02(80アイテム) | 4GB | 229MB | 265MB | 正常完了、runtime_error 0件 |
| A08(140アイテム) | 12GB | 219MB | 261MB | 正常完了、runtime_error 0件 |
| A08(140アイテム) | 4GB | 220MB | 262MB | 正常完了、runtime_error 0件 |

**結論**: ローカルで到達できる最大規模(140アイテム・2コンテナ)では、複製評価器のVA使用量は
4GB/12GBのどちらの上限にも遠く及ばず(頭打ち265MB)、`evaluate()`内でのRLIMIT_AS枯渇は
再現できなかった。本番シーンがローカルより大幅に大きい場合は除外しきれないが、
**「メモリが最有力仮説」という前提は、少なくともローカル最大規模では裏付けが取れなかった**。

### 0.4 副産物(n=4とは無関係の潜在バグ)

検証中、`max_mem`をfloatで渡すと`runner.py:15`の`resource.setrlimit`が
`TypeError: 'float' object cannot be interpreted as an integer`で子プロセスごと即死することを
発見した。`except ImportError`では捕まらない。ただし本番は`open()`まで到達している
(=起動直後には死んでいない)ので、これはn=4の説明にはなり得ない。`src/`は本番と同じ写しで
変更しない方針のため記録のみに留める。

---

## ステップ1: 例外の自己申告テレメトリ + ラッチ緩和

### 1-A. 例外自己申告テレメトリ

n=4のときだけ、壁時計の埋め込み先をPhase37の帯(158.0〜161.5s、0.5s刻み)から切り離し、
専用の帯(162.00〜165.15s、0.05s刻み)へ置き換える。

```
T = 162.00 + 0.05 × code       code = 16·a + b   (0〜63 → T は 162.00〜165.15)
```

| b | 例外クラス | b | 例外クラス |
|---|---|---|---|
| 0 | MemoryError | 8 | RuntimeError |
| 1 | pybullet.error | 9 | OSError/IOError |
| 2 | KeyError | 10 | AssertionError |
| 3 | IndexError | 11 | ZeroDivisionError |
| 4 | AttributeError | 12 | RecursionError |
| 5 | ValueError | 13 | OverflowError |
| 6 | TypeError | 14 | その他(クラス名文字列を別途記録) |
| 7 | TimeoutError | 15 | 予約 |

`a`(0〜3、3で頭打ち)は最初の失敗までに成功した候補数。isinstance判定はサブクラスを
親クラスより先に判定する(`RecursionError`→`RuntimeError`、`TimeoutError`→`OSError`)。

実装: `agents/mysolver/ordering.py`の`_classify_exception()`、`TELEMETRY_N4_BASE_S=162.00`・
`TELEMETRY_N4_STEP_S=0.05`(いずれも`MYSOLVER_TELEMETRY_N4_*`で上書き可)。既定無効
(`MYSOLVER_TELEMETRY=0`)時は分岐にすら入らない。

**検証結果**(A01、budget=15/60s、`MYSOLVER_REPLICA_FORCE_FAIL=runtime`+
`MYSOLVER_REPLICA_FORCE_EXC`+`MYSOLVER_REPLICA_FORCE_FAIL_AFTER`で個別に発火):

| ケース | 期待T | 実測elapsed | 判定 |
|---|---:|---:|---|
| b=pybullet.error, a=0 | 162.05 | 162.061s | OK |
| b=KeyError, a=0 | 162.10 | 162.100s | OK |
| b=AttributeError, a=0 | 162.20 | 162.206s | OK |
| b=ValueError, a=0 | 162.25 | 162.258s | OK |
| b=TypeError, a=0 | 162.30 | 162.301s | OK |
| b=TimeoutError, a=0 | 162.35 | 162.354s | OK |
| b=RuntimeError, a=0 | 162.40 | 162.402s | OK |
| b=RuntimeError, a=1 | 163.20 | 163.204s | OK |
| b=RuntimeError, a=2 | 164.00 | 164.001s | OK |

9/9ケース全て設計どおり(実測は目標+9〜15ms、Phase37と同傾向)。a=1/2はcand_poolに
複数候補が必要なためbudget=60sで検証(budget=15sではcand_poolが1件しか集まらず
a≥1を検証できなかった)。

### 1-B. 取り置きが構築を打ち切ったかの1ビット

`planner.SearchBudget`には元々`hard_expired`(hard_deadlineに実際に到達したか)属性が
存在しており、これを流用した。本番既定(`DEFAULT_TIME_BUDGET=120`, `reserve_s=45`)では
`hard_deadline = start + min(165-45, 120*1.4) = start+120`が常に前者(取り置き由来の項)で
決まるため、`use_replica and total_budget.hard_expired`は「取り置きが構築を実際に
短くしたか」をほぼ直接意味する。`ordering.LAST_BUILD_WALL_CUT`(モジュールグローバル、
同一プロセス内で`optimize()`→`policy()`が共有)に記録し、`agent.py`の`policy()`が
**最初の呼び出し1回だけ**6.7s(打ち切りあり)/6.2s(打ち切りなし)まで埋める。

**検証**(local_eval.py、本番同様のサブプロセス経由、`MYSOLVER_TELEMETRY=1`):

| シーン | LAST_BUILD_WALL_CUT | 目標T | 実測policy_time | timeout(8.0s)に対する余裕 |
|---|---|---:|---:|---:|
| A06(打ち切りあり) | True | 6.7s | **6.71s** | 1.29s |
| A02(打ち切りなし) | False | 6.2s | **6.20s** | 1.80s |

両分岐とも設計どおりに発火し、8.0sタイムアウトに対して安全マージンを確保した。**採用**。

### 1-C. ラッチの候補単位化

従来(Phase36)は1候補の失敗でそのシーンの複製評価を丸ごと諦めていた。
`MYSOLVER_REPLICA_LATCH_MODE`(既定`per_candidate`)で、失敗した候補だけ飛ばして
次候補の評価を続け、**2回連続で失敗した場合だけ**シーン単位のラッチに落とす形に変更した
(`'scene'`でPhase36の旧挙動に戻せる)。

**正常系でのビット単位不変性検証**(決定的8シーン B01-B04, P04, A01-A03、budget=30s、
Step0時点のコミット(HEAD, `7e9d0c1`)とのgit worktree比較): **8/8一致**。

### 1-E. 新フォーマットのベースライン(Mac移行後の一致確認用)

```
MYSOLVER_REPLICA_SELECT=0 MYSOLVER_UNITS_PER_SEC=2.00e7 PYTHONPATH=. \
  .venv/bin/python tools/measure_regime.py --config-path 'configs/gen/suite_*.json' \
  --module-path agents/mysolver/ --repeats 1 \
  --out results/phase38_baseline_off_codespaces.json --label baseline_off_codespaces
```

| 指標 | mean | max |
|---|---:|---:|
| fill_strict | 24.29 | — |
| fill_loose | 36.21 | — |
| composite_strict | 70.43 | — |
| composite_loose | 73.84 | — |
| optimize_time | 87.86s | 165.01s(HARD_WALL_LIMIT=165s) |
| policy_time | 0.484s | 1.89s(timeout=8s) |

総所要2601s(約43.4分)。REPLICA_SELECT=0(ρ-test無効)・UNITS_PER_SEC=2.00e7(提出既定)。

### 1-F. 提出用zip

- ベース: 現在のリポジトリ状態(1-A/1-B/1-Cを含む)
- **変更点は`MYSOLVER_TELEMETRY`の既定を`'0'`→`'1'`に固定することのみ**
  (SIGNATEはzipアップロードで環境変数を渡せないため、Phase36/37と同じ制約)
- **リポジトリの追跡ファイル(`agents/mysolver/ordering.py`)は変更していない**(既定`'0'`のまま)
- 出力: `submissions/mysolver_submit_phase38_probe.zip`(11エントリ、10ファイル+ディレクトリ)
- **SHA256**: `77059c0ad6b9d8ca54df592c7e3e73699e86d3863ee94c9619af89dfa0c7333c`
- zip内コードとリポジトリコードで、決定的8シーン・同一budget(15s)のbuild_order出力(`order`)が
  **8/8完全一致**することを確認した(CPU負荷のないクリーンな環境で実施。並行して重い計測
  ジョブを走らせた状態での初回検証は7/8で疑似的な不一致が出たが、これはPHASE38自身の
  1-A/B/C変更とは無関係の**壁時計競合によるノイズ**であることをrepo-vs-repo再検証で確認済み
  — `hard_deadline`が真の壁時計に依存するため、他プロセスとのCPU競合下では同一コードでも
  リスタート回数が振れうる。この振れは既存の`total_budget.hard_deadline`設計に内在するもので
  本フェーズの変更が原因ではない)。

### 1-G. push

```
git add -f results/phase38_baseline_off_codespaces.json submissions/mysolver_submit_phase38_probe.zip
git add agents/mysolver/ordering.py agents/mysolver/agent.py agents/mysolver/replica.py README.md \
  tools/gen_tech_doc.py docs/技術対策.docx
git commit && git push
```

**方針からの1点逸脱**: 指示は`git add -f results/`（ディレクトリ全体）だったが、`results/`配下には
`results-dir-tracking-policy`（過去分phase10〜36は遡って追加しない）の対象外である
未追跡ファイルが98件残っている（phase12〜25の生ログ等）。これらを一緒に追加すると既存方針と
矛盾するため、本コミットでは新規生成した`phase38_baseline_off_codespaces.json`だけを
スコープして追加した。

---

## まとめ

| 項目 | 結果 |
|---|---|
| メモリ仮説(RLIMIT_AS枯渇) | ローカル最大規模(140アイテム)で再現できず、否定的材料 |
| 1-A 例外自己申告テレメトリ | 9/9検証OK、既定無効時ビット単位不変 |
| 1-B 取り置き打ち切りビット | 2分岐ともOK、timeout比1.29s以上の余裕あり、採用 |
| 1-C ラッチ候補単位化 | 8/8ビット単位不変(正常系)、既定有効 |
| 1-E ベースライン | fill_strict 24.29 / composite_strict 70.43 / optimize_time mean 87.86s max 165.01s |
| 1-F zip | ~~SHA256 `77059c0a...`~~ **提出せず**。下記「修正指示への対応」参照 |

**この時点のzip(`77059c0a...`)は提出しなかった。** 1-E実測で`optimize_time max=165.01s`
(HARD_WALL_LIMITそのものに到達)が判明し、n=4の符号帯(162.00〜165.15s)が自然な壁時計と
衝突しうることが分かったため、提出前に以下の修正指示が入った。詳細は次節。

---

## 追記: 提出前の修正(ステップA〜G)

Phase38報告(上記)を受け、提出前に2つの問題が見つかった。

1. **n=4の符号帯(162.00〜165.15s)が自然値と衝突する**: 1-Eの実測で`optimize_time max
   = 165.01s`が出ており、埋め込み無しの自然な壁時計がそのまま符号帯に入るため
   `162.00+0.05×60=165.00s`(a=3, b=12 RecursionError)などと誤読しうる。1-Cのラッチ
   候補単位化で複製評価の試行回数が増える分、本番でも壁に張り付くシーンは増える方向。
   さらにn=4帯がPhase37のn=6/7帯(161.0/161.5s)より上にあるため、`optimization`は
   全シーンのmaxしか報告されず、n=4が出ているシーンがあるとn=6/7(ρ-testが完走した
   証拠)が隠れてしまう。
2. **1-Eのベースラインは壁時計拘束ありで取っており、Mac移行後の非拘束計測とは
   原理的に一致しない**。このままでは移行チェックが必ず失敗する。

### ステップA: HARD_WALL_LIMIT等の環境変数化

以下を環境変数化した(**すべて既定値は不変**)。

| 定数 | ファイル | 既定値 | 環境変数 |
|---|---|---:|---|
| `HARD_WALL_LIMIT` | ordering.py | 165.0 | `MYSOLVER_HARD_WALL_LIMIT` |
| `HARD_WALL_FACTOR` | ordering.py | 1.4 | `MYSOLVER_HARD_WALL_FACTOR` |
| `POLICY_HARD_WALL` | agent.py | 6.0 | `MYSOLVER_POLICY_HARD_WALL` |
| `RUN_ORDER_HARD_WALL`(旧`hard_wall`引数既定値) | replica.py | 6.0 | `MYSOLVER_REPLICA_RUN_ORDER_HARD_WALL` |
| `REPLICA_RESERVE_S` | ordering.py | 45.0 | `MYSOLVER_REPLICA_RESERVE_S`(既に対応済み、Phase35から) |

env化を見送ったもの(理由付き):
- `DEFAULT_TIME_BUDGET`(ordering.py、120.0): `agent.py`の`MYSOLVER_OPTIMIZE_BUDGET`経由で
  既に間接的にオーバーライド可能なため、定数自体の直接env化は不要と判断。
- `PER_STEP_TIME_BUDGET` / `MAX_VALIDATE_SLICE` / `FINAL_MARGIN` / `CONSTRUCT_SLICE` /
  `POLICY_TIME_BUDGET`: これらは「名目秒」であり`UNITS_PER_SEC`経由で決定的にユニットへ
  換算される値で、実際の打ち切りは消費ユニット数で決まる(**壁時計there自体には依存しない**、
  各定数のコメントに明記済み)。Phase17で「総予算に依存しない固定値であることが本質的」と
  意図的に設計されているため、env化して不用意に触れる余地を作らない判断とした。

**A-3検証**: 決定的8シーン(B01-B04, P04, A01-A03)、budget=30s、既定値のまま(env未設定)、
Step0時点のコミット(`b209253`)とのgit worktree比較で **8/8ビット単位一致**(CPU負荷のない
クリーンな環境で実施)。

### ステップB: policyテレメトリの4値化

`T_policy = 6.20 + 0.15×(2·any_success + wall_cut)` → 6.20/6.35/6.50/6.65。
`any_success`(そのシーンで複製評価が完走したか、`stopped=='done'`)を新たに
`ordering.LAST_ANY_SUCCESS`として記録し、`wall_cut`(既存の`LAST_BUILD_WALL_CUT`)と
組み合わせる。1-Bの2値実装を置き換えた。

**B-2検証**(local_eval.py、本番同様のサブプロセス経由、4分岐すべて実測):

| code | 期待T | シーン/設定 | 実測policy_time | 判定 |
|---:|---:|---|---:|---|
| 0 (any=0,cut=0) | 6.20 | P01(既積みあり、is_applicable=False) | 6.21s | OK |
| 1 (any=0,cut=1) | 6.35 | A06(既定設定、自然にwall_cut=True・評価は間に合わず) | 6.36s | OK |
| 2 (any=1,cut=0) | 6.50 | A07(既定設定、構築が余裕を持って完了) | 6.50s | OK |
| 3 (any=1,cut=1) | 6.65 | A06(`MYSOLVER_REPLICA_RESERVE_S=90`で人為的に評価枠を拡張) | 6.65s | OK |

4/4すべて設計どおり。timeout(8.0s)に対し最大6.65sで余裕1.35s。

**副産物(重要な観察)**: A02をB-2検証の初期試行で**既定設定のまま**流したところ、
以前(Step0-1、in-process計測)は`wall_cut=False`だったのに対し、今回の**subprocess経由**
実行では`wall_cut=True`(code=3, 6.65s)が観測された。同一シーン・同一既定設定でも、
実行コンテキスト(in-process直接呼び出し vs 本番同様のspawn subprocess)が変わるだけで
`hard_expired`の判定が反転しうることを直接示す実例であり、下記ステップDの認識と整合する。

### ステップB-3: 提出用zipのみHARD_WALL_LIMIT=155.0

判別の保険として、**zip内のordering.pyだけ**`HARD_WALL_LIMIT`の既定を155.0に下げた
(自然な壁時計上限を~161sへ落とし、162.00s以上のn=4帯を完全にクリーンにする)。
リポジトリの追跡ファイルは165.0のまま変更していない。

### ステップC: 壁時計非拘束ベースラインの再取得

```
MYSOLVER_HARD_WALL_LIMIT=3000 MYSOLVER_REPLICA_SELECT=0 MYSOLVER_UNITS_PER_SEC=2.00e7 \
  PYTHONPATH=. .venv/bin/python tools/measure_regime.py --config-path 'configs/gen/suite_*.json' \
  --module-path agents/mysolver/ --repeats 1 \
  --out results/phase38_baseline_off_nowall.json --label baseline_off_nowall
```

| 指標 | mean | max |
|---|---:|---:|
| fill_strict | 24.29 | — |
| fill_loose | 36.21 | — |
| composite_strict | 70.43 | — |
| composite_loose | 73.84 | — |
| optimize_time | 83.61s | **151.76s**(165s天井への張り付き解消、C-3確認済み) |
| policy_time | 0.473s | 1.89s |

総所要2479s(約41.3分)。**これがMac移行チェックの唯一の正しい期待値**
(`results/phase38_baseline_off_codespaces.json`は壁時計拘束ありの参考値として残す)。

suite平均(fill_strict/composite_strict)は拘束あり版とほぼ同値(24.29→24.29、70.43→70.43)
だったが、これは個々のシーンの値が変化していないことを意味しない——後述のとおり
シーン単位では壁時計コンテキストに応じて振れており、たまたま正負が打ち消し合って
suite平均が近い値になったに過ぎない可能性が高い(個別シーンの前後比較は本フェーズでは
未実施、次フェーズの課題)。

### ステップD: 決定性についての認識の格上げ

Phase36のoff側`optimize_time max`は154.69s、Phase38 1-Eは同一構成で165.01s——同じ設定で
10s以上ずれていた。ステップBの副産物(A02のwall_cutがin-process/subprocessで反転)、
ステップA-3・F-2で観測された「並行CPU負荷下では同一コードでも8シーン中1件が不一致」も
すべて同根の現象である。

1-F時点の分析は「`hard_deadline`が真の壁時計に依存するため、競合下でリスタート回数が
振れる」という**メカニズムの指摘としては正しかった**が、結論を「本フェーズの変更が
原因ではない」で止めていた。この結論は事実として誤りではないが、**問題を過小評価していた**。
正しい認識は次のとおり:

> **Phase17が確保したはずの決定性は、壁時計が律速する条件下では成立していない。**
> `total_budget.hard_deadline`(`planner.SearchBudget`)は真の壁時計(`time.perf_counter()`)
> に依存するため、(a) 実行コンテキスト(in-process/subprocess/RLIMIT_ASの有無)、
> (b) 同時実行中の他プロセスとのCPU競合、(c) 実行環境そのものの速度(Codespaces/Mac/
> サーマルスロットリング)のいずれによっても、**同一コード・同一シーン・同一設定で
> 結果が変わりうる**。これはユニット予算(`UNITS_PER_SEC`)側の決定性(Phase17の本来の
> 主張)を否定するものではなく、その決定性は「壁時計のhard_deadlineに一度も到達しない」
> という条件付きでしか成立しない、という限定がついていたことに気づけていなかった。

**恒久ルールに追加**(`docs/migration_to_mac.md`に記載):
「ローカル計測は必ず壁時計を非拘束(`MYSOLVER_HARD_WALL_LIMIT=3000`)で行う。壁時計が
律速する条件では同一コードでも結果が振れ、A/Bが成立しない。165.0は提出時のみ。」

理由: 移行先のMacは1.4GHz・8GB・サーマルスロットリングありで、壁時計律速が常態になる
可能性が高い。非拘束は「あれば便利」ではなく**必須条件**である。

### ステップE: results/の保全

`results/`配下の未追跡98件(phase12〜36の生ログ)を、追跡方針を変えずに1ファイルへ
アーカイブして保全した。

```
tar czf results_archive_phase12-36.tar.gz results/
```

サイズと内訳は次節「報告」参照。

### ステップF: zip再生成とpush

- `MYSOLVER_TELEMETRY`既定を`'1'`に固定、`HARD_WALL_LIMIT`既定を`'155.0'`に固定
  (**いずれもzip内のみ**、リポジトリ追跡ファイルは`'0'`/`165.0`のまま)
- 新SHA256は次節「報告」に記載(旧`77059c0a...`は破棄・未提出)
- zip内コードとリポジトリコードで決定的8シーン(budget=15s)のbuild_order出力が
  **クリーンな環境で8/8一致**することを確認済み

判定は次回提出の`optimization`実測値(162.00〜165.15sの帯、zip内では155.0天井のため
自然値との衝突は解消)と`policy`実測値(6.20/6.35/6.50/6.65のいずれか)が返ってきてから
行う。
