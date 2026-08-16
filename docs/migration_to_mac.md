# Codespaces → Mac 移行手順書(Phase38 ステップG)

Codespacesはいずれ消える。このリポジトリだけから、Mac側で同じ開発サイクル(仮説→実装→
26シーンA/B→提出)を再開できることを保証するための手順書。

---

## 1. clone

```bash
git clone git@github.com:MasahiroTatsuta/baggage-packing.git
cd baggage-packing
```

SSH鍵をMacに登録していない場合はHTTPS版:
```bash
git clone https://github.com/MasahiroTatsuta/baggage-packing.git
```

---

## 2. venv構築

```bash
python3 -m venv .venv
source .venv/bin/activate
pip install --upgrade pip
```

依存パッケージの正確なバージョンは `env_snapshot.txt`(リポジトリ直下、Codespacesの
環境を記録したもの)を参照すること。**torchは入れない**(このリポジトリのどこからも
`import torch` されていない。requirements的なファイルが無いプロジェクトなので、
`env_snapshot.txt` の `pip freeze` 出力から主要パッケージだけを抜いてインストールする)。

```bash
pip install $(grep -E '^(numpy|scipy|pybullet|gymnasium|pillow|Pillow)==' env_snapshot.txt)
```

**注意(numpyのBLAS)**: LinuxのCodespacesはOpenBLASでビルドされたnumpyを使っている
可能性が高いが、macOSは標準でAccelerateフレームワークを使う。`numpy.show_config()` の
出力が env_snapshot.txt に記録済みなので、Mac側でも同じコマンドを打って**BLAS実装が
違うことを認識した上で**数値比較すること(浮動小数点の丸め方がわずかに変わりうる)。

---

## 3. 恒久ルール: ローカル計測は壁時計を必ず非拘束にする

```bash
export MYSOLVER_HARD_WALL_LIMIT=3000
```

**165.0(既定値)は提出時のみ使う値。** Mac(1.4GHz・8GB・サーマルスロットリングあり)は
壁時計律速が常態になりうる環境で、`agents/mysolver/ordering.py` の
`total_budget.hard_deadline` は真の壁時計(`time.perf_counter()`)に依存するため、
壁時計が律速する条件下では**同一コード・同一シーンでも実行のたびに結果が振れる**
(Phase38ステップD、詳細は [`results/phase38_report.md`](../results/phase38_report.md) §D)。
非拘束にしないままA/Bを取ると、コードの差ではなく実行タイミングの差を「効果」と
誤認するリスクがある。

同様に `MYSOLVER_POLICY_HARD_WALL`(既定6.0)・`MYSOLVER_HARD_WALL_FACTOR`(既定1.4)も
環境変数化済み(Phase38ステップA)。通常はHARD_WALL_LIMITだけ触れば足りる。

---

## 4. デスクトップスクリプト3本

`scripts/` 配下に用意済み。プロジェクトルートで実行する。

- **`bp_check.sh`**: 移植後の健全性チェック。決定的8シーン(B01-B04, P04, A01-A03)の
  `build_order` 出力を壁時計非拘束で表示し、軽量スモーク(A01, budget=15s)を1本流す。
  副作用なし(resultsに書かない)。まず最初にこれを通すこと。
- **`bp_ab.sh <label> ["ENV1=V1 ENV2=V2"]`**: 26シーンA/B(off=`MYSOLVER_REPLICA_SELECT=0`
  固定 / on=`MYSOLVER_REPLICA_SELECT=1`+追加env)を壁時計非拘束・`UNITS_PER_SEC=2.00e7`
  (提出既定)で実行し、`composite_strict` の対応ありt検定(mean/std/SE/t、−2.0pt超の
  悪化シーン一覧)まで自動で出す。結果は `results/bp_ab_<label>_{off,on}.json`。
- **`bp_push.sh "<message>" <file1> [file2 ...]`**: 指定したファイルだけを
  `git add -f` してcommit・push し、`origin/main` のresults/追跡ファイル数と
  (zip引数があれば)SHA256の一致を検証する。**`results/` をディレクトリ丸ごと
  addする呼び出しはできない設計**(過去分を遡って追加しない方針を機械的に守るため)。

3本とも `bash -n` の構文チェック済み。実行前に `chmod +x scripts/*.sh` は不要
(リポジトリに実行ビット付きでコミット済み)。

---

## 5. 移行チェックの期待値

`results/phase38_baseline_off_nowall.json`(壁時計非拘束・`REPLICA_SELECT=0`・
`UNITS_PER_SEC=2.00e7`、26シーン)が**唯一の正しい期待値**。
壁時計拘束ありの `results/phase38_baseline_off_codespaces.json` は参考値としてのみ
残しており、移行チェックには使わないこと(拘束条件が違うため原理的に一致しない)。

| 指標 | mean | max |
|---|---:|---:|
| fill_strict | 24.29 | — |
| fill_loose | 36.21 | — |
| composite_strict | 70.43 | — |
| composite_loose | 73.84 | — |
| optimize_time | 83.61s | 151.76s(HARD_WALL_LIMIT非拘束、165s天井に張り付かないことを確認済み) |
| policy_time | 0.473s | 1.89s(timeout=8s) |

(REPLICA_SELECT=0固定・UNITS_PER_SEC=2.00e7・26シーン、Phase38ステップC実測)

Mac側で `bash scripts/bp_ab.sh migration_check` 相当(off側のみ)を実行し、
`fill_strict` / `composite_strict` の mean が上表と**大きくずれていれば**(ノイズ床は
fill±0.90pt程度、Phase10実測)、移植ミスかもしれないので先に疑うこと。多少のずれ
(BLAS差・pybulletバージョン差による物理演算の丸め誤差蓄積)は許容範囲内である
可能性があるため、まず`bp_check.sh`の決定的8シーンが一致するかを先に見る
(こちらはpybullet物理演算を経由しない`build_order`のみのテストなので、
一致すれば少なくとも探索ロジック側は移植できている)。

---

## 6. Codespacesのスペック(比較基準)

| 項目 | 値 |
|---|---|
| CPU | AMD EPYC 7763(2 vCPU) |
| メモリ | 7.8GB(スワップ無し) |
| optimize_time(壁時計拘束あり、REPLICA_SELECT=0、26シーン) | mean 87.86s / max 165.01s(Phase38 1-E) |
| optimize_time(壁時計非拘束、REPLICA_SELECT=0、26シーン) | mean 83.61s / max 151.76s(Phase38 ステップC) |

Mac側でこの表と桁違いに遅ければ(例: optimize_timeのmeanが2倍以上)、サーマル
スロットリングか、venvのnumpy/pybulletが想定と違うビルド(BLAS未最適化など)である
可能性を疑うこと。

---

## 7. results/の過去ログ保全

Codespaces時代の生ログ(phase12〜36、98ファイル)は `results-dir-tracking-policy` により
個別にはgit追跡していない。Codespaces削除で失われないよう、1ファイルにアーカイブして
`results_archive_phase12-36.tar.gz` としてリポジトリに追加した(Phase38ステップE)。

```bash
tar xzf results_archive_phase12-36.tar.gz
```

で `results/` 配下に展開できる。**このアーカイブを展開したからといって、それらのファイルを
個別にgit追跡対象へ昇格させるわけではない**(方針は変えていない、あくまで保全目的)。

---

## 8. 既知のリスク(Mac固有)

- `src/`(agents/mysolver・toolsを除く評価基盤側)は本番評価基盤の写しであり、
  **Linux側の挙動を変える修正は禁止。** ただしmacOS固有の実行不能問題に対しては、
  **Darwin限定かつLinuxではno-opとなる形の修正のみ許容する。** 該当箇所は必ず
  単独コミットにし、このセクションに一覧で記録すること。
  - `src/ground_handling/runner.py`: `resource.setrlimit(resource.RLIMIT_AS, ...)`が
    子プロセス起動時に呼ばれる。macOSはdyld共有キャッシュ等が起動時から広大な
    仮想アドレス空間を予約するため、RLIMIT_ASを16GB未満に設定しようとすると
    `ValueError: current limit exceeds maximum limit`で失敗する(Phase38ステップG
    Mac側で実機確認、0.5GB〜16GBはすべて失敗・100GBはOK)。既定の`max_mem=4`は
    この範囲に収まるため、パッチ前は子プロセスが起動直後に`ValueError`未捕捉で
    即死していた(`except ImportError`はValueErrorを捕まえない)。
    **Darwin限定でValueErrorも握りつぶすよう修正済み**(Linux側は従来どおり例外を
    送出し、挙動はバイト単位で不変)。影響がないことの裏付け: Phase38 §0.3の
    VmPeak実測は最大265MBで、4GB制限はLinuxでも一度もbindingしていない。
  - `注記`: 本ファイルはgit追跡されている(過去の「非追跡」という記述は誤り)。
    「変更禁止」の趣旨はLinux側の挙動保持であり、Darwin限定no-op修正は対象外。
- `agents/mysolver/replica.py` の `MYSOLVER_REPLICA_VMLOG=1`(VA診断、既定無効)は
  `/proc/self/status` を読むが、`try/except Exception` で既に保護されており、
  macOSで有効化しても例外は握りつぶされて「read failed」と表示されるだけで
  クラッシュはしない(Phase38ステップG-4で確認済み、コード変更は不要だった)。
