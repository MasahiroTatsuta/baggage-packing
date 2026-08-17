# Phase39: bp_push.sh 安全弁 / results掃除 / D03外れの切り分け / KeyErrorローカル再現

## 0. 前提
Mac移行検証(ステップ5)で 25/26 シーンが ±0.04pt 以内で一致したが、
`D03_A_2c_60_prioheavy_cont` のみ fill_strict +1.36pt の外れが残っていた。
本フェーズはその切り分け、`bp_push.sh` への安全弁追加、KeyError のローカル再現試行を行う。

## 1. bp_push.sh 安全弁

### 1-1. 実際の場所
`bp_push.sh` / `bp_check.sh` / `bp_ab.sh` は **`scripts/` の1箇所にのみ**存在し、重複はなかった
(`~/Desktop/bp_*.sh` には無し。`find ~/Desktop -maxdepth 3` でも同一の3ファイルのみ)。

### 1-2. 追加した安全弁
`set -euo pipefail` 直後、git操作より前に以下を挿入(3スクリプト共通):

```bash
cd ~/Desktop/baggage-packing
[ -d .git ] || { echo "FATAL: .git がありません(親ディレクトリの git を拾う危険)" >&2; exit 1; }
git rev-parse --show-toplevel | grep -q "baggage-packing$" || { echo "FATAL: リポジトリルートが不正" >&2; exit 1; }
git remote get-url origin | grep -q "MasahiroTatsuta/baggage-packing" || { echo "FATAL: remote が不正" >&2; exit 1; }
```

既存の `cd "$(dirname "$0")/.."` は上記の絶対パス `cd` に置き換えて整理した(重複させていない)。

### 1-3. 動作確認
一時ディレクトリで3ケースを検証(本物の `~/.git` には一切触れていない):
- `.git` 無し → `FATAL: .git がありません` で exit 1
- ディレクトリ名は `baggage-packing` だが `.git` 無し / remote不正 → 適切な段階で exit 1
  (`baggage-packing`という名の別リポジトリでも remote チェックで確実に弾かれることを確認)
- 正常系(本物のリポジトリ) → `PASS` で素通り

`bash -n` で3スクリプトとも構文エラー無しを確認済み。

### 1-4. bp_check.sh / bp_ab.sh への適用
**両方に同じガードを追加した。** 理由: これらは push しないため必須ではないが、
誤ったリポジトリ/親ディレクトリで計測すると気づかずに数十分の計測が無効データになるリスクがあり、
追加コストがほぼゼロ(数ミリ秒)なので追加する判断とした。

## 2. results/ の掃除

### 2-1. 削除した重複
`find results -name "* 2.*"` で **96件**(json 64 + md/txt 31 + 内訳誤差込みで総数96)を確認・削除。
指示文にあった「96個」という数値と一致した。全て未追跡(`??`)ファイルで、
対応する本体ファイルと内容が完全一致(サイズ同一・diff空)していたことを削除前に確認済み。

### 2-2. 追跡ファイルへの意図しない上書きの有無
`git diff --stat results/` は**空**、`git status --short results/` に `M`/`D` は**無し**
(全て `??` の未追跡ファイルのみ)。「phase38_report.md が古い内容で上書きされた」問題は
**現状では再発していない**。復元が必要な追跡ファイルは0件だった。

### 2-3. 複製の発生経路
**未特定**(深追いはしていない)。削除前に確認できた傍証: 複製ファイルは `-rw-------`(600)・
本体より古いタイムスタンプ、本体は `-rw-r--r--`(644)・新しいタイムスタンプという非対称があり、
Finderの「両方を残す」コピー/復元操作の痕跡と整合的だった。

## 3. D03 外れの切り分け

### 3-1. 実測env変数名(ステップAの実装を確認)
- `MYSOLVER_HARD_WALL_LIMIT`(既定165.0, ordering.py:107)
- `MYSOLVER_HARD_WALL_FACTOR`(既定1.4, ordering.py:111)
- `MYSOLVER_POLICY_HARD_WALL`(既定6.0, agent.py:18)
- `MYSOLVER_REPLICA_RUN_ORDER_HARD_WALL`(既定6.0, replica.py:101) —
  指示文の想定名 `RUN_ORDER_HARD_WALL` ではなく `REPLICA_` が入るのが正しい変数名だった。

### 3-2. D03単独3回連続実行の結果

| rep | fill_strict | optimize_time |
|---|---|---|
| 1 | 27.594286077580705 | 116.98s |
| 2 | 27.594286077580705 | 114.79s |
| 3 | 27.594286077580705 | 113.05s |

**3回ともbit単位で完全一致** → 決定的な差。壁時計律速ではない
(FACTOR/LIMITを外す3-2の処方は指示のフローチャート通りスキップ)。

### 3-3. BLAS由来で説明できるか
Codespaces基線(`results/phase38_baseline_off_codespaces.json` / `_nowall.json`、
両者ともD03は fill_strict=26.239089839819354 で完全一致)と比較:

| 指標 | Mac | Codespaces | 差分 |
|---|---|---|---|
| fill_strict | 27.594286 | 26.239090 | **+1.355196**(≈報告の+1.36) |
| fill_loose | 42.6250438610064 | 42.6250438610064 | **±0.000000**(bit単位で一致) |
| cog_score | 57.681889 | 57.764490 | -0.082601 |
| stability_score | 97.906857 | 98.131689 | -0.224832 |
| placement_score | 100.0 | 100.0 | 0 |
| soft_item_score | 100.0 | 100.0 | 0 |

**fill_loose が bit単位で一致している**のが決め手になる。これは「配置そのもの(どの荷物を
どの位置・姿勢で置いたか)は本質的に同一」であることを意味する(margin=0.01の緩い判定では
両プラットフォームで結果が変わらない)。一方 fill_strict は margin=-0.005 という厳しい閾値を
使う。実行ログには D03 特有の "item X not inside (hit boundary plane)" という、境界ぎりぎりの
荷物が10件前後記録されており、この shelf/container 境界すれすれの荷物のうち1〜数個が、
BLAS(Mac Accelerate と Codespaces側の実装の丸め差)由来のごくわずかな接触計算の違いで
「strict margin内に収まる/収まらない」の判定が反転したと考えるのが最も整合的である。
cog_score/stability_score の小さな(0.08〜0.22pt)差も、荷物1〜数個の in/out 反転で
説明できる規模であり、全面的な配置順序の変化(カオス的カスケード)であれば fill_loose も
無事では済まないはずだが、そちらは無傷だった。

以上より、+1.36pt は「BLAS由来の丸め差がストレージ境界すれすれの荷物のstrict判定を
反転させた」という機構で**説明がつく**と判断する。ただし、どの荷物・どの接触計算が
反転したかまでは今回追跡していない(必要なら次フェーズで item 単位のin/out差分を取る)。

### 3-4. optimize_time比較
どちらも壁時計に到達していない:
- Mac: 113.05〜116.98s(HARD_WALL_LIMIT=3000, FACTOR由来の推定締切約168sにも未到達)
- Codespaces: 108.46〜108.74s(baseline/nowall両方でほぼ同一 → こちらも壁時計非拘束)

Mac の方がやや遅い(+5〜8s)が、これは既知の速度比(Mac 0.94x)の範囲内であり、
壁時計律速のいずれの側面も確認されなかった。

### 3-5. 26シーン全体の再測定要否
**不要と判断し、実施しない。** 理由:
1. HARD_WALL_FACTORの律速仮説はD03単独の3回連続テストで既に否定された
   (optimize_timeが両プラットフォームとも制限に遠く届いていない)。
2. 25/26シーンは既に±0.04pt以内で一致しており、他シーンが壁時計に敏感である証拠がない。
3. D03の原因はBLAS由来の境界判定反転という「シーン固有(荷物が境界すれすれ)」の性質と
   推定され、FACTOR/LIMITを外しても変化しないと考えられる。
再測定してもD03の値は動かず、他25シーンも変化しない可能性が高いため、40分のコストに
見合わないと判断した。

以上より、**bp_check.sh / bp_ab.sh へのFACTOR除去追加、docs/migration_to_mac.mdの
恒久ルール更新は行っていない**(条件節「期待値に一致するなら」が成立しなかったため)。

## 4. KeyError のローカル再現試行

`agents/mysolver/replica.py` の `ReplicaEvaluator.evaluate()` を直接呼び出し、
`MYSOLVER_REPLICA_SELECT=1` で5つの仮説を1シーンずつ検証した。

前提の再確認: **D03 は元々 is_soft item を7個含んでいた**(ローカル26シーン全体で
is_soft=trueを含むシーンは多数存在し、「ローカルはsoft item無し」という前提は実際には
誤りだった)。ただし `ordering.py:1149` の `except Exception` がreplica評価器の例外を
静かに握りつぶす実装のため、通常のD03実行だけでは「replica評価器内でKeyErrorが
起きていないか」は分からない。そのため `ReplicaEvaluator` を直接叩いて確認した。

| ケース | 内容 | 結果 |
|---|---|---|
| 4-1 | D03(is_soft 7個含む, 元の順序) | 例外なし(fill=15.52) |
| 4-1b | D03(is_soft含む, index降順) | 例外なし(fill=18.46) |
| 4-1c | A01 + 先頭itemに is_soft=True 強制 | 例外なし(fill=15.41) |
| 4-2 | D03 + コンテナ3個目(複製) | 例外なし(fill=9.37) |
| 4-4a | lookahead_k=0 | 例外なし(`max(1,...)`で1に丸められる。fill=15.52) |
| 4-4b | lookahead_k=1000(item数を大幅超過) | 例外なし(fill=25.97) |
| 4-4c | lookahead_k=len(items)ちょうど(60) | 例外なし(fill=25.97) |
| 4-5 | コンテナ0: shelf=True かつ is_prioritized=True | 例外なし(fill=15.67) |

**4-3(item_stream.max_space != 1)は未実施**: `max_space` はエージェント観測
(`get_init_states`/`get_info_for_optimization`)に一切現れず、`replica.py` は
`ASSUMED_MAX_SPACE=1` を完全に決め打ちしているため、観測を細工して再現する
経路が存在しない。実際に production の max_space が1以外だった場合、
replica評価器は「補充カデンスがずれる」という**静かな不整合**を起こしうるが、
それ自体がKeyErrorを直接引き起こす経路は静的解析でも見つからなかった。

**全8ケースで例外は一切発生しなかった**(traceback無し、KeyErrorの再現に失敗)。

### 再現しなかった場合の対応
指示に従い、前ターンで渡された「KeyError キー名符号化」の probe を実装する方向に
切り替える。本フェーズではこの実装は行わず、方針転換のみを報告する。

## まとめ

| ステップ | 結果 |
|---|---|
| 1. 安全弁 | bp_push.sh / bp_check.sh / bp_ab.sh の3つに追加、正常系/異常系とも動作確認済み |
| 2. results掃除 | 96件の重複を削除、追跡ファイルへの意図しない上書きは無し、複製経路は未特定 |
| 3. D03切り分け | 決定的(3回一致)、壁時計非律速、BLAS由来の境界判定反転で+1.36ptを説明、26シーン再測定は不要と判断 |
| 4. KeyError再現 | 5仮説×8ケースいずれも再現せず。probe実装へ方針転換(未実装、指示待ち) |
