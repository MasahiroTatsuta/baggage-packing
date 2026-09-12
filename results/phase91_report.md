# Phase91 報告: 未解決事項の決着と防御フェーズへの移行

**実施日: 2026-09-02。** コード変更なし・新規zip作成なし。記録の追記とドキュメント
更新のみ。主枠(`mysolver_submit_cc_rcl_w3.zip`)・2枠目(`mysolver_submit_rest020.zip`)
は変更していない。

## 先出し結論

1. **safexy026 は未解決ではなく決着済みだった。** public 53.62(緩2 57.185 比 −3.57)。
   `SAFETY_MARGIN_XY` は両側悪化の山型で既定 0.022 が頂点、**不採用**で確定。
   総括表 #6 を「保留」→「不採用」に修正。総括表の他の行に本番結果の転記漏れは無し。
2. **他戦略へのフェーズ1予算配分は着手しない。** w2=−18 という実測、ローカル信号の
   非転移性、限界収益の逓減、機会費用から、**探索フェーズの完全終了を宣言する。**
3. **防御フェーズ(現物確認)を主枠 w3・2枠目 rest020 に対して再実施。** SHA256 一致・
   全定数意図どおり・決定的8シーンで zip版とリポジトリ版が **両zipとも 8/8 ビット単位
   一致**・差分ファイルは全て意図どおり。唯一の未確認は SIGNATE 画面での実選択の
   目視突合(ユーザー操作必須)。

---

## ステップ1: safexy026 の決着(記録訂正)

### 結果

| zip | `SAFETY_MARGIN_XY` | public | num_placed_items | 緩2(57.185)比 |
|---|---:|---:|---:|---:|
| `mysolver_submit_safexy018.zip` | 0.018 | 54.00 | — | −3.19 |
| **既定(緩2 = 主枠土台)** | **0.022** | **57.185** | 0.6434 | (基準) |
| `mysolver_submit_safexy026.zip` | 0.026 | **53.62** | **0.5965** | **−3.57** |

- `safexy026.zip` の SHA256 `6eb91e020fff8402925f4571ddae0dcca2b8109937f97ddb6ccf44ae476e39ac`
  は現物と一致(Phase76-77 記録どおり)。zip 内 `geometry.py` の
  `SAFETY_MARGIN_XY` 既定は `'0.026'` で焼き込み済みを確認。
- `SAFETY_MARGIN_XY` は **両側悪化の山型**(0.018 で −3.19、0.026 で −3.57)。
  既定値 0.022 が最適点。**不採用で確定。**

### 訂正した記録

- `docs/submission_policy.md` §5「幾何定数の探索」行: 「`safexy026` の本番結果待ち」
  → 「不採用(両側悪化を確認、既定 0.022 が頂点)」+ カーブを明記。
- `docs/submission_policy.md` に §13-1 を新設し、結果と経緯を記録。
- `results/phase90_report.md` 総括表 #6: 「本番結果が記録に見当たらず(未解決)/保留」
  → 「public 53.62(−3.57)/不採用(両側悪化を確認、0.022が頂点)」。
- `results/phase90_report.md` §3-3 の「すぐ確認すべき未解決事項」に決着注記を追記。

### 総括表の他の行の点検(指示 1-2)

`results/phase90_report.md` §3-2 の総括表 #1〜#20 を確認した。**本番結果が存在するのに
表へ未反映だった行は #6 のみ。** 他の行:

- 本番提出済みで結果反映済み: #1, #3, #4, #5, #7, #8, #9, #10, #11, #12, #13, #18, #19
- 本番未提出(ローカルのみ): #15(look-ahead), #17(BRKGA), #20(window内訳)
- 未実装: #16(MCTS)
- 調査のみで軸として存在せず: #14(online policy)

補足(表の行ではない小さな抜け): Phase74 の `mysolver_submit_c0300.zip`(centroid 0.300)
の本番結果は Phase75 報告に転記されていない。ただし centroid のピークは
0.225→57.06 / 0.25→57.18 / 0.275→56.94 のクリーンな山で既に確定しており、
c0300(更に緩めた側)は降下の確認にしかならないため、実害のない記録の抜けとして
留める(このセッションに c0300 の本番結果自体が無いため追記もできない)。

---

## ステップ2: 他戦略へのフェーズ1予算配分

### 判断: 当初「着手しない」→ ユーザー要請で実施 → 実測で否定

Phase90 (3-3) が挙げた「体積優先(`strategy_orders[0]`)以外の3戦略
(count_first / big_first / layer_first)にもフェーズ1並みの決定的構築予算を与える」案。
当初は下記の理由で「着手しない」と判断したが、ユーザーの要請で最小実験を実施した。

### 実測結果(2026-09-02)

`agents/mysolver/ordering.py` に `MYSOLVER_PHASE1_EXTRA_STRATEGY`(既定 `''` = 無効、
`bp_check.sh` 8/8 ビット一致で無害を確認)を追加。w3 の3本のフェーズ1 window
(15,25,None)の**最後の1本の種**を体積優先から他戦略に差し替え、26シーン+sample_config
(28件)を本番相当120秒予算で実測。対照は Phase88 の w3 実測(751/1558)。

| 戦略 | w3一致 | 改善 | 悪化 | ネット | 判定 |
|---|---:|---:|---:|---:|:---:|
| count_first | 27/28 | 0 | C03 −6 | **−6** | ❌ |
| big_first | 27/28 | 0 | C03 −6 | **−6** | ❌ |
| layer_first | 27/28 | 0 | C03 −6 | **−6** | ❌ |

**3戦略が1個単位で完全同一**(C03 の −6 まで一致)。含意:

1. **フェーズ1では体積優先が28シーン全てで優越。** どの並び順を種にしても追加リスタートは
   一度も勝たず、最終採用順序は常に体積優先のまま(27シーン完全一致)。`ordering.py`
   Phase9 のコメント(「他戦略を混ぜると回帰する」)が cc_rcl+w3 土台でも成立。
2. **C03 の −6 は戦略の中身と無関係な副作用。** 3本目のリスタートが体積優先と違うだけで
   予算消費がずれ、フェーズ2の乱数リスタートの結果が変わる(Phase16 の予算非単調性)。
   戦略を変えても −6 が動かないのがその証拠。この機構は「無害な足し算」ですらない。

**→ この軸は実測で死んでいる。** `MYSOLVER_PHASE1_EXTRA_STRATEGY` は
`BEAM_SOFT_LAST` 等と同じく env ゲート・既定無効のまま失敗の記録として残す。
`docs/submission_policy.md` §13-2 に反映。生データ: `scratchpad/sweep_{count,big,layer}_first.json`。

### (当初の)着手しない理由 — 実測はこれを裏付けた

1. **手元の唯一の実測がこの方向に不利。** w2(体積優先フェーズ1を3本→2本に削減)は
   ローカル −18(Phase88-89)。最小実験(2-2 が例示した「体積優先を2本に減らし
   1本を別戦略へ」)は、その −18 の体積優先劣化を**先払い**した上で、未検証の別戦略の
   1リスタートがそれ以上を取り戻すことに賭ける。軸単位の過去最大利得(w3 の net+34、
   しかもフル再チューニングの結果)を天井と見ても、外来の単発リスタートが実質
   net+18 超を出す見込みは薄く、ローカルにその兆候を出す手段も無い。
2. **ローカル信号では判定できない。** Phase70(loose2: ローカル劣位→本番勝利)、
   Phase80(BEAM_SOFT_LAST: ローカル +0.89pp 予測→本番 −0.97pp)、Phase89-90
   (w4/comp_1520: net 正でも悪化シーン多→本番で符号反転 or 未達)と、弱い信号は
   本番転移せず符号すら反転する事例が積み上がっている。着手を正当化するには
   「悪化シーン数が少ない強い信号」が必要だが、Phase90 は「15を残す」という遥かに
   制約の強い部分仮説でさえ、頑健性の基準を満たす候補を作れなかった。新しいソート順
   戦略は原理的な制約が更に緩く、また「弱く高リスクな候補」に終わる公算が高い。
3. **限界収益が明確に逓減。** Phase73 以降 20 軸のうち本番で明確に効いたのは
   支持閾値緩和 / cutcorner / rcl / フェーズ1優先(w3)の4軸のみ。直近3フェーズ
   (88-90)の探索(window 数・内訳)は全て本番またはローカルで否定的。「フェーズ1
   優先」の洞察は w3 で既に +0.349 を回収済みで、同じ発想を別戦略へ再適用するのは
   二次的な賭けにすぎない。
4. **機会費用と下振れ。** 実装+検証で半日〜1日、加えて本番検証は提出枠と約1.5時間の
   反映待ちを消費する。残り約5週間は、期待値 +0.2 程度の上積みを狙うより、
   防御(最終選択・パッケージング事故の防止)に充てる方が妥当。主枠は public 57.894、
   保険枠は REST_CLEARANCE 軸という独立の故障モードを持ち、現在地は追加の攻めを
   要求していない。

賛成側の論拠(3戦略はソート順そのものが違い、window とは質的に異なる多様性。
cutcorner×rcl の加法性に近い)は妥当性がある。しかし cutcorner/rcl が加法的に効いた
のは各々が**安価**——下振れが小さく、ほぼ無リスクで試せた——からであって、
新戦略は体積優先の予算を必ず削るため安価ではない。機序が独立でも、参入料
(w2 相当の −18)が想定利得を先に食う構図では期待値が立たない。

### この軸の決着と次の候補

**「他戦略へのフェーズ1配分」は決着(3戦略とも実測でネット −6・改善ゼロ)。**
Phase73〜91 の 21 軸で本番寄与したのは支持閾値緩和 / cutcorner / rcl / フェーズ1優先(w3)の
4軸のみ。主枠 w3・2枠目 rest020 は変更なし。

**次の候補(#1、zip 作成済み・本番結果待ち)**: オンライン `policy()` が毎回
`POLICY_HARD_WALL`(6.0s)で打ち切られている(Phase84)ため、これを 6.5s に上げて
全配置判断の探索を深くする。

- `submissions/mysolver_submit_w3_pw65.zip`、SHA256
  `4ddc33defa3c4028c6389bdbba414c944471e54f0f2f749079e5e1e501dd2e1b`
- w3 zip を展開して `agent.py` の `MYSOLVER_POLICY_HARD_WALL` 既定を `'6.0'→'6.5'` の
  **1行だけ**書き換えて再圧縮。他10ファイルは w3 zip とバイト単位一致。
- 決定的8シーンの `build_order` は **8/8 ビット単位で w3 と一致**(`POLICY_HARD_WALL` は
  オフラインでは不使用)。全定数 grep も w3 と同一 + 壁時計のみ変更。
- **ローカルでは判定不能**(ローカル policy 時間 ~0.1s、壁に触れない)。本番提出でのみ判定。
- 判定基準・#2〜#4 は `docs/submission_policy.md` §13-5。

**本番結果(2026-09-02)**: public **57.89384**(w3 57.894 と3桁で同一=前後)、
num_placed_items 0.6443、policy 時間 6.172s(w3 の ~6.05s から +0.12s)。
**壁を 6.5s に上げても policy は 6.17s で自然完了**(6.5s の壁には未到達)——従来の
6.0s 壁は探索を約0.12s 早く切っていただけで、その分の追加探索はスコアを動かさなかった。
**判定: null。#1 不採用、#2 も見送り(探索時間の限界価値がゼロと直接判明)。
§13-5 の系統は打ち切り。** `pw65.zip` は失敗の記録として保管、主枠 w3 は無変更。

---

## ステップ3: 防御フェーズ(現物確認)

過去の報告を信用せず、この時点の zip 現物に対してゼロから再実行した(Phase85 の再実施)。
検証スクリプト: `verify_zip.py`(scratchpad、`build_order` の決定的8シーン出力を
zip展開版とリポジトリ版で別プロセス実行して突合。リポジトリ版には zip が焼き込んだ
env 既定値に一致する環境変数を明示指定)。`time_budget=30.0`、`MYSOLVER_HARD_WALL_LIMIT=3000`。

### (3-1) SHA256 の再計算と照合

| zip | 再計算した SHA256 | 記録 | 一致 |
|---|---|---|:---:|
| `mysolver_submit_cc_rcl_w3.zip`(主枠) | `a7bdef7ee08fa4386d54b4be9938cabf31354407608888640f788e18533cd48a` | §5・§11(Phase89)・`phase89_report.md` | ✅ |
| `mysolver_submit_rest020.zip`(2枠目) | `e13ee35996c5bcc6819f62657b30d271521abe8efcae6c0b2fb0380238e3ff12` | §5・§6(Phase85)・`phase76/77_report.md` | ✅ |

### (3-2) 全定数の grep

zip を展開したソースから `os.environ.get(...)` の既定値を直接確認した。

**主枠 w3**(`mysolver/`):

| 定数 | 値 | 期待 |
|---|---:|---|
| `MIN_UNION_SUPPORT_RATIO` | 0.35 | 緩2 ✅ |
| `MIN_SUPPORT_SPAN_RATIO` | 0.4 | 緩2 ✅ |
| `MAX_SUPPORT_CENTROID_OFFSET` | 0.25 | 緩2 ✅ |
| `INCLUSION_MARGIN` | −0.012 | 既定 ✅ |
| `SAFETY_MARGIN_XY` | 0.022 | 既定 ✅ |
| `REST_CLEARANCE` | 0.016 | 既定 ✅ |
| `MYSOLVER_PHASE1_WINDOWS`(既定) | `15,25,None` | **w3 ✅** |
| `MYSOLVER_PHASE1_SLICE_S`(既定) | `33.333333333333336` | **w3 ✅** |
| `MYSOLVER_PHASE1_ALLOW_PARTIAL`(既定) | `0` | 既定 ✅ |
| `CUTCORNER_CANDIDATES` | `1` | cutcorner有効 ✅ |
| `CUTCORNER_N_Y_SAMPLES` | 5 | 既定(cc_strongではない)✅ |
| `_RCL_SHUFFLE` / `_RCL_FRACTION` | `1` / 0.3 | rcl有効・k既定 ✅ |
| `_BEAM_SOFT_LAST` | `0` | 既定無効 ✅ |
| `REPAIR` / `ALNS` / `BRKGA` | `0` / `0` / `0` | 全て無効 ✅ |
| `WALL_MODE` / `DFTRC_STRATEGY` | `0` / `0` | 無効 ✅ |
| `LOOKAHEAD_DEPTH` | `0` | 無効 ✅ |
| `MYSOLVER_TELEMETRY` / `TELEMETRY_PHASE2` | `0` / `0` | 無効 ✅ |
| `HARD_WALL_LIMIT` | 165.0 | 既定 ✅ |
| `REPLICA_SELECT` / `REPLICA_METRIC` | `0` / `fill` | 既定 ✅ |
| `FALLBACK_SAFE_POS` / `FALLBACK_AVOID_OBSTACLES` | `1` / `1` | 既定 ✅ |
| `STRICT_SUPPORT_DISABLE` | `0` | 既定 ✅ |

**2枠目 rest020**(`mysolver/`、Phase77時点のスナップショット):

| 定数 | 値 | 期待 |
|---|---:|---|
| `MIN_UNION_SUPPORT_RATIO` / `SPAN` / `CENTROID` | 0.35 / 0.4 / 0.25 | 緩2 ✅ |
| `INCLUSION_MARGIN` | −0.012 | 既定 ✅ |
| `SAFETY_MARGIN_XY` | 0.022 | 既定 ✅ |
| `REST_CLEARANCE` | **0.020** | **rest020 ✅** |
| `REPAIR` / `ALNS` / `WALL_MODE` | `0` / `0` / `0` | 無効 ✅ |
| `HARD_WALL_LIMIT` | 165.0 | 既定 ✅ |
| `REPLICA_SELECT` / `REPLICA_METRIC` | `0` / `fill` | 既定 ✅ |
| `FALLBACK_SAFE_POS` / `FALLBACK_AVOID_OBSTACLES` | `1` / `1` | 既定 ✅ |
| `STRICT_SUPPORT_DISABLE` | `0` | 既定 ✅ |
| Phase78以降のフラグ(cutcorner/rcl/lookahead/brkga/beam_soft_last/telemetry_phase2/phase1_windows) | **存在しない** | Phase77 スナップショットとして正 ✅ |

### (3-3) 決定的8シーンのビット単位検証(zip版 vs リポジトリ版)

| シーン | 主枠 w3(zip vs repo) | 2枠目 rest020(zip vs repo) |
|---|:---:|:---:|
| B01 (1c,40,plain) | MATCH | MATCH |
| B02 (1c,40,shelf) | MATCH | MATCH |
| B03 (2c,80,prio) | MATCH | MATCH |
| B04 (2c,80,noprio) | MATCH | MATCH |
| P04 (1c,pre8,shelf) | MATCH | MATCH |
| A01 (1c,40,plain) | MATCH | MATCH |
| A02 (1c,80,plain) | MATCH | MATCH |
| A03 (1c,40,shelf) | MATCH | MATCH |
| **合計** | **8/8 ビット単位一致** | **8/8 ビット単位一致** |

zip 展開版(`mysolver` パッケージ、env 未指定=焼き込み既定)と、現行リポジトリ版
(`agents.mysolver`、zip の焼き込み値に一致する環境変数を明示指定)を別プロセスで
実行して `build_order` 出力を比較。**両zipとも 8/8 で完全一致。** zip ファイル自体の
破損・改変が無く、Phase82〜90 のリポジトリ変更がこれら2zipの依拠ロジックに影響して
いないことを実測確認した(Phase85 と同じ結論を現時点で再現)。

### (3-4) リポジトリ追跡ファイルとの差分

**主枠 w3** — 3ファイル・計7行のみ差分、いずれも env 既定値の焼き込み:

| ファイル | 差分行 |
|---|---|
| `planner.py` | `MIN_UNION_SUPPORT_RATIO` `0.55→0.35` / `MIN_SUPPORT_SPAN_RATIO` `0.6→0.4` / `MAX_SUPPORT_CENTROID_OFFSET` `0.15→0.25` / `CUTCORNER_CANDIDATES` 既定 `'0'→'1'` |
| `ordering.py` | `_PHASE1_WINDOWS_ENV` 既定 `''→'15,25,None'` / `PHASE1_SLICE_S` 既定 `str(CONSTRUCT_SLICE)→'33.333333333333336'` |
| `simulate.py` | `_RCL_SHUFFLE` 既定 `'0'→'1'` |

`geometry.py` は現行リポジトリと**完全一致**(幾何既定値 −0.012 / 0.022 / 0.016)。
`__init__.py` / `agent.py` / `alns.py` / `reach.py` / `replica_scorer.py` / `replica.py`
も完全一致。**意図しない変更なし。**

**2枠目 rest020** — 4ファイル差分:

| ファイル | 内容 |
|---|---|
| `geometry.py` | `REST_CLEARANCE` 既定 `0.016→0.020`(意図どおり)+ Phase78 以降に追加された cutcorner 関連関数が**無い**分の差 |
| `ordering.py` / `planner.py` / `simulate.py` | Phase78 以降の機能追加(cutcorner / rcl / lookahead / brkga / beam_soft_last / phase1 windows / telemetry_phase2、いずれも既定無効)が rest020 側に無い分の差 |

REST_CLEARANCE 以外の差分は全て「後発の機能追加(既定無効)」であり「既存ロジックの
変更」ではない。これを (3-3) の 8/8 ビット一致が裏づける。`__init__.py` / `agent.py` /
`alns.py` / `reach.py` / `replica_scorer.py` / `replica.py` は完全一致。
**意図どおり(異常なし)。**

### (3-5) まとめ表

`docs/submission_policy.md` §13-3 に「最終確認済み 2026-09-02」として記録した
(本報告の (3-1)〜(3-4) と同内容)。

### 唯一の未確認事項

**SIGNATE 提出画面で「選択中」の2ファイルの目視突合はこのセッションでは実行不能**
(ブラウザ / API アクセス手段なし)。Phase37 の誤アップロード前例があるため、
**ユーザーによる目視確認が必須**:

- 主枠: `mysolver_submit_cc_rcl_w3.zip`(SHA256 `a7bdef7e…3cd48a`)
- 2枠目: `mysolver_submit_rest020.zip`(SHA256 `e13ee359…8e3ff12`)

---

## ステップ4: 締切に向けたチェックリスト

`docs/submission_policy.md` §13-4 に記載(§7 は Phase85 時点のものとして注記を付し、
最新版が §13-4 である旨を明示)。要点:

- 主枠 = `mysolver_submit_cc_rcl_w3.zip`(public 57.894)、
  2枠目 = `mysolver_submit_rest020.zip`(public 55.96)。
- 2026-10-05 頃(選択期限 2026-10-12 の約1週間前)に SIGNATE の実選択を目視再確認。
- 締切(2026-10-19 23:55)直前の新規提出は避ける(反映に約1.5時間、Phase37 実測)。
- zip・コードは凍結。新規 zip は作成しない(探索フェーズ終了)。
- Phase91 の現物確認結果(§13-3)を最終記録として保持。

---

## やっていないこと

- コードの変更・新規 zip の作成(記録の追記・ドキュメント更新のみ)。
- 本番の集計スコアからの足切り閾値・シーン数の逆算。
- `.gitignore` の書き換え・force push。
- 主枠(w3)・2枠目(rest020)の zip への変更。

## 生成物一覧

- `results/phase91_report.md`(本ファイル)
- `docs/submission_policy.md`(§5 該当行の訂正、§7 に注記、§13 を新設)
- `results/phase90_report.md`(総括表 #6 の訂正、§3-3 に決着注記)

コード変更なし。
