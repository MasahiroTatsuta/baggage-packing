# 引き継ぎ資料

作成: 2026-09-14 / 引き継ぎ元: Claude Code (Opus 5) / 引き継ぎ先: Codex

**最初に読むもの**: この文書 → `docs/SUMMARY.md`(全体像) → `docs/EXPERIMENT_LOG.md`(全軸の詳細)。
フェーズ順の一次ログは `docs/submission_policy.md`(1600行超)。

---

## 1. いまどこにいるか

| 項目 | 値 |
|---|---|
| **主枠(SIGNATE で選択中)** | `submissions/mysolver_submit_w3pe_lr_ordcount.zip` |
| **public** | **61.092** |
| SHA256 | `d70d503bc33e79b7633f0c36eed3246b1f84af3af30fe907e727fc873f3e73a3` |
| 順位 | 108位付近 |
| **選択期限** | **2026-10-12** / 最終締切 2026-10-19 23:55 |
| ブランチ | `main`(= `origin/main`、`exp/wallbuild` と同一) |

リーダーボード(2026-09-10): 50位 64.544 / 20位 67.280 / 10位 68.409 / 1位 70.993。
**上位10者程度がスクリーニング通過 → 成果審査**(METI プレスリリース由来、未確認)。
もし正しければ **10位(68.409)が実質カットラインで、現在 +7.3 の差**。

### 提出済みで結果待ちのもの

**`~/Desktop/mysolver_submit_w3pe_lr_oc_corrres.zip`**
(SHA256 `7a4dbe41080d866165dac74ae728fc9ff760ae051bbdb7087c822ec5a72eac9d`)

ユーザーに提出を依頼したところで引き継ぎになった。**結果が出たら
`docs/submission_policy.md` §14 に追記すること。** ローカルでは配置数 +5 で
サブスコア5指標中4つが主枠超え(詳細は §4)。

---

## 2. 次にやること(優先順)

### (A) 最優先: 外部Deep Research 8施策のうち**未検証4本**を潰す

`~/Downloads/compass_artifact_wf-bf20987d-e520-512a-8e78-4f7a006e0227_text_markdown.md`
(ユーザーが取得した外部調査レポート)の施策リスト。**当初、引き継ぎ元は8本中1本しか
実装せず残りを推論で切っていた**が、ユーザーの指摘で検証に着手したところ、
**推論で「効かない」と切った施策1が唯一の正の結果だった**。同じ轍を踏まないこと。

| 施策 | 状態 | 結果 |
|---|---|---|
| 1 corridor reservation | ✅ 実装・A/B・zip 作成 | **配置数 +5、5指標中4つで主枠超え。提出依頼済み** |
| 2 MACS | ✅ 実装・A/B | −15。提出見送り |
| 3 support-polygon | ✅ 実装・提出 | **−2.24 不採用** |
| **4 CVaR / 分位点選択** | ❌ **未着手** | — |
| **5 noisy-robust ILS** | ❌ **未着手** | — |
| **6 ラダー4段構成** | ❌ **未着手** | — |
| **7 2コンテナ割当** | ❌ **未着手** | — |
| 8 PCT/GOPT | 実行困難 | 学習済み重みの同梱が必要、2コア GPUなし |

各施策の原文と、引き継ぎ元が当初つけた(そして外した)評価は
`docs/submission_policy.md` §14-16 に記録してある。

#### 施策4 の実装メモ(引き継ぎ元の下調べ)

「optimize() の180秒で K本の順列を作り、各順列を複数回評価して**配置数の
最悪値/下側分位点(CVaR)**が最大のものを選ぶ」。`ordering.build_order` の
`_better()`(現在は `(配置数, 体積)` の辞書式)を分布評価に変える。
`simulate` 側に `score_noise` 引数が既にあるので摂動注入に使える。

**注意点(引き継ぎ元の見立て、要検証)**: 分散を取るのが正しいのは閾値の**下**に
いるときだけで、本番シーンの約70%は既に足切りを通過している(§3-2)。
通過済みシーンで分散を取るのは純粋に有害。ただしこの見立ては検証していない。

### (B) 未着手の別軸: 真の詰み249個

Phase96 の総当たり測定で、死亡25局面のうち **11局面は合法解が存在しない**
(残り14局面は救済可能で、last-resort が回収済み)。この249個は「そこに至らせない」
上流の問題で、Phase73〜103 の38軸と WBC 15ラウンドが跳ね返された壁。

**ただし decoder 系(DBLF)と corridor_reserve は、最悪シーン A08(140個中19個)を
26〜27 に改善している。** 上流の詰みに初めて手が届いた系統なので、ここを深掘りする
価値はある。

---

## 3. 絶対に踏んではいけない落とし穴

引き継ぎ元が実際に踏んだものだけを書く。

### 3-1. `except Exception: pass` が握り潰す —— 提出物は必ず実動作確認

corridor_reserve の zip 初版は **一度も動いていなかった**。インライン移植した
`_mx_prune` が `WBC_MIN_SPACE` を参照して `NameError` になり、コードの
`except Exception: pass` がそれを握り潰していた。スコアは「それらしい」値
(= 主枠と同じ 19/20)が出るので気づけない。

**提出物を作ったら、必ず zip の中身を `agents/mysolver/` に入れ替えて実行し、
A/B と同じ数値が出ることを確認すること。** 静的チェック(AST での未定義名検出)は
このバグを検出できなかった。

```bash
SP=/tmp/work
cp -R agents/mysolver $SP/repo_bak
rm -rf agents/mysolver && cp -R $SP/build_X/mysolver agents/mysolver
env MYSOLVER_HARD_WALL_LIMIT=3000 MYSOLVER_UNITS_PER_SEC=2.00e7 PYTHONPATH=. \
  .venv/bin/python tools/measure.py --config-path configs/gen/suite_A08_2c_140_extreme.json \
  --repeats 1 --optimize-budget 135 --out $SP/zipcheck.json --label zipcheck
rm -rf agents/mysolver && cp -R $SP/repo_bak agents/mysolver   # 必ず戻す
```

### 3-2. 決定性チェックには `MYSOLVER_HARD_WALL_LIMIT=3000` を付ける

付けずに回すと、マシンが重いときに壁時計(既定165秒)が探索を早く切り、
**実装バグでもないのに 7/8 になる**。引き継ぎ元は Phase107 でこれに1時間溶かした。
A/B には最初から付いていたが決定性チェックには付いていなかった。

```bash
PYTHONPATH=. MYSOLVER_OPTIMIZE_BUDGET=40 MYSOLVER_LASTRESORT=0 \
  MYSOLVER_HARD_WALL_LIMIT=3000 \
  .venv/bin/python tools/phase63_determinism_check.py --out /tmp/det.json
```

### 3-3. 重い候補評価は `policy_timeout`(8秒)を壊す

`SearchBudget` は「1回の `_evaluate_candidates` は数ms相当」を前提にしている。
MACS / corridor_reserve のように候補ごとに空間分割する処理はこれを壊す。

**実測比率: 本番の policy はローカルの約10倍遅い**
(ローカル max 0.505〜0.623s / 本番 6.17〜6.35s、`POLICY_HARD_WALL=6.0` の壁に張り付き)。
ローカルで +0.186s なら本番 +1.9s → **8.07s でタイムアウト真上**。
超えるとランダム手を返して即死する。

対策として `_heavy_eval_allowed(budget)` を入れてある(`hard_deadline` の
2.5秒前で重い評価を開始しない)。**新しい重い処理を足すときは必ずこれを通すこと。**
時間の実測は `tools/phase63_policy_timing.py`。

### 3-4. 自分が入れた安全弁が測定を壊す

MACS の初回測定 −30 は、タイムアウト対策で入れた予算課金
(`MACS_UNIT_COST=20 × |spaces| × K`、1評価あたり6.4万ユニット)が過大で、
**MACS の効果ではなく探索が痩せたことを測っていた**。課金を外すと4シーンで
−25 → −11 に回復(A05 は主枠超え)。**新しい計上を足したら、必ず
「課金なし」条件と比較して交絡を切り分けること。**

### 3-5. ローカル A/B は効果量の予測に使えない

| 手 | ローカル配置率増 | 本番配置率増 | 転移率 |
|---|---:|---:|---:|
| last-resort | +1.29pp | +1.88pp | 1.46 |
| ordcount | +2.51pp | +0.51pp | 0.20 |
| oc_cf | +4.47pp | **−0.40pp** | **−0.09** |

**ローカル増分が大きいほど転移が悪い(26シーンへの過適合)。**
これは測定設計の欠陥ではなく問題クラスの構造的性質である
(MDPI *Algorithms* 2026, 19(9):707 が「constraint density が上がると
Kendall's W が 0.69→0.08 に低下」と報告)。

**ローカルは「壊れていないことの確認」に使い、採否は public で決めること。**

さらに: **`soft_item_score` はローカルでは判定できない**。ローカルは足切りを
適用していないため全バリアントが 97〜99.5 に張り付く(本番は 62〜70)。

### 3-6. 機序が説明できても効くとは限らない

引き継ぎ元はこのセッションで機序ベースの予測を4回立てて**3回外した**
(cogfilter / dblf / soft_clearance)。**機序の説明は「試す理由」にはなっても
「効く根拠」にはならない。**

### 3-7. メモリ

このMacは空きが逼迫しやすく(Chrome が20%超)、A/B が2回 kill された。
26シーンを一度に回さず `half1`/`half2` に分けること。

---

## 4. 直近の実験結果(未記録分を含む)

### corridor reservation (Phase109、提出依頼済み・結果待ち)

```
配置数 786 → 791 (+5)   改善10 / 悪化11 / 不変5
A08_2c_140_extreme  19 → 26 (+7)    P02_pre10  28 → 35 (+7)
D05_tall            20 → 26 (+6)    C03_prio   36 → 40 (+4)
A02_1c_80_plain     29 → 20 (−9)    D04_flat   28 → 23 (−5)

             ordcount  corr_res
fill            28.23    28.07   −0.16
cog             62.73    63.32   +0.59
stability       97.44    97.47   +0.03
placement       98.21    99.52   +1.31
soft_item       98.40    99.04   +0.64
```

### MACS との対比(これが今回いちばんの知見)

| | 評価キー | 配置数 |
|---|---|---:|
| MACS(施策2) | 残る**最大の単一凸空間**の体積 | −15 |
| corridor_reserve(施策1) | **前面から到達できる**空き空間の体積合計 | **+5** |

同じ maximal-space を使いながら結果が分かれた。
**「大きな空間を残す」ではなく「到達できる空間を残す」が効く。**
MACS は到達性を見ないので「入れない大空間」を残してしまう。

### decoder 置換(Phase105-107、すべて不採用)

| 版 | public | 特徴 |
|---|---:|---|
| dblf | 60.672 | **fill/cog/stability の3指標で主枠超え**、負けは soft_item −7.95 一点 |
| dblf_hard | 59.606 | soft_item は部分回復も、decoder 混合で配置数 −1.04pp |
| dblf + soft立入禁止帯 | 未提出 | ローカル −24、soft_item も逆に低下 |

---

## 5. 使い方

### 環境

```bash
python3.12 -m venv .venv
.venv/bin/pip install numpy==2.5.1 scipy==1.18.0 gymnasium==1.2.3 pillow==10.3.0
# pybullet は macOS でソースビルドが必要。-D_DARWIN が zlib の fdopen を潰す既知バグを回避:
CFLAGS="-Dfdopen=fdopen" CXXFLAGS="-Dfdopen=fdopen" .venv/bin/pip install pybullet==3.2.7
```

### 26シーン A/B(1条件あたり約1.5時間)

```bash
ls configs/gen/suite_*.json | head -13 > /tmp/half1.txt
ls configs/gen/suite_*.json | tail -13 > /tmp/half2.txt
for H in 1 2; do
  env MYSOLVER_HARD_WALL_LIMIT=3000 MYSOLVER_UNITS_PER_SEC=2.00e7 \
      MYSOLVER_LASTRESORT=1 MYSOLVER_ORDER_BY_COUNT=1 <新フラグ> PYTHONPATH=. \
    .venv/bin/python tools/measure.py --config-path $(cat /tmp/half$H.txt | tr '\n' ' ') \
    --repeats 1 --optimize-budget 135 --out results/phaseNNN_half$H.json --label x$H
done
```

対照は `results/phase98_ab_ordcount.json`(= 現主枠、配置数 786)。

### 提出パッケージング

**本番 zip に対する最小差分**で作る(リポジトリからビルドすると既定OFFの実験コードを
同梱してしまう)。ベースは `mysolver_submit_w3pe_lr_ordcount.zip`。

```bash
unzip -q submissions/mysolver_submit_w3pe_lr_ordcount.zip -d /tmp/build
# /tmp/build/mysolver/*.py の env 既定値を書き換える
(cd /tmp/build && zip -qr ../new.zip ./mysolver)
shasum -a 256 ../new.zip     # SHA256 を submission_policy.md に必ず記録
cp ../new.zip ~/Desktop/     # 保全ルール: zip は .gitignore 対象
```

**その後 §3-1 の実動作確認を必ず行う。**

### 3D可視化(構造的な発見に効いた)

```bash
PYTHONPATH=. MYSOLVER_LASTRESORT=1 MYSOLVER_ORDER_BY_COUNT=1 MYSOLVER_OPTIMIZE_BUDGET=135 \
  .venv/bin/python tools/trace_episode.py \
  --config configs/gen/suite_A08_2c_140_extreme.json --out results/trace_A08.json
```

ユーザーがこのビューアを見て「左がガラ空き」と指摘したことから、
`transport_x_bounds` がコンテナ長の22%を封鎖していることが判明した
(修正自体は失敗、`docs/EXPERIMENT_LOG.md` §7)。**数百回のA/Bで見つからなかった
構造を目視が掘り当てた**ので、行き詰まったら可視化に戻ること。

---

## 6. 運用ルール(ユーザーとの取り決め)

### 方針: 「メダルか無しか」(2026-09-09 変更)

目標が銅メダル(≒60、実際は上位10者=68.4の可能性)である以上、そこに届かなければ
主枠を守る意味がない。**2枠目は保険ではなく最も分散の大きい賭けに充てる。**

### 提出は希少資源ではない

SIGNATE は **1日5回**投稿でき、締切まで約35日 = 約175回。
**枠を消費するのは選択期限(10-12)であって提出ではない。**
Phase89/90 の頑健性ゲート(「悪化シーンが少ない強い信号でないと採用しない」)は
**最終枠の選択にのみ適用し、提出可否には適用しない。**

### 禁止事項(運営規約)

運営が**評価関数のパラメータ解析を NG と明言**している
(`docs/submission_policy.md` §4、Phase61 追記)。したがって:

- 各指標の重みを提出結果から逆算しない
- 足切り閾値そのもの(何個で発動するか)を逆算しない

複数の仮説を並べた感度分析(`tools/scorer.py` の `CUTOFF_CANDIDATES`)は、
閾値を特定せずに意思決定するための手段として許容範囲と判断している。

### ユーザー操作が必須なもの

- **SIGNATE への提出**(エージェントからは実行できない)
- **選択期限前の目視確認**: 提出画面で「選択中」が主枠になっているか。
  **2026-10-05 頃を目安に**。これを怠ると全部が無駄になる
- 締切直前の新規提出は避ける(反映に約1.5時間、Phase37 実測)

---

## 7. 押さえておくべき設計知識

### 足切りは階段関数

last-resort の緩和レベルを上げた3提出で、**Lv1 と Lv2 のサブスコアが小数まで
完全一致**した(= 同じシーン集合しか通過していない)。配置数は単調増加(0.6498 →
0.6534)なのに public は Lv3 で初めて跳ねた(58.498 → 60.824)。

**配置数の小さな改善は public に転移しない。跨ぐまで積んで初めて得点になる。**
これは Phase73 以降の多くの軸が「ローカルで net +数個」を追って全滅した理由の説明。

### 足切りは本番シーンの約3割を0点にしている

本番サブスコアとローカル(足切り未適用)の比から通過率を推定すると 69〜79%。
1シーンあたり **跨ぎ1回 = +2.75pt / soft違反1回 = −0.19pt(14:1)**。
配置数を増やす施策とサブスコアを守る施策のトレードはこの比率で判断する。

### 死因は100%が同じ1経路

`planner.plan()` が合法手ゼロを返すと `agent.py` の `_fallback_place_pos()` に落ちる。
この関数は**床レベルの9点(1コンテナ・1荷物・向き0固定)しか見ない**。
一方、総当たり測定では**死亡局面に実在する合法解の z 源は 100% が既配置荷物の上、
床は0件**だった。

**死亡時の `orientation = 0` はこのフォールバックのハードコード値**であり、
ソルバが向きを選び損ねた結果ではない。死因判別の簡便な指紋として使える。

### 効いたものと効かなかったもの

| カテゴリ | 結果 |
|---|---|
| **実行可能集合の拡大** | last-resort **+2.33**(唯一の大きな前進) |
| 目的関数の入れ替え | ordcount **+0.27** |
| **到達可能な空き空間の最大化** | corridor_reserve(ローカル +5、本番結果待ち) |
| 再ランク付け(定数・重み・種・候補順位) | 15本以上で最良 +0.27、最悪 −5.19 |
| 探索機構の高度化(beam/MCTS/BRKGA/ALNS/LDBC/CDR) | すべて不採用 |
| 構築規則の再定式化(WBC) | 15ラウンドでストップロス |

---

## 8. 環境変数の一覧(現在の主枠で有効なもの)

| 変数 | 主枠での値 | 意味 |
|---|---|---|
| `MYSOLVER_LASTRESORT` | 1 | 合法手ゼロ時の段階的救済ラダー |
| `MYSOLVER_LASTRESORT_MAX_LEVEL` | 3 | ラダーの最終段 |
| `MYSOLVER_ORDER_BY_COUNT` | 1 | リスタート比較を(配置数, 体積)の辞書式へ |
| `MYSOLVER_PHASE1_WINDOWS` | 15,25,None | フェーズ1の決定的リスタート |
| `MYSOLVER_PHASE1_SLICE_S` | 33.333… | 各リスタートの秒配分 |
| `MYSOLVER_MIN_UNION_SUPPORT_RATIO` | 0.35 | 緩2閾値(山型の頂点、確定) |
| `MYSOLVER_CUTCORNER_CANDIDATES` | 1 | 切り欠き面沿いの候補生成 |
| `MYSOLVER_RCL_SHUFFLE` | 1 | 上位30%シャッフル |
| `MYSOLVER_PLAN_EXECUTOR` / `PACK_LNS` | 1 | plan-executor / PACK_LNS |
| `DEFAULT_TIME_BUDGET` | 135.0 | offline 予算(zip に焼き込み) |

既定OFFの実験フック(すべて 8/8 ビット単位一致を確認済み):
`MACS` / `CORRIDOR_RESERVE` / `DECODER` / `FLOOR_FIRST` / `LASTRESORT_COG_FILTER` /
`LASTRESORT_SUPPORT_FIRST` / `LASTRESORT_KEEP_PRIORITY_CLEARANCE` /
`TRANSPORT_X_BY_HEIGHT` / `SOFT_CLEARANCE_XY` / `PHASE1_PRIMARY_STRATEGY`
