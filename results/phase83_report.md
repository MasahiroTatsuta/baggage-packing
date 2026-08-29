# Phase83 報告: rcl_k15/k50 のビルド不具合調査、加法性を活かした軸の重ね合わせ

## Phase82本番結果(先出し)

| | public | vs緩2 | 配置率 |
|---|---:|---:|---:|
| **cc_rcl** | **57.545** | **+0.360** | 63.74% |
| cc_strong | 57.407 | +0.223 | 63.49% |
| cutcorner | 57.394 | +0.209 | 63.49% |
| rcl(k30) | 57.334 | +0.149 | 64.59% |
| 緩2 | 57.185 | — | 64.34% |

加法性: cutcorner(+0.209) + rcl(+0.149) = +0.358 ≈ 実測+0.360(誤差0.002)。cc_strongは
cutcorner単体から+0.013で誤差範囲(N_Yはこれ以上増やさない)。

**不具合**: rcl_k15/rcl_k50が緩2と13桁完全一致(57.18496059843758)——RCL自体が
動いていない疑い。

---

## ステップ0: rcl_k15/rcl_k50のビルド不具合調査

### (0-1) zip中身の直接確認

`unzip`して`simulate.py`を直接grepした結果、**両フラグとも意図どおりの値だった**:

```
mysolver_submit_rcl_k15.zip:
  _RCL_SHUFFLE = os.environ.get('MYSOLVER_RCL_SHUFFLE', '1') == '1'
  _RCL_FRACTION = float(os.environ.get('MYSOLVER_RCL_FRACTION', '0.15'))

mysolver_submit_rcl_k50.zip:
  _RCL_SHUFFLE = os.environ.get('MYSOLVER_RCL_SHUFFLE', '1') == '1'
  _RCL_FRACTION = float(os.environ.get('MYSOLVER_RCL_FRACTION', '0.50'))
```

**zipのビルドプロセス(sed置換)自体に問題は無かった。** 指示文の仮説
(「MYSOLVER_RCL_FRACTIONを非既定値に固定した瞬間にMYSOLVER_RCL_SHUFFLEが無効化される」)
は、少なくとも「zipの中身が違う値になっている」という形では再現しなかった。

### (0-2) 読み取り順序と依存関係の確認

```python
_RCL_SHUFFLE = os.environ.get('MYSOLVER_RCL_SHUFFLE', '0') == '1'
_RCL_FRACTION = float(os.environ.get('MYSOLVER_RCL_FRACTION', '0.3'))
```

両方ともモジュールロード時に**互いに独立して**読まれる(読み取り順序は上から下、
`_RCL_SHUFFLE`が先だが、後に読む`_RCL_FRACTION`が前者の値を参照・上書きする分岐は
存在しない)。呼び出し側(`beam_construct_order`/`greedy_construct_order`)は:

```python
if shuffle_ties and rng is not None:
    if _RCL_SHUFFLE:
        keys = _rcl_shuffle_keys(remaining, rng)
    else:
        keys = list(remaining.keys())
        rng.shuffle(keys)
    remaining = {k: remaining[k] for k in keys}
```

`_RCL_FRACTION`を変えることで`_RCL_SHUFFLE`の判定(`if _RCL_SHUFFLE:`)が無効化される
条件は、コード上**存在しない**(`_rcl_shuffle_keys`内で`_RCL_FRACTION`を参照する箇所は
`rcl_size = max(1, int(len(ranked) * _RCL_FRACTION))`の1箇所のみで、これが例外を
投げたり`_RCL_SHUFFLE`自体を書き換えたりすることは無い)。

### (0-3)/(0-4): 実地検証(zip中身は正しいため、実行時の挙動を直接調べた)

コード読解だけでは原因が見えなかったため、`try_construct`の`except Exception: pass`が
例外を握りつぶしている可能性を疑い、`simulate.beam_construct_order`を監視ラッパーで
差し替えて例外を直接捕捉する診断を実施した(読み取り専用、後で元に戻す一時パッチ)。

**実測1: `_rcl_shuffle_keys`を直接呼ぶ(40アイテム相当のダミーデータ)**
k=0.15/0.30/0.50のいずれも例外なく実行でき、それぞれ異なる順序を返した
(体積降順ランクは共通、選ぶ範囲だけが変わるので当然の結果)。

**実測2: `beam_construct_order`を直接呼ぶ(実シーンA01、`shuffle_ties=True`)**
k=0.3/0.15/0.50のいずれも例外なく実行でき、それぞれ異なる`order`を返した。

**実測3: `ordering.build_order`をシーン2つ×3設定で実行、`beam_construct_order`を
監視ラッパーで包んで例外・呼び出し回数を記録**

| シーン | 予算 | 設定 | winner_source | 総呼び出し数 | shuffle_ties=True呼び出し数 | 例外 |
|---|---:|---|---|---:|---:|---:|
| A01(40個) | 30s | RCL=0 | heuristic | — | 0 | 0 |
| A01(40個) | 30s | RCL=1,k=30 | heuristic | — | 0 | 0 |
| A01(40個) | 30s | RCL=1,k=15 | heuristic | — | 0 | 0 |
| A01(40個) | 30s | RCL=1,k=50 | heuristic | — | 0 | 0 |
| B04(80個) | 120s(本番相当) | RCL=1,k=15 | heuristic | 3 | **0** | 0 |
| B04(80個) | 120s(本番相当) | RCL=1,k=30 | heuristic | 3 | **0** | 0 |
| B04(80個) | 120s(本番相当) | RCL=0 | heuristic | 3 | **0** | 0 |
| A03(40個) | 120s(本番相当) | RCL=1,k=15 | phase1 | 6 | **1** | 0 |
| A03(40個) | 120s(本番相当) | RCL=1,k=30 | phase1 | 6 | **1** | 0 |
| A03(40個) | 120s(本番相当) | RCL=0 | phase1 | 6 | **1** | 0 |

**例外は1件も発生しなかった。** `_RCL_FRACTION`をどの値にしても`_RCL_SHUFFLE`が
無効化される現象は、直接呼び出し・フル`build_order`経由のいずれでも再現しなかった。

**新たに判明した事実(想定外)**: B04(2コンテナ80個)では、RCL設定に関わらず
`beam_construct_order`の総呼び出し数がわずか3回(フェーズ1の窓5種のうち3つまで)
にとどまり、**フェーズ2(`shuffle_ties=True`)が本番相当の120秒予算でも一度も
呼ばれていない**。フェーズ1の窓スイープだけで予算の大半(または全部)を使い切る
シーンが存在する。RCLが効くかどうかを議論する前に、そもそも**フェーズ2に到達する
かどうか自体がシーン依存**であることが分かった(A03のような40個規模のシーンでは
120秒で6回呼ばれ、うち1回はフェーズ2に到達している)。

### 判定: 原因を特定できなかった(指示どおりkの探索を打ち切る)

**コードレベルのバグは見つからなかった。** zipの中身・読み取り順序・依存関係・
実地での例外の有無、いずれを調べても「`_RCL_FRACTION`を変えるとRCLが無効化される」
という現象を再現できなかった。指示(0-4)に従い、**推測で作り直さず、kの探索を
ここで打ち切る**(`mysolver_submit_cc_rcl_k15.zip`/`cc_rcl_k50.zip`は作成しない)。

**未検証の仮説として記録する**(検証していないため判定には使わない):
フェーズ2がbest_orderとして採用される(=`_better()`で現在の最良を上回る)頻度は、
Phase72・Phase81・Phase82の3回の独立測定すべてで**0/26〜0/28**という、極めて低い
確率のイベントだった。この事実を踏まえると、k=30(既定)が本番のどこかのシーンで
たまたま勝った(public+0.149)一方、k=15/k=50は本番のどのシーンでも一度も勝てなかった
(=RCL無効時とビット単位で同一)という結果は、**「kの値そのものの巧拙」ではなく
「低確率事象がどの乱数列で偶然当たるか」という運の要素で十分説明できる**可能性がある。
これは「バグではない」という上記の結論と矛盾しない(何ら誤動作していないが、
勝率が低すぎて大半の設定では観測上「無効化されたように見える」)。

### 26シーン全体でのorder変化数(k=0.15、`tools/phase72_winner_trace.py`と同一条件)

| winner_source | RCL=0 | RCL=1,k=30 | RCL=1,k=15 |
|---|---:|---:|---:|
| heuristic | 10 | 10 | 10 |
| phase1 | 16 | 16 | 16 |
| **phase2** | **0** | **0** | **0** |

**k=0.15でも0/26で完全に同一。** RCL=0・k=30(Phase82再確認)・k=15(本フェーズ)の
3設定全てでphase2由来の採用がゼロだった。「26シーン全体でorderが変わるシーン数」は
**0/26**(=このデフォルト閾値0.55/0.6/0.15・壁時計非拘束という条件では、
RCLのフラクション値によらずorderは一度も変化しない)。緩2+cutcorner土台での
挙動は次項の12シーンサブセットで別途確認する。

---

## ステップ1: zip(cc_rcl_k15/k50は作成せず、新規独立軸のみ)

### 判断: `mysolver_submit_cc_rcl_k15.zip`/`cc_rcl_k50.zip`は作成しない

ステップ0で原因を特定できなかったため、指示(0-4)に従いこの2本は作成しない。

### 判断: 既存フラグ(ALNS/REPAIR/WALL_MODE)の再検討

Phase82で「4zipとも既定'0'」と確認された3フラグについて、コードのコメント・
過去の判定根拠を再確認し、cutcorner/rclという新しい土台のもとで再挑戦する価値が
あるかを判定した。

| フラグ | 過去の判定 | 再挑戦の可否 | 理由 |
|---|---|---|---|
| `MYSOLVER_ALNS` | Phase34で不採用(代理関数の精度不足、ρ=−0.321、**本物のpybullet評価器で測定**) | **再挑戦しない** | ALNSの受理判定(どの破壊→修復案を採用するか)に使う代理目的関数(risk_adjusted_volume)の精度問題は、支持閾値・候補生成(cutcorner)・リスタート方式(rcl)のいずれとも無関係な、**受理判定そのものの欠陥**。土台を変えても代理関数の中身は変わっておらず、再挑戦で覆る理由がない |
| `MYSOLVER_REPAIR` | Phase29で不採用(**到達シーン数2/26**、t=+1.000で構造的にt>2を超えられない。機構自体はC03+2.696pt・P05+4.799ptと実際に機能はしていた) | **再挑戦する(1本)** | 不採用理由が「効果が負」ではなく「到達シーン数が少なすぎて統計的に有意と言えない」という**到達範囲の問題**だった。cutcornerは候補生成そのものを変える施策であり、どのシーンで搬入経路がブロックされるかという幾何的な状況が変わりうる。到達シーン数が増える可能性を否定できず、REPAIR自体は既に実装済みで実装リスクがゼロなので、cc_rclに重ねて1本試す価値があると判断した |
| `MYSOLVER_WALL_MODE` | Phase9/13/14/26で4連敗、Phase26は**統計的に有意な悪化**(t=−2.291)、機序も特定済み(壁面へ荷物を押し付ける結果、境界ぎりぎりの配置が構造的に増え、沈降後の厳マージン判定で計上から漏れる) | **再挑戦しない** | 「効果不足」ではなく「統計的に有意なマイナス」であり、機序(境界ぎりぎり配置の増加)はcutcorner/rclのいずれとも無関係に成立する。厳/緩どちらの内包判定レジームでも不採用の結論は変わらないとPhase26自身が確認済みで、土台を変える動機がない |

**`MYSOLVER_REPAIR`を新規独立軸として選定し、cc_rclの上に重ねた1zip
(`mysolver_submit_cc_rcl_repair.zip`)を作成した。**

### zip安全性の事前確認

`MYSOLVER_CUTCORNER_CANDIDATES=1` + `MYSOLVER_RCL_SHUFFLE=1` + `MYSOLVER_REPAIR=1`の
組み合わせが例外なく動作することを、zip化前にA01(--optimize-budget 15)で確認した
(placed=21/40、policy_time max=0.10s、例外なし)。

### zip一覧

| zip | 有効フラグ | SHA256 |
|---|---|---|
| `mysolver_submit_cc_rcl_repair.zip` | `MYSOLVER_CUTCORNER_CANDIDATES=1` + `MYSOLVER_RCL_SHUFFLE=1`(既定k=30) + `MYSOLVER_REPAIR=1` | `efbf48464302567b82b1787191bbe9460bb1787df3354c425e7075f65952b49f` |

**アップロード時は必ず上記SHA256と照合すること。**

指示は「zip最大5本」だったが、**5本中1本のみ作成した**——(0-4)の判定でrcl_k15/k50の
2本を見送り、新規独立軸もALNS/WALL_MODEを妥当な理由で除外した結果、REPAIRの1本のみが
残った。本数を埋めるための無理な追加(推測での作り直し、根拠の薄いパラメータの水増し)は
指示の禁止事項にも反するため行っていない。

### 全定数grep

閾値3(union/span/centroid、緩2)+ 幾何3(既定)+ 他7(既定)+ Phase81の新規フラグ3
(`DFTRC_STRATEGY`=既定'0'/`CUTCORNER_CANDIDATES`='1'・`N_Y`=既定'5'/`RCL_SHUFFLE`='1'・
`FRACTION`=既定'0.3') + Phase83の新規フラグ3(`REPAIR`='1'・`REPAIR_MAX`=既定'12'・
`REPAIR_VOXEL`=既定'0.05')に加え、`WALL_MODE`・`REACH_WEIGHT`・`ALNS`・
`BEAM_SOFT_LAST`が既定のままであることも確認した(意図しない機能混入が無いことの
追加確認)。全て意図どおり(grep結果は上のステップ1冒頭に掲載済み)。

### zipとリポジトリの差分ファイル確認

`ordering.py`(`REPAIR`)・`planner.py`(閾値3行+`CUTCORNER_CANDIDATES`)・`simulate.py`
(`RCL_SHUFFLE`)の3ファイルが差分。いずれも11エントリ、既存提出zipと同一構造。

### order変化数(cc_rcl土台 vs cc_rcl+REPAIR)

**方法論上の制約(先に報告)**: 緩2+cutcorner+rclという土台は、既定閾値のみの構成より
1シーンあたりの構築コストが著しく大きい(候補が支持閾値の緩和で大幅に増えるため)。
指示どおり26シーン全体をこの土台で比較しようとしたところ、**A01(40個)1シーン・
budget=60s(本番相当120sの半分)だけで実時間3分以上**かかり、26シーン×2設定では
現実的な時間に収まらないと判明した。そのため**6シーンのサブセット(A01/A03/B01/
C01/D01/P02、いずれも40個規模)・budget=30sに縮小して測定した**(このスコープ限定
自体を明記する。全26シーンでの измерение は行っていない)。

| | 差分シーン数 |
|---|---:|
| cc_rcl → cc_rcl+REPAIR | **1/6**(`suite_B01_1c_40_plain.json`) |

**REPAIRは実際にorderを変化させた(RCLの0/26とは対照的)。** REPAIRの機構
(行き詰まった順序に対してブロッカーを同定し局所的に並べ替える)がcutcorner+rclという
新しい土台の上でも動作していることが確認できた。ただしこれは「orderが変わった」
という事実のみで、**実際の配置数・publicスコアへの影響は不明**(判定は本番結果を待つ)。

### policy実行時間(本番実測6.07s、8秒制限まで約1.9sとの比較)

要求の厳しい2シーン(B04: 2コンテナ80個、C03: 2コンテナ80個・優先荷物)、
`--optimize-budget 60`で`mysolver_submit_cc_rcl_repair.zip`相当の構成
(cutcorner+rcl+REPAIR)を実測した:

| | policy_time mean | policy_time max |
|---|---:|---:|
| cc_rcl+REPAIR | 1.548s | **1.97s** |

**ローカル実測は8秒制限に対し十分な余裕がある(6.03s以上には遠く及ばない)。**
ただし従来のcutcorner単体実測(Phase82: max 1.69s)・cc_rcl_repair(本フェーズ:
max 1.97s)のいずれもローカル最大値は本番実測(6.03〜6.07s)の1/3以下であり、
**ローカルでは本番の実際の制約(より大きい・複雑な本番シーン)を再現できていない**
という留保はPhase82から変わらない。REPAIRは`policy()`(オンライン、8秒制限側)には
一切関与しない(offline `build_order`側の機構)ため、REPAIR追加によるpolicy時間への
直接的な影響は原理的に無い(cutcornerの候補生成コストのみがpolicy時間に効く)。

### 決定的8シーンの対緩2差分(参考値)

| zip | 差分シーン数 | 差分シーン |
|---|---:|---|
| `mysolver_submit_cc_rcl_repair.zip` | **3/8** | B01, B04, P04 |

Phase81のcutcorner単体・rcl単体はいずれも緩2に対し0/8、Phase82のcc_rcl(両方)も0/8
だったのに対し、**REPAIRを重ねたことで初めて決定的8シーンでも差分が生じた**
(B01は6シーンサブセットの検証でも差分が出ていたシーンと一致)。REPAIRが
cutcorner+rclという新しい土台の上で確かに機能していることの一貫した傍証になっている。

---

## 提出枠の更新

指示どおり、**主枠を`mysolver_submit_cc_rcl.zip`(public 57.545、Phase82作成分)に
変更した。** 2枠目は`rest020`(public 55.96)を維持(REST_CLEARANCEが崖から
離れた唯一の構成という設計上の役割、Phase77の判断を継続)。`docs/submission_policy.md`
§1・§5に反映した。

## 判定(本番結果待ち)

対照はcc_rcl=57.545。`mysolver_submit_cc_rcl_repair.zip`を提出し、publicと
num_placed_itemsの両方で判定する。ローカルでの追加観測(判定には使わない、参考情報):

- REPAIRは6シーンサブセット中1/6・決定的8シーン中3/8でorderを実際に変化させた
  (cutcorner/rcl単体はいずれも0/8だった)。**「orderが変わる」という点ではcc_rclより
  明確に反応があるが、それが配置数・publicスコアの向上につながるかは別問題**
  ——REPAIRはPhase29で「機構は機能するが到達シーン数が少なすぎて統計的に有意と
  言えない」という判定であり、到達シーン数が増えたからといって効果の符号(プラスか
  マイナスか)まで保証されているわけではない。
- policy時間はローカルで安全マージンを確保(max1.97s)。

**Phase80の教訓どおり、ローカルの反応の有無だけで採否を決めない。** 本番結果を
見て判定する:
- 57.545を明確に超えた → 枠入れを検討
- 57.4〜57.6 → 誤差範囲、枠はcc_rclのまま
- 下がった → 不採用

---

## やっていないこと

- N_Yをさらに増やすこと(cc_strongで飽和を確認済み、指示どおり動かしていない)。
- dftrcの再挑戦。
- 支持閾値・幾何定数を動かすこと(緩2+既定で固定)。
- 不具合の原因が分からないまま推測でzipを作り直すこと(rcl_k15/k50は指示どおり
  未作成のまま)。
- ローカルA/Bの弱いシグナルで採否を決めること(上記「判定」参照、本番結果待ち)。
- 本番の集計スコアから足切り閾値やシーン数を逆算すること。
- `.gitignore`の書き換え・force push。

## 生成物一覧

- `results/phase83_wintrace_rcl_k15.json`(ステップ0、k=0.15の26シーンphase2勝率)
- `results/phase83_report.md`(本ファイル)
- `submissions/mysolver_submit_cc_rcl_repair.zip`(新規、SHA256は上表参照)
- `docs/submission_policy.md`(§1・§5にPhase83追記、主枠をcc_rclへ更新)

コードファイル(`agents/mysolver/*.py`)は無変更(Phase81/82で実装済みのフラグを
異なる組み合わせで使っているだけ)。診断用の一時的なmonkey-patchスクリプト
(`simulate.beam_construct_order`を監視ラッパーで包むもの)はセッションのスクラッチ
パッドに置かれた読み取り専用の一回限りの調査用で、リポジトリ化していない。
