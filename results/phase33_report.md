# Phase 33 報告：床面限定risk_volをpublicで直接検証(ルール5の一時適用除外) + ALNS接頭辞再開の実現可能性確認

作成日: 2026-08-13
対象コミット: (最終コミットで確定、直前 ba4edef)

**本報告は中間報告。** タスク1(public提出)は zip 作成・ローカル検証まで完了しているが、
実際のSIGNATE Web UIへのアップロードはこの環境から実行できない(認証情報・ブラウザ操作
いずれも無い)。public スコアの入手を待って判定・確定を追記する。タスク2(ALNS実現可能性)
は完了しており、本報告に結果を記載する。

変更ファイル:
`agents/mysolver/simulate.py`(`RISK_SLACK_FACES` 既定を `'all'`→`'floor'` に変更、
+ ALNS調査専用フック `resume_state`/`snapshot_after`/`snapshot_out` を追加、
既定Noneで無改変)、`tools/phase33_prefix_resume.py`(新規、接頭辞再開のビット単位一致検証)。

**測定条件**: ローカル実測は `MYSOLVER_UNITS_PER_SEC=1.55e7`(タスク2)。
提出用ビルドは `MYSOLVER_UNITS_PER_SEC=2.00e7`(既定のまま、変更なし)。

---

## 1. タスク1: 床面限定risk_volの提出(準備完了・提出はブロック中)

### 1.1 実施内容

1. `agents/mysolver/simulate.py` の `RISK_SLACK_FACES` 既定を `'all'` → `'floor'` に変更。
   影響範囲は Phase31 と同じく `simulate_order` 内の `risk_adjusted_volume` 集計のみで、
   hard legality(`check_inclusion_batch`)・`_score()` のboundary_term・online policy は
   一切変更していない(コード上のコメント・既存の構造から確認、新規の変更なし)。
2. `planner.UNITS_PER_SEC` は既に既定 `2.00e7`(提出用の値)であることを確認(変更不要)。
3. **決定的5シーン(B01-B04, P04)での無変更確認(ビット単位)**:
   `tools/measure_regime.py` を新しい既定(`floor`)のまま実行し、Phase31時点の記録
   (`results/phase31_noleak.json`、既定`all`時点)と突き合わせ。

   ```
   完全一致(ビット単位)のシーン: 5/5
   fill_strict/fill_loose/cog_score/stability_score/placement_score/soft_item_score
   すべて差分 0.000
   ```

   これらのシーンは pattern B(`optimize=False`)のため `build_order`(ひいては
   `simulate_order`)自体が呼ばれず、`RISK_SLACK_FACES` の値に構造的に影響されない
   ことを実測でも裏取りした。
4. `mysolver_submit.zip` を再生成(`agents/mysolver/` 直下、8ファイル、`__pycache__`除外)。

### 1.2 ブロック理由

SIGNATEへの提出はWeb UI経由のログイン+アップロードが必要で、この実行環境には
SIGNATEの認証情報もブラウザ操作の手段も無い(過去フェーズの記録にも自動提出の
形跡が無く、一貫して人間側の操作だったとみられる)。したがって:

- **`mysolver_submit.zip` はビルド・検証済みだが未提出。**
- ユーザーがこのzipをSIGNATEに提出し、public スコアを共有してもらう必要がある。

### 1.3 判定(スコア入手後に追記)

指示書の判定基準(ベースライン53.64比):
- +0.3以上 → 既定を`floor`で確定、選択則の残り(未回収+1.69)を追う
- −0.1〜+0.3 → 効果はノイズ内、既定を`all`に戻す
- −0.1未満 → 明確な悪化、既定を`all`に戻し路線を閉じる

**(公開スコア入手後、ここに実測結果と最終判定を追記する。)**

---

## 2. タスク2: ALNSの実現可能性確認(接頭辞再開)

### 2.1 実装したフック

`agents/mysolver/simulate.py` の `simulate_order` に3つの調査専用引数を追加した
(すべて既定None、指定しない限り無改変であることを実測確認済み——後述2.2)。

- `snapshot_after=k`: k個目を配置した直後(refill後)の完全な内部状態
  (containers/pool/残order/累積量5種/違反カウント2種/消費ユニット)を `snapshot_out`
  に書き込み、その場でロールアウトを止める。
- `resume_state=snapshot`: `snapshot_after` で取った状態から続きを流し込む
  (containers/orderを最初から構築し直さない)。
- ALNS本体(destroy/repair・受理則)は実装していない。フックのみ。

### 2.2 (2-1) ビット単位一致の検証

optimize有効の先頭3シーン(A01, A02, A03、シーンID昇順)で、
`results/phase29_cand_g1.json` に記録済みの勝者順序(新規ロールアウトなし)を使い、

- FULL: 順序を最初から通した結果
- RESUME: n_placed の 1/4, 1/2, 3/4 相当のステップkでスナップショットし、
  そこから再開した結果

を比較した(`tools/phase33_prefix_resume.py`)。

| scene | k点 | bitwise_match | 備考 |
|---|---|---|---|
| A01_1c_40_plain | 6/13/19 (n_placed=26) | 3/3 | diff_placed_volume/diff_risk_vol とも0 |
| A02_1c_80_plain | 5/10/15 (n_placed=21) | 3/3 | 同上 |
| A03_1c_40_shelf | 6/13/19 (n_placed=26) | 3/3 | 同上 |

**9/9件で `placed_ids`/`placed_volume`/`risk_vol`/`violation_ratio`/`stability_risk`
すべて完全一致。接頭辞再開は実装可能と確認できた。**

またフック自体を使わない通常呼び出し(`resume_state`/`snapshot_after`/`snapshot_out`を
渡さない既存の呼び出し)が無改変であることも、A01の記録済みrisk_vol(1.5079097051369992)
との突き合わせで確認済み(差分0.0)。

### 2.3 (2-2) スナップショットのコスト

同じ9件でスナップショットを pickle 化してバイト数を実測。

| scene | k | snapshot bytes |
|---|---:|---:|
| A01 | 6/13/19 | 4.4KiB / 7.4KiB / 10.0KiB |
| A02 | 5/10/15 | 4.0KiB / 6.1KiB / 8.2KiB |
| A03 | 6/13/19 | 4.3KiB / 7.3KiB / 9.7KiB |

配置済み荷物数にほぼ比例して数KiB〜10KiB程度。荷物数40〜80のシーンでこの規模なので、
メモリコストは無視できる(ALNSで多数のスナップショットを保持しても数MB規模に収まる)。

### 2.4 (2-3) 1反復のコスト比と、端数予算に入る反復数

Phase29 §3.3 が「リスタートループが捨てている端数」として実測した5シーン
(A05/C02/C03/D01/P05)で、以下を実測した:

- **1反復のコスト** = 記録済み勝者順序をスナップショット(m=末尾15%相当の荷物数、
  最小1個)し、そこから `resume_state` で再開して最後まで流し込む壁時計。
  「末尾m個だけ作り直す」の**評価(simulate)側**のコストに相当する
  (構築側の探索アルゴリズム自体はPhase34で実装、本フェーズでは測っていない——後述の注記参照)。
- **全構築1回のコスト** = `beam_construct_order`(構築)+ `simulate_order`(validate)を
  1回ずつ実測した合計(`try_construct` が実際に1リスタートごとに行っている処理と同じ組み合わせ)。

| scene | 全構築1回(構築+validate) | 1反復(末尾mのresume) | m | 比率 | 端数(Phase29実測) | 反復数目安 |
|---|---:|---:|---:|---:|---:|---:|
| A05_2c_80_prio | 19.90s | 0.32s | 4 | **1.6%** | 12.6s | ~39.8 |
| C02_2c_55_shelfprio | 38.32s | 1.48s | 5 | **3.9%** | 1.7s | ~1.1 |
| C03_2c_80_prio | 25.78s | 1.24s | 4 | **4.8%** | 8.5s | ~6.8 |
| D01_A_1c_40_softheavy | 6.45s | 0.16s | 3 | **2.5%** | 39.7s | ~250.3 |
| P05_C_2c_pre8_shelfprio | 31.95s | 1.01s | 3 | **3.2%** | 9.4s | ~9.3 |

(壁時計ベースの実測のため多少の測定ノイズはある——独立に2回測定し、比率は
1.6〜4.8%の範囲で再現した。A01-A03での参考測定(§2.4冒頭)でも、末尾7〜20個の
再評価が全構築1回の3.2〜16.6%に収まっており、上記5シーンの結果と整合する。)

**全シーンで比率が判定基準の1/3(33%)を大きく下回った(最大でも4.8%)。**
副産物として、`beam_construct_order`(構築)は `simulate_order`(validate)より
実測で3〜10倍重いことも分かった(例: C02は construct 31.7s vs validate 10.2s)。
現行の `try_construct` は構築後に**同じ順序をもう一度 `simulate_order` で丸ごと再生**
しており(`validate()`)、この構築コストの支配性は接頭辞再開の恩恵をさらに大きくする
——ALNSで「末尾mだけ構築し直す」場合、構築コストもm個分だけで済むと期待できる
(ただし構築側の局所探索アルゴリズムはPhase34で実装するため、比率はあくまで
「評価(resume-simulate)側」の実測であり、「構築側」は概算の域を出ない)。

**留意点**: C02は端数予算が1.7秒しかなく、反復数目安は約1.1回にとどまる
(比率自体は3.5%で基準を満たすが、母数が小さいシーンでは反復回数が限られる)。
D01のように端数39.7秒・比率3.0%のシーンでは200回超の反復が理論上入る余地がある。
シーンごとに端数予算のばらつきが大きいため、実装時は「固定反復回数」ではなく
「端数予算に収まるだけ反復する」設計(Phase29の修正フェーズと同じ、予算駆動の
anytime設計)が引き続き妥当と考えられる。

### 2.5 判定

**1反復(resume-simulate側)のコストは全構築1回の1/3を大きく下回る(1.7〜4.7%)。
ALNSは端数予算に収まる。Phase34で実装を推奨する。**

---

## 3. Phase34への推奨

- **タスク1の判定を先に確定させる**(public スコア入手待ち)。効果が転移していれば
  (+0.3以上)、選択則側の残り(オラクル差+2.638のうち未回収+1.69)も並行して
  価値があるが、Phase32の結論通り「選択則の改良自体は打ち切り」の方針は維持する。
- **ALNSは実現可能と確認できたため、Phase34で実装に着手する。** 設計方針:
  - Phase29の衝突駆動リスタート(`REPAIR`)の枠組み(stall_info→item_blockers→
    多手移動候補生成)を、「順序全体の作り直し」ではなく「接頭辞再開+末尾mだけ
    resume-simulate」に置き換える。
  - 構築側(末尾mの並べ替え候補生成)は本フェーズでは未実装・未計測。Phase29の
    `_advance_before`/`_advance_to_front`/`_delay_blockers` のような軽量な手を
    まず接頭辞再開に載せ替えるのが最小の変更幅になる。
  - 反復回数は固定値ではなく端数予算(`total_budget.remaining()`)に応じたanytime設計。
  - Phase29で不採用だった主因(「(a)の大半は自分の順序修正の射程外」)は選択則の
    問題ではなく到達可能性の問題だったため、ALNS化そのものがこの主因を解消するとは
    限らない。ALNSの効果はPhase29と同じ「衝突を修正できるシーンの母数」に律速される
    可能性がある点は事前に認識しておく(Phase29 §結果参照)。

---

## 4. 再現手順

```bash
# タスク1: 決定的5シーンの無変更確認
MYSOLVER_UNITS_PER_SEC=1.55e7 PYTHONPATH=. .venv/bin/python tools/measure_regime.py \
  --config-path configs/gen/suite_B01_1c_40_plain.json configs/gen/suite_B02_1c_40_shelf.json \
                configs/gen/suite_B03_2c_80_prio.json configs/gen/suite_B04_2c_80_noprio.json \
                configs/gen/suite_P04_B_1c_pre8_shelf.json \
  --module-path agents/mysolver/ --repeats 1 --out /tmp/phase33_decisive5_floor.json \
  --label phase33_decisive5_floor_default
PYTHONPATH=. .venv/bin/python tools/phase29_cmp.py \
  --before results/phase31_noleak.json --after /tmp/phase33_decisive5_floor.json

# タスク1: 提出パッケージング
cd agents && zip -r ../mysolver_submit.zip mysolver -x "*__pycache__*" -x "*.pyc"

# タスク2: 接頭辞再開のビット単位一致(A01-A03)
MYSOLVER_UNITS_PER_SEC=1.55e7 PYTHONPATH=. .venv/bin/python tools/phase33_prefix_resume.py \
  --cand results/phase29_cand_g1.json \
  --scenes A01_1c_40_plain A02_1c_80_plain A03_1c_40_shelf \
  --out results/phase33_prefix_resume.json

# タスク2: 1反復のコスト比(A05/C02/C03/D01/P05、Phase29端数実測シーン)
MYSOLVER_UNITS_PER_SEC=1.55e7 PYTHONPATH=. .venv/bin/python tools/phase33_iter_cost.py \
  --cand results/phase29_cand_g1.json results/phase29_cand_g2.json \
  --out results/phase33_iter_cost.json
```
