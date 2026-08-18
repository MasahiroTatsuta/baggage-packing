# Phase47 報告: 配布された sample_config で本番相当の入力を測る

## 0. 前提

`configs/sample_config.json` は運営配布の本物のシーン定義であり、Phase38で
validatorパラメータ(inclusion_margin=-0.005 / start_z=0.08 / safety_margin=0.015 /
ceiling_margin=0.018 / settle_wait_step=300)がreplica.pyの決め打ち値と完全一致することを
確認済みのため、本番と同じ設定で走らせられる。

## ツールの修正(報告に先立って)

`tools/diagnose_stacking.py` に、5成分すべてを記録するよう機能追加する過程で
**順序バグ**を発見・修正した。従来は `Scorer.evaluate()`(内部で fill→placement→
soft→cog→**stability**の順に計算し、**stabilityは蓋をして揺らす破壊的な計算**)を
1回のブラックボックス呼び出しにした後で、本ツール独自に `_find_stacking_pairs()`と
geoダンプを行っていた。これは**揺らし後の状態**を数えることになり、
placement_score/soft_item_scoreが確定した時点(揺らし前)の状態と食い違っていた。

- **影響を受けないもの**: `placement_score`/`soft_item_score` 自体(`evaluate()`内部で
  stability計算より前に確定するため、Phase44〜46で報告した数値はいずれも正しい)。
- **影響を受けたもの**: 本ツールが追加で出す `n_stacking_pairs`/`n_prio_crushed_pairs`/
  `n_soft_crushed_pairs` と、Phase44の幾何クロスチェックに使ったgeoダンプ
  (揺らし後の位置を見ていた)。ただし揺らしは「輸送中の穏やかな振動を模した上で
  再沈静化」する処理であり、下敷き関係が大きく入れ替わるほどの撹拌ではないため、
  「違反0件」という**結論そのものが覆る可能性は低い**と判断している(placement_score/
  soft_item_score自体が揺らし前の値で0件なのは事実として動かない)。過去フェーズの
  再計測はスコープ外とし、本フェーズ以降の計測はすべて修正後のツールで行った。
  `Scorer.evaluate()`と同じ順序(fill→placement→soft→_find_stacking_pairs/geo→cog→
  stability)で個別に呼ぶよう修正した。

---

## ステップ1: sample_config を走らせる

`MYSOLVER_HARD_WALL_LIMIT=3000 PYTHONPATH=. python tools/diagnose_stacking.py --config-path configs/sample_config.json --out results/phase47_sample_config_diag.json`

読み込み部分の変更は不要だった(`glob.glob('configs/sample_config.json')` は
ワイルドカードが無くても存在するファイルパスをそのまま1要素のリストで返す)。
`configs/sample_config.json` 自体は未変更。

| 指標 | タスク000 | タスク001 |
|---|---:|---:|
| fill_score | 27.97 | 22.85 |
| cog_score | 55.87 | 56.64 |
| stability_score | 97.86 | 97.90 |
| **placement_score** | **100.00** | **100.00** |
| **soft_item_score** | **100.00** | **100.00** |
| 配置数/総数 | 23/41 | 25/42 |
| episode_status | Stopped in the middle. Did not satisfy ['is_included', 'is_valid', 'is_placed_safe'] | 同左 |
| 積み重なりペア数 | 16 | 13 |
| 優先下敷き数 | 0 | 0 |
| ソフト下敷き数 | 0 | 0 |
| 配置された優先手荷物 | 3 | 1 |
| 配置されたソフト貨物 | 0 | 7 |

### 判定

**両タスクとも100.00のまま。** 本番相当の入力(配布sample_config、本番同一の
validatorパラメータ)を実際に走らせても違反は再現しなかった。

指示の分岐に従い判定する:**「100のまま」→ 判定基準の食い違いが濃厚。** ただし
これは同時に、**「本番相当の入力に対してローカル評価系(tools/scorer.py)が
満点を返す」という事実そのもの**でもある。したがって、
**本番の57.6/47.15はこのローカル評価系では原理的に再現できない**という結論になる。
`+13.61`の空白地帯は、少なくとも現状の`tools/scorer.py`の実装とローカル環境の
組み合わせでは測れる状態にない。fill/cog側のレバーに戻る判断材料として記録する。

---

## ステップ2: prio比率を「下げる」方向の検証

`tools/gen_suite_ratio_stress.py` に `lowprio5`(prio目標5%)/`lowprio10`(prio目標10%)を
追加した(既存の6シーンは同一シード順序を保ったため無変更。決定的に確認済み)。
soft比率はsample_configに合わせて30%に固定。prio=5%・n=40は期待値2個で二項乱数の
ばらつきにより0個になる回もあったため(既定シード5006では実際に0個)、
プールに優先手荷物が2個以上存在するseed=5014へ明示的に差し替えた(理由をコード
コメントに記録)。

| シーン | 実測soft% | 実測prio% | 配置数 | 配置prio | 配置soft | ペア数 | placement | soft_item |
|---|---:|---:|---:|---:|---:|---:|---:|---:|
| ratiostress_lowprio5 | 30%(実測) | 5%(2/40) | 23/40 | **1** | 5 | 15 | 100.0 | 100.0 |
| ratiostress_lowprio10 | 30%(実測) | 10%(3/40) | 18/40 | **2** | 0 | 11 | 100.0 | 100.0 |

配置された優先手荷物が1個・2個という、まさに「1件の違反でplacementが0や50になりうる」
分母の小さい状況でも、**違反は0件で placement_score は100.00のまま**だった。

分母が小さい(1〜2個)状況でも回避できているという事実は、ステップ1の結論
(「plannerが本当に回避できている、判定基準の食い違いが濃厚」)をさらに補強する。

---

## ステップ3: sample_config構成のローカルスイート反映の検討

**実施しなかった。** 指示の実施条件(「ステップ1で違反が再現した場合のみ実施する」)が
成立しなかったため(ステップ1は「100のまま」で確定し、違反は再現しなかった)。

---

## まとめ

Phase44〜47を通じて、以下の**36件のローカル計測**(26既存シーン+6構成比stress+
sample_config2件+lowprio2件)のいずれでも `placement_score`/`soft_item_score` は
一度も100.00を下回らなかった。**本番相当の入力(sample_config、本番同一validator設定)
を含めて再現しない**ことが確定したため、**原因は`tools/scorer.py`の判定基準
(接触閾値0.7・許容誤差・上方向の定義等)が本番の非公開判定式と食い違っていることに
ほぼ絞られた**。この先の直接検証手段は無い(本番の判定式が非公開のため)。
次の一手として、`+13.61`の空白地帯を諦めて既存のfill/cogレバーへ戻るか、
`tools/scorer.py`の閾値・許容誤差を意図的に変えて(ただし今回は指示により変更禁止)
違反が出る条件を探るか、の判断が必要になる。

---

## 変更ファイル

- `tools/diagnose_stacking.py`(順序バグ修正 + cog_score/stability_score記録を追加)
- `tools/gen_suite_ratio_stress.py`(lowprio5/lowprio10シーンを追加)
- `configs/gen/ratiostress_lowprio5.json` / `ratiostress_lowprio10.json`(新規、2シーン)
- `tools/suite_manifest_ratio_stress.json`(更新)
- `results/phase47_sample_config_diag.json`(ステップ1の実測データ)
- `results/phase47_lowprio_diag.json`(ステップ2の実測データ)
- `results/phase47_report.md`(本ファイル)

`configs/sample_config.json`・既存26シーン(`configs/gen/suite_*.json`)・
`tools/scorer.py` はいずれも変更していない。
