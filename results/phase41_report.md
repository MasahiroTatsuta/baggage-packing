# Phase41 報告: replica.py を「未知の欠損に対して恒久的に頑健にする」防御的書き直し

## 0. 背景

`agents/mysolver/replica.py` の `ReplicaEvaluator` は、環境から渡される観測データ
(`container_list` / item 情報)を辞書として読んでいる。想定外の欠損があると `KeyError` で
評価が失敗する可能性がある。原因キーは未特定のため、特定を待たず「未知の欠損に対して
恒久的に頑健にする」(欠損があってもその候補だけ諦めて次に進む)方針で書き直した。

---

## ステップ1: 辞書アクセスの棚卸し

環境由来の observation dict(container_list の要素 `c` / item info の要素 `it`)への
角括弧アクセスを全て洗い出し、必須/代替可を判定した(判定基準: `Container`/`Item`
データクラスの既定値の有無、`items.py`/`containers.py`)。

| 行(旧) | キー | 対象 | 判定 | 根拠 |
|---|---|---|---|---|
| 231 | `c['index']` | container | 必須 | `Container.index` に既定値なし |
| 232 | `c['thickness']` | container | 必須 | `Container.thickness` に既定値なし |
| 233 | `c['length']` | container | 必須 | `Container.length` に既定値なし |
| 234 | `c['width']` | container | 必須 | `Container.width` に既定値なし |
| 235 | `c['height']` | container | 必須 | `Container.height` に既定値なし |
| 236 | `c['cut_x']` | container | 代替可 | `Container.cut_x = 0.3` |
| 237 | `c['cut_y']` | container | 代替可 | `Container.cut_y = 0.3` |
| 238 | `c['shelf']` | container | 代替可 | `Container.require_shelf = False` |
| 239 | `c['is_prioritized']` | container | 代替可 | `Container.is_prioritized = False` |
| 240 | `c['center'][2]` | container | 必須(※) | `buffer` 自体は既定値0.01を持つが、`center` は244行目(offset_x復元)にも使われ、そちらに対応する既定値が無い二重役割キーのため合成として必須扱い |
| 243 | `c['center'][0]` | container | 必須(※同上) | `Container.offset_x` に既定値なし(build()側で動的計算) |
| 263 | `it['index']` | item | 必須(既に対策済み) | 264-267行で try/except KeyError → None 済み |

`c.get('packed_items', [])` は変更前から `.get` 済みで対策不要。

---

## ステップ2: 防御的な書き直し

- `_containers_config()`: 代替可能キー(`cut_x`/`cut_y`/`shelf`/`is_prioritized`)を
  `.get(key, 既定値)` に置き換え。既定値は `Container` のフィールド既定値そのまま
  (勝手な値を作らない)。必須キー欠損・型異常は `KeyError`/`TypeError`/`AttributeError`/
  `IndexError` を自前で捕捉し `None` を返す(戻り値 `dict | None`)。
- `reset()`: 戻り値を `bool` に変更。`_containers_config()` が `None` を返した場合、
  および pybullet 環境構築中に同4例外が起きた場合は `False` を返す(例外を外に投げない)。
- `run_order()`: 既存の `by_idx` の try/except KeyError → None パターンはそのまま維持しつつ、
  メソッド全体を try/except で包み同4例外を吸収して `None` を返すよう統一。
- `evaluate()`: `reset()` の戻り値を確認して `False` なら `None` を返すよう変更。
  さらに将来の変更に対する二重の安全弁として同4例外を捕捉する try/except を追加。
  **`FORCE_FAIL=='runtime'` の障害注入(Phase38の例外自己申告テレメトリ検証用)はこの
  try/except の外側に置いたまま維持**(意図的にこの防御の対象外。理由は§4参照)。

diff: `git diff agents/mysolver/replica.py`(138 insertions, 104 deletions)。

---

## ステップ3: 動作確認

### 3-1. キー欠損テスト(13ケース、B01シーン実データで検証)

`_containers_config`/`run_order` に渡すデータから、キーを1つずつ削って `ReplicaEvaluator.evaluate()`
を直接呼んだ結果:

| キー | 種別 | 結果 |
|---|---|---|
| container.index | 必須 | OK(None, 例外なし) |
| container.thickness | 必須 | OK(None, 例外なし) |
| container.length | 必須 | OK(None, 例外なし) |
| container.width | 必須 | OK(None, 例外なし) |
| container.height | 必須 | OK(None, 例外なし) |
| container.center | 必須 | OK(None, 例外なし) |
| container.cut_x | 代替可 | OK(継続, fill=15.45 ※既定値0.3で幾何が変わるため実測と差、クラッシュしないことが確認点) |
| container.cut_y | 代替可 | OK(継続, fill=27.98) |
| container.shelf | 代替可 | OK(継続, fill=28.17 = ベースラインと一致。B01は元々shelf不要のため既定値Falseと実測が偶然一致) |
| container.is_prioritized | 代替可 | OK(継続, fill=28.17 = ベースライン一致) |
| item.height | 必須(Itemデータクラス) | OK(None, 例外なし) |
| item.mass | 代替可(Item.mass=1.0、dataclass既定値が自動適用) | OK(継続, fill=25.95) |
| 候補単位の独立性 | 1候補目を壊れたデータで評価→None、直後に同一インスタンスで2候補目を正常データで評価→ベースラインと完全一致(fill=28.1705) | OK |

**13件中NG 0件。**(検証スクリプト: セッションのスクラッチパッドに保存、リポジトリには含めない)

### 3-2. 決定的8シーンのビット単位不変(B01-B04, P04, A01-A03)

変更前(`git stash`で復元した原版)と変更後で `build_order()` の出力順序を比較。

**8/8 完全一致**(先頭10件・全長とも同一)。

```
B01: n=40 first10=[30, 13, 37, 26, 38, 22, 9, 3, 2, 17]
B02: n=40 first10=[27, 7, 6, 29, 38, 19, 23, 34, 12, 15]
B03: n=80 first10=[62, 36, 37, 7, 24, 30, 48, 9, 50, 25]
B04: n=80 first10=[51, 43, 55, 53, 23, 68, 63, 66, 54, 61]
P04: n=34 first10=[16, 15, 18, 22, 24, 3, 0, 21, 5, 29]
A01: n=40 first10=[13, 3, 35, 23, 17, 14, 30, 38, 2, 0]
A02: n=80 first10=[11, 26, 53, 60, 58, 74, 63, 8, 72, 22]
A03: n=40 first10=[13, 37, 28, 35, 22, 15, 25, 32, 29, 7]
```

`bp_check.sh` の軽量スモーク(A01, budget=15s)も例外なく完走。

### 3-3. 26シーンA/B(`bp_ab.sh phase41_defensive`)

- **OFF側**(`MYSOLVER_REPLICA_SELECT=0`、replica.pyを一切使わない経路)と対照
  `results/phase40_baseline_off_mac.json` を比較 → **26/26シーンで composite_strict・
  fill_strict とも完全一致(diff 0.000)**。OFF側はreplica.pyを構造的に通らないため
  この一致は当然の結果だが、他コードへの副作用がないことのスモークテストとして機能する。
- **ON側**(`MYSOLVER_REPLICA_SELECT=1`、replica.pyが実際に動く経路)は26シーン完走、
  例外・トレースバックなし。

  | 指標 | OFF | ON | 差分 |
  |---|---:|---:|---:|
  | composite_strict mean | 70.443 | 70.645 | +0.201 (t=1.007, 有意差なし) |
  | fill_strict mean | 24.340 | 26.011 | +1.671 |
  | -2.0pt超悪化シーン | — | 0件 | — |

  ON側の値がOFF側と異なるのはreplica.py(複製評価器)が候補順序を実際に選び直す設計上の
  想定どおりであり、悪化シーンゼロ・例外ゼロという結果は「防御的書き直しが正常系の判定ロジックを
  壊していないこと」を裏付ける。なお本フェーズの目的はバグ修正であり性能改善ではないため、
  t検定の有意性そのものは採否基準の対象外。

---

## 4. 発見事項(スコープ外につき今回は未対応)

`ordering.py` の `build_order()`(1179-1182行目)は `rep.evaluate()` が `None` を返した場合、
無条件で「壁時計deadline超過」と解釈しシーン全体を即ラッチする設計になっている
(従来 `None` は `run_order()` の deadline チェックと `FORCE_FAIL=='deadline'` のみが
返す値だったため)。

本フェーズで観測データ欠損時にも `None` を返すようにしたことで、**同じ `None` に
「壁時計超過」と「観測データ欠損」という2つの原因が衝突する**。従来(例外を投げていた
版)は `except Exception as e:` 経路(`REPLICA_LATCH_MODE=per_candidate`)で
「1候補失敗→続行、2回連続失敗でラッチ」という緩やかな扱いだったが、`None` 化により
**1候補目の失敗で即座にシーン全体をラッチ**するようになる。

container_list はシーン内の全候補で共有されるため実害は限定的(欠損があればどのみち
大半の候補が同様に失敗する)と考えられるが、「1候補が失敗しても残りの候補の評価が続く」
という要件を ordering.py 側は現状厳密には満たしていない。今回は依頼スコープを replica.py
に限定し、`ordering.py` の `got is None` 分岐(Phase38の壁時計テレメトリ埋め込みと密結合)
には手を入れていない。詳細と対応方針案はローカルメモリ
(`replica-none-vs-deadline-conflation`)に記録した。

---

## 5. 変更ファイル

- `agents/mysolver/replica.py`(本体)
- `results/bp_ab_phase41_defensive_off.json` / `results/bp_ab_phase41_defensive_on.json`(3-3の実測)
- `results/phase41_report.md`(本ファイル)
