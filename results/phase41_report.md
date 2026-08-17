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

## 5. 変更ファイル(Phase41時点)

- `agents/mysolver/replica.py`(本体)
- `results/bp_ab_phase41_defensive_off.json` / `results/bp_ab_phase41_defensive_on.json`(3-3の実測)
- `results/phase41_report.md`(本ファイル)

---
---

# Phase42 追記: None の多義性解消 + cut_x/cut_y/shelf の格下げ

## 0. 背景(Phase41 §4 の転記)

Phase41 §4 で報告した問題は、実はスコープ外ではなく Phase41 本来の目的
(「1候補が失敗しても残りの候補の評価が続く」)を無効化する退行だった。以下、
セッション外に残らないローカルメモリ(`replica-none-vs-deadline-conflation`)に
記録していた内容をそのままここへ転記する。

> `agents/mysolver/ordering.py` の `build_order`(1147行目付近)は `rep.evaluate()` の
> 戻り値が `None` のとき、無条件で `rstats['stopped']='wall_deadline'; latched=True; break`
> にする(1179-1182行目)。これは元々「壁時計 deadline 超過」専用のシグナルとして設計されている
> (`run_order()` の deadline チェック、`FORCE_FAIL=='deadline'` のみが None を返す前提)。
>
> Phase41(`replica.py` を「未知の欠損に対して恒久的に頑健にする」防御的書き直し)で
> `_containers_config`/`reset`/`run_order`/`evaluate` が観測データの欠損
> (KeyError/TypeError/AttributeError/IndexError)に対しても例外を投げず `None` を返す
> ようにした。これにより **同じ None という戻り値に2つの異なる原因(壁時計超過 / 観測データ欠損)
> が衝突する**。
>
> 影響: 例外を投げていた頃は ordering.py の `except Exception as e:` 経路
> (`REPLICA_LATCH_MODE=per_candidate` 既定)で「1候補失敗→続行、2回連続失敗でラッチ」
> という緩やかな扱いだったが、None を返すようになると `got is None:` 分岐に落ちて
> **1回目の失敗で即座にシーン全体をラッチ**してしまう。観測データの欠損はシーン内の
> 全候補で同じ container_list を共有するため実害は限定的(どのみち大半の候補で失敗する)
> と考えられるが、規定どおりの「1候補だけ諦めて残りを評価する」という要求を
> ordering.py 側は現状満たせていない。
>
> **意図的にordering.pyは変更していない**(タスクの依頼スコープが replica.py に限定されて
> おり、ordering.py の `got is None` 分岐は Phase38 の壁時計テレメトリ埋め込みと
> 密結合していて、不用意に触ると `FORCE_FAIL=='deadline'` のテスト仕組みや n=4/n=5 の
> 符号化を壊しかねないため)。

さらに本番の実測は a=0(1候補目で失敗)であることが Phase38 で確定している。この状態で
「1候補目の失敗=シーン全体を諦める」のままにすると、**Phase41の効果が最も出てほしい
ケースを自分で潰すことになる**ため、本フェーズで対応した。

---

## ステップ1: None の多義性を解消する

### 1-1. 設計案の選定

3案を検討した:

- (a) `evaluate()` の戻り値を `(status, result)` のタプルにする(`status` は
  `'ok'`/`'deadline'`/`'data_error'`)
- (b) 専用の番兵オブジェクトを2種類用意する(`_DeadlineSentinel`/`_DataErrorSentinel`)
- (c) `evaluate()` の戻り値は `dict | None` のまま据え置き、直近の失敗理由を
  `REPLICA_STATS` 経由で ordering.py が読む

**(a) を採用した。** 理由:

1. (b) は「どちらの番兵か」を `isinstance`/`is` で判定できるだけで、**例外分類情報
   (どの例外クラスだったか)を運べない**。Phase38 の `_classify_exception()` は
   例外オブジェクトそのもの(`type(exc)`と`exc.__traceback__`)を必要とするため、
   番兵化すると「テレメトリのために原因例外を別途どこかに保持する」仕組みが結局
   追加で必要になり、(a)より複雑になる。
2. (c) は `REPLICA_STATS` の更新タイミングが `run_order()` 内の複数の return
   ポイント(by_idx欠損・deadline超過・outer except)に分散し、「いつ読むべきか」の
   同期をordering.py側に追加で作り込む必要がある。関数の戻り値だけで完結する(a)より
   状態管理の経路が増え、Phase38が既に密結合している箇所を壊すリスクが上がる。
3. (a) は payload に **原因例外オブジェクトそのもの**を積めるため、ordering.py が
   受け取った `payload` をそのまま `_classify_exception()` に渡せば、Phase38の
   `exc_class`/`exc_a`/`exc_b`/`exc_code` 符号化(壁時計テレメトリの n=4 埋め込み)を
   **一切変更せずに**再利用できる。`exc.__traceback__` は例外が replica.py 内で
   捕捉された時点のフレームを保持したままなので、例外を投げていた頃と全く同じ
   分類結果になる。これが最優先事項(「壁時計テレメトリ埋め込みを壊さないこと」)を
   満たす決め手になった。

### 1-2. 実装

`agents/mysolver/replica.py`:
- `ReplicaEvaluator.__init__` に `self._last_error: Exception | None = None` を追加
  (`_containers_config`/`reset` が捕捉した例外を一時保持するサイドチャネル)。
- `_containers_config()`/`reset()`: 例外捕捉時に `self._last_error = e` を追加(戻り値の
  型 `dict | None` / `bool` 自体は変更しない)。
- `run_order()`: 戻り値を `dict | None` から `(status, payload)` のタプルに変更。
  `'ok'` → `(status, result_dict)`、deadline超過 → `('deadline', None)`、
  by_idx欠損や outer except で捕捉した例外 → `('data_error', 例外)`。
- `evaluate()`: 同じく `(status, payload)` を返す。`reset()` が `False` を返したら
  `('data_error', self._last_error)`。`FORCE_FAIL=='deadline'` は `('deadline', None)`。
  **`FORCE_FAIL=='runtime'` の障害注入(Phase38検証用)は意図的にこの変更の対象外**
  ——従来どおり素の例外を投げ続け、ordering.py の `except Exception as e:` 経路を通す
  (この経路自体がPhase38のテレメトリ検証対象そのものなので、data_error化すると
  検証手段を失う)。

`agents/mysolver/ordering.py`:
- `_classify_exception()` の直後に `_record_replica_failure(rstats, exc, rank)` を新設。
  従来 `except Exception as e:` 節にインラインで書かれていた
  exc_events追記 + exc_class/exc_a/exc_b/exc_code の初回確定ロジックをそのまま関数化。
- `rep.evaluate(...)` の呼び出しを `status, payload = rep.evaluate(...)` に変更。
  - `except Exception as e:`(replica.py が捕捉しなかった例外。pybullet.error/
    MemoryError/RuntimeError や `FORCE_FAIL=='runtime'` の障害注入など)→
    `_record_replica_failure(rstats, e, rank)` を呼んで**従来どおり**候補単位ラッチ。
  - `status == 'deadline'` → 従来どおり無条件でシーン全体ラッチ(壁時計が尽きているので
    当然)。
  - `status == 'data_error'`(**新設**)→ `_record_replica_failure(rstats, payload, rank)`
    を呼んで、上の `except Exception as e:` 節と**全く同じ**候補単位ラッチ
    (`REPLICA_LATCH_MODE=per_candidate` の2回連続失敗でシーンラッチ)を適用する。
  - `status == 'ok'` → 従来どおり(`got = payload` として以降の処理はそのまま)。

diff: `git diff agents/mysolver/replica.py agents/mysolver/ordering.py`。

### 1-3. 動作確認

B01シーンの先頭16アイテムのみを使い(各リスタートを軽くして複数候補を素早く確保するため)、
`ordering.build_order()` の `_replica_mod` をフェイクの `ReplicaEvaluator` に差し替えて
4パターンを検証した(`ordering.REPLICA_STATS` を直接読む)。

| ケース | 入力パターン | 結果 |
|---|---|---|
| 1候補目 data_error → 以降 ok | KeyError → ok,ok,... | **全ランク候補(n_ranked=2)が実際に呼ばれ、evaluated=1、`latched=False`・`stopped='done'`**。exc_code=2(KeyError)。**「1候補目の失敗で2候補目以降が評価されなくなる」という Phase41 §4 の退行が解消したことを直接示す**。 |
| 先頭2候補が連続 data_error | TypeError, TypeError | 2回呼ばれた時点で `stopped='runtime_error'`・`latched=True`(3候補目は呼ばれない)。`REPLICA_LATCH_MODE=per_candidate` の「2回連続失敗でラッチ」どおり。exc_code=6(TypeError)。 |
| 1候補目 deadline | deadline | 1回だけ呼ばれて即 `stopped='wall_deadline'`・`latched=True`。**deadline由来は従来どおり即ラッチすることを確認**(2候補目は呼ばれない)。 |
| 非連続 data_error(1,3候補目が失敗、2候補目は成功) | — | このシーン/予算では n_ranked が2までしか安定して確保できず(3候補以上を要するこのケースは)スキップ。上記3ケースで候補単位ラッチの主要な分岐(継続/2連続ラッチ/deadline即ラッチ)は網羅済み。 |

---

## ステップ2: cut_x / cut_y の判定を格下げ、他キーの再点検

### 2-1〜2-2. cut_x / cut_y → 必須へ

Phase41 §3-1 で `cut_x` 欠損時に既定値0.3で継続すると `fill=15.45`
(実測ベースライン28.17)という**別人の値**を返すことが判明していた。これは
クラッシュせず沈黙して間違った値を返す分、クラッシュより悪い(候補選択則はこの値の
argmaxを取るため)。`cut_x`/`cut_y` を「代替可」から「必須」へ格下げし、
欠損時は他の必須キーと同じ `data_error` としてその候補をスキップするようにした
(`_containers_config()` の `float(c['cut_x'])`/`float(c['cut_y'])` を `.get()` から
素の `[]` アクセスに戻し、既存の except で吸収させる)。

### 2-3. 他の代替可キーの再点検(判定基準: 幾何・質量・物理係数に影響するか)

| キー | 対象 | 旧判定 | 再点検結果 | 根拠 |
|---|---|---|---|---|
| `shelf`(→`require_shelf`) | container | 代替可 | **格下げ(必須)** | `Container.create()`(containers.py)は `require_shelf` に応じて `shelf_volume` を計算し、`self.volume` から差し引く。この `volume` は `Evaluator.calculate_fill_rate()`(evaluator.py)の `total_container_volume`(=fillの**分母**)にそのまま使われるため、`shelf` 欠損時に既定値Falseで継続すると **fillの値そのものが変わる**(cut_x/cut_yと同一クラスの危険)。さらに `shelf_bullet_id`/`small_shelf_bullet_id` は質量0の物理ボディとして実際に生成され、配置判定(`check_inclusion`/`check_transport_path`/`place_item`)も物理的に変わる。**タスク文中の「shelf は幾何に影響しないので代替可のままでよい」という前提は、コード上の根拠と矛盾するため今回訂正した。** |
| `is_prioritized` | container | 代替可 | **維持(代替可)** | `containers.py` 内で `Container.create()`/volume計算のどこにも参照されていない(`get_item_info_in_containers()` での書き出し専用)。幾何・質量・物理係数への影響はゼロ。ただし `planner.py` は `is_prioritized`(container/item双方)を意思決定の入力に多用しており(`prio_term`/`tier`割当/配置制限など約10箇所)、既定値Falseが実態と異なれば**選ばれる配置系列自体は変わりうる**。これは「幾何・質量・物理係数」という今回の判定基準の対象外(方策の意思決定バイアス)なので格下げはしないが、注意点として明記する。 |
| `mass`/`lateralFriction`/`rollingFriction`/`spinningFriction`/`restitution`/`angularDamping`/`is_soft` | item(Itemデータクラスのオプションフィールド) | (Phase41表になし。`Item(**dict)`のdataclass既定値で暗黙に補われる) | **既知のリスクとして報告のみ、今回は対応しない** | 判定基準に照らすと `mass`=質量そのもの、摩擦・反発係数・減衰=物理係数そのものであり、本来は`cut_x`/`shelf`と同じ危険区分に入る。ただし影響範囲は1アイテムの沈み込み方・摩擦挙動に限られ、fillの**分母**(コンテナ全体の幾何)には影響しないため、`shelf`ほど深刻ではない。Phase41の棚卸し表(角括弧アクセスの列挙)にはそもそも含まれていない(dataclassの言語機能による暗黙補完のため)。必須化するには `Item(**dict)` 呼び出し前に明示的な欠損チェックを追加する実装が別途必要で、本フェーズのスコープ(cut_x/cut_yの再点検に端を発したステップ2)を超えるため、今回はコード変更せず記録に留める。 |

---

## ステップ3: 再検証

### 3-1. キー欠損テスト再実行(14ケース)

Phase41の13ケースに、cut_x/cut_yの回帰確認(「以前はfill=15.45を返していたが、今回は
data_errorになりargmaxを汚染しない」)を1件追加し、必須/代替可のリストをステップ2の
判定に合わせて更新して再実行した。

**14件中NG 0件。** cut_x/cut_y/shelf は data_error(KeyError)、is_prioritized は
既定値False継続(fill=28.1705でベースライン一致)、候補単位の独立性も維持を確認。

### 3-2. 決定的8シーンのビット単位不変

Phase41コミット済み版(`de9e403`)と現在の作業ツリーで `build_order()` の出力を比較。

**8/8 完全一致**(B01-B04, P04, A01-A03、先頭10件・全長とも同一)。
`bp_check.sh` の軽量スモークも同一のfill_score=27.22で完走。

### 3-3. 26シーンA/B(`bp_ab.sh phase42_latch_fix`)

- **OFF側**: `results/phase40_baseline_off_mac.json` と **26/26シーンで完全一致(diff 0.000)**。
- **ON側**: 26シーン完走、例外・トレースバックなし、**-2.0pt超の悪化シーンは0件**。

  | 指標 | Phase41 ON | Phase42 ON | 差分 |
  |---|---:|---:|---:|
  | composite_strict mean | 70.645 | 70.645 | 0.000 |
  | fill_strict mean | 26.011 | 26.011 | 0.000(Phase41報告の+1.671はOFF比較。ON自体はPhase41から不変) |
  | composite_strict vs OFF(t検定) | t=1.007 | t=1.007 | — |

  **Phase42のON側26シーン結果は、シーン単位でPhase41のON側と完全一致(diff 0.000、
  26/26)だった。** 実運用の26シーンには元々欠損データが無いため、`(status, payload)`
  タプル化・`cut_x`/`cut_y`/`shelf`の必須格上げのどちらも通常経路の判定結果には
  一切影響しないことが、8シーンのビット単位一致(3-2)よりさらに広い26シーン全件で
  裏付けられた。

---

## 4. 変更ファイル(Phase42時点)

- `agents/mysolver/replica.py`(戻り値を `(status, payload)` タプルへ、cut_x/cut_y/shelf を必須へ)
- `agents/mysolver/ordering.py`(`_record_replica_failure()` 新設、`got is None` を
  `status` 判定に置き換え、data_error を候補単位ラッチへ)
- `results/bp_ab_phase42_latch_fix_off.json` / `results/bp_ab_phase42_latch_fix_on.json`(3-3の実測)
- `results/phase41_report.md`(本ファイル、Phase42追記)

---
---

# Phase43 追記: 提出まとめ

## 0. 目的

Phase42(replica.pyの防御的書き直し + 候補単位ラッチの復活)が本番の実行時エラー
(n=4、a=0で1候補目から失敗)を解消するかを確認する。**ローカル26シーンには欠損データが
無いため(Phase42 §3-3で確認済み)、この効果はローカルでは検証できず、本番提出でのみ
判定できる。**

## ステップ1: 提出用zip

### 1-1〜1-2. 生成

診断用の仕掛けは追加せず、リポジトリの現状(Phase42コミット `be66351`)をそのまま
`agents/mysolver/` から zip 化した(`cd agents && zip -r ../submissions/mysolver_submit_phase42.zip ./mysolver -x '*__pycache__*' -x '*.pyc'`。既存の `submissions/mysolver_submit_phase38_probe.zip` と同じ
「`mysolver/` 直下に10個の.pyファイル」という構造を再現)。

- **出力先**: `submissions/mysolver_submit_phase42.zip`
- **SHA256**: `5bd659e0c9036541e69d6ec620cfd5afa82804603b1ff2d960cd7b8973b064af`

### 1-4. 環境変数の既定値(zip内 `mysolver/ordering.py` を直接grep)

```
$ unzip -p submissions/mysolver_submit_phase42.zip mysolver/ordering.py | grep "os.environ.get('MYSOLVER_TELEMETRY'\|...HARD_WALL_LIMIT'\|...REPLICA_SELECT'\|...REPLICA_METRIC'\|...REPLICA_LATCH_MODE'"
HARD_WALL_LIMIT = float(os.environ.get('MYSOLVER_HARD_WALL_LIMIT', '165.0'))
REPLICA_SELECT = os.environ.get('MYSOLVER_REPLICA_SELECT', '1') == '1'
MYSOLVER_TELEMETRY = os.environ.get('MYSOLVER_TELEMETRY', '0') == '1'
REPLICA_METRIC = os.environ.get('MYSOLVER_REPLICA_METRIC', 'fill')
REPLICA_LATCH_MODE = os.environ.get('MYSOLVER_REPLICA_LATCH_MODE', 'per_candidate')
```

5項目とも指示どおりの既定値(TELEMETRY='0'、HARD_WALL_LIMIT='165.0'、REPLICA_SELECT='1'、
REPLICA_METRIC='fill'、REPLICA_LATCH_MODE='per_candidate')。診断用の仕掛け(FORCE_FAIL等)は
そもそも既定で空文字・無効化されており、今回コードも変更していない。

### 1-3. zip内コードとリポジトリのビット単位一致(8/8)

zipを別ディレクトリへ展開し、`sys.path` に展開先の親ディレクトリとリポジトリルートの
両方を通して `mysolver.ordering`(zip版)と `agents.mysolver.ordering`(リポジトリ版)を
別々のパッケージとして同一プロセスに import し、決定的8シーンの `build_order()` 出力を
比較した。

```
[B01] repo_n=40 zip_n=40 OK 完全一致
[B02] repo_n=40 zip_n=40 OK 完全一致
[B03] repo_n=80 zip_n=80 OK 完全一致
[B04] repo_n=80 zip_n=80 OK 完全一致
[P04] repo_n=34 zip_n=34 OK 完全一致
[A01] repo_n=40 zip_n=40 OK 完全一致
[A02] repo_n=80 zip_n=80 OK 完全一致
[A03] repo_n=40 zip_n=40 OK 完全一致
```

**8/8 完全一致**(順序の長さだけでなく `build_order()` の返り値そのものを `==` 比較、
先頭要素だけでなく全長一致)。

**アップロード時は必ずこのSHA256(`5bd659e...b064af`)と照合すること**
(Phase37で別のzipを誤ってアップロードした事例があるため)。

---

## ステップ2: 判定基準(次回提出結果を見るときのために先出し)

提出後の `fill_score` を見て判断する。**判定は fill_score の1点で足りる。**

- `fill_score` が **`38.09476291926298` から動いた** →
  ρ-testが本番で動き出した。Phase35以降はじめての前進。次は
  `MYSOLVER_REPLICA_METRIC=composite` の提出を検討する(ローカル26シーンA/Bで
  t=2.424・悪化0件を確認済み、詳細はPhase37/38報告参照)。
- `fill_score` が **`38.09476291926298` のまま13桁一致** →
  まだ失敗している。原因は観測データの欠損ではなかったことになる。その場合は
  `REPLICA_STATS` の `_last_error`(またはordering.py側の `exc_class`/`exc_code`)に
  本番で何が入るかを読む手段が無いと切り分けが進まないため、方法を改めて相談する。

---

## ステップ3: 検証スクリプトのリポジトリ化

Phase41 §3-1(13ケース)・Phase42 §3-1(14ケース)・Phase42 §1-3(ラッチ動作検証)は
いずれもセッションのスクラッチパッドに置かれておりリポジトリに残っていなかった。
次に replica.py / ordering.py を触るときの回帰テストとして `tools/` 配下に置いた:

- **`tools/test_replica_missing_keys.py`**(Phase41 §3-1 + Phase42 §3-1 相当、14ケース):
  観測データから必須キー/代替可能キーを1つずつ削り、`ReplicaEvaluator.evaluate()` が
  `('data_error', 例外)` / `('ok', dict)` を正しく返し分けることを確認する。

  ```
  MYSOLVER_HARD_WALL_LIMIT=3000 PYTHONPATH=. python tools/test_replica_missing_keys.py
  ```

- **`tools/test_replica_latch.py`**(Phase42 §1-3 相当、3〜4ケース):
  `ordering.build_order()` の `_replica_mod` をフェイクの `ReplicaEvaluator` に
  差し替え、`data_error`(候補単位ラッチ)と `deadline`(即ラッチ)が正しく区別され、
  「1候補目失敗→2候補目以降が実際に評価される」ことを確認する。

  ```
  MYSOLVER_HARD_WALL_LIMIT=3000 PYTHONPATH=. python tools/test_replica_latch.py
  ```

いずれも終了コード0=全ケースOK、副作用なし(results/には書かない)。リポジトリルートで
実行すること(`sys.path.insert(0, '.')` を前提にしている、他の `tools/` 配下のスクリプトと
同じ流儀)。

---

## 4. 変更ファイル(Phase43時点)

- `submissions/mysolver_submit_phase42.zip`(提出用zip、診断用仕掛けなし)
- `tools/test_replica_missing_keys.py`(Phase41/42のキー欠損テストをリポジトリ化)
- `tools/test_replica_latch.py`(Phase42のラッチ動作テストをリポジトリ化)
- `results/phase41_report.md`(本ファイル、Phase43追記)
