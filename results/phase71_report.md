# Phase71 報告: 緩3の提出、主枠の再入れ替え、順序制御(is_soft/is_prioritized)の調査

## 結論(先出し)

**緩3(`mysolver_submit_loose3.zip`)を作成した**(判定は本番結果待ち)。
**主枠を緩2(public 57.18)に、2枠目を緩1(public 56.05)に更新し、Phase55は
枠から外した。** 順序制御の調査では、**is_softを最優先キーとする「ハード先積み」
規則はordering.pyに既に存在する**が、**ランダムリスタート(Phase2)ではこの規則が
シャッフルにより壊れる**ことがコードから確認できた。

---

## 緩2の本番結果(再掲)

public 56.05 → **57.18(+1.14)**。配置率62.25% → **64.34%(+2.09pp)**。

| 成分 | 変化 | 換算寄与 | 割合 |
|---|---:|---:|---:|
| soft_item | +5.25 | +0.752 | 66.0% |
| placement | +1.65 | +0.236 | 20.8% |
| fill | +0.71 | +0.203 | 17.8% |
| stability | +0.18 | +0.026 | 3.4% |
| **cog** | **−0.42** | **−0.060** | **−8.0%** |

配置率の伸びは加速している(緩1: +0.88pp → 緩2: +2.09pp)。**cogが初めて
マイナスに転じた**——支持を緩めすぎるコストが出始めた兆候として記録する。

---

## ステップ1: 緩3のzip

### (1-1) 閾値3定数をzip内でのみ既定値へ固定

`agents/mysolver/planner.py`のzipビルド用コピーのみ書き換え(リポジトリ追跡
ファイルは無変更):

```diff
-MIN_UNION_SUPPORT_RATIO = float(os.environ.get('MYSOLVER_MIN_UNION_SUPPORT_RATIO', '0.55'))
+MIN_UNION_SUPPORT_RATIO = float(os.environ.get('MYSOLVER_MIN_UNION_SUPPORT_RATIO', '0.25'))
-MIN_SUPPORT_SPAN_RATIO = float(os.environ.get('MYSOLVER_MIN_SUPPORT_SPAN_RATIO', '0.6'))
+MIN_SUPPORT_SPAN_RATIO = float(os.environ.get('MYSOLVER_MIN_SUPPORT_SPAN_RATIO', '0.3'))
-MAX_SUPPORT_CENTROID_OFFSET = float(os.environ.get('MYSOLVER_MAX_SUPPORT_CENTROID_OFFSET', '0.15'))
+MAX_SUPPORT_CENTROID_OFFSET = float(os.environ.get('MYSOLVER_MAX_SUPPORT_CENTROID_OFFSET', '0.30'))
```

`git diff --stat agents/mysolver/`は空。zipビルド用コピーとリポジトリの差分もこの3行のみ。

他7項目をzip内コードから直接grep:

```
planner.py:1394: MYSOLVER_STRICT_SUPPORT_DISABLE', '0'
agent.py:39:     MYSOLVER_FALLBACK_SAFE_POS', '1'
agent.py:47:     MYSOLVER_FALLBACK_AVOID_OBSTACLES', '1'
ordering.py:108: MYSOLVER_HARD_WALL_LIMIT', '165.0'
ordering.py:475: MYSOLVER_REPLICA_SELECT', '0'
ordering.py:505: MYSOLVER_TELEMETRY', '0'
ordering.py:526: MYSOLVER_REPLICA_METRIC', 'fill'
```

**7項目すべて指示どおりの既定値。**

### (1-2) zip生成・決定的8シーンでの差分確認

- **出力先**: `submissions/mysolver_submit_loose3.zip`(11ファイル、既存構造と同一)
- **SHA256**: `c72ba8afd17d8332fa1c84e122256efdda495015fadc707bb85be517e6adddc7`

決定的8シーン(B01-B04, P04, A01-A03)でzip版とリポジトリ版の`build_order`を比較:

```
[B01] repo_n=40 zip_n=40 一致
[B02] repo_n=40 zip_n=40 ★差分あり
[B03] repo_n=80 zip_n=80 一致
[B04] repo_n=80 zip_n=80 一致
[P04] repo_n=34 zip_n=34 一致
[A01] repo_n=40 zip_n=40 一致
[A02] repo_n=80 zip_n=80 一致
[A03] repo_n=40 zip_n=40 ★差分あり

差分ありシーン数: 2/8(B02, A03)
```

**緩1・緩2と同じ2/8だが、対象シーンが変わった。** 緩1・緩2はいずれも(A01, A03)で
差分が出ていたが、緩3では**A01が一致に戻り、代わりにB02で新たに差分が出た**。
差分件数は単調に増えるわけではなく、閾値を動かすたびにどのシーンが動くかが
非単調に変化することが実測で分かる(参考値であり、28シーン全体の効果の大小を
代表するものではない)。

### (1-3) アップロード時の照合

**アップロード時は必ずSHA256
(`c72ba8afd17d8332fa1c84e122256efdda495015fadc707bb85be517e6adddc7`)と照合すること。**

### 判定基準(先出し)

- **57.18を超えた** → 緩4(0.15/0.2/0.35)へ。ただしcog/stabilityの符号を必ず確認する。
- **57.18前後** → 閾値軸は飽和。緩2を主枠に確定し、次の軸へ。
- **下がった** → 緩2が最適点。主枠を緩2に確定。

現時点では提出・判定は未実施。zip作成とローカル確認のみ完了。

---

## ステップ2: 提出枠の更新

- **主枠**: `submissions/mysolver_submit_loose2.zip`(public 57.18)
- **2枠目**: `submissions/mysolver_submit_phase67_loose1.zip`(public 56.05)
- Phase55(53.64)は枠から外した。
- `docs/submission_policy.md`§1にPhase71追記、§5の主枠・2枠目まとめ表を更新した。

---

## ステップ3: 「ハード先積み」の順序制御調査(実装なし・報告のみ)

### (3-1) `ordering.build_order`はis_soft/is_prioritizedを順序決定にどう使っているか

`agents/mysolver/ordering.py`の4つの初期順序戦略
(`_strategy_volume_desc`/`_strategy_count_first`/`_strategy_big_first`/`_strategy_layer_first`、
L125-176)は、いずれも同じ形のソートキーを使っている:

```python
def key(item):
    return (
        1 if item.get('is_soft', False) else 0,      # 第1キー: is_soft
        <戦略固有の値>,                                  # 第2キー
        <戦略固有の値>,                                  # 第3キー
        0 if item.get('is_prioritized', False) else 1,  # 第4キー(最弱): is_prioritized
    )
```

**事実1: is_softは第1キー(最優先・支配的)。** これは「スコアの一項」ではなく、
**タプルの辞書式比較で完全に支配する厳格な分割**である——非ソフト(ハード)荷物は
1件残らずソフト荷物より前に並ぶ。**「ハードを先に積む」という規則は、既にコードに
存在し、しかも4戦略すべてで最も強い形(ハード制約に近い絶対順序)で実装されている。**
コード中のコメント(L123)も明示的に「is_softを最後段にする制約は共通」と述べている。

**事実2: is_prioritizedは第4キー(最弱・最後のタイブレーク)。** 第1〜第3キーが
連続値(体積・質量・寸法)であるため、実質的に同点になることはほぼ無く、
**is_prioritizedによる並べ替えは実質的に発動しない(no-opに近い)。**

**したがって指示書の前提(「無ければ未着手の単純施策」)はis_softについては
成立しない——既に最強の形で実装済みである。** is_prioritizedについては
「先頭寄りにする」という意図はコード上にあるが、キーの序列上ほぼ機能していない。

### 事実3: forbidden_hit自体は順序と無関係にplanner側のハード制約

`ordering.py`のコメント(L189-191)にある通り、「非優先(非ソフト)荷物を優先
(ソフト)荷物の上に乗せない」という制約は**planner.pyが候補生成時にハード制約
として強制する**ため、**どの順序で積んでも下敷きそのものは発生しない**
(Phase65/66が特定した`forbidden_hit`による`fail_support`は、この制約により
候補が却下されて配置に失敗する現象であり、順序を変えても制約自体は変わらない)。
順序が影響しうるのは「その制約に抵触する候補しか残っていない状況に、どの荷物が
追い込まれるか」という頻度の方であり、制約の有無ではない。

### 事実4(コード調査で新たに判明): ランダムリスタート(Phase2)ではこの規則が壊れる

`build_order`(L669〜)は、初期順序戦略で作った4種の並び(`strategy_orders`、
いずれもis_soft最優先で構築済み)を種にして、フェーズ1(決定的、`use_noise=False`)
とフェーズ2(ノイズあり、`use_noise=True`)の2段階で貪欲/ビーム構築
(`simulate.beam_construct_order`)を試す。

`beam_construct_order`(`agents/mysolver/simulate.py` L413〜)の該当箇所:

```python
remaining = {item['index']: dict(item) for item in item_list}
if shuffle_ties and rng is not None:
    keys = list(remaining.keys())
    rng.shuffle(keys)
    remaining = {k: remaining[k] for k in keys}
```

`shuffle_ties=True`はフェーズ2(ノイズありリスタート、`try_construct`のノイズ
分岐)でのみ渡される。**この`rng.shuffle(keys)`は「同点内でのシャッフル」ではなく、
`remaining`全体のキー順序を無条件に完全シャッフルする**(名前の「ties」に反して、
同点かどうかを判定していない)。各ステップの候補可視範囲(`window`)は
`list(st['remaining'].values())[:window]`で決まるため、**フェーズ2ではis_soft
最優先の並びが起点から完全に破壊された状態で貪欲構築が進む**——ハード荷物と
ソフト荷物が可視ウィンドウの先頭からランダムに混在しうる。

フェーズ1(`use_noise=False`、`shuffle_ties`を渡さない)はこの影響を受けず、
種の並び(is_soft最優先)がそのままウィンドウ可視順に反映される。

**best_orderは、フェーズ1・フェーズ2のうち目的関数
(risk調整済み体積 − placementペナルティ)が最も高かった順序が採用される**ため、
最終的に選ばれる順序がハード先積みを保っているかどうかは、**どちらのフェーズの
どのリスタートが勝ったか**に依存し、コード調査だけでは確定できない
(実測には別途の計測が必要で、本フェーズでは実装・計測を行っていない)。

### まとめ(実装はしない)

- is_softに関する「ハード先積み」規則は**既に存在し、初期シードでは最強の形**で
  実装されている。
- is_prioritizedの「先頭寄り」規則は**存在するが、キー序列上ほぼ機能していない**。
- forbidden_hit自体は順序に関わらない候補生成時のハード制約であり、順序が
  変えられるのは「その制約に抵触する頻度」のみ。
- **フェーズ2(ノイズありリスタート)の`shuffle_ties`は、名前に反してis_soft
  最優先の並びを完全に破壊する**ため、最終的な`best_order`がハード先積みを
  保っているかは実測しないと分からない。

---

## 生成物一覧

- `submissions/mysolver_submit_loose3.zip`(新規、SHA256:
  `c72ba8afd17d8332fa1c84e122256efdda495015fadc707bb85be517e6adddc7`)
- `results/phase71_report.md`(本ファイル)
- `docs/submission_policy.md`(§1・§5を更新)

`agents/mysolver/`(リポジトリ追跡ファイル)・`tools/scorer.py`・既存configは
いずれも無変更。本番の集計スコアから足切り閾値やシーン数を逆算する分析、
および順序制御ロジックの実装変更は行っていない。
