# Phase56 報告: 修正済み提出物の作成と、残り期間の方針確定

## 結論(先出し)

**提出用zip(`mysolver_submit_phase55.zip`)を作成し、8/8ビット単位一致・全7環境変数の
既定値を確認した。** コード調査の結果、Step2で問うた「planner.plan()がNoneを返す状況を
減らす」レバーは**新規の未着手領域ではなく、既に2つの主要な下位分類(Phase30の
(i)幾何で入らない・(iii)通路封鎖、合計77%)がそれぞれALNS(Phase34)・reach.item_blockers
順序修正(Phase29)で攻略済み・不採用確定していた**ことが判明した。この事実を踏まえ、
**残り期間の推奨方針は「現状(53.64)の防衛」とする**(詳細は本文参照)。

---

## ステップ1: 提出用zipの生成

### (1-1)(1-2) zip生成・SHA256

診断用の仕掛けは追加せず、リポジトリ現状(Phase55コミット`672bad0`)をそのまま
`agents/mysolver/`からzip化した(Phase43と同一手順):

```
cd agents && zip -r ../submissions/mysolver_submit_phase55.zip ./mysolver -x '*__pycache__*' -x '*.pyc'
```

- **出力先**: `submissions/mysolver_submit_phase55.zip`(11ファイル、`mysolver/`直下に
  .pyファイル10個——既存の提出zipと同じ構造)
- **SHA256**: `85bd11db422ed2fce01ca10c8f286ff608564abc2b4a720848a2bfaee8a8418e`

### 環境変数の既定値(zip内コードを直接grep)

```
$ unzip -p submissions/mysolver_submit_phase55.zip mysolver/ordering.py | grep "os.environ.get('MYSOLVER_TELEMETRY'\|...HARD_WALL_LIMIT'\|...REPLICA_SELECT'\|...REPLICA_METRIC'"
HARD_WALL_LIMIT = float(os.environ.get('MYSOLVER_HARD_WALL_LIMIT', '165.0'))
REPLICA_SELECT = os.environ.get('MYSOLVER_REPLICA_SELECT', '0') == '1'
MYSOLVER_TELEMETRY = os.environ.get('MYSOLVER_TELEMETRY', '0') == '1'
REPLICA_METRIC = os.environ.get('MYSOLVER_REPLICA_METRIC', 'fill')

$ unzip -p submissions/mysolver_submit_phase55.zip mysolver/agent.py | grep "os.environ.get('MYSOLVER_FALLBACK_"
FALLBACK_INCLUSION_MARGIN = float(os.environ.get('MYSOLVER_FALLBACK_INCLUSION_MARGIN', '-0.005'))
FALLBACK_CLEARANCE_EPS = float(os.environ.get('MYSOLVER_FALLBACK_CLEARANCE_EPS', '1e-4'))
FALLBACK_SAFE_POS = os.environ.get('MYSOLVER_FALLBACK_SAFE_POS', '1') == '1'
```

**7項目すべて指示どおりの既定値**(TELEMETRY='0'、HARD_WALL_LIMIT='165.0'、
REPLICA_SELECT='0'、REPLICA_METRIC='fill'、FALLBACK_SAFE_POS='1'、
FALLBACK_INCLUSION_MARGIN='-0.005'、FALLBACK_CLEARANCE_EPS='1e-4')。

### (1-3) zip内コードとリポジトリの決定的8シーンビット単位一致

zipを別ディレクトリへ展開し、`mysolver.ordering`(zip版)と`agents.mysolver.ordering`
(リポジトリ版)を同一プロセスに import して`build_order()`の返り値を`==`比較した
(Phase43と同一手法):

```
[B01] repo_n=40 zip_n=40 OK 完全一致
[B02] repo_n=40 zip_n=40 OK 完全一致
[B03] repo_n=80 zip_n=80 OK 完全一致
[B04] repo_n=80 zip_n=80 OK 完全一致
[P04] repo_n=34 zip_n=34 OK 完全一致
[A01] repo_n=40 zip_n=40 OK 完全一致
[A02] repo_n=80 zip_n=80 OK 完全一致
[A03] repo_n=40 zip_n=40 OK 完全一致

8/8 完全一致
```

### (1-4) アップロード時の照合

**アップロード時は必ずSHA256(`85bd11db...a8418e`)と照合すること**
(Phase37で別のzipを誤ってアップロードした事例があるため)。

### 判定基準の確認(提出後、本番結果が出た時点で判定すること)

- `fill_score`が`38.09476291926298`**から動けば**: 本番ではフォールバック発火時に
  壁際の余裕が残っているケースがあった=前進。
- **13桁一致のままなら**: 本番でもis_validが律速——このレバーは閉じる
  (本報告のステップ2の結論と整合)。

---

## ステップ2: 残り期間の方針整理

### (2-1) `planner.plan()`がNoneを返す状況を減らせる余地があるか(コード調査)

**探索の網羅性**(`agents/mysolver/planner.py::_search_best`/`plan`より):

- `plan()`は「現在の1アイテムだけ」ではなく、**可視プール内の全アイテム**
  (`for pool_idx in range(n_pool)`、オンライン呼び出しは`n_pool=min(len(pool_list), MAX_POOL_ITEMS=20)`)
  × 全ユニーク向き(`_unique_orientations`)× 全コンテナ × y層(`Y_SLICE_COUNT=2`段階の
  搬入経路規律)× 候補XYグリッド(`BASE_GRID_DENSITY=2`)を総当たりする。
  合法手が1つも見つからなければ、**同じ予算内でグリッド密度を2倍
  (`RETRY_GRID_DENSITY=4`)にした最終リトライ**をさらに行い、それでも0件なら
  初めて`None`を返す。**「現在のアイテムしか見ていない」という懸念は誤りで、
  可視プール全件・複数解像度で網羅的に探索した上での`None`である。**
- 候補の合法性評価(`_evaluate_candidates`)は着地面(support)判定に加え、
  `_y_sweep_unreachable_mask`による**搬入経路(y方向スイープ)到達可能性の
  事前フィルタ**を組み込んでおり、単純な「その場に収まるか」だけでなく
  「そこまで運べるか」も評価対象に含む。
  ※オンライン呼び出しは`lookahead_k`(見える手荷物数)がパターンA/B/Cで
  1/10/5に固定されており`MAX_POOL_ITEMS=20`を下回るため、現行シーンでは
  この上限による見落としは実質発生しない。

**Phase30の診断「(i)幾何で入らない」との異同**:

Phase30の(i)は、エピソード終了時点の**残存アイテム全体**について、voxel粒度の
静的容積解析(`fit_nosupport==0`=支持を無視しても入る位置が皆無)で判定した
**事後の集計指標**である。一方Phase54/55が特定した「フォールバック発火時に
is_validで落ちる」は、**その瞬間の1候補**(内壁からのクリアランスのみを
考慮したもの)が**搬送経路上で既配置荷物と衝突する**という、`check_transport_path`
固有の判定である。**両者は同じ現象(「置き場がもう無い」)を指しうるが、厳密には
別の区分に対応する**——is_valid(搬送経路衝突)はPhase30の**(iii)通路封鎖**
(`fit_nosupport>0`だが到達不能)に近く、(i)幾何で入らない(そもそも空間が無い)
そのものではない可能性が高い。`planner.plan()`自体の候補生成が
`_y_sweep_unreachable_mask`で到達可能性を事前フィルタしていることを踏まえると、
`plan()`の`None`はPhase30の(i)・(iii)**両方を包含した上位概念**と考えるのが妥当。

**この2つの下位分類は、いずれも既に個別に攻略が試みられ、不採用が確定している:**

| Phase30の分類 | シーン数/残体積シェア | 攻略の試み | 結果 |
|---|---|---|---|
| (i) 幾何で入らない | 48%/54.8%(最大区分) | **Phase34: ALNS(破壊→修復)** | **不採用**(t検定の採用基準に遠く届かず。既定`MYSOLVER_ALNS=0`のまま。報告書に「探索の作り方を変える路線はここで閉じる」と明記) |
| (iii) 通路封鎖 | 29%/30.4%(2番手) | **Phase29: `reach.item_blockers`による3手の順序修正** | **不採用**(6シーン中4シーンで悪化、採点指標が動いたのは1/26シーン。到達シーン数が少なすぎてt検定を原理的に通過できず) |

**つまり「plan()がNoneを返す状況を減らす」という方向性は、今回新たに発見された
未着手のレバーではなく、その中身(合計77%)は既に2つの異なる手法で攻略され、
いずれも統計的な採用基準を満たせず打ち切られていた領域である。** Phase54/55が
今回明らかにしたのは「フォールバック自体が壊れていた」という**別の、
より単純で確実なバグ**であり、これは正しく修正した(Phase55)。しかし
その先にある「plan()のNoneそのものを減らす」は、既存の探索改良では
すでに手詰まりであることが、今回のコード調査で改めて裏付けられた。

### (2-2) 残り2ヶ月で取り得る選択肢

| 選択肢 | 期待値 | 実現可能性 |
|---|---:|---|
| ρ-test原因究明 | +0.48(完全に機能した場合の上限。Phase44試算) | 中〜低。メカニズムは既知だが、本番で機能しない原因の特定に診断ビルドが要り、規約確認が未了のまま保留中 |
| plan()のNone低減(探索改良の再挑戦) | 上限不明だが、対象の77%は既に2手法で攻略・不採用確定 | **低**。同じ土俵での再挑戦は過去4回(Phase29/32/33/34)いずれも不採用という実績があり、新規性のある切り口が無い限り再挑戦の根拠が薄い |
| placement_score/soft_item_score | +13.61(合計、完全に機能した場合) | 極めて低い。ローカルで一度も乖離を再現できず(Phase44-47)、原因不明のまま |
| stability_score | +0.39(実質それ未満) | 低い。物理エンジンの沈降残差が支配的(Phase52) | 
| シーン規模仮説 | — | 棄却済み(Phase53) |
| **現構成の防衛**(53.64を守る) | 0(現状維持) | **高**。Phase55の修正はリスクゼロで確実に正しく、それ以外は着手済みで全て低確度 |

### (2-3) 推奨方針

**「現構成の防衛」を推奨する。** 理由:

1. **今回の調査(Phase41〜56、16フェーズ)で、統計的な採用基準を満たした改善は
   ゼロ件だった。** 試みたレバーは(a) ρ-test防御化・診断(Phase41-44)、
   (b) placement/soft_item乖離調査(Phase44-47)、(c) ε制約cog(Phase48-49)、
   (d) シーン規模仮説(Phase53)、(e) sudden death根本原因(Phase54-55、
   **唯一の実装済み修正、ただしリスクゼロの安全策であって公開スコアの
   実効果は未確認**)。これ以前のPhase29-34でも(f) 到達可能性駆動リスタート・
   (g) Borda選択則・(h) 床面限定risk_vol・(i) ALNS、いずれも不採用。
   **「まだ試していない」ではなく、「試した主要な筋は軒並み統計的に否定された」
   という事実が既に積み上がっている。**
2. **残る未着手レバー(ρ-test)は上限+0.48と小さく、かつ実行の前提条件
   (診断ビルドの規約確認)が本フェーズ時点でも未解決のまま。** これに
   残り2ヶ月を投じても、着手できる保証がなく、着手できても期待値が小さい。
3. **Phase55の修正はダウンサイドがない**(t=-0.34、有意な悪化なし、
   fill/placement/soft_itemは完全不変)——**唯一「実装して損はしない」と
   確信を持って言えるレバーであり、既に実装・提出済み。** これ以上、
   確度の低い探索改良に工数を割くより、**この提出の本番結果を見て
   fill_scoreが動くかどうか(ステップ1の判定基準)を確認し、動いた場合にのみ
   `plan()`低減の再挑戦を検討する**、という**受動的な監視体制**に切り替える方が、
   残り期間の使い方として合理的である。
4. 「防衛」は「何もしない」ではなく、**今回特定した既知の情報(REPLICA_SELECT
   既定値・境界マージンの取り扱い等)が将来また静かに壊れないよう、
   `docs/migration_to_mac.md`§5.2の恒久ルールや`bp_check.sh`の自動照合体制を
   維持すること**を含む——これらは既に整備済みであり、追加の実装コストなしに
   継続できる。

---

## 変更ファイル

- `submissions/mysolver_submit_phase55.zip`(新規、提出用zip)
- `results/phase56_report.md`(本ファイル)

`agents/mysolver/`・`tools/scorer.py`・既存26シーン・`.gitignore`はいずれも
無変更(ステップ2は報告のみ、実装は行っていない)。
