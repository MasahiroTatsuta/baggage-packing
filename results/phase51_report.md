# Phase51 報告: A03基準値不一致の原因を確定 — MYSOLVER_REPLICA_SELECT既定値の変更(Phase44)

## 結論(先出し)

**原因を完全に特定した。環境ドリフトではなく、Phase44(コミット`fe42180`)で
`MYSOLVER_REPLICA_SELECT`の既定値を`'1'`→`'0'`に変更した、意図した仕様変更の
副作用だった。** `bp_check.sh`の決定的8シーン確認は`build_order()`を直接呼ぶだけで
この環境変数を明示指定しないため、既定値の変更をそのまま受けていた。A03は
既積み荷物が無い(`is_applicable=True`)シーンのため、既定`REPLICA_SELECT=1`だった
Phase41〜43はρ-test(複製評価器)が自動的に働き、その実結果が
`first10=[13,37,28,35,22,15,25,32,29,7]`だった。Phase44で既定が`0`になって以降は
ρ-test無効の経路(純粋なshadow simulatorのrisk_vol選択)を通り、
`first10=[38,0,6,26,7,33,21,25,1,19]`が正しい既定挙動になった。

**`phase40_baseline_off_mac.json`は引き続き有効な対照であり、26シーンA/Bを
再開してよい**(この基線は常に`MYSOLVER_REPLICA_SELECT`を明示指定して測定されており、
既定値変更の影響を一切受けていないため)。

---

## ステップ1: git履歴からA03の値の初出と変遷を追う

### (1-1)(1-2) 時系列表と、実行結果か転記かの判定

`git log -p -- results/ scripts/` でA03を含む全コミットを洗い出し、`build_order()`の
`first10`が明示的に記録されている箇所と、各報告書の文面(「実測した」か「確認済み」と
だけ書いて値を再掲しているか)を突き合わせた。

| コミット(日時) | 内容 | A03の状態 | 実行結果 or 転記 |
|---|---|---|---|
| `e579accb`(08-16 13:01 UTC) | Codespaces環境記録(env_snapshot.txt) | — | — |
| `a99f271`(08-16 13:40 UTC) | **Phase38: bp_check.sh新規作成、Mac移行ベースライン取得** | `phase38_baseline_off_nowall.json`のfill_strict=17.78497195156464(**現行値と一致**) | 26シーンA/B測定(`REPLICA_SELECT=0`明示)としての実行結果 |
| `3bdc41e`(08-16 13:48 UTC) | Phase38 G-5: クリーンクローン検証、A01/A02/A03のfill_strict/composite_strict比較表 | fill_strict=17.78497195156464(こちらも**現行値と一致**、beforeafter完全一致) | 実行結果(クリーンクローン再取得) |
| `de9e403`(08-17 15:05 JST、**Phase41**) | replica.py防御的書き直し | `first10=[13,37,28,35,22,15,25,32,29,7]`(8/8ビット単位不変として報告) | **実行結果**(この時点の既定`REPLICA_SELECT='1'`のもとで`bp_check.sh`相当を実際に実行) |
| `be66351`(08-18 02:18 JST、**Phase42**) | replica.py/ordering.py改修 | 同上の値で「8/8完全一致」と報告 | **実行結果**(直前コミットから再実行して確認、既定`REPLICA_SELECT='1'`のまま) |
| `2f67cce`(08-18 02:28 JST、**Phase43**) | 提出用zip作成・8シーンビット単位一致確認 | 同上の値でzip内コードとリポジトリのbuild_order出力を比較 | **実行結果**(既定`REPLICA_SELECT='1'`のまま、Phase44の変更はまだ入っていない) |
| `fe42180`(08-18 14:13 JST、**Phase44**) | **`REPLICA_SELECT`既定値を`'1'`→`'0'`に変更** | 8シーン確認は**再実行していない**(「26シーンA/B off側は明示指定なので無関係」という理由で「再測定不要」と判断。ただしこれは26シーンA/B側の議論であり、`bp_check.sh`の直接呼び出しは検証対象に含まれていなかった) | **この時点から「転記」に切り替わる**——Phase44の報告書自体はA03の値を明記していないが、以後の暗黙の前提(「決定的8シーンは不変」)が実際には検証されないまま持ち越された |
| Phase45〜48(08-18 14:13〜18:31 JST) | placement/soft_item調査、ε制約cog着手前調査 | `bp_check.sh`の8シーン確認は**一度も再実行されていない**(いずれもスコープ外) | 該当なし(A03の値に言及なし) |
| `500f4a9`(08-18 18:31 JST、**Phase49**) | ε制約cog作業1、`bp_check.sh`を**Phase43以来はじめて再実行** | `first10=[38,0,6,26,7,33,21,25,1,19]`(過去の記録と不一致と判明) | **実行結果**(既定`REPLICA_SELECT='0'`、Phase44の変更が反映された状態で初めて再実行された) |
| `e712ea3`(08-18 19:46 JST、**Phase50**) | 環境ドリフト調査(壁時計・BLAS) | 同上の値、原因を「未特定」として記録 | 実行結果の再確認(5回+8シーン×2回) |

**判定**: Phase41〜43は実際に`REPLICA_SELECT='1'`(当時の既定値)のもとでbp_check.sh
相当を実行した**正真正銘の実行結果**であり、捏造や転記ミスではない。**Phase44が
既定値を変更した際、`bp_check.sh`の8シーン確認を再実行しなかったため、以後の
「A03=`[13,37,...]`」という前提はPhase44〜48のどの報告書にも明示的な再検証の記述が
なく、暗黙のうちに古い値のまま引き継がれていた**(Phase45〜48は実際にはA03の値に
一切言及していない——「転記」というより「検証の空白期間」に近い)。

### (1-3) Mac移行(Phase38)前後で値が変わっているか

`a99f271`・`3bdc41e`(いずれもPhase38、Mac移行の最初期コミット)の時点で、26シーン
全件評価(`REPLICA_SELECT=0`明示)によるA03のfill_strictは**既に**17.78497195156464
——これは現行の(Phase44以降の)`first10=[38,...]`の値と一致する。**Mac移行の前後で
「26シーンA/B用のREPLICA_SELECT=0経路」のA03の挙動は変わっていない。** 変わったのは
`bp_check.sh`の直接`build_order()`呼び出し(REPLICA_SELECT既定値に依存する経路)
だけであり、これはPhase38ではなくPhase44のタイミングで変わった。

---

## ステップ2: D03と同じ検証をA03に適用する

### (2-1)(2-2) A03のfill_strict/fill_loose/composite_strict、Codespaces基線との比較

`results/phase38_baseline_off_codespaces.json`と`results/phase40_baseline_off_mac.json`
(いずれも`REPLICA_SELECT=0`明示・26シーン)でA03を直接比較:

| 指標 | Codespaces | Mac | 差分 |
|---|---:|---:|---:|
| fill_strict | 17.784972 | 17.784972 | **+0.000000(完全一致)** |
| fill_loose | 30.451092 | 30.451092 | **+0.000000(完全一致)** |
| composite_strict | 67.853454 | 67.853454 | **+0.000000(完全一致)** |

**fill_strictも含めて完全一致。D03のような「fill_looseは一致するがfill_strictだけ
ずれる(境界判定反転)」パターンには該当しない**——A03はそもそもCodespaces-Mac間で
**一切ずれていない**(D03とは別種、というより「ずれていない」)。これは(1-3)の結論
(Mac移行時点でA03に変化は無かった)と整合する。

### (2-3) 26シーン全件の差分表(Codespaces基線 vs Mac基線)

`results/phase38_baseline_off_codespaces.json` と `results/phase40_baseline_off_mac.json`
を26シーン全件で突き合わせた(Phase39が報告していなかった全件表)。

| シーン | fill_strict差分 | fill_loose差分 |
|---|---:|---:|
| A01〜A08, B01〜B04, C01〜C03, D01/D02/D04/D05, P01〜P06(21シーン) | 0.000000 | 0.000000 |
| **D03_A_2c_60_prioheavy_cont** | **+1.3552** | 0.000000 |

**26シーン中、差分があるのはD03の1件のみ(fill_strictだけ、fill_looseは一致)。**
A03を含む残り25シーンは**fill_strict・fill_loose・composite_strictすべて完全一致**。
Phase39の「25/26が±0.04以内」という要約(全件表なし)は、実際には「25/26が
**diff 0.000000**」という、より強い一致だったことが今回の全件表で判明した。
D03のみ実際に境界判定が反転しており(fill_loose一致・fill_strictのみ変化という
パターンはPhase39の「BLAS由来の境界判定反転」という結論と整合)、**A03はこの現象とは
無関係**(そもそも差分ゼロ)。

---

## ステップ3: 結論と対応

### (3-1) 該当: 「移行時点から違っていた」の亜種——正確には「Phase44のREPLICA_SELECT
既定値変更で切り替わった」であり、「移行(Codespaces→Mac)」自体が原因ではない

決定的な直接証拠として、**現行コード(HEAD)に`MYSOLVER_HARD_WALL_LIMIT=3000
MYSOLVER_REPLICA_SELECT=1`を明示指定してA03単体を実行すると、過去の基準値
`first10=[13,37,28,35,22,15,25,32,29,7]`が完全に再現する**ことを確認した
(所要16.67s)。同一の`.venv`・同一のHEADコードで、環境変数1つを変えるだけで
双方の値が再現するため、環境(BLAS等)由来ではなく**確定的なコード分岐の結果**である
ことが証明された。念のため、Mac移行時点のコミット(`a99f271`)のコードを
`git worktree`で取り出し、現行`.venv`で`build_order(time_budget=30.0)`を実行したところ
`first10=[13,37,28,35,22,15,25,32,29,7]`(旧基準値)が出た——**同一環境で新旧コードを
比較する厳密な対照実験**により、コード側の変化(REPLICA_SELECT既定値)が原因であると
確定した。

対応(指示3-1の各項目):

- **A03を`bp_baseline_8scenes.json`で`status: "unresolved"`から`status: "stable"`へ
  格上げした。** 現在の値(`[38,0,6,26,7,33,21,25,1,19,...]`)を確定基準値として
  記録し、原因(Phase44のREPLICA_SELECT既定値変更)を`_historical_note`に明記した。
- `bp_check.sh`を更新し、**8シーン全件をstrict照合**(不一致ならexit 1)に変更した
  (以前はA03だけ`unresolved`として警告扱いにしていた)。
- **`phase40_baseline_off_mac.json`は対照として引き続き有効**であり、書き換えは
  行っていない(指示どおり)。**26シーンA/Bを再開してよい。**
- `docs/migration_to_mac.md`の「セッションをまたいだドリフト」という記述
  (Phase50が追記したもの)を訂正した——実際には「ドリフト」ではなく、
  「`bp_check.sh`がREPLICA_SELECTを明示指定していなかったために、Phase44の意図した
  既定値変更を静かに拾ってしまった」という、**1回きりの、原因が特定された変化**である。

### (3-2)(3-3)

該当なし(原因は判明したため)。

---

## 変更ファイル

- `scripts/bp_baseline_8scenes.json`(A03を`stable`に格上げ、原因説明を追記)
- `scripts/bp_check.sh`(8シーン全件をstrict照合に変更)
- `docs/migration_to_mac.md`(§2 BLAS注記の訂正、§5.0 strict化の反映、
  §5.1の「ドリフト」記述を「原因特定済み」に全面差し替え)
- `results/phase51_report.md`(本ファイル)

`phase40_baseline_off_mac.json`・既存26シーン・`tools/scorer.py`はいずれも無変更。
