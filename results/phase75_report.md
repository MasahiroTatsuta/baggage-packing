# Phase75 報告: centroid スイープ完了 → 幾何定数を本番で直接試す

## 結論(先出し)

1. **centroid スイープはピーク確定。** 0.225→57.06 / **0.25→57.18** / 0.275→56.94。
   頂上付近は幅 0.24 でほぼ平坦(配置率も 63.84 / **64.34** / 63.40 と同じ山形)。
   `MAX_SUPPORT_CENTROID_OFFSET` を主レバーとした支持閾値の全収穫は
   **53.64 → 57.18(+3.54)**。**閾値軸は終了。**
2. **主枠は緩2(`mysolver_submit_loose2.zip`、public 57.18)で確定・変更なし。**
   **2枠目を緩1(56.05)から `mysolver_submit_c0225.zip`(public 57.06)へ差し替え。**
3. ローカル A/B は本番の主効果を検出できない(`tools/scorer.py` に足切りが無い、
   Phase61)。centroid と同じく「実測の裏づけなく公式値より約7mm 保守的に
   置かれた自制」である幾何定数3つを、本番で1本ずつ直接試す。

---

## ステップ1: 提出枠の更新

| 枠 | 変更前 | 変更後 | public |
|---|---|---|---:|
| 主枠 | `mysolver_submit_loose2.zip`(緩2) | 変更なし | 57.18 |
| 2枠目 | `mysolver_submit_phase67_loose1.zip`(緩1) | **`mysolver_submit_c0225.zip`** | 56.05 → **57.06** |

理由: 最終評価は public と別シーンのため多様性にも価値があるが、頂上付近が
平坦で3構成(c0225 / 緩2 / c0275)の挙動差が小さい以上、public の高い方を採るのが素直。
`docs/submission_policy.md` §1 に Phase75追記、§5 の枠表を更新した。

---

## ステップ2-1: `INCLUSION_MARGIN` / `REST_CLEARANCE` の env 化(リポジトリ追跡ファイル)

`agents/mysolver/geometry.py`:

```
INCLUSION_MARGIN = float(os.environ.get('MYSOLVER_INCLUSION_MARGIN', '-0.012'))
REST_CLEARANCE   = float(os.environ.get('MYSOLVER_REST_CLEARANCE',   '0.016'))
```

- 既定値は現行値(−0.012 / 0.016)で不変。Phase60 の `SAFETY_MARGIN_XY` と同じ扱い
  (このコミット単体では挙動を変えない)。
- `SAFETY_MARGIN_XY` は Phase60 で env 化済み(`MYSOLVER_SAFETY_MARGIN_XY`、既定 0.022)。
- **決定的8シーン(B01–B04, P04, A01–A03)を既定値で `bash scripts/bp_check.sh`
  実行 → 8/8 が `scripts/bp_baseline_8scenes.json` とビット単位一致。** 軽量スモーク
  (A01, budget=15s)も例外なく完走。

`git status --porcelain` の追跡ファイル差分は `agents/mysolver/geometry.py` のみ。

---

## ステップ2-2: 提出用zip 3本

いずれも **緩2 の閾値(union/span/centroid = 0.35 / 0.4 / 0.25)を土台**にし、
**1本につき幾何定数を1つだけ**公式値に向けて1段動かした。

| zip | 動かした定数 | 現行既定 | この zip | 公式値 |
|---|---|---:|---:|---:|
| (F) `mysolver_submit_incl008.zip`   | `INCLUSION_MARGIN` | −0.012 | **−0.008** | −0.005 |
| (G) `mysolver_submit_safexy018.zip` | `SAFETY_MARGIN_XY` | 0.022  | **0.018**  | 0.015 |
| (H) `mysolver_submit_rest012.zip`   | `REST_CLEARANCE`   | 0.016  | **0.012**  | —(現行から1段) |

`INCLUSION_MARGIN` は公式値ちょうど(−0.005)にはしていない。`geometry.py` に
記録された「−0.016 付近で特定コンテナ形状(cut corner付近)の候補が急減する崖」は
方向こそ逆だが、**公式の閾値そのものでは余裕がゼロになる**ため1段手前に留めた。

### ステップ2-3: 各zipの検証

#### 全定数の grep(zip内コードから直接)

3zip共通で以下を確認(値は3zipとも同一):

```
planner.py:
  MIN_UNION_SUPPORT_RATIO       = '0.35'      (緩2)
  MIN_SUPPORT_SPAN_RATIO        = '0.4'       (緩2)
  MAX_SUPPORT_CENTROID_OFFSET   = '0.25'      (緩2)
  MYSOLVER_STRICT_SUPPORT_DISABLE  既定 '0'
ordering.py:
  HARD_WALL_LIMIT               = '165.0'
  REPLICA_SELECT                = '0'
  MYSOLVER_TELEMETRY            = '0'
  REPLICA_METRIC                = 'fill'
agent.py:
  FALLBACK_SAFE_POS             = '1'
  FALLBACK_AVOID_OBSTACLES      = '1'
```

幾何定数(geometry.py、zipごとに1つだけ差し替え):

| zip | `INCLUSION_MARGIN` | `SAFETY_MARGIN_XY` | `REST_CLEARANCE` |
|---|---:|---:|---:|
| (F) incl008   | **−0.008** | 0.022 | 0.016 |
| (G) safexy018 | −0.012 | **0.018** | 0.016 |
| (H) rest012   | −0.012 | 0.022 | **0.012** |

#### リポジトリ追跡ファイルとの差分

各zipの `agents/mysolver/` 全10ファイルを作業ツリー(= HEAD + ステップ2-1のenv化)と
1ファイルずつ SHA 比較:

- 差分があるのは `planner.py`(緩2閾値の3行)と `geometry.py`(当該1定数の1行)のみ。
- 他8ファイル(alns / reach / simulate / replica / __init__ / replica_scorer / agent /
  ordering)は全zipで作業ツリーとビット単位一致。
- `git status --porcelain` の追跡ファイル差分は `geometry.py`(env化)のみ。zipは
  `.gitignore` の `*.zip` 対象で未追跡。

#### SHA256(アップロード時に必ず照合すること)

| zip | サイズ | SHA256 |
|---|---:|---|
| `mysolver_submit_incl008.zip`   | 121172 | `c29ed11b5ccd9d17526fc4a1ccf692445a50cd6ae668c35376fa66e993c6d613` |
| `mysolver_submit_safexy018.zip` | 121172 | `cea29d7e73eebaacdc749e8e917f86fe1c4960dc51117a56f14a34b861668c0d` |
| `mysolver_submit_rest012.zip`   | 121170 | `e5f2376ddd01a6ed11290eee29fd7f16727f9bf108ada602a68a0cb33d579753` |

いずれも 11 エントリ(`mysolver/` + 10ファイル)、既存提出zipと同一構造。`__pycache__` /
`.pyc` は含まない。

#### 決定的8シーンの差分(参考値)

`build_order`(オフライン順序)を、各定数を env で与えて8シーン実行し比較:

| 構成 | vs 素の基準(0.55/0.6/0.15) | vs 緩2(0.35/0.4/0.25) | 差分シーン(vs緩2) |
|---|---:|---:|---|
| 緩2 単体(土台) | 2/8 | 0/8 | — |
| (F) incl008 (−0.008)   | 3/8 | **1/8** | B02 |
| (G) safexy018 (0.018)  | 4/8 | **4/8** | B02, A01, A02, A03 |
| (H) rest012 (0.012)    | 6/8 | **6/8** | B01, B02, B04, A01, A02, A03 |

- 緩2 単体が 2/8(A01, A03)なのは Phase71 の記録と一致(サニティチェック)。
- オフライン順序への影響は `REST_CLEARANCE` > `SAFETY_MARGIN_XY` > `INCLUSION_MARGIN`。
- ただし Phase71 で確認したとおり、支持閾値の効きは主にオンライン配置判断側に出る。
  この差分シーン数は28シーン全体の効果の大小を代表せず、あくまで参考値。

---

## 判定(先出し)

対照は **緩2 = public 57.18**。**public だけでなく必ず `num_placed_items` も見る**
(配置率が伸びているかがこの軸が生きているかの一次判定)。

- 57.18 を明確に超えた → その定数が新しいレバー。枠入れ替えを検討し、さらに1段緩める。
- 57.18 前後 → その定数は効かない。
- 下がった → 現行値が適正。**centroid と違い、この定数には実測の意味があった**ことになる。

3本とも枠には入れない(判定用)。1日5回の提出上限内で順次投入する。

---

## 生成物一覧

- `submissions/mysolver_submit_incl008.zip`(新規、未追跡)
- `submissions/mysolver_submit_safexy018.zip`(新規、未追跡)
- `submissions/mysolver_submit_rest012.zip`(新規、未追跡)
- `agents/mysolver/geometry.py`(`INCLUSION_MARGIN` / `REST_CLEARANCE` を env 化、既定値不変)
- `docs/submission_policy.md`(§1 に Phase75追記、§5 の枠表を更新)
- `results/phase75_report.md`(本ファイル)

`tools/scorer.py`・既存 config・`.gitignore` は無変更。本番の集計スコアから
足切り閾値やシーン数を逆算する分析は行っていない。
