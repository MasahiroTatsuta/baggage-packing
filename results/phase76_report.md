# Phase76 報告: 幾何定数を「逆方向」(より保守的な側)に振る

## 結論(先出し)

Phase75 で3定数を公式値へ1段近づけた結果は**3本とも誤り**だった:

| 定数 | 変更 | public | num_placed% | 判定 |
|---|---|---:|---:|---|
| `INCLUSION_MARGIN` | −0.012 → −0.008 | 57.180 | — | **完全に無効(−0.005)。以後触らない** |
| `SAFETY_MARGIN_XY` | 0.022 → 0.018 | 54.00 | — | **−3.19** |
| `REST_CLEARANCE` | 0.016 → 0.012 | **9.19** | 36.42 | **−48.0。崖の縁だった** |

- **`rest012` は足切りの直接的な証拠**: fill 22.01 に対し cog 3.02 / stability 4.17 /
  placement 4.50 / soft_item 5.00。運営が言った「一定数置けないと fill 以外は0点」が
  そのまま観測された。足切りは実在し、効果は桁違いに大きい。
- **解釈**: planner の判定は AABB による解析的近似、本物は pybullet の1cm刻み
  シミュレーション(Phase61 §3-1)。余分なマージンはこのモデル誤差を吸収するために
  必要だった。`MAX_SUPPORT_CENTROID_OFFSET` だけが例外で、あれは「支持の質」という
  別種の自制。→ `docs/official_spec.md` §4 に記録。

**残り2枠で逆方向(現行値よりさらに保守的な側)を試す。**

---

## ステップ2: 提出用zip 2本

いずれも **緩2 の閾値(union/span/centroid = 0.35 / 0.4 / 0.25)を土台**にし、
**1本につき幾何定数を1つだけ**現行値から +1段動かした。
`INCLUSION_MARGIN` は Phase75 で効果ゼロと判明したため触らない(既定 −0.012 のまま)。

| zip | 動かした定数 | 現行既定 | この zip | 方向 |
|---|---|---:|---:|---|
| (I) `mysolver_submit_rest020.zip`   | `REST_CLEARANCE`   | 0.016 | **0.020** | +1段(保守側) |
| (J) `mysolver_submit_safexy026.zip` | `SAFETY_MARGIN_XY` | 0.022 | **0.026** | +1段(保守側) |

`REST_CLEARANCE` は 0.012 で −48 という崖があるため、0.020 も崖の可能性を考えて
いきなり 0.024 等へは飛ばさず、+1段に留めた。

### 全13定数の grep(zip内コードから直接)

2zip共通(閾値3 + 他7):

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

幾何定数3(geometry.py、zipごとに1つだけ差し替え):

| zip | `INCLUSION_MARGIN` | `SAFETY_MARGIN_XY` | `REST_CLEARANCE` |
|---|---:|---:|---:|
| (I) rest020   | −0.012 | 0.022 | **0.020** |
| (J) safexy026 | −0.012 | **0.026** | 0.016 |

### リポジトリ追跡ファイルとの差分

各zipの `agents/mysolver/` 全10ファイルを作業ツリーと1ファイルずつ SHA 比較:

- 差分は `planner.py`(緩2閾値の3行)と `geometry.py`(当該1定数の1行)のみ。
- 他8ファイル(alns / reach / simulate / replica / __init__ / replica_scorer / agent /
  ordering)は全zipで作業ツリーとビット単位一致。
- `git status --porcelain` の追跡ファイル差分は Phase75 由来の `geometry.py`(env化)・
  `docs/submission_policy.md` と、本フェーズの `docs/official_spec.md` ・
  `results/phase76_report.md` のみ。zip は `.gitignore` の `*.zip` 対象で未追跡。
- **既定値は変更していない**(zip 内で env の既定文字列を差し替えているだけ)。

### SHA256(アップロード時に必ず照合すること)

| zip | サイズ | SHA256 |
|---|---:|---|
| `mysolver_submit_rest020.zip`   | 121171 | `e13ee35996c5bcc6819f62657b30d271521abe8efcae6c0b2fb0380238e3ff12` |
| `mysolver_submit_safexy026.zip` | 121172 | `6eb91e020fff8402925f4571ddae0dcca2b8109937f97ddb6ccf44ae476e39ac` |

いずれも 11 エントリ(`mysolver/` + 10ファイル)、既存提出zipと同一構造。`__pycache__` /
`.pyc` は含まない。

### 決定的8シーンの対緩2 差分(参考値)

`build_order`(オフライン順序)を env で各定数を与えて8シーン実行し、緩2(0.35/0.4/0.25、
幾何は既定)と比較:

| 構成 | vs 緩2 | 差分シーン |
|---|---:|---|
| (I) rest020 (REST_CLEARANCE=0.020)   | **4/8** | B02, A01, A02, A03 |
| (J) safexy026 (SAFETY_MARGIN_XY=0.026) | **3/8** | A01, A02, A03 |

参考: Phase75 の保守化と逆方向(rest012=6/8、safexy018=4/8)。オフライン順序への
影響は `REST_CLEARANCE` > `SAFETY_MARGIN_XY` の傾向で一貫。ただし Phase71 で確認した
とおり効きは主にオンライン配置判断側に出るため、28シーン全体の効果を代表しない
参考値。

---

## 判定(先出し)

対照は **緩2 = public 57.18**。**public だけでなく必ず `num_placed_items` も見る。**

- 57.18 を超えた → その定数はまだ保守側に振る余地がある。さらに1段。
- 57.18 前後 → 現行値が最適点。その定数は終了。
- 下がった → 現行値が最適点。両側を確認したことになるので終了。

2本とも枠には入れない(判定用)。1日5回の提出上限内で順次投入する。

---

## 記録(`docs/official_spec.md` §4 に追記)

- **公式値(1.5cm / 5mm)は「本物の判定閾値」であり、planner の近似モデルがそのまま
  使ってよい値ではない。** モデル誤差を吸収する余裕が必要で、実測では現行値のほうが良い。
- **足切りは実在する。`rest012`(配置率 36.42%)で fill 以外が 3〜5 まで落ちた。**
  ただし閾値そのもの(何個で発動するか)は逆算しない(禁止事項)。

---

## 生成物一覧

- `submissions/mysolver_submit_rest020.zip`(新規、未追跡)
- `submissions/mysolver_submit_safexy026.zip`(新規、未追跡)
- `docs/official_spec.md`(§4 を新設)
- `docs/submission_policy.md`(§1 に Phase76追記)
- `results/phase76_report.md`(本ファイル)

`agents/mysolver/`(リポジトリ追跡ファイル)・`tools/scorer.py`・既存 config・
`.gitignore` は無変更。本番の集計スコアから足切り閾値やシーン数を逆算する分析は
行っていない。
