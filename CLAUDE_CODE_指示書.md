# Claude Code 指示書 — NEDO Challenge コンテスト2「積付アルゴリズム」ベースライン構築

この文書は、ローカルの Claude Code に最初に渡す作業指示です。目的は「**評価基盤で1手も失敗しない合法貪欲ベースライン Agent**」と「**ローカル評価ループ＋自前スコアラ**」を作ること。まずは順位より「全課題でエラー無く完走し、fill_score を稼ぐ」を最優先とする。

---

## 0. 前提・分業（重要）

- OS: Intel Mac / メモリ8GB。**アーキは評価基盤(x86_64 Ubuntu 24.04)と一致**。
- 分業: **編集・git・このClaude Code操作はmacホスト**、**python実行・テスト・zip作成はDockerコンテナ内**。理由: シミュレータが `multiprocessing(spawn)` + `shared_memory` + `resource.RLIMIT_AS` などLinux前提の挙動に依存するため、実行は必ずコンテナ内で本番一致させる。
- 配布物 `simulator/` をリポジトリのルートとして使う（`docker-compose.yml` がここにある）。

### 環境の起動と実行方法
```bash
# 初回ビルド＆起動（simulator/ で実行）
cd simulator
docker compose up -d --build     # service名: dev1 / container名: gh_env / マウント: .:/workspace

# コンテナ内でサンプル実行（動作確認）
docker compose exec dev1 bash -lc "cd /workspace && PYTHONPATH=. python scripts/run_test.py --module-path agents/base/ --verbose True"

# 結果は results/evaluation_results.json に出力される
```
- **`PYTHONPATH=.` 必須**: 付けないと `ModuleNotFoundError: No module named 'src'` になる（`scripts/` から `src` を import するため、プロジェクトルートをパスに通す）。
- **Python 3.12 必須**: コードは3.12専用のf-string文法を使用。Mac素のpythonでは `SyntaxError` になるので必ずコンテナ内(py312)で実行する。
- `--module-path` は**末尾スラッシュ必須**（`run_test.py` が `agents/base/`→`agents.base.agent` に変換）。モジュール名は常に `agent`、クラス名は `Agent` 固定。
- Docker Desktop の割当は CPU4 / メモリ5〜6GB程度に。重い多シーン実験は将来クラウドVMへ逃がす。

---

## 1. 作るもの（リポジトリ構成）

`simulator/` 配下に以下を追加する。既存 `src/`, `configs/`, `scripts/` は改変しない。

```
simulator/
├── agents/
│   ├── base/agent.py              # 既存テンプレ（参照用）
│   └── mysolver/
│       ├── agent.py               # ★提出エントリ（Agentクラス）
│       ├── geometry.py            # コンテナ/荷物の幾何・座標変換・合法性判定
│       ├── planner.py             # 候補生成＋スコアリング（配置決定ロジック）
│       └── ordering.py            # optimize用の積付順序決定
├── tools/
│   ├── local_eval.py              # 複数config一括実行＋集計（ホスト起動→exec）
│   └── scorer.py                  # 自前スコアラ（fill/cog/placement/soft/stability近似）
└── configs/
    └── (テスト用に派生configを増やす場合はここ)
```

提出時は `agents/mysolver/` を zip 化（`agent.py` がフォルダ直下に来る形。`sample_submit.zip` 同様）。

---

## 2. Agent API 契約（`agents/mysolver/agent.py`）

`Agent` クラスに3メソッドを実装。評価基盤は**別プロセスで生成**し、各呼び出しに制限時間を課す。**タイムアウトするとそのプロセスは kill→再起動され内部状態が消える**。したがって **policy は毎回 `observation` から状態を再構築する設計**にし、内部メモリに依存しない（堅牢性のため必須）。

```python
class Agent:
    def __init__(self, module_path: str): ...
        # 重い初期化は避ける（init_timeout=10s）

    def get_init_states(self, init_states: dict) -> None:
        # init_states: {'optimize': bool, 'lookahead_k': int, 'container_list': [...]}
        # コンテナ仕様を保持（寸法, cut, center, n_vecs, points, volume, shelf, is_prioritized）

    def optimize(self, item_list: list) -> list[int]:
        # optimize=True時のみ、開始直後に1回。制限180s。
        # 全荷物の index を過不足なく並べ替えた完全順列を返す。ordering.py に委譲。

    def policy(self, observation: dict) -> dict:
        # 毎ステップ。制限8s。planner.py に委譲。
        # 返り値:
        #   {'item_idx': int, 'container_idx': int,
        #    'place_pos': np.ndarray([x,y,z], dtype=np.float32),  # ローカル相対座標
        #    'orientation': int}  # 0..5
```

### observation の中身（policy）
- `depth_map`: `np.ndarray`、形状 `(num_containers, 64, 64)` のハイトマップ（**runnerが共有メモリから復元して渡す**。そのまま `observation['depth_map']` で参照可）。
- `pool_list`: 今選べる荷物 dict のリスト（最大 `lookahead_k` 個。**終盤は len が減る**ので `item_idx` 範囲に注意）。
- `container_list`: 各コンテナ dict（下記）。`packed_items` に既配置荷物（pos/orn付き）。
- `optimize`, `lookahead_k`。

### 荷物 dict（pool_list / packed_items 共通）
`index, length(X), width(Y), height(Z), mass, is_prioritized(bool), is_soft(bool), belongs_to(int|None), pos(tuple|None), orn(quat|None), lateralFriction, rollingFriction, spinningFriction, restitution, angularDamping`。ソフト時のみ `contactStiffness, contactDamping, linearDamping` が追加。

### コンテナ dict
`index, length(X), width(Y奥行), height(Z), cut_x, cut_y, thickness, center(world tuple), n_vecs(list[tuple]), points(list[tuple]), volume, shelf(bool), is_prioritized(bool), packed_items(list)`。

---

## 3. 座標系・幾何（合法配置の要）

`place_pos` は**コンテナ・ローカルの相対座標**で、原点は `(offset_x, 0, 0)`。
- **`offset_x = container['center'][0]`**（ローカル中心のx=0がworld上でcenter.xに一致するため）。world変換は `world = (place_pos.x + offset_x, place_pos.y, place_pos.z)`。y,zは不変。
- ローカル座標範囲の目安:
  - x ∈ [-length/2, length/2]（中心0）。**片側X端の上部が cut_x×cut_y で斜めに切り欠かれている**（小さい脇棚が x≈-length/2+cut_x/2 に存在）。
  - y ∈ [-width/2, width/2]。**挿入は y=-width/2（手前）側から**。評価は手前→奥(+Y)へ、次に横(±X)へ「まっすぐ押し込む」軌道で干渉判定。
  - z: 床面上面 ≈ `thickness`。**床置きなら item中心z ≈ thickness + (回転後の半高)**。天井 ≈ height。棚あり時は棚上面 `height/2 + thickness + buffer` も接地面。
- `orientation`(0..5) と回転後寸法（`utils.ORNS`/`get_half_ext`と一致させる）:
  - 0:[L,W,H] 1:[L,H,W] 2:[H,W,L] 3:[W,L,H] 4:[W,H,L] 5:[H,L,W]
  - 半寸は各成分/2。**内包・干渉判定はこの回転後半寸で行う**こと。
- depth_map 画素(u,v)→ローカル(x,y): `x = pos_low[0] + (u/64)*(pos_high[0]-pos_low[0])`, `y = pos_low[1] + (v/64)*(...)`（`pos_lim` は action config、sampleは low=-100/high=100 なので実際は各コンテナ寸法でクリップして使う）。

---

## 4. 評価・検証ルール（これを事前に自己再現し、非合法手を絶対に出さない）

評価基盤は毎ステップで以下を検証し、**1つでもNGならそのシーンは即終了→その時点の状態で採点**（sudden death）。`src/ground_handling/validator.py` を正とし、planner側で**同等のチェックを配置前に行って合格する手だけを出す**。

1. **形式**: action のキー集合一致、`place_pos` 各要素が `pos_lim`[low,high]内、`container_idx/item_idx/orientation` が範囲内。
2. **内包判定**（`check_inclusion` と同式）: world の候補中心 `t` について、全面で `dot(n_vec, t - point) + dot(|n_vec|, half_lwh) <= inclusion_margin(≈0.02)`。
3. **搬入経路の干渉**（`check_transport_path`）: 手前(y=-width/2)から z を合わせて生成→ +Y 方向→ +X 方向へ移動する軌道上で、既配置荷物・壁・棚に **1.5cm(safety_margin) 以内接近で衝突=失敗**。planner では簡易に「回転後AABB＋1.5cmマージンで既配置AABBと重なる候補を除外」＋「手前から目標までの掃引経路(y,x掃引)を既配置AABBと交差判定」で近似する。
4. **定着判定**（`place_item`）: 目標へ置いて物理を数ステップ進め、初期位置からのズレ>閾値 or 角度>30°なら失敗。→ **支持面（下に十分な接地）と重心が乗る位置**を選ぶ。投入は目標の**約8cm上(start_z)から落下**（接地面上なら落下なし）。
5. **タイムアウト**: optimize>180s→デフォルト順、policy>8s→**ランダム手強制＋プロセス再起動**。→ policyは**8秒に十分な余裕**（READMEは各制限より1秒以上速くを推奨）。

> 実装方針: まずAABBベースの保守的な合法性判定で「確実に入る・ぶつからない・下が支えられている」候補のみ許可。既配置が傾いている(非軸平行)場合もあるので、既配置AABBは実姿勢(orn)から算出した外接AABBを使う（保守側）。

---

## 5. 実装タスク（フェーズ順・小さくコミット）

### Phase 0: 動作確認
- `docker compose up -d --build` → base agent で `run_test.py` を通し、`results/evaluation_results.json` が出ることを確認。中身の `place_states`・`time_results`・`evaluation` を読めるようにする。

### Phase 1: 合法貪欲ベースライン（最優先）
- `geometry.py`: 座標変換(local↔world, offset_x=center.x)、回転後半寸(get_half_ext相当)、内包判定、AABB算出、掃引干渉の近似判定、床/棚の接地z算出。
- `planner.py`: 各ステップで **(pool内の各item)×(orientation 0..5)×(候補位置)** を評価し、**合法な手のうち最良**を返す。
  - 候補位置生成: コンテナ内をXY格子（例2〜5cm刻み）＋既配置の上面/隣接、ハイトマップ由来の低い谷から生成。z は接地面に載せる（Deepest-Bottom-Left / skyline 的に「なるべく低く・奥・端」優先）。
  - 合法フィルタ: §4の1〜4を事前判定して合格のみ残す。
  - スコア（合法候補の優先順位）: 低重心(z小)・接地支持面積大・奥/壁接触・空間効率、＋ルール（優先手荷物は下段/優先コンテナへ、ソフトは上段/上に非ソフトを載せない）。
  - **合法手が全く無い場合のみ**やむを得ず失敗（それ以上積めない）。まず「置ける限り置く」を徹底。
- `ordering.py`: optimize は「重い・大きい・優先を先（下段）に、ソフトを後（上段）に」の決定的ソートから開始。完全順列で返す。
- `agent.py`: 上記を束ねる。**policyは毎回observationから再構築**（内部状態非依存）。

### Phase 2: ローカル評価ループ＋自前スコアラ
- `tools/scorer.py`: 配布 `evaluator.py` は fill と積載率のみ。cog/placement/soft/stability を pybullet 終端状態から自前算出（重心=質量加重座標、placement/soft=上方向接触の属性チェック、stability=蓋をして重力を揺らし変位計測）。**重みは非公開**なので総合は「各サブスコアの素点」を並べて可視化し、投稿フィードバックで較正する。
- `tools/local_eval.py`: 複数 config を順次(逐次)実行し、シーンごとの status/各サブスコア/所要時間を集計表示。8GBなので並列にしない。

### Phase 3以降（ベースライン安定後）
- lookaheadを使った1〜数手先読み、A課題のoffline順序最適化、ALNSで順序/配置を改善。RLは最後の検討枠。

---

## 6. 受け入れ基準（Phase 1完了の定義）
- `configs/sample_config.json` の全 task が `status: success`（`format_error`/例外なし）。
- 全 task で `place_states.is_included / is_valid / is_placed_safe` が最終手まで True 基調（早期終了が「これ以上物理的に積めない」時のみ）。
- `time_results.policy < 7s`、`optimization < 170s`（余裕を持つ）。
- 少なくとも複数個の荷物を積み、`fill_score > 0`。ランダムfallbackに落ちていない（ログにtimeoutが出ない）。

---

## 7. 提出パッケージング
```bash
cd simulator/agents
zip -r mysolver_submit.zip ./mysolver
# 生成した mysolver_submit.zip を SIGNATE に投稿（1日5回まで）
```
- 提出前チェック: ネット接続処理を含めない／`requirements.txt` は Dockerfile 未導入ライブラリのみ記載（基本は numpy/pybullet/gymnasium で完結させる）／メモリ12GB・policy8s以内。

---

## 8. 禁止・境界（実装で踏まないこと）
- 特定 config/コンテナ寸法/荷物順への**ハードコード禁止**（隠しテストは全く別）。汎用実装。
- 評価時**ネット接続不可**。有償API不可。外部データはオープンなもののみ。
- 期間中コード/モデルのチーム外共有禁止（公式Discordのみ可）。

---

## 参照ファイル（配布 simulator 内、正とする実装）
- API契約: `agents/base/agent.py`
- 実行フロー/タイムアウト/fallback: `src/ground_handling/app.py`, `runner.py`
- 観測生成: `src/ground_handling/env.py`（`_get_obs`, `get_init_states`）
- 検証: `src/ground_handling/validator.py`（内包/搬入経路/定着）
- 幾何/座標: `src/ground_handling/containers.py`（`local_to_global`, `center`, `n_vecs/points`, `volume`）, `utils.py`（`ORNS`, `get_half_ext`）
- 充填率例: `src/ground_handling/evaluator.py`
- 設定例: `configs/sample_config.json`（action.pos_lim, orientations, agent.timeout, camera 64x64 等）
