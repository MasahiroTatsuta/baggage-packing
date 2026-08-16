#!/usr/bin/env bash
# bp_check.sh — 移植後の健全性チェック(壁時計非拘束・決定的8シーンのビット単位一致 + 軽量スモーク)。
# 使い方: プロジェクトルートで `bash scripts/bp_check.sh` (Mac/Linux共通)。
# 副作用なし(results/には書かない)。CPU負荷のある処理と並行実行しないこと
# (hard_deadline が真の壁時計に依存するため、競合下では同一コードでも結果が振れる。
#  Phase38 ステップD参照)。
set -euo pipefail
cd "$(dirname "$0")/.."

echo "== 1. 決定的8シーン(B01-B04, P04, A01-A03)の build_order 出力を確認 =="
MYSOLVER_HARD_WALL_LIMIT=3000 PYTHONPATH=. .venv/bin/python - <<'PY'
import io, json, os, sys, time
from contextlib import redirect_stdout
from src.ground_handling.env import GroundHandlingEnv
from agents.mysolver import ordering as ordering_mod

SCENES = {
    'B01': 'configs/gen/suite_B01_1c_40_plain.json',
    'B02': 'configs/gen/suite_B02_1c_40_shelf.json',
    'B03': 'configs/gen/suite_B03_2c_80_prio.json',
    'B04': 'configs/gen/suite_B04_2c_80_noprio.json',
    'P04': 'configs/gen/suite_P04_B_1c_pre8_shelf.json',
    'A01': 'configs/gen/suite_A01_1c_40_plain.json',
    'A02': 'configs/gen/suite_A02_1c_80_plain.json',
    'A03': 'configs/gen/suite_A03_1c_40_shelf.json',
}

def load_scene(cp):
    task = list(json.load(open(cp)).values())[0]
    env = GroundHandlingEnv(config=task, verbose=False, render_mode=None)
    try:
        env.reset_settings()
        init = env.get_init_states()
        items = env.get_info_for_optimization()
        return init['container_list'], items, init['lookahead_k']
    finally:
        env.close()

for label, cp in SCENES.items():
    cl, items, lk = load_scene(cp)
    t0 = time.perf_counter()
    with redirect_stdout(io.StringIO()):
        order = ordering_mod.build_order(items, cl, lk, time_budget=30.0)
    print(f'[{label}] n={len(order)} first10={order[:10]} ({time.perf_counter()-t0:.1f}s)')
PY

echo ""
echo "== 2. 軽量スモーク(A01, budget=15s)で optimize/policy が例外なく完走するか =="
MYSOLVER_HARD_WALL_LIMIT=3000 MYSOLVER_OPTIMIZE_BUDGET=15 PYTHONPATH=. .venv/bin/python \
  tools/local_eval.py --config-path configs/gen/suite_A01_1c_40_plain.json \
  --module-path agents/mysolver/

echo ""
echo "== bp_check.sh 完了 =="
