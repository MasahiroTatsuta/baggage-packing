#!/usr/bin/env bash
# bp_ab.sh — 26シーンA/B(off/on)を壁時計非拘束で実行し、mean/std/SE/tを算出する。
# 使い方: bash scripts/bp_ab.sh <label> [追加のenv代入 "KEY=VAL KEY2=VAL2..."]
#   例: bash scripts/bp_ab.sh replica_select "MYSOLVER_REPLICA_SELECT=1"
# off側は常に MYSOLVER_REPLICA_SELECT=0(現行の対照群)。on側は引数で渡した env を追加適用する。
# 既存の採否基準(t>2 かつ悪化シーン少数)に従って判断すること。実行順はoff→onの逐次
# (並行実行しない。理由はbp_check.sh冒頭コメント参照)。
set -euo pipefail
cd "$(dirname "$0")/.."

LABEL="${1:?usage: bp_ab.sh <label> [\"ENV1=V1 ENV2=V2\"]}"
EXTRA_ENV="${2:-}"

echo "== off側(対照): MYSOLVER_REPLICA_SELECT=0 =="
env MYSOLVER_HARD_WALL_LIMIT=3000 MYSOLVER_REPLICA_SELECT=0 MYSOLVER_UNITS_PER_SEC=2.00e7 \
  PYTHONPATH=. .venv/bin/python tools/measure_regime.py \
  --config-path 'configs/gen/suite_*.json' --module-path agents/mysolver/ --repeats 1 \
  --out "results/bp_ab_${LABEL}_off.json" --label "${LABEL}_off"

echo ""
echo "== on側: MYSOLVER_REPLICA_SELECT=1 ${EXTRA_ENV} =="
env MYSOLVER_HARD_WALL_LIMIT=3000 MYSOLVER_REPLICA_SELECT=1 MYSOLVER_UNITS_PER_SEC=2.00e7 \
  ${EXTRA_ENV} PYTHONPATH=. .venv/bin/python tools/measure_regime.py \
  --config-path 'configs/gen/suite_*.json' --module-path agents/mysolver/ --repeats 1 \
  --out "results/bp_ab_${LABEL}_on.json" --label "${LABEL}_on"

echo ""
echo "== t検定(composite_strict, シーン単位の対応あり) =="
PYTHONPATH=. .venv/bin/python - "$LABEL" <<'PY'
import json, math, sys
label = sys.argv[1]
off = json.load(open(f'results/bp_ab_{label}_off.json'))['per_scene']
on = json.load(open(f'results/bp_ab_{label}_on.json'))['per_scene']
diffs = []
for k in off:
    if k in on:
        d = on[k]['composite_strict']['mean'] - off[k]['composite_strict']['mean']
        diffs.append((k, d))
if not diffs:
    print('警告: per_sceneのキーが一致しない。手動で比較すること。')
else:
    n = len(diffs)
    mean = sum(d for _, d in diffs) / n
    var = sum((d - mean) ** 2 for _, d in diffs) / (n - 1) if n > 1 else 0.0
    se = math.sqrt(var / n) if n > 1 else float('nan')
    t = mean / se if se else float('nan')
    worse = [k for k, d in diffs if d < -2.0]
    print(f'n={n} mean={mean:.3f} std={math.sqrt(var):.3f} SE={se:.3f} t={t:.3f}')
    print(f'-2.0pt超の悪化シーン: {worse}')
PY
