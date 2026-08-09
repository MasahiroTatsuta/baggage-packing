#!/bin/bash
# Phase24 ターゲット2 の A/B 実行。
#   uniform : Phase23 と完全に同一(不感帯 0.0805 を全天面へ一律適用)
#   surface : 天面が直置き面(床 / 棚上面レベル)なら不感帯 0.0005、それ以外は 0.0805
# 26シーン・repeats=1(決定性が確保されているため)。逐次実行。
set -e
cd /workspaces/baggage-packing

MODE=${1:?usage: phase24_ab.sh <uniform|surface> [outfile]}
OUT=${2:-results/phase24_suite_${MODE}.json}

echo "===== phase24 A/B mode=${MODE} start: $(date) ====="
MYSOLVER_CORRIDOR_DB_MODE=${MODE} PYTHONPATH=. .venv/bin/python tools/measure_regime.py \
    --config-path 'configs/gen/suite_*.json' --module-path agents/mysolver/ \
    --repeats 1 --out "${OUT}" --label "phase24_${MODE}"
echo "===== phase24 A/B mode=${MODE} done: $(date) ====="
