#!/bin/bash
# Phase25a ターゲット2: UNITS_PER_SEC の26シーンA/B。
#   BEAM_WIDTH=1(既定, Phase22相当)固定で UNITS_PER_SEC のみを変える単一変更。
#   26シーン・repeats=1(決定性が確保されているため。Phase17/19以来の方針)。逐次実行。
set -e
cd /workspaces/baggage-packing

UPS=${1:?usage: phase25a_ups_ab.sh <units_per_sec> [outfile]}
OUT=${2:-results/phase25a_suite_ups_${UPS}.json}

echo "===== phase25a UNITS_PER_SEC=${UPS} start: $(date) ====="
MYSOLVER_UNITS_PER_SEC=${UPS} PYTHONPATH=. .venv/bin/python tools/measure_regime.py \
    --config-path 'configs/gen/suite_*.json' --module-path agents/mysolver/ \
    --repeats 1 --out "${OUT}" --label "phase25a_ups_${UPS}"
echo "===== phase25a UNITS_PER_SEC=${UPS} done: $(date) ====="
