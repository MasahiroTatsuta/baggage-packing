#!/bin/bash
# Phase16 ターゲット1: optimize予算再掃引(30/60/120/165秒)。
# optimize有効シーン(pattern A/C、21シーン)のみを対象に、各予算×3回平均を
# fill_strict/fill_loose両方で計測する。逐次実行、各予算の完了ごとにJSONを書き出す。
set -e
cd /workspaces/baggage-packing

CONFIGS=$(ls configs/gen/suite_*.json | grep -v 'suite_B0[1-4]_' | grep -v 'suite_P04_')

for BUDGET in 30 60 120 165; do
    echo "===== budget=${BUDGET}s start: $(date) ====="
    PYTHONPATH=. .venv/bin/python tools/measure_regime.py \
        --config-path $CONFIGS \
        --module-path agents/mysolver/ \
        --repeats 3 \
        --optimize-budget ${BUDGET} \
        --out results/phase16_budget_b${BUDGET}.json \
        --label "phase16_budget_b${BUDGET}"
    echo "===== budget=${BUDGET}s done: $(date) ====="
done

echo "ALL DONE: $(date)"
