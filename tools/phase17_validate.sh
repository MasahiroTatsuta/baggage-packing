#!/bin/bash
# Phase17 検証: 決定性リファクタ後のKPI計測。
#
#  (a) P02 の反復追試(budget 30 / 120、各5回)で fill_strict の std が縮むこと
#  (b) budget 30/60/120/165 の再掃引でシーン単位のスプレッドが縮み、単調性が改善すること
#  (c) 26シーンで両レジームの fill がノイズ床を超えて悪化しないこと
#
# 決定性が確認できた後は repeats を絞る(§5 計測プロトコルの再評価)。
set -e
cd /workspaces/baggage-packing

STAGE=${1:-all}

run() { echo "===== $* : $(date) ====="; }

if [ "$STAGE" = "p02" ] || [ "$STAGE" = "all" ]; then
    run "P02 x5 @budget30"
    PYTHONPATH=. .venv/bin/python tools/measure_regime.py \
        --config-path configs/gen/suite_P02_A_1c_pre10.json \
        --module-path agents/mysolver/ --repeats 5 --optimize-budget 30 \
        --out results/phase17_p02_b30_after5.json --label phase17_p02_b30_after5
    run "P02 x5 @budget120"
    PYTHONPATH=. .venv/bin/python tools/measure_regime.py \
        --config-path configs/gen/suite_P02_A_1c_pre10.json \
        --module-path agents/mysolver/ --repeats 5 --optimize-budget 120 \
        --out results/phase17_p02_b120_after5.json --label phase17_p02_b120_after5
fi

if [ "$STAGE" = "sweep" ] || [ "$STAGE" = "all" ]; then
    # phase16 と同じ対象(optimize有効21シーン)・同じ予算水準。決定性が確保されたため
    # repeats=1 で計測する(phase16 は repeats=3)。
    CONFIGS=$(ls configs/gen/suite_*.json | grep -v 'suite_B0[1-4]_' | grep -v 'suite_P04_')
    for BUDGET in 30 60 120 165; do
        run "sweep budget=${BUDGET}"
        PYTHONPATH=. .venv/bin/python tools/measure_regime.py \
            --config-path $CONFIGS --module-path agents/mysolver/ \
            --repeats 1 --optimize-budget ${BUDGET} \
            --out results/phase17_budget_b${BUDGET}.json --label phase17_budget_b${BUDGET}
    done
fi

if [ "$STAGE" = "suite" ] || [ "$STAGE" = "all" ]; then
    run "26 scenes @default(120) x3"
    PYTHONPATH=. .venv/bin/python tools/measure_regime.py \
        --config-path 'configs/gen/suite_*.json' --module-path agents/mysolver/ \
        --repeats 3 --out results/phase17_suite_after3.json --label phase17_suite_after3
fi

if [ "$STAGE" = "old6" ] || [ "$STAGE" = "all" ]; then
    run "old 6 scenes (constraints)"
    PYTHONPATH=. .venv/bin/python tools/local_eval.py \
        --config-path configs/sample_config.json configs/gen/gen_2containers_patternB.json \
        configs/gen/gen_2containers_priority.json configs/gen/gen_shelf_patternA.json \
        configs/gen/gen_manyitems_patternA.json \
        --module-path agents/mysolver/ | tee results/phase17_old6.txt
fi

echo "STAGE ${STAGE} ALLDONE: $(date)"
