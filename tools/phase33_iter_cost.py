"""
tools/phase33_iter_cost.py

Phase33 タスク2(2-3): ALNS「1反復」のコストが「全構築1回」の何割になるかを、
Phase29 §3.3 が「リスタートループが捨てている端数」として実測した5シーン
(A05/C02/C03/D01/P05)で実測する。新規探索は行わない
(既存の候補順序を1回構築・1回評価・1回resumeするだけ)。

  - 全構築1回のコスト = beam_construct_order(構築) + simulate_order(validate) を
    1回ずつ実測した合計(ordering.try_construct が1リスタートごとに行う処理と同じ組)。
  - 1反復のコスト = results/phase29_cand_*.json に記録済みの勝者順序を、末尾m個
    (n_placedの15%相当、最小1個)の手前でスナップショットし、simulate.simulate_order の
    resume_state で再開して最後まで流し込む壁時計(「末尾mだけ作り直す」の評価側コスト)。

実行:
    MYSOLVER_UNITS_PER_SEC=1.55e7 PYTHONPATH=. .venv/bin/python tools/phase33_iter_cost.py \
        --cand results/phase29_cand_g1.json results/phase29_cand_g2.json \
        --out results/phase33_iter_cost.json
"""
import argparse
import io
import json
import os
import sys
import time
from contextlib import redirect_stdout

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from src.ground_handling.env import GroundHandlingEnv
from agents.mysolver import geometry as geo
from agents.mysolver import ordering as ordering_mod
from agents.mysolver import planner
from agents.mysolver import simulate as simulate_mod

# Phase29 §3.3 実測: リスタートループが1回も追加のリスタートを始められず捨てていた
# 端数予算(名目秒、MYSOLVER_UNITS_PER_SEC=1.55e7 換算)。
LEFTOVER_BUDGET_S = {
    'A05_2c_80_prio': 12.6,
    'C02_2c_55_shelfprio': 1.7,
    'C03_2c_80_prio': 8.5,
    'D01_A_1c_40_softheavy': 39.7,
    'P05_C_2c_pre8_shelfprio': 9.4,
}


def load_scene(label):
    task = list(json.load(open(f'configs/gen/suite_{label}.json')).values())[0]
    env = GroundHandlingEnv(config=task, verbose=False, render_mode=None)
    try:
        env.reset_settings()
        init = env.get_init_states()
        container_list = init['container_list']
        lookahead = init['lookahead_k']
        items = env.get_info_for_optimization()
    finally:
        try:
            env.close()
        except Exception:
            pass
    items_by_index = {it['index']: it for it in items}
    prepacked_ids = geo.initial_prepacked_ids(container_list)
    return container_list, items, items_by_index, lookahead, prepacked_ids


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--cand', nargs='+', required=True)
    ap.add_argument('--out', default=None)
    args = ap.parse_args()

    cands = []
    for f in args.cand:
        cands.extend(json.load(open(f)))
    cand_by_label = {c['label']: c for c in cands}

    simulate_mod.RISK_SLACK_FACES = 'all'  # 選択則には無関係、Phase29記録との整合のため固定

    results = []
    for label, leftover_s in LEFTOVER_BUDGET_S.items():
        print(f'--- {label} start ---', flush=True)
        c = cand_by_label[label]
        rec = c['records'][c['winner_idx']]
        order = rec['order']
        container_list, item_list, items_by_index, lookahead, prepacked_ids = load_scene(label)
        lookahead = max(1, int(lookahead or 1))
        print(f'  loaded scene, n_items={len(item_list)}', flush=True)

        # 1反復のコスト: 記録済み勝者順序を末尾m個(n_placedの15%、最小1個)の手前でスナップ
        # ショットし、resume_state で再開して最後まで流し込む。
        budget_f = planner.SearchBudget.from_seconds(600.0)
        buf = io.StringIO()
        with redirect_stdout(buf):
            rec_full_result = simulate_mod.simulate_order(
                container_list, items_by_index, order, lookahead, budget_f,
                prepacked_ids=prepacked_ids, stability_weight=ordering_mod.STABILITY_PENALTY_WEIGHT)
        rec_n_placed = len(rec_full_result[0])
        m = max(1, round(0.15 * rec_n_placed))
        k = max(0, rec_n_placed - m)
        print(f'  rec_n_placed={rec_n_placed} m={m} k={k}', flush=True)

        snapshot_out = {}
        budget_s = planner.SearchBudget.from_seconds(600.0)
        buf = io.StringIO()
        with redirect_stdout(buf):
            simulate_mod.simulate_order(
                container_list, items_by_index, order, lookahead, budget_s,
                prepacked_ids=prepacked_ids, stability_weight=ordering_mod.STABILITY_PENALTY_WEIGHT,
                snapshot_after=k, snapshot_out=snapshot_out)

        budget_r = planner.SearchBudget.from_seconds(600.0)
        buf = io.StringIO()
        t0 = time.perf_counter()
        with redirect_stdout(buf):
            simulate_mod.simulate_order(
                None, items_by_index, None, lookahead, budget_r,
                prepacked_ids=prepacked_ids, stability_weight=ordering_mod.STABILITY_PENALTY_WEIGHT,
                resume_state=snapshot_out)
        resume_elapsed = time.perf_counter() - t0
        print(f'  resume_elapsed={resume_elapsed:.3f}s', flush=True)

        # 全構築1回のコスト: beam_construct_order(構築) + simulate_order(validate)。
        default_items = ordering_mod.STRATEGIES[0](item_list)
        window = ordering_mod.WINDOW_CANDIDATES[0]
        budget_c = planner.SearchBudget.from_seconds(600.0)
        buf = io.StringIO()
        t0 = time.perf_counter()
        with redirect_stdout(buf):
            built_order = simulate_mod.beam_construct_order(
                container_list, default_items, budget_c,
                per_step_time_budget=ordering_mod.PER_STEP_TIME_BUDGET,
                window=window, prepacked_ids=prepacked_ids, beam_width=1)
        construct_elapsed = time.perf_counter() - t0
        print(f'  construct_elapsed={construct_elapsed:.3f}s', flush=True)

        budget_v = planner.SearchBudget.from_seconds(600.0)
        buf = io.StringIO()
        t0 = time.perf_counter()
        with redirect_stdout(buf):
            full_result = simulate_mod.simulate_order(
                container_list, items_by_index, built_order, lookahead, budget_v,
                prepacked_ids=prepacked_ids, stability_weight=ordering_mod.STABILITY_PENALTY_WEIGHT)
        validate_elapsed = time.perf_counter() - t0
        total_1restart = construct_elapsed + validate_elapsed
        n_placed_full = len(full_result[0])

        ratio = resume_elapsed / total_1restart if total_1restart > 0 else float('nan')
        n_iters = leftover_s / resume_elapsed if resume_elapsed > 0 else float('nan')

        print(f'{label}: n_items={len(item_list)} n_placed(built)={n_placed_full} '
              f'n_placed(rec)={rec_n_placed} m(tail)={m} construct={construct_elapsed:.3f}s '
              f'validate={validate_elapsed:.3f}s total_1restart={total_1restart:.3f}s '
              f'resume_tail={resume_elapsed:.3f}s ratio={ratio:.3f} '
              f'leftover={leftover_s}s -> n_iterations~={n_iters:.1f}', flush=True)
        results.append({'label': label, 'n_items': len(item_list), 'n_placed_built': n_placed_full,
                         'n_placed_rec': rec_n_placed, 'm_tail': m, 'construct_s': construct_elapsed,
                         'validate_s': validate_elapsed, 'total_1restart_s': total_1restart,
                         'resume_tail_s': resume_elapsed, 'ratio': ratio,
                         'leftover_s': leftover_s, 'n_iterations_est': n_iters})
        if args.out:
            json.dump(results, open(args.out, 'w'), indent=1, ensure_ascii=False)

    print('DONE', flush=True)


if __name__ == '__main__':
    main()
