"""
tools/phase33_prefix_resume.py

Phase33 タスク2(2-1, 2-2): ALNS の「接頭辞再開」が実装可能かどうかの確認専用ツール。
ALNS本体は実装しない。simulate.simulate_order に足した調査専用フック
(resume_state/snapshot_after/snapshot_out, 既定Noneで無改変)を使い、

  (a) 順序を最初から通した結果(FULL)
  (b) ステップkでスナップショットし、k+1から再開した結果(RESUME)

が全指標でビット単位一致するかを、optimize有効の先頭3シーン(A01-A03, シーンID昇順)で
検証する。候補順序は results/phase29_cand_g1.json に記録済みの winner order をそのまま
使う(新規ロールアウトなし)。

あわせて (2-2) スナップショット1回のコスト(壁時計・sys.getsizeof系のメモリ概算)を
複数の k で実測する。

実行:
    MYSOLVER_UNITS_PER_SEC=1.55e7 PYTHONPATH=. .venv/bin/python tools/phase33_prefix_resume.py \
        --cand results/phase29_cand_g1.json \
        --scenes A01_1c_40_plain A02_1c_80_plain A03_1c_40_shelf \
        --out results/phase33_prefix_resume.json
"""
import argparse
import io
import json
import os
import pickle
import sys
import time
from contextlib import redirect_stdout

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from src.ground_handling.env import GroundHandlingEnv
from agents.mysolver import geometry as geo
from agents.mysolver import ordering as ordering_mod
from agents.mysolver import planner
from agents.mysolver import simulate as simulate_mod


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
    return container_list, items_by_index, lookahead, prepacked_ids


def run_full(container_list, items_by_index, order, lookahead, prepacked_ids):
    budget = planner.SearchBudget.from_seconds(600.0)
    buf = io.StringIO()
    t0 = time.perf_counter()
    with redirect_stdout(buf):
        result = simulate_mod.simulate_order(
            container_list, items_by_index, order, lookahead, budget,
            prepacked_ids=prepacked_ids, stability_weight=ordering_mod.STABILITY_PENALTY_WEIGHT)
    elapsed = time.perf_counter() - t0
    return result, elapsed


def run_with_snapshot(container_list, items_by_index, order, lookahead, prepacked_ids, k):
    """k個目の配置直後でスナップショットを取り、その場でロールアウトを止める。
    戻り値: (snapshot dict, スナップショット取得までの壁時計)。"""
    budget = planner.SearchBudget.from_seconds(600.0)
    snapshot_out = {}
    buf = io.StringIO()
    t0 = time.perf_counter()
    with redirect_stdout(buf):
        simulate_mod.simulate_order(
            container_list, items_by_index, order, lookahead, budget,
            prepacked_ids=prepacked_ids, stability_weight=ordering_mod.STABILITY_PENALTY_WEIGHT,
            snapshot_after=k, snapshot_out=snapshot_out)
    elapsed = time.perf_counter() - t0
    return snapshot_out, elapsed


def run_resume(items_by_index, lookahead, prepacked_ids, snapshot):
    budget = planner.SearchBudget.from_seconds(600.0)
    buf = io.StringIO()
    t0 = time.perf_counter()
    with redirect_stdout(buf):
        result = simulate_mod.simulate_order(
            None, items_by_index, None, lookahead, budget,
            prepacked_ids=prepacked_ids, stability_weight=ordering_mod.STABILITY_PENALTY_WEIGHT,
            resume_state=snapshot)
    elapsed = time.perf_counter() - t0
    return result, elapsed


def snapshot_bytes(snapshot):
    """pickle化したバイト数でスナップショットのメモリコストを概算する。"""
    return len(pickle.dumps(snapshot))


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--cand', required=True)
    ap.add_argument('--scenes', nargs='+', required=True)
    ap.add_argument('--out', default=None)
    args = ap.parse_args()

    cands = json.load(open(args.cand))
    cand_by_label = {c['label']: c for c in cands}

    simulate_mod.RISK_SLACK_FACES = 'all'  # Phase31/32の実測と揃える(選択肢に無関係な引数)

    rows = []
    for label in args.scenes:
        c = cand_by_label[label]
        rec = c['records'][c['winner_idx']]
        order = rec['order']
        n_total = len(order)

        container_list, items_by_index, lookahead, prepacked_ids = load_scene(label)
        lookahead = max(1, int(lookahead or 1))

        (full_result, full_elapsed) = run_full(
            container_list, items_by_index, order, lookahead, prepacked_ids)
        placed_ids_full, placed_volume_full, risk_vol_full, viol_full, srisk_full = full_result
        n_placed_full = len(placed_ids_full)
        print(f'{label}: FULL n_placed={n_placed_full}/{n_total} '
              f'placed_volume={placed_volume_full:.6f} risk_vol={risk_vol_full:.6f} '
              f'elapsed={full_elapsed:.3f}s', flush=True)

        # k は n_placed_full の 1/4, 1/2, 3/4 の3点で確認する。
        ks = sorted({max(1, n_placed_full // 4), max(1, n_placed_full // 2),
                     max(1, (3 * n_placed_full) // 4)})
        ks = [k for k in ks if k < n_placed_full]

        for k in ks:
            snapshot, snap_elapsed = run_with_snapshot(
                container_list, items_by_index, order, lookahead, prepacked_ids, k)
            if not snapshot:
                print(f'  k={k}: スナップショット未取得(n_placed_full以上か行き詰まり)', flush=True)
                continue
            snap_size = snapshot_bytes(snapshot)

            resume_result, resume_elapsed = run_resume(
                items_by_index, lookahead, prepacked_ids, snapshot)
            placed_ids_r, placed_volume_r, risk_vol_r, viol_r, srisk_r = resume_result

            bitwise_match = (
                placed_ids_r == placed_ids_full
                and placed_volume_r == placed_volume_full
                and risk_vol_r == risk_vol_full
                and viol_r == viol_full
                and srisk_r == srisk_full
            )
            row = {
                'label': label, 'k': k, 'n_placed_full': n_placed_full,
                'n_total': n_total,
                'bitwise_match': bitwise_match,
                'placed_ids_full_tail': placed_ids_full[k:],
                'placed_ids_resume_tail': placed_ids_r[k:],
                'diff_placed_volume': placed_volume_r - placed_volume_full,
                'diff_risk_vol': risk_vol_r - risk_vol_full,
                'snapshot_bytes': snap_size,
                'snapshot_elapsed_s': snap_elapsed,
                'full_elapsed_s': full_elapsed,
                'resume_elapsed_s': resume_elapsed,
            }
            rows.append(row)
            print(f'  k={k}/{n_placed_full}: bitwise_match={bitwise_match} '
                  f'snapshot={snap_size / 1024:.1f}KiB '
                  f'diff_pv={row["diff_placed_volume"]:.2e} diff_rv={row["diff_risk_vol"]:.2e}',
                  flush=True)

        if args.out:
            json.dump(rows, open(args.out, 'w'), indent=1, ensure_ascii=False)

    n_ok = sum(1 for r in rows if r['bitwise_match'])
    print(f'\n合計 {len(rows)} 件中 {n_ok} 件がビット単位一致')


if __name__ == '__main__':
    main()
