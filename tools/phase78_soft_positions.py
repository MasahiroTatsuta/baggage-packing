"""Phase78 ステップ1-1: 本番に近い time_budget=150s で全シーンの build_order を1回ずつ
実行し、採用順序でのソフト荷物(is_soft)の出現位置が理想(全ソフトが末尾)からどれだけ
崩れているかを測る。読み取り専用(build_order の戻り値を観測するだけ)。

    MYSOLVER_HARD_WALL_LIMIT=3000 PYTHONPATH=. .venv/bin/python \\
        tools/phase78_soft_positions.py --budget 150 \\
        --out results/phase78_soft_positions_b150.json
"""
import argparse, glob, json, os, sys, time

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
from src.ground_handling.env import GroundHandlingEnv
from agents.mysolver import ordering as ordering_mod


def parse_args():
    p = argparse.ArgumentParser()
    p.add_argument('--config-path', default='configs/gen/suite_*.json')
    p.add_argument('--budget', type=float, default=150.0)
    p.add_argument('--out', default='/tmp/phase78_soft_positions.json')
    return p.parse_args()


def load_scene(task):
    env = GroundHandlingEnv(config=task, verbose=False, render_mode=None)
    try:
        env.reset_settings()
        init = env.get_init_states()
        items = env.get_info_for_optimization()
        return init['container_list'], items, init['lookahead_k']
    finally:
        env.close()


def analyse(order, items_by_index):
    n = len(order)
    soft_pos = [i for i, idx in enumerate(order)
                if items_by_index.get(idx, {}).get('is_soft', False)]
    n_soft = len(soft_pos)
    if n_soft == 0 or n <= 1:
        return {'n_items': n, 'n_soft': n_soft, 'mean_ratio': None, 'ideal_ratio': None,
                'diff': None, 'first_soft_pos': None, 'first_soft_ratio': None,
                'hard_after_first_soft': None}
    denom = max(n - 1, 1)
    mean_ratio = sum(p / denom for p in soft_pos) / n_soft
    n_hard = n - n_soft
    ideal = sum((n_hard + j) / denom for j in range(n_soft)) / n_soft
    first = soft_pos[0]
    hard_after = sum(1 for idx in order[first + 1:]
                     if not items_by_index.get(idx, {}).get('is_soft', False))
    return {'n_items': n, 'n_soft': n_soft, 'mean_ratio': mean_ratio, 'ideal_ratio': ideal,
            'diff': mean_ratio - ideal, 'first_soft_pos': first,
            'first_soft_ratio': first / denom, 'hard_after_first_soft': hard_after}


def main():
    args = parse_args()
    paths = sorted(glob.glob(args.config_path))
    results = {}
    for cp in paths:
        for tk, task in json.load(open(cp)).items():
            label = os.path.basename(cp).replace('suite_', '').replace('.json', '')
            cl, items, lk = load_scene(task)
            ibi = {it['index']: it for it in items}
            t0 = time.perf_counter()
            order = ordering_mod.build_order(items, cl, lk, time_budget=args.budget)
            dt = time.perf_counter() - t0
            row = analyse(order, ibi)
            row['elapsed_sec'] = dt
            row['winner'] = dict(ordering_mod.LAST_BUILD_DIAGNOSTICS)
            results[label] = row
            d = row['diff']
            print(f"[{label:26}] n_soft={row['n_soft']:2}/{row['n_items']:3} "
                  f"mean={_f(row['mean_ratio'])} ideal={_f(row['ideal_ratio'])} "
                  f"diff={_f(d)} first_soft@{row['first_soft_pos']}"
                  f"(r={_f(row['first_soft_ratio'])}) hard_after={row['hard_after_first_soft']} "
                  f"({dt:.0f}s)", flush=True)

    with open(args.out, 'w') as f:
        json.dump(results, f, indent=2)

    rows = [(k, v) for k, v in results.items() if v['diff'] is not None]
    big = [(k, v) for k, v in rows if abs(v['diff']) > 0.05]
    print(f"\n=== サマリ ===")
    print(f"ソフト有りシーン: {len(rows)}")
    print(f"|diff| > 0.05 のシーン数: {len(big)}")
    for k, v in sorted(big, key=lambda kv: kv[1]['diff']):
        print(f"  {k:26} diff={v['diff']:+.3f} first_soft_ratio={v['first_soft_ratio']:.3f} "
              f"hard_after_first_soft={v['hard_after_first_soft']}/{v['n_items']-v['n_soft']}")
    print(f"wrote {args.out}")


def _f(x):
    return 'None' if x is None else f'{x:.3f}'


if __name__ == '__main__':
    main()
