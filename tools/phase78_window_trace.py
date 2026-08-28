"""Phase78 ステップ1-2: フェーズ1の決定的貪欲構築(simulate.beam_construct_order、体積優先
シード・ノイズ無し・beam_width=1)を WINDOW_CANDIDATES の全 window で再現し、
`MYSOLVER_BEAM_TRACE=1` の読み取り専用トレース(simulate.LAST_BEAM_TRACE)から

  - window がハード/ソフト境界をまたいだステップ数(spans_boundary)
  - window 内にハードが残っているのにソフトが選ばれたステップ数(soft_before_hard)
  - その最初のステップと、その時点の window 内ハード残数 / 全体ハード残数

を集計する。build_order の探索ロジックは一切変更していない(env 既定ではトレース分岐に
入らず 8/8 ビット単位不変を確認済み)。

    MYSOLVER_HARD_WALL_LIMIT=3000 MYSOLVER_BEAM_TRACE=1 PYTHONPATH=. .venv/bin/python \\
        tools/phase78_window_trace.py --out results/phase78_window_trace.json
"""
import argparse, glob, json, os, sys

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
from src.ground_handling.env import GroundHandlingEnv
from agents.mysolver import ordering, simulate, planner
from agents.mysolver import geometry as geo

WINDOWS = ordering.WINDOW_CANDIDATES  # [15, 20, 25, 30, None]


def parse_args():
    p = argparse.ArgumentParser()
    p.add_argument('--config-path', default='configs/gen/suite_*.json')
    p.add_argument('--out', default='/tmp/phase78_window_trace.json')
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


def run_window(cl, items, window):
    seed = ordering._strategy_volume_desc(items)
    prepacked = geo.initial_prepacked_ids(cl)
    simulate.LAST_BEAM_TRACE.clear()
    # フェーズ1の1 window あたりの実配分と同じ(CONSTRUCT_SLICE=20 名目秒、anytime)。
    budget = planner.SearchBudget.from_seconds(ordering.CONSTRUCT_SLICE)
    simulate.beam_construct_order(
        cl, seed, budget,
        per_step_time_budget=ordering.PER_STEP_TIME_BUDGET,
        rng=None, score_noise=0.0, shuffle_ties=False,
        window=window, prepacked_ids=prepacked, beam_width=ordering.BEAM_WIDTH,
    )
    trace = list(simulate.LAST_BEAM_TRACE)
    steps = len(trace)
    spans = [t for t in trace if t['spans_boundary']]
    sbh = [t for t in trace if t['soft_before_hard']]
    first_sbh = sbh[0] if sbh else None
    return {
        'steps': steps,
        'n_spans_boundary': len(spans),
        'n_soft_before_hard': len(sbh),
        'first_soft_before_hard': None if first_sbh is None else {
            'step': first_sbh['step'], 'pool_hard_left': first_sbh['pool_hard'],
            'rem_hard_left': first_sbh['rem_hard'], 'rem_soft_left': first_sbh['rem_soft'],
        },
    }


def main():
    args = parse_args()
    assert os.environ.get('MYSOLVER_BEAM_TRACE') == '1', 'set MYSOLVER_BEAM_TRACE=1'
    results = {}
    agg = {w: {'n_spans_boundary': 0, 'n_soft_before_hard': 0, 'scenes_with_sbh': 0}
           for w in map(str, WINDOWS)}
    for cp in sorted(glob.glob(args.config_path)):
        for tk, task in json.load(open(cp)).items():
            label = os.path.basename(cp).replace('suite_', '').replace('.json', '')
            cl, items, lk = load_scene(task)
            n_soft = sum(1 for it in items if it.get('is_soft', False))
            per_w = {}
            for w in WINDOWS:
                r = run_window(cl, items, w)
                per_w[str(w)] = r
                a = agg[str(w)]
                a['n_spans_boundary'] += r['n_spans_boundary']
                a['n_soft_before_hard'] += r['n_soft_before_hard']
                if r['n_soft_before_hard'] > 0:
                    a['scenes_with_sbh'] += 1
            results[label] = {'n_items': len(items), 'n_soft': n_soft, 'windows': per_w}
            summary = ' '.join(
                f"w{w}:{per_w[str(w)]['n_spans_boundary']}sp/{per_w[str(w)]['n_soft_before_hard']}sbh"
                for w in WINDOWS)
            print(f"[{label:26}] n_soft={n_soft:2} | {summary}", flush=True)

    with open(args.out, 'w') as f:
        json.dump({'per_scene': results, 'aggregate': agg}, f, indent=2)
    print("\n=== window 別 集計(全シーン合算)===")
    print(f"{'window':>8} | {'境界またぎ step 合計':>18} | {'soft先取り step 合計':>20} | {'該当シーン数':>10}")
    for w in WINDOWS:
        a = agg[str(w)]
        print(f"{str(w):>8} | {a['n_spans_boundary']:>18} | {a['n_soft_before_hard']:>20} | "
              f"{a['scenes_with_sbh']:>10}")
    print(f"wrote {args.out}")


if __name__ == '__main__':
    main()
