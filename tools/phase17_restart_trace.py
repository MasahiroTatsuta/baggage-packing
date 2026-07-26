"""
tools/phase17_restart_trace.py

Phase17: 「予算を増やしたとき、build_order は同じリスタート系列の“先頭N個”を辿っているか」を
直接確かめる高速チェック。

budget を変えて build_order を走らせ、各リスタートについて
  (通し番号, フェーズ, window, 種戦略, 配られたユニット, 得られた順序のdigest, validateスコア)
を記録する。予算に対して単調に改善するには、**リスタート i の結果が総予算 N に依存しない**
=「小さい予算の系列が大きい予算の系列の接頭辞になっている」ことが必要十分に近い。

1.8時間かかる26シーン掃引を回さなくても、この接頭辞性だけで設計の可否を判定できる。

実行例:
    PYTHONPATH=. .venv/bin/python tools/phase17_restart_trace.py \
        --config-path configs/gen/suite_P02_A_1c_pre10.json --budgets 30 60 120
"""
import argparse
import hashlib
import json
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from agents.mysolver import ordering, simulate
from src.ground_handling.env import GroundHandlingEnv

TRACE: list[dict] = []


def _install():
    orig_construct = simulate.greedy_construct_order

    def probed(container_list, item_list, budget, **kw):
        order = orig_construct(container_list, item_list, budget, **kw)
        TRACE.append({
            'i': len(TRACE),
            'window': kw.get('window'),
            'noise': kw.get('score_noise', 0.0),
            'slice_units': round(budget.limit / 1e6, 3),
            'digest': hashlib.sha256(json.dumps(order).encode()).hexdigest()[:12],
        })
        return order

    simulate.greedy_construct_order = probed


def run(task_config, budget):
    TRACE.clear()
    env = GroundHandlingEnv(config=task_config, verbose=False, render_mode=None)
    try:
        env.reset_settings()
        init = env.get_init_states()
        items = env.get_info_for_optimization()
        ordering.build_order(items, init.get('container_list'), init.get('lookahead_k'),
                             time_budget=budget)
        return list(TRACE)
    finally:
        try:
            env.close()
        except Exception:
            pass


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--config-path', required=True)
    ap.add_argument('--budgets', type=float, nargs='+', default=[30, 60, 120])
    args = ap.parse_args()

    _install()
    with open(args.config_path) as f:
        cfg = json.load(f)
    task_config = next(iter(cfg.values()))

    traces = {}
    for b in args.budgets:
        t = run(task_config, b)
        traces[b] = t
        print(f'\n=== budget={b:g}: {len(t)} restarts')
        for r in t:
            print(f'   #{r["i"]:<3} win={str(r["window"]):<5} noise={r["noise"]:<5} '
                  f'slice={r["slice_units"]:>8.2f}Mu  {r["digest"]}')

    print('\n=== 接頭辞性の判定(小予算の系列が大予算の系列の先頭と一致するか) ===')
    bs = sorted(traces)
    for lo, hi in zip(bs, bs[1:]):
        a, b = traces[lo], traces[hi]
        n = min(len(a), len(b))
        k = next((i for i in range(n) if a[i]['digest'] != b[i]['digest']), n)
        verdict = 'PREFIX-OK' if k == len(a) else f'DIVERGES at #{k}'
        print(f'   b{lo:g}({len(a)}) vs b{hi:g}({len(b)}): 一致 {k}/{len(a)}  -> {verdict}')


if __name__ == '__main__':
    main()
