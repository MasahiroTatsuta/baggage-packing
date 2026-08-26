"""Phase72: build_order()のbest_orderがフェーズ1由来かフェーズ2由来かを実測する
(読み取り専用の診断。ordering.LAST_BUILD_DIAGNOSTICSを読むだけで、探索の挙動は変えない)。

26シーン+sample_configそれぞれについて、agents.mysolver.ordering.build_order()を
リポジトリ既定の壁時計非拘束(MYSOLVER_HARD_WALL_LIMIT=3000)で1回呼び出し、
- 採用されたbest_orderの由来(winner_source: heuristic/phase1/phase2/repair/alns/replica_select)
- 種となった戦略名(winner_strategy)
- 採用順序でのソフト荷物(is_soft)の出現位置(0=先頭,1=末尾で正規化した比率)の分布
- 初期シード(4戦略、いずれもis_soft最優先)での理想値(=ソフト荷物は全て後半に固まるはず)
  との比較
を記録する。**現在の支持閾値はリポジトリ既定(0.55/0.6/0.15)のまま、閾値環境変数は
一切設定しない**(緩1〜3の閾値とは混ぜない)。

実行方法:
    MYSOLVER_HARD_WALL_LIMIT=3000 PYTHONPATH=. .venv/bin/python \\
        tools/phase72_winner_trace.py --config-path 'configs/gen/suite_*.json' \\
        --out results/phase72_winner_trace.json
"""
import argparse
import glob
import json
import os
import sys
import time

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from src.ground_handling.env import GroundHandlingEnv
from agents.mysolver import ordering as ordering_mod


def parse_args():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('--config-path', default='configs/gen/suite_*.json')
    p.add_argument('--out', default='/tmp/phase72_winner_trace.json')
    return p.parse_args()


def load_scene(cp_task):
    env = GroundHandlingEnv(config=cp_task, verbose=False, render_mode=None)
    try:
        env.reset_settings()
        init = env.get_init_states()
        items = env.get_info_for_optimization()
        return init['container_list'], items, init['lookahead_k']
    finally:
        env.close()


def soft_position_stats(order, items_by_index):
    n = len(order)
    if n == 0:
        return {'n_soft': 0, 'mean_pos_ratio': None, 'positions_ratio': []}
    soft_positions = []
    for i, idx in enumerate(order):
        item = items_by_index.get(idx)
        if item is not None and item.get('is_soft', False):
            soft_positions.append(i / max(n - 1, 1))
    mean_ratio = sum(soft_positions) / len(soft_positions) if soft_positions else None
    return {'n_soft': len(soft_positions), 'mean_pos_ratio': mean_ratio,
            'positions_ratio': soft_positions}


def ideal_seed_stats(item_list):
    """4戦略シード(is_soft最優先)での理想的なソフト位置比率(=ハードが尽きた後に
    ソフトが並ぶ場合の平均位置)を計算する。全戦略共通でis_softは第1キーなので
    どの戦略でも同一の理想値になる。"""
    n = len(item_list)
    n_soft = sum(1 for it in item_list if it.get('is_soft', False))
    n_hard = n - n_soft
    if n_soft == 0 or n <= 1:
        return None
    # ハードが0..n_hard-1、ソフトがn_hard..n-1に並ぶ理想順序での平均位置比率
    positions = [(n_hard + j) / max(n - 1, 1) for j in range(n_soft)]
    return sum(positions) / len(positions)


def main():
    args = parse_args()
    paths = sorted(glob.glob(args.config_path))
    results = {}
    for cp in paths:
        tasks = json.load(open(cp))
        for tk, task in tasks.items():
            label = f'{os.path.basename(cp)}::{tk}' if len(tasks) > 1 else os.path.basename(cp)
            container_list, items, lookahead_k = load_scene(task)
            items_by_index = {it['index']: it for it in items}
            t0 = time.perf_counter()
            order = ordering_mod.build_order(items, container_list, lookahead_k, time_budget=30.0)
            dt = time.perf_counter() - t0
            diag = dict(ordering_mod.LAST_BUILD_DIAGNOSTICS)
            pos_stats = soft_position_stats(order, items_by_index)
            ideal = ideal_seed_stats(items)
            row = {
                'winner_source': diag.get('winner_source'),
                'winner_strategy': diag.get('winner_strategy'),
                'n_items': len(items),
                'n_soft': pos_stats['n_soft'],
                'mean_soft_pos_ratio': pos_stats['mean_pos_ratio'],
                'ideal_soft_pos_ratio': ideal,
                'positions_ratio': pos_stats['positions_ratio'],
                'elapsed_sec': dt,
            }
            results[label] = row
            print(f"[{label}] winner={row['winner_source']}({row['winner_strategy']}) "
                  f"n_soft={row['n_soft']}/{row['n_items']} "
                  f"mean_soft_pos={row['mean_soft_pos_ratio']} ideal={row['ideal_soft_pos_ratio']} "
                  f"({dt:.1f}s)", flush=True)

    with open(args.out, 'w') as f:
        json.dump(results, f, indent=2)
    print(f"wrote {args.out}")


if __name__ == '__main__':
    main()
