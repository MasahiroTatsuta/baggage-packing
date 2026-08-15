"""
tools/phase35_gate1.py

Phase35 ステップ1のゲート1: **26シーン測定の前に**「複製評価器による選び直しが
build_order の出力を何シーンで変えるか」を数える。

t の上限は 5√k/√(26−k)(Phase34 導出、k=1→1.000 / 2→1.443 / 3→1.806 / 4→2.132)。
**t>2 には最低4シーン必要**で、3以下なら測定せず打ち切る。

複製評価器は「代理の1位とは違う候補が実評価で勝った」ときだけ出力を変えるので、
有効側の1パスで到達シーン数が数えられる(A/Bの2パスは要らない)。

実行:
    MYSOLVER_REPLICA_SELECT=1 MYSOLVER_UNITS_PER_SEC=1.55e7 PYTHONPATH=. \
        .venv/bin/python tools/phase35_gate1.py --out results/phase35_gate1.json
"""
import argparse
import glob
import io
import json
import os
import sys
import time
from contextlib import redirect_stdout

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from src.ground_handling.env import GroundHandlingEnv
from agents.mysolver import ordering as ordering_mod
from agents.mysolver import replica as replica_mod


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--config-path', nargs='+', default=['configs/gen/suite_*.json'])
    ap.add_argument('--budget', type=float, default=ordering_mod.DEFAULT_TIME_BUDGET)
    ap.add_argument('--out', default=None)
    args = ap.parse_args()

    paths = []
    for pat in args.config_path:
        paths.extend(sorted(glob.glob(pat)) or [pat])

    rows = []
    for path in paths:
        label = os.path.basename(path).replace('suite_', '').replace('.json', '')
        task = list(json.load(open(path)).values())[0]
        env = GroundHandlingEnv(config=task, verbose=False, render_mode=None)
        try:
            env.reset_settings()
            init = env.get_init_states()
            if not env.optimize:
                print(f'{label:32s} optimize=False(build_order を呼ばない = 構造的に不変)')
                rows.append({'label': label, 'optimize': False, 'changed': False})
                continue
            items = env.get_info_for_optimization()
            container_list = init['container_list']
            lookahead = init['lookahead_k']
        finally:
            try:
                env.close()
            except Exception:
                pass

        applicable = replica_mod.is_applicable(container_list)
        ordering_mod.REPLICA_STATS.clear()
        t0 = time.perf_counter()
        with redirect_stdout(io.StringIO()):
            ordering_mod.build_order(items, container_list, lookahead, time_budget=args.budget)
        elapsed = time.perf_counter() - t0
        st = dict(ordering_mod.REPLICA_STATS)
        row = {'label': label, 'optimize': True, 'applicable': applicable,
               'elapsed_s': elapsed, 'n_cand': st.get('n_cand'),
               'n_ranked': st.get('n_ranked'), 'evaluated': st.get('evaluated', 0),
               'winner_rank': st.get('winner_rank'), 'winner_fill': st.get('winner_fill'),
               'changed': bool(st.get('changed')), 'stopped': st.get('stopped'),
               'rows': st.get('rows')}
        rows.append(row)
        rr = row['rows'] or []
        fills = ' '.join(f"{r['real_fill']:.1f}" for r in rr)
        print(f'{label:32s} 適用={applicable} 候補={row["n_cand"]} 実評価={row["evaluated"]} '
              f'勝者rank={row["winner_rank"]} 変化={row["changed"]} '
              f'[実fill: {fills}] 停止={row["stopped"]} ({elapsed:.0f}s)', flush=True)
        if args.out:
            json.dump(rows, open(args.out, 'w'), indent=1, ensure_ascii=False)

    opt = [r for r in rows if r.get('optimize')]
    app = [r for r in opt if r.get('applicable')]
    changed = [r for r in opt if r.get('changed')]
    print('\n========== ゲート1 ==========')
    print(f'optimize有効シーン   : {len(opt)} / {len(rows)}')
    print(f'複製評価器の適用対象 : {len(app)} (既積みありは適用外)')
    print(f'**出力が変わったシーン**: {len(changed)} -> {[r["label"] for r in changed]}')
    k = len(changed)
    import math
    ceil = 5 * math.sqrt(k) / math.sqrt(26 - k) if 0 < k < 26 else float('nan')
    print(f't値の構造的上限(k={k}): {ceil:.3f}')
    print('判定: ' + ('26シーン測定へ進む' if k >= 4 else '打ち切り(t>2を原理的に通過できない)'))


if __name__ == '__main__':
    main()
