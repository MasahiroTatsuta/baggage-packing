"""
tools/phase34_gate1.py

Phase34 ゲート1: **26シーン測定の前に**「ALNS が build_order の出力を何シーンで変えるか」
を数える。

Phase28 は1シーン、Phase29 は2シーンしか動かず、どちらも t 値の構造的上限
(k シーンしか動かないとき t は √k で頭打ち)で落ちた。効果量を測る前に **到達範囲** を
確認するのがこのツールの役割である。

ALNS は「厳密に改善したときだけ採用する」山登りなので、
**採用が1回でも起きた ⇔ ALNS 無効時と出力が変わる** が成り立つ。したがって
ALNS 有効の1パスだけで到達シーン数が数えられる(A/Bの2パスは要らない)。

判定(指示書):
  8シーン以上 → 26シーン測定へ進む
  4〜7シーン  → t の上限を確認した上で進む
  3シーン以下 → 打ち切って報告

実行:
    MYSOLVER_ALNS=1 MYSOLVER_UNITS_PER_SEC=1.55e7 PYTHONPATH=. \
        .venv/bin/python tools/phase34_gate1.py --out results/phase34_gate1.json
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
                rows.append({'label': label, 'optimize': False})
                continue
            items = env.get_info_for_optimization()
            container_list = init['container_list']
            lookahead = init['lookahead_k']
        finally:
            try:
                env.close()
            except Exception:
                pass

        ordering_mod.ALNS_STATS.clear()
        t0 = time.perf_counter()
        with redirect_stdout(io.StringIO()):
            ordering_mod.build_order(items, container_list, lookahead, time_budget=args.budget)
        elapsed = time.perf_counter() - t0
        st = dict(ordering_mod.ALNS_STATS)
        iters = st.get('iter_s') or []
        row = {
            'label': label, 'optimize': True, 'elapsed_s': elapsed,
            'n_items': st.get('n_items'), 'fraction_s': st.get('fraction_s'),
            'validate_units_s': st.get('validate_units_s'),
            'n_snapshots': st.get('n_snapshots'),
            'n_eval': st.get('n_eval', 0), 'n_accept': st.get('n_accept', 0),
            'gain': st.get('gain', 0.0), 'stopped': st.get('stopped'),
            'ops': st.get('ops'),
            'iter_s_mean': (sum(iters) / len(iters)) if iters else None,
            'iter_s_max': max(iters) if iters else None,
            'changed': bool(st.get('n_accept', 0) > 0),
        }
        rows.append(row)
        ops = row['ops'] or {}
        print(f'{label:32s} 端数={row["fraction_s"] or 0:6.1f}s validate実費={row["validate_units_s"] or 0:5.2f}s '
              f'評価={row["n_eval"]:3d} 採用={row["n_accept"]:2d} '
              f'gain={row["gain"]:+.4f} 1反復={row["iter_s_mean"] or 0:5.2f}s '
              f'停止={row["stopped"]} '
              f'[occ {ops.get("occupier", {}).get("accepted", 0)}/{ops.get("occupier", {}).get("evaluated", 0)} '
              f'blk {ops.get("blocker", {}).get("accepted", 0)}/{ops.get("blocker", {}).get("evaluated", 0)} '
              f'wst {ops.get("worst", {}).get("accepted", 0)}/{ops.get("worst", {}).get("evaluated", 0)}] '
              f'({elapsed:.0f}s)', flush=True)
        if args.out:
            json.dump(rows, open(args.out, 'w'), indent=1, ensure_ascii=False)

    opt = [r for r in rows if r.get('optimize')]
    changed = [r for r in opt if r.get('changed')]
    reached = [r for r in opt if r.get('n_eval', 0) > 0]
    print('\n========== ゲート1 ==========')
    print(f'optimize有効シーン        : {len(opt)} / {len(rows)}')
    print(f'ALNS が1回でも評価したシーン: {len(reached)}')
    print(f'**出力が変わったシーン**   : {len(changed)}  -> {[r["label"] for r in changed]}')
    verdict = ('26シーン測定へ進む' if len(changed) >= 8
               else ('t の上限を確認の上で進む' if len(changed) >= 4 else '打ち切り(t>2を原理的に通過できない)'))
    print(f'判定                      : {verdict}')


if __name__ == '__main__':
    main()
