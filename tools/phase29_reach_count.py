"""
tools/phase29_reach_count.py

Phase29: **26シーン測定に入る前の足切り**(Phase28 §5.1 の教訓)。
衝突駆動リスタート(MYSOLVER_REPAIR=1)が、実際に build_order の出力順序を変えるシーンが
何件あるかを数える。t 値は動いたシーン数 k に対しておおむね √k で頭打ちになるので、
k が 4〜5 に届かないなら 26シーン測定を回す意味が無い。

各シーンで build_order を2回(REPAIR=0 / REPAIR=1)走らせ、返る順序を直接比較する。
"""
import argparse
import importlib
import io
import json
import os
import sys
import time
from contextlib import redirect_stdout

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from src.ground_handling.env import GroundHandlingEnv


def build(label, repair):
    os.environ['MYSOLVER_REPAIR'] = '1' if repair else '0'
    import agents.mysolver.ordering as O
    importlib.reload(O)
    task = list(json.load(open(f'configs/gen/suite_{label}.json')).values())[0]
    env = GroundHandlingEnv(config=task, verbose=False, render_mode=None)
    try:
        env.reset_settings()
        init = env.get_init_states()
        cl = init['container_list']
        la = init['lookahead_k']
        items = env.get_info_for_optimization()
    finally:
        try:
            env.close()
        except Exception:
            pass
    t0 = time.perf_counter()
    buf = io.StringIO()
    with redirect_stdout(buf):
        order = O.build_order(items, cl, la)
    return list(order), time.perf_counter() - t0, buf.getvalue()


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--labels', nargs='+', required=True)
    ap.add_argument('--out', default=None)
    args = ap.parse_args()

    rows = []
    for label in args.labels:
        off, t_off, _ = build(label, False)
        on, t_on, log = build(label, True)
        changed = off != on
        n_repair = log.count('[repair]   ')
        rows.append({'label': label, 'changed': changed, 'sec_off': t_off, 'sec_on': t_on,
                     'n_repair_attempts': n_repair})
        print(f'{label:28s} 順序が変わった={changed}  修正の試行={n_repair}回  '
              f'{t_off:.1f}s -> {t_on:.1f}s', flush=True)
        if args.out:
            json.dump(rows, open(args.out, 'w'), indent=1, ensure_ascii=False)
    print(f'\n  到達したシーン(順序が変わった): '
          f'{sum(1 for r in rows if r["changed"])}/{len(rows)}')
    print(f'  変わったシーン: {[r["label"] for r in rows if r["changed"]]}')


if __name__ == '__main__':
    main()
