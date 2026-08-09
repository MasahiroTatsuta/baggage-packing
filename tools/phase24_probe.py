"""
tools/phase24_probe.py

Phase24 ターゲット2の事後診断: 不感帯の帯域別化(`surface` モード)が
**候補の意思決定にどれだけ届いているか**を実行時に数える。

26シーンA/Bで 25/26 のシーンが1ビットも変わらなかったため、
「機序は実在するのに、決定時点では発火していない」という仮説を検証する。

各 `_corridor_excess` 呼び出しについて uniform / surface の両方を計算し、
  ・守るべき通路がある候補(min_limit が有限)の数
  ・不感帯が縮む(=拘束面が直置き帯にある)候補の数
  ・excess>0 になる候補の数(uniform / surface)
  ・argmax が動いた plan() 呼び出しの数
を数える。src/ と agents/ は変更しない(実行時にラップするだけ)。

    PYTHONPATH=. .venv/bin/python tools/phase24_probe.py \
        --config-path configs/gen/suite_A01_1c_40_plain.json ...
"""
import argparse
import glob
import json
import os
import sys
from contextlib import redirect_stdout

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

import numpy as np

from src.ground_handling.env import GroundHandlingEnv
import agents.mysolver.planner as planner
from agents.mysolver.agent import Agent

STATS = {}


def _reset():
    STATS.clear()
    STATS.update(calls=0, cands=0, protected=0, db_shrunk=0,
                 excess_uniform=0, excess_surface=0, argmax_moved=0)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--config-path', nargs='+', required=True)
    ap.add_argument('--out', default=None)
    args = ap.parse_args()

    paths = []
    for pat in args.config_path:
        m = sorted(glob.glob(pat))
        paths.extend(m if m else [pat])

    orig = planner._corridor_excess

    def wrapped(container, half, wx, wy, wz, obstacles):
        planner.CORRIDOR_DB_MODE = 'uniform'
        u = orig(container, half, wx, wy, wz, obstacles)
        planner.CORRIDOR_DB_MODE = 'surface'
        s = orig(container, half, wx, wy, wz, obstacles)
        planner.CORRIDOR_DB_MODE = 'uniform'
        STATS['calls'] += 1
        STATS['cands'] += wx.shape[0]
        # 不感帯が縮んだ候補 = surface のほうが excess が大きい、または
        # uniform では 0 だが surface では正
        STATS['db_shrunk'] += int(np.count_nonzero(s > u + 1e-12))
        STATS['excess_uniform'] += int(np.count_nonzero(u > 1e-12))
        STATS['excess_surface'] += int(np.count_nonzero(s > 1e-12))
        return u

    planner._corridor_excess = wrapped
    out = {}
    try:
        for cp in paths:
            with open(cp) as f:
                cfg = json.load(f)
            for tid, tc in cfg.items():
                label = f'{os.path.basename(cp)}::{tid}'
                _reset()
                env = GroundHandlingEnv(config=tc, verbose=False, render_mode=None)
                try:
                    with open(os.devnull, 'w') as dn, redirect_stdout(dn):
                        env.reset_settings()
                        agent = Agent('agents/mysolver/')
                        agent.get_init_states(env.get_init_states())
                        if env.optimize:
                            env.set_item_order(list(agent.optimize(env.get_info_for_optimization())))
                        env.reset_item_stream()
                        obs, info = env.reset(seed=42)
                        term = trunc = False
                        while not term and not trunc:
                            obs, r, term, trunc, info = env.step(agent.policy(obs))
                finally:
                    try:
                        env.close()
                    except Exception:
                        pass
                st = dict(STATS)
                out[label] = st
                c = max(st['cands'], 1)
                print(f'[{label}] cands={st["cands"]} '
                      f'excess>0(uniform)={st["excess_uniform"]} ({st["excess_uniform"]/c*100:.2f}%) '
                      f'excess>0(surface)={st["excess_surface"]} ({st["excess_surface"]/c*100:.2f}%) '
                      f'不感帯が縮んだ候補={st["db_shrunk"]} ({st["db_shrunk"]/c*100:.2f}%)', flush=True)
    finally:
        planner._corridor_excess = orig

    if args.out:
        with open(args.out, 'w') as f:
            json.dump(out, f)
    tot = {k: sum(v[k] for v in out.values()) for k in next(iter(out.values()))}
    c = max(tot['cands'], 1)
    print(f'\n=== TOTAL over {len(out)} scenes ===')
    print(f'評価した候補 (corridor 計算対象): {tot["cands"]}')
    print(f'excess>0 (uniform, 現行)        : {tot["excess_uniform"]} ({tot["excess_uniform"]/c*100:.2f}%)')
    print(f'excess>0 (surface)              : {tot["excess_surface"]} ({tot["excess_surface"]/c*100:.2f}%)')
    print(f'不感帯が縮んだ候補              : {tot["db_shrunk"]} ({tot["db_shrunk"]/c*100:.2f}%)')


if __name__ == '__main__':
    main()
