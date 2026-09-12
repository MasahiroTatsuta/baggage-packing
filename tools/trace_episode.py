"""現行主枠(ordcount)でエピソードを1本走らせ、3D可視化用のトレースJSONを書き出す。

各ステップについて「その時点で積まれている全荷物の実際の静止姿勢(沈降後)」を記録するので、
スクラブして沈み込み・傾き・崩れをそのまま観察できる。死亡ステップでは、拒否された
action と status(is_included/is_valid/is_placed_safe)も残す。

読み取り専用(src/・configs/・agents/ は一切変更しない)。

実行例:
    PYTHONPATH=. MYSOLVER_LASTRESORT=1 MYSOLVER_ORDER_BY_COUNT=1 \\
      .venv/bin/python tools/trace_episode.py \\
      --config configs/gen/suite_A08_2c_140_extreme.json --out results/trace_A08.json
"""
import argparse
import json
import os
import sys
import time
import traceback

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from src.ground_handling.agent_factory import AgentFactory
from src.ground_handling.env import GroundHandlingEnv
from src.ground_handling.runner import TimedAgentRunner


def r(x, n=4):
    if isinstance(x, (list, tuple, np.ndarray)):
        return [r(v, n) for v in x]
    return round(float(x), n)


def snapshot(env):
    """その時点で各コンテナに積まれている全荷物の、実際の静止姿勢を採る。"""
    out = []
    for c in env.container_manager.containers:
        c.update_packed_items(env.client)
        for it in c.packed_items:
            if it.pos is None:
                continue
            out.append({
                'i': it.index, 'c': c.index,
                'd': r([it.length, it.width, it.height]),
                'p': r(it.pos), 'q': r(it.orn, 5),
                's': int(bool(it.is_soft)), 'pr': int(bool(it.is_prioritized)),
                'm': r(it.mass, 2),
            })
    return out


def containers_geom(env):
    out = []
    for c in env.container_manager.containers:
        out.append({
            'index': c.index, 'offset_x': r(c.offset_x), 'thickness': r(c.thickness),
            'length': r(c.length), 'width': r(c.width), 'height': r(c.height),
            'cut_x': r(c.cut_x), 'cut_y': r(c.cut_y),
            'require_shelf': bool(c.require_shelf),
            'is_prioritized': bool(c.is_prioritized),
            'center': r(getattr(c, 'center', (0, 0, 0))),
        })
    return out


def run(task_config, module_path='agents/mysolver/'):
    agent_module = '.'.join(module_path.split('/')) + 'agent'
    factory = AgentFactory(module_name=agent_module, class_name='Agent', module_path=module_path)
    env = GroundHandlingEnv(config=task_config, verbose=False, render_mode=None)
    try:
        env.reset_settings()
        init_states = env.get_init_states()
        a = task_config['agent']
        runner = TimedAgentRunner(agent_factory=factory, allowed_methods=a['allowed_methods'],
                                   max_mem=a.get('max_mem', 4), verbose=False)
        runner.call('get_init_states', time_out_sec=a['init_timeout'], fallback=None, init_states=init_states)
        if env.optimize:
            items = env.get_info_for_optimization()
            order, _ = runner.call('optimize', time_out_sec=a['optimization_timeout'],
                                    fallback=list(env.stream_manager.all_indices), item_list=items)
            env.set_item_order(order)
        env.reset_item_stream()
        obs, info = env.reset(seed=42)

        geom = containers_geom(env)
        steps = [{'n': 0, 'action': None, 'status': None, 'items': snapshot(env)}]
        total = env.num_total_items
        terminated = truncated = False
        n = 0
        death = None
        while not terminated and not truncated:
            act, _ = runner.call('policy', time_out_sec=a['policy_timeout'],
                                  fallback=env.action_space.sample(), observation=obs)
            pool = obs.get('pool_list', [])
            ii = int(act.get('item_idx', 0))
            chosen = pool[ii] if 0 <= ii < len(pool) else None
            obs, reward, terminated, truncated, info = env.step(act)
            n += 1
            st = info.get('status', {})
            rec = {
                'n': n,
                'action': {
                    'item_index': (chosen or {}).get('index'),
                    'container_idx': int(act.get('container_idx', 0)),
                    'pos': r(np.asarray(act.get('place_pos'), dtype=float)),
                    'orn': int(act.get('orientation', 0)),
                    'dims': r([(chosen or {}).get('length', 0), (chosen or {}).get('width', 0),
                               (chosen or {}).get('height', 0)]),
                    'is_soft': int(bool((chosen or {}).get('is_soft', False))),
                    'is_prioritized': int(bool((chosen or {}).get('is_prioritized', False))),
                },
                'status': {k: bool(v) for k, v in st.items()},
                'items': snapshot(env),
            }
            steps.append(rec)
            if terminated and not env.stream_manager.is_empty():
                death = {'n': n, 'status': rec['status'], 'action': rec['action']}
        return {
            'total_items': total,
            'n_steps': n,
            'n_placed': len(steps[-1]['items']),
            'death': death,
            'containers': geom,
            'steps': steps,
        }
    finally:
        try:
            env.close()
        except Exception:
            pass


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument('--config', required=True)
    ap.add_argument('--module-path', default='agents/mysolver/')
    ap.add_argument('--out', required=True)
    args = ap.parse_args()
    d = json.load(open(args.config))
    tk = list(d)[0]
    t0 = time.perf_counter()
    try:
        res = run(d[tk], args.module_path)
    except Exception:
        print(traceback.format_exc())
        raise
    res['scene'] = os.path.basename(args.config)
    res['elapsed_sec'] = round(time.perf_counter() - t0, 1)
    with open(args.out, 'w') as f:
        json.dump(res, f, separators=(',', ':'))
    sz = os.path.getsize(args.out) / 1e6
    print(f"[{res['scene']}] steps={res['n_steps']} placed={res['n_placed']}/{res['total_items']} "
          f"death={'yes' if res['death'] else 'no'} out={args.out} ({sz:.1f} MB, {res['elapsed_sec']}s)")


if __name__ == '__main__':
    main()
