"""Phase63: MYSOLVER_FALLBACK_AVOID_OBSTACLES=0 時にビット単位で旧挙動と不変であることを
確認するための、決定的8シーン(scripts/bp_check.shと同じB01-B04,P04,A01-A03)のフル
エピソード実行・アクション列ハッシュ化ツール。

読み取り専用(configs/・src/は一切変更しない)。

実行方法(リポジトリルートで):
    PYTHONPATH=. .venv/bin/python tools/phase63_determinism_check.py --out /tmp/phase63_det.json
"""
import argparse
import hashlib
import json
import os
import sys
import time

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from src.ground_handling.agent_factory import AgentFactory
from src.ground_handling.env import GroundHandlingEnv
from src.ground_handling.runner import TimedAgentRunner

SCENES = {
    'B01': 'configs/gen/suite_B01_1c_40_plain.json',
    'B02': 'configs/gen/suite_B02_1c_40_shelf.json',
    'B03': 'configs/gen/suite_B03_2c_80_prio.json',
    'B04': 'configs/gen/suite_B04_2c_80_noprio.json',
    'P04': 'configs/gen/suite_P04_B_1c_pre8_shelf.json',
    'A01': 'configs/gen/suite_A01_1c_40_plain.json',
    'A02': 'configs/gen/suite_A02_1c_80_plain.json',
    'A03': 'configs/gen/suite_A03_1c_40_shelf.json',
}


def _json_default(o):
    if isinstance(o, (np.floating,)):
        return float(o)
    if isinstance(o, (np.integer,)):
        return int(o)
    if isinstance(o, np.ndarray):
        return o.tolist()
    return str(o)


def run_one_scene(cp: str, module_path='agents/mysolver/') -> dict:
    task = list(json.load(open(cp)).values())[0]
    agent_factory = AgentFactory(module_name='agents.mysolver.agent', class_name='Agent', module_path=module_path)
    env = GroundHandlingEnv(config=task, verbose=False, render_mode=None)
    try:
        env.reset_settings()
        init_states = env.get_init_states()
        runner = TimedAgentRunner(agent_factory=agent_factory, allowed_methods=task['agent']['allowed_methods'],
                                   max_mem=task['agent'].get('max_mem', 4), verbose=False)
        runner.call('get_init_states', time_out_sec=task['agent']['init_timeout'], fallback=None,
                    init_states=init_states)
        if env.optimize:
            item_list = env.get_info_for_optimization()
            optimized_order, _ = runner.call(
                'optimize', time_out_sec=task['agent']['optimization_timeout'],
                fallback=list(env.stream_manager.all_indices), item_list=item_list)
            env.set_item_order(optimized_order)
        env.reset_item_stream()
        obs, info = env.reset(seed=42)
        terminated = truncated = False
        actions = []
        n_step = 0
        while not terminated and not truncated:
            action, _ = runner.call('policy', time_out_sec=task['agent']['policy_timeout'],
                                     fallback=env.action_space.sample(), observation=obs)
            actions.append({'item_idx': int(action['item_idx']), 'container_idx': int(action['container_idx']),
                             'orientation': int(action['orientation']),
                             'place_pos': [float(x) for x in np.asarray(action['place_pos']).tolist()]})
            obs, reward, terminated, truncated, info = env.step(action)
            n_step += 1
        n_placed = sum(len(c.packed_items) for c in env.container_manager.containers)
        digest = hashlib.sha256(json.dumps(actions, default=_json_default).encode()).hexdigest()
        return {'n_steps': n_step, 'n_placed': n_placed, 'action_digest': digest, 'n_actions': len(actions)}
    finally:
        try:
            env.close()
        except Exception:
            pass


def main():
    p = argparse.ArgumentParser()
    p.add_argument('--out', default='/tmp/phase63_det.json')
    args = p.parse_args()
    results = {}
    for label, cp in SCENES.items():
        t0 = time.perf_counter()
        r = run_one_scene(cp)
        r['elapsed_sec'] = time.perf_counter() - t0
        results[label] = r
        print(f"[{label}] n_steps={r['n_steps']} n_placed={r['n_placed']} digest={r['action_digest'][:16]} "
              f"({r['elapsed_sec']:.1f}s)")
    with open(args.out, 'w') as f:
        json.dump(results, f, indent=2)
    print(f'wrote {args.out}')


if __name__ == '__main__':
    main()
