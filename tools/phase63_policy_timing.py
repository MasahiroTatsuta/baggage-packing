"""Phase63 (2-4): policy()の実行時間(全手番)を計測する。
`agent.py`は変更しない(既存の`MYSOLVER_FALLBACK_AVOID_OBSTACLES`で挙動を切り替えるだけ)。

実行方法:
    PYTHONPATH=. .venv/bin/python tools/phase63_policy_timing.py --config-path 'configs/gen/suite_*.json'
"""
import argparse
import glob
import json
import os
import sys
import time

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from src.ground_handling.agent_factory import AgentFactory
from src.ground_handling.env import GroundHandlingEnv
from src.ground_handling.runner import TimedAgentRunner


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
        times = []
        while not terminated and not truncated:
            t0 = time.perf_counter()
            action, _ = runner.call('policy', time_out_sec=task['agent']['policy_timeout'],
                                     fallback=env.action_space.sample(), observation=obs)
            times.append(time.perf_counter() - t0)
            obs, reward, terminated, truncated, info = env.step(action)
        return {'times': times, 'n_steps': len(times), 'max': max(times), 'mean': sum(times) / len(times)}
    finally:
        try:
            env.close()
        except Exception:
            pass


def main():
    p = argparse.ArgumentParser()
    p.add_argument('--config-path', action='append', default=None)
    args = p.parse_args()
    patterns = args.config_path or ['configs/gen/suite_*.json']
    flag = os.environ.get('MYSOLVER_FALLBACK_AVOID_OBSTACLES', '1')
    all_times = []
    paths = sorted({p for pat in patterns for p in glob.glob(pat)})
    for cp in paths:
        r = run_one_scene(cp)
        all_times.extend(r['times'])
        print(f"[{os.path.basename(cp)}] n_steps={r['n_steps']} max={r['max']:.3f}s mean={r['mean']:.3f}s "
              f"last_step={r['times'][-1]:.3f}s")
    print(f"=== flag={flag} 全体: n={len(all_times)} mean={sum(all_times)/len(all_times):.3f}s "
          f"max={max(all_times):.3f}s ===")


if __name__ == '__main__':
    main()
