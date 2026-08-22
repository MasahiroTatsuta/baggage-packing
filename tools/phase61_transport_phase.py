"""Phase61 §3-2: sudden death(cause=is_valid)を起こした手番について、
`validator.check_transport_path` の判定が Y区間(phase1)/X区間(phase2) の
どちらで失敗したかを集計する診断ツール。

`_move_item` の呼び出しを監視するだけで、`src/ground_handling` 側のコードは
一切変更しない(読み取り専用)。check_transport_path は:
  - phase1(Y方向掃引)が失敗 → その場でreturn(_move_item呼び出しは1回のみ)
  - phase1が成功 → phase2(X方向掃引)を実行 → その結果をreturn(_move_item呼び出しは2回)
なので、is_valid=False の手番で _move_item が何回呼ばれ、最後の呼び出しが
どちらのphaseだったかを見れば、Y区間/X区間のどちらで落ちたかが厳密に分かる。

実行方法(リポジトリルートで):
    PYTHONPATH=. .venv/bin/python tools/phase61_transport_phase.py \\
        --config-path 'configs/gen/suite_*.json' --out results/phase61_transport_phase_26.json
"""
import argparse
import glob
import json
import os
import sys
import time
import traceback

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from src.ground_handling.agent_factory import AgentFactory
from src.ground_handling.env import GroundHandlingEnv
from src.ground_handling.runner import TimedAgentRunner
from src.ground_handling.validator import PlacementValidator


def parse_args():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('--config-path', default='configs/gen/suite_*.json')
    p.add_argument('--module-path', default='agents/mysolver/')
    p.add_argument('--out', default='/tmp/phase61_transport_phase.json')
    return p.parse_args()


def _install_move_item_probe():
    """PlacementValidator._move_item をラップし、呼び出しごとの is_packable を
    インスタンス属性 `_phase61_calls` に記録する。元のメソッドは呼ぶだけで変更しない。"""
    original = PlacementValidator._move_item

    def wrapped(self, *args, **kwargs):
        is_packable, current_pos = original(self, *args, **kwargs)
        if not hasattr(self, '_phase61_calls'):
            self._phase61_calls = []
        self._phase61_calls.append(bool(is_packable))
        return is_packable, current_pos

    PlacementValidator._move_item = wrapped
    return original


def _classify(calls: list) -> str:
    if not calls:
        return 'no_move_item_call'
    if len(calls) == 1:
        return 'Y区間(phase1)' if not calls[0] else 'unknown(phase1成功なのに1回のみ)'
    if len(calls) == 2:
        return 'X区間(phase2)' if not calls[1] else 'unknown(2回とも成功)'
    return f'unknown(呼び出し{len(calls)}回)'


def run_one_scene(task_config: dict, module_path: str, agent_module: str) -> dict:
    agent_factory = AgentFactory(module_name=agent_module, class_name='Agent', module_path=module_path)
    env = GroundHandlingEnv(config=task_config, verbose=False, render_mode=None)
    try:
        env.reset_settings()
        init_states = env.get_init_states()
        allowed_methods = task_config['agent']['allowed_methods']
        max_mem = task_config['agent'].get('max_mem', 4)
        runner = TimedAgentRunner(agent_factory=agent_factory, allowed_methods=allowed_methods,
                                   max_mem=max_mem, verbose=False)
        runner.call('get_init_states', time_out_sec=task_config['agent']['init_timeout'],
                    fallback=None, init_states=init_states)

        if env.optimize:
            item_list = env.get_info_for_optimization()
            optimized_order, _ = runner.call(
                'optimize', time_out_sec=task_config['agent']['optimization_timeout'],
                fallback=list(env.stream_manager.all_indices), item_list=item_list)
            if not env.set_item_order(optimized_order):
                return {'status': 'optimize_failed'}

        env.reset_item_stream()
        obs, info = env.reset(seed=42)
        terminated = False
        truncated = False
        n_step = 0
        death = None

        while not terminated and not truncated:
            action, _ = runner.call('policy', time_out_sec=task_config['agent']['policy_timeout'],
                                     fallback=env.action_space.sample(), observation=obs)
            env.validator._phase61_calls = []
            obs, reward, terminated, truncated, info = env.step(action)
            n_step += 1
            status = info.get('status', {})

            if terminated and not env.stream_manager.is_empty():
                cause = None
                if not status.get('is_included', True):
                    cause = 'is_included'
                elif not status.get('is_valid', True):
                    cause = 'is_valid'
                elif not status.get('is_placed_safe', True):
                    cause = 'is_placed_safe'
                else:
                    cause = 'unknown'
                phase = None
                if cause == 'is_valid':
                    phase = _classify(getattr(env.validator, '_phase61_calls', []))
                death = {'cause': cause, 'n_placed_at_death': n_step - 1, 'transport_phase': phase,
                         'move_item_calls': getattr(env.validator, '_phase61_calls', [])}

        completed = env.stream_manager.is_empty()
        return {
            'status': 'ok',
            'n_steps': n_step,
            'completed_without_sudden_death': completed,
            'death': death,
        }
    except Exception:
        return {'status': f'error: {traceback.format_exc().splitlines()[-1]}'}
    finally:
        try:
            env.close()
        except Exception:
            pass


def main():
    args = parse_args()
    _install_move_item_probe()
    agent_module = '.'.join(args.module_path.split('/')) + 'agent'
    paths = sorted(glob.glob(args.config_path))
    results = {}
    tally = {}
    for cp in paths:
        d = json.load(open(cp))
        for tk, task in d.items():
            label = f'{os.path.basename(cp)}::{tk}'
            t0 = time.perf_counter()
            r = run_one_scene(task, args.module_path, agent_module)
            r['elapsed_sec'] = time.perf_counter() - t0
            results[label] = r
            death = r.get('death')
            if death:
                key = f"{death['cause']}" + (f" / {death['transport_phase']}" if death['transport_phase'] else '')
                tally[key] = tally.get(key, 0) + 1
                print(f"[{label}] SUDDEN DEATH cause={death['cause']} phase={death['transport_phase']} "
                      f"({r['elapsed_sec']:.1f}s)")
            else:
                tally['no_death'] = tally.get('no_death', 0) + 1
                print(f"[{label}] completed={r.get('completed_without_sudden_death')} ({r['elapsed_sec']:.1f}s)")

    print('=== tally ===')
    for k, v in sorted(tally.items(), key=lambda kv: -kv[1]):
        print(f'  {k}: {v}')

    with open(args.out, 'w') as f:
        json.dump({'results': results, 'tally': tally}, f, indent=2)
    print(f"wrote {args.out}")


if __name__ == '__main__':
    main()
