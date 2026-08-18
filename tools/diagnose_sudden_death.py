"""sudden death(episode早期終了)の直接原因を特定する診断ツール(Phase54)。

`src/ground_handling/env.py::step()` は `is_included`/`is_valid`/`is_placed_safe`
のいずれか1つでもFalseになった時点で即座に `terminated=True` にする(リトライなし)。
本ツールはpolicyループを自前で回し、**終了を引き起こした直前のaction**(荷物属性・
狙った place_pos・向き)と、**どの検査がなぜ落ちたか**(`check_inclusion`が使う
6/7面の平面判定 `dots` — 正の値がinclusion_marginを超えた分だけ「はみ出し量」)を
記録する。`src/ground_handling/validator.py::check_inclusion` のロジックをそのまま
複製して呼ぶだけで、validator.py自体は一切変更しない。

読み取り専用(`src/`・`tools/scorer.py`・`configs/` は一切変更しない)。

実行方法(リポジトリルートで):
    MYSOLVER_HARD_WALL_LIMIT=3000 PYTHONPATH=. .venv/bin/python tools/diagnose_sudden_death.py \\
        --config-path 'configs/gen/suite_*.json' --out results/phase54_sudden_death_26.json
"""
import argparse
import glob
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
from src.ground_handling.utils import get_half_ext


def parse_args():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('--config-path', default='configs/gen/suite_*.json')
    p.add_argument('--module-path', default='agents/mysolver/')
    p.add_argument('--out', default='/tmp/diagnose_sudden_death.json')
    return p.parse_args()


def _inclusion_dots(container, item, target_pos, target_orn_idx) -> list:
    """validator.py::check_inclusion() のdots計算をそのまま複製(副作用なし)。"""
    half_lwh = get_half_ext([item.length, item.width, item.height], target_orn_idx)
    n_vecs = np.array(container.n_vecs)
    points = np.array(container.points)
    dots = (n_vecs * (np.array(target_pos) - points)).sum(axis=1) + np.dot(np.abs(n_vecs), half_lwh)
    return dots.tolist()


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
        last5 = []
        death = None
        inclusion_margin = env.validator.inclusion_margin

        while not terminated and not truncated:
            action, _ = runner.call('policy', time_out_sec=task_config['agent']['policy_timeout'],
                                     fallback=env.action_space.sample(), observation=obs)

            item_idx = action['item_idx']
            container_idx = action['container_idx']
            orn_idx = action['orientation']
            target_item = env.stream_manager.get_item(item_idx)
            target_container = env.container_manager.get_container(container_idx)
            item_snapshot = None
            dots = None
            global_pos = None
            if target_item is not None and target_container is not None:
                global_pos = list(target_container.local_to_global(action['place_pos']))
                dots = _inclusion_dots(target_container, target_item, global_pos, orn_idx)
                item_snapshot = {
                    'index': target_item.index, 'length': target_item.length,
                    'width': target_item.width, 'height': target_item.height,
                    'mass': target_item.mass, 'is_soft': target_item.is_soft,
                    'is_prioritized': target_item.is_prioritized,
                }

            obs, reward, terminated, truncated, info = env.step(action)
            n_step += 1
            status = info.get('status', {})

            step_record = {
                'n_step': n_step, 'item': item_snapshot, 'place_pos_local': action['place_pos'],
                'place_pos_global': global_pos, 'container_idx': container_idx, 'orientation': orn_idx,
                'status': status, 'inclusion_dots': dots,
            }
            last5.append(step_record)
            if len(last5) > 5:
                last5.pop(0)

            if terminated and not env.stream_manager.is_empty():
                # sudden death: どの検査が落ちたかを判定(env.pyのチェーンの順序どおり)
                if not status.get('is_included', True):
                    cause = 'is_included'
                elif not status.get('is_valid', True):
                    cause = 'is_valid'
                elif not status.get('is_placed_safe', True):
                    cause = 'is_placed_safe'
                else:
                    cause = 'unknown'
                overshoot = None
                if dots is not None:
                    excess = [d - inclusion_margin for d in dots]
                    overshoot = {'max_excess': max(excess), 'all_excess': excess}
                death = {
                    'cause': cause, 'n_placed_at_death': n_step - 1,
                    'death_item_index_in_pool': item_idx,
                    'death_item': item_snapshot, 'place_pos_global': global_pos,
                    'orientation': orn_idx, 'container_idx': container_idx,
                    'inclusion_dots': dots, 'inclusion_margin': inclusion_margin,
                    'overshoot': overshoot,
                }

        completed = env.stream_manager.is_empty()
        return {
            'status': 'ok',
            'total_items': env.num_total_items,
            'n_steps': n_step,
            'n_placed_final': sum(len(c.packed_items) for c in env.container_manager.containers),
            'completed_without_sudden_death': completed,
            'death': death,
            'last5_steps': last5,
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
    agent_module = '.'.join(args.module_path.split('/')) + 'agent'
    paths = sorted(glob.glob(args.config_path))
    results = {}
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
                print(f"[{label}] SUDDEN DEATH at step {death['n_placed_at_death']+1} "
                      f"cause={death['cause']} item_idx_in_pool={death['death_item_index_in_pool']} "
                      f"max_excess={death['overshoot']['max_excess'] if death['overshoot'] else None} "
                      f"({r['elapsed_sec']:.1f}s)")
            else:
                print(f"[{label}] completed={r.get('completed_without_sudden_death')} "
                      f"n_placed={r.get('n_placed_final')} ({r['elapsed_sec']:.1f}s)")

    def _json_default(o):
        if isinstance(o, (np.floating,)):
            return float(o)
        if isinstance(o, (np.integer,)):
            return int(o)
        if isinstance(o, np.ndarray):
            return o.tolist()
        return str(o)

    with open(args.out, 'w') as f:
        json.dump(results, f, indent=2, default=_json_default)
    print(f"wrote {args.out}")


if __name__ == '__main__':
    main()
