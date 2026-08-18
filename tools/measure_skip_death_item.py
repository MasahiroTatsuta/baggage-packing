"""「あと1手」の価値を定量する(Phase54 ステップ2)。

`results/phase54_sudden_death_26.json`(Step1診断結果)から各シーンの sudden death を
引き起こしたアイテムのグローバルindexを読み取り、そのアイテム1個だけを
`item_stream.item_list` から除去したシーン(メモリ上でのみ、configファイルは
書き換えない)で agent.optimize()→policyループ→Scorer.evaluate() を実行し、
5成分を実測する。既存26シーン(configs/gen/suite_*.json)は一切変更しない。

読み取り専用(`src/`・`tools/scorer.py`・`configs/` は一切変更しない)。

実行方法(リポジトリルートで):
    MYSOLVER_HARD_WALL_LIMIT=3000 PYTHONPATH=. .venv/bin/python tools/measure_skip_death_item.py \\
        --death-json results/phase54_sudden_death_26.json \\
        --config-path 'configs/gen/suite_*.json' --out results/phase54_skip_death_26.json
"""
import argparse
import copy
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
from tools.scorer import Scorer


def parse_args():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('--death-json', required=True)
    p.add_argument('--config-path', default='configs/gen/suite_*.json')
    p.add_argument('--module-path', default='agents/mysolver/')
    p.add_argument('--out', default='/tmp/measure_skip_death_item.json')
    return p.parse_args()


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
            obs, reward, terminated, truncated, info = env.step(action)
            n_step += 1
            status = info.get('status', {})
            if terminated and not env.stream_manager.is_empty():
                if not status.get('is_included', True):
                    cause = 'is_included'
                elif not status.get('is_valid', True):
                    cause = 'is_valid'
                elif not status.get('is_placed_safe', True):
                    cause = 'is_placed_safe'
                else:
                    cause = 'unknown'
                death = {'cause': cause, 'n_step': n_step}

        containers = env.container_manager.containers
        scorer = Scorer(client=env.client, config=task_config)
        fill_score, _ = scorer.calculate_fill_score(containers)
        placement_score = scorer.calculate_placement_score(containers)
        soft_item_score = scorer.calculate_soft_item_score(containers)
        cog_score = scorer.calculate_cog_score(containers)
        stability_score = scorer.calculate_stability_score(containers)

        return {
            'status': 'ok',
            'total_items': env.num_total_items,
            'n_steps': n_step,
            'n_placed_final': sum(len(c.packed_items) for c in containers),
            'completed_without_second_death': env.stream_manager.is_empty(),
            'second_death': death,
            'fill_score': fill_score,
            'cog_score': cog_score,
            'stability_score': stability_score,
            'placement_score': placement_score,
            'soft_item_score': soft_item_score,
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
    death_data = json.load(open(args.death_json))
    paths = sorted(glob.glob(args.config_path))
    results = {}
    for cp in paths:
        d = json.load(open(cp))
        for tk, task in d.items():
            label = f'{os.path.basename(cp)}::{tk}'
            rec = death_data.get(label)
            if not rec or not rec.get('death'):
                print(f'[{label}] スキップ対象なし(sudden deathが記録されていない)')
                continue
            skip_index = rec['death']['death_item']['index']
            n_placed_before = rec['death']['n_placed_at_death']

            task_copy = copy.deepcopy(task)
            items = task_copy['item_stream']['item_list']
            before_n = len(items)
            task_copy['item_stream']['item_list'] = [it for it in items if it['index'] != skip_index]
            after_n = len(task_copy['item_stream']['item_list'])
            if after_n != before_n - 1:
                print(f'[{label}] WARNING: 除去件数が想定外(before={before_n} after={after_n})')

            t0 = time.perf_counter()
            r = run_one_scene(task_copy, args.module_path, agent_module)
            r['elapsed_sec'] = time.perf_counter() - t0
            r['skipped_item_global_index'] = skip_index
            r['n_placed_before_skip_run'] = n_placed_before
            results[label] = r
            print(f"[{label}] skip idx={skip_index} -> n_placed={r.get('n_placed_final')} "
                  f"fill={r.get('fill_score')} stability={r.get('stability_score')} "
                  f"placement={r.get('placement_score')} soft={r.get('soft_item_score')} "
                  f"second_death={r.get('second_death')} ({r['elapsed_sec']:.1f}s)")

    with open(args.out, 'w') as f:
        json.dump(results, f, indent=2)
    print(f"wrote {args.out}")


if __name__ == '__main__':
    main()
