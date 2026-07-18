"""
tools/local_eval.py

`src/ground_handling/app.py` の EvaluationApp.run() と同じ流れ
(env初期化 -> get_init_states -> [optimize] -> policy loopをterminated/truncatedまで実行)
をそのまま踏襲しつつ、最後に本家Evaluator(fillのみ)の代わりに tools/scorer.py の Scorer で
5指標(fill/cog/stability/placement/soft) + num_placed_items + status + 処理時間 を算出する
ローカル評価ループ。

メモリ節約のため、config・シーンを並列実行はせず1つずつ逐次実行する。
src/ と configs/ は一切変更しない。

実行例:
    PYTHONPATH=. .venv/bin/python tools/local_eval.py \
        --config-path configs/sample_config.json --module-path agents/mysolver/
"""
import argparse
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

METRIC_KEYS = ['fill_score', 'cog_score', 'stability_score', 'placement_score', 'soft_item_score']


def parse_args():
    parser = argparse.ArgumentParser()
    parser.add_argument('--config-path', nargs='+', default=['configs/sample_config.json'],
                         help='評価するconfigファイルのパス(複数指定可)')
    parser.add_argument('--module-path', default='agents/base/', help='エージェントモジュールのパス(末尾/込み)')
    parser.add_argument('--render-mode', default=None, help='human or None')
    parser.add_argument('--verbose', action='store_true', help='envの詳細ログを出す')
    return parser.parse_args()


def run_one_scene(task_config: dict, module_path: str, agent_module: str, render_mode, verbose: bool) -> dict:
    """1シーン分を app.py と同じ手順で実行し、Scorerで5指標を算出して結果dictを返す"""
    agent_factory = AgentFactory(module_name=agent_module, class_name='Agent', module_path=module_path)
    env = GroundHandlingEnv(config=task_config, verbose=verbose, render_mode=render_mode)
    runner = None
    optimize_time = 0.0
    policy_time = 0.0
    place_states: dict = {}

    try:
        env.reset_settings()
        init_states = env.get_init_states()

        allowed_methods = task_config['agent']['allowed_methods']
        max_mem = task_config['agent'].get('max_mem', 4)
        runner = TimedAgentRunner(agent_factory=agent_factory, allowed_methods=allowed_methods,
                                   max_mem=max_mem, verbose=verbose)
        runner.call('get_init_states', time_out_sec=task_config['agent']['init_timeout'],
                    fallback=None, init_states=init_states)

        if env.optimize:
            item_list = env.get_info_for_optimization()
            optimized_order, opt_elapsed = runner.call(
                'optimize', time_out_sec=task_config['agent']['optimization_timeout'],
                fallback=list(env.stream_manager.all_indices), item_list=item_list)
            optimize_time = max(optimize_time, opt_elapsed)
            if not env.set_item_order(optimized_order):
                return {
                    'status': 'Invalid index or data type for optimization',
                    'metrics': None, 'optimize_time': optimize_time, 'policy_time': policy_time,
                }

        env.reset_item_stream()
        obs, info = env.reset(seed=42)

        terminated = False
        truncated = False
        while not terminated and not truncated:
            action, policy_elapsed = runner.call('policy', time_out_sec=task_config['agent']['policy_timeout'],
                                                  fallback=env.action_space.sample(), observation=obs)
            policy_time = max(policy_time, policy_elapsed)
            obs, reward, terminated, truncated, info = env.step(action)
            place_states = {k: v for k, v in info['status'].items()}

        if env.stream_manager.is_empty():
            status = 'The packing has been completed successfully.'
        else:
            failed = [k for k, v in place_states.items() if v is False]
            status = f'Stopped in the middle. Did not satisfy {failed}' if failed else 'Stopped in the middle.'

        scorer = Scorer(client=env.client, config=task_config)
        metrics = scorer.evaluate(env.container_manager.containers, env.num_total_items)

        return {'status': status, 'metrics': metrics, 'optimize_time': optimize_time, 'policy_time': policy_time}

    except Exception:
        return {
            'status': f'error: {traceback.format_exc().splitlines()[-1]}',
            'metrics': None, 'optimize_time': optimize_time, 'policy_time': policy_time,
        }
    finally:
        try:
            env.close()
        except Exception:
            pass
        if runner is not None:
            try:
                runner.close()
            except Exception:
                pass


def print_table(rows: list[dict]):
    headers = ['scene', 'fill', 'cog', 'stability', 'placement', 'soft', 'placed%', 'opt[s]', 'policy[s]', 'status']
    widths = [26, 7, 7, 9, 9, 7, 8, 7, 9, 42]

    def fmt(values):
        return ' | '.join(str(v).ljust(w)[:w] for v, w in zip(values, widths))

    print(fmt(headers))
    print('-' * (sum(widths) + 3 * (len(widths) - 1)))
    for row in rows:
        m = row['metrics']
        if m is None:
            values = [row['scene'], '-', '-', '-', '-', '-', '-',
                      f"{row['optimize_time']:.2f}", f"{row['policy_time']:.2f}", row['status']]
        else:
            values = [
                row['scene'],
                f"{m['fill_score']:.2f}", f"{m['cog_score']:.2f}", f"{m['stability_score']:.2f}",
                f"{m['placement_score']:.2f}", f"{m['soft_item_score']:.2f}",
                f"{m['num_placed_items'] * 100:.1f}",
                f"{row['optimize_time']:.2f}", f"{row['policy_time']:.2f}", row['status'],
            ]
        print(fmt(values))


def main():
    args = parse_args()
    module_path = args.module_path
    agent_module = '.'.join(module_path.split('/')) + 'agent'  # scripts/run_test.pyと同じ組み立て方

    all_rows = []
    for config_path in args.config_path:
        with open(config_path) as f:
            config = json.load(f)

        for task_id, task_config in config.items():
            scene_label = f'{os.path.basename(config_path)}::{task_id}'
            print(f'\n--- running {scene_label} ---')
            t0 = time.time()
            result = run_one_scene(task_config, module_path, agent_module, args.render_mode, args.verbose)
            print(f'  done in {time.time() - t0:.1f}s (status: {result["status"]})')
            result['scene'] = scene_label
            all_rows.append(result)

    print('\n=== シーン別結果 ===')
    print_table(all_rows)

    scored_rows = [r for r in all_rows if r['metrics'] is not None]
    if scored_rows:
        print('\n=== 平均 ===')
        for key in METRIC_KEYS:
            avg = sum(r['metrics'][key] for r in scored_rows) / len(scored_rows)
            print(f'{key:16s}: {avg:.2f}')
        avg_placed = sum(r['metrics']['num_placed_items'] for r in scored_rows) / len(scored_rows)
        print(f'{"num_placed_items":16s}: {avg_placed * 100:.1f}%')
        print(f'(scored {len(scored_rows)}/{len(all_rows)} scenes)')
    else:
        print('\nすべてのシーンでエラーが発生したため平均は計算できません。')


if __name__ == '__main__':
    main()
