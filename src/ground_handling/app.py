import time
import traceback
import json
import os

from .runner import TimedAgentRunner
from .agent_factory import AgentFactory
from .env import GroundHandlingEnv


class EvaluationApp:
    def __init__(self, config_path: str, module_path: str, agent_module: str, agent_class: str, result_dir: str, result_fname: str):
        with open(config_path) as f:
            self.config = json.load(f)
        self.module_path = module_path
        self.agent_module = agent_module
        self.agent_class = agent_class
        self.result_dir = result_dir

        self.result_path = os.path.join(result_dir, result_fname)

    def run(self, render_mode: str | None = None, verbose: bool = True):
        results = {}
        if verbose:
            time_overall = time.time()

        agent_factory = AgentFactory(
            module_name = self.agent_module,
            class_name = self.agent_class,
            module_path = self.module_path
        )
        env = None
        runner = None
        for task_id, task_config in self.config.items():
            if verbose:
                print(f'\n--- {task_id} ---\n')
            try:
                message = 'ok'
                status = 'success'
                terminated = False
                truncated = False
                optimize_time = 0
                policy_time = 0


                # 環境の初期化
                results[task_id] = {}
                env = GroundHandlingEnv(config = task_config, verbose = verbose, render_mode = render_mode)
                env.reset_settings()
                init_states = env.get_init_states()

                # agentの初期化
                allowed_methods = task_config['agent']['allowed_methods']
                max_mem = task_config['agent'].get('max_mem', 4)
                runner = TimedAgentRunner(agent_factory = agent_factory, allowed_methods = allowed_methods, max_mem=max_mem, verbose = verbose)
                runner.call('get_init_states', time_out_sec = task_config['agent']['init_timeout'], fallback = None, init_states = init_states)

                if env.optimize:
                    print('Perform Optimization!')
                    item_list = env.get_info_for_optimization()
                    optimized_order, opt_elapsed_time = runner.call('optimize',
                                                                    time_out_sec = task_config['agent']['optimization_timeout'],
                                                                    fallback = list(env.stream_manager.all_indices),
                                                                    item_list = item_list)
                    optimize_time = max(optimize_time, opt_elapsed_time) # 最大時間
                    is_optimized = env.set_item_order(optimized_order)
                    if not is_optimized:
                        status = 'format_error'
                        results[task_id].update({
                            'evaluation': None,
                            'message': 'Invalid index or data type for optimization',
                            'status': status
                            }
                        )
                        break
                env.reset_item_stream()

                obs, info = env.reset(seed=42)
                place_states = {}
                while not terminated and not truncated:
                    action, policy_elapsed_time = runner.call('policy',
                                                              time_out_sec = task_config['agent']['policy_timeout'],
                                                              fallback = env.action_space.sample(),
                                                              observation = obs)
                    policy_time = max(policy_time, policy_elapsed_time) # 最大時間
                    obs, reward, terminated, truncated, info = env.step(action)
                    place_states = {k: v for k, v in info['status'].items()}


                evaluation_report = env.evaluate()
                results[task_id].update({
                    'evaluation': evaluation_report,
                    'message': message,
                    'status': status
                })
                results[task_id].update({'place_states': place_states})
                time_results = {
                    'optimization': optimize_time,
                    'policy': policy_time
                }
                results[task_id].update({'time_results': time_results})

                if verbose:
                    print('\n  results')
                    for k, v in results[task_id].items():
                        print('   ', k, v)
            except Exception as e:
                status = 'format_error'
                message = traceback.format_exc()
                print(message)
                results[task_id].update({
                    'evaluation': None,
                    'message': message,
                    'status': status
                })
                break

            finally:
                if env is not None:
                    try:
                        env.close()
                    except Exception:
                        pass
                if runner is not None:
                    try:
                        runner.close()
                    except Exception:
                        pass

        if verbose:
            print(time.time()-time_overall)
        os.makedirs(self.result_dir, exist_ok=True)
        with open(self.result_path, 'w') as f:
            json.dump(results, f, indent=4)
