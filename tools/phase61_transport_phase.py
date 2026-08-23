"""Phase61 §3-2 / Phase62 §1: sudden death(cause=is_valid)を起こした手番について、
`validator.check_transport_path` の判定が Y区間(phase1)/X区間(phase2) のどちらで
失敗したかを集計し(Phase61)、さらに Phase62 では、その死亡アクションが
`planner.plan()` の legal1/legal2 を実際に通過した候補だったのか、それとも
`planner.plan()` がNone(候補なし)を返し legal1/legal2 を一切通らない
`agent.py::_fallback_place_pos` が使われたのかを切り分ける(死亡アクションが
subprocess内で決定されるため、同一入力でplanner.plan()を直接呼び直して検証する)。

`src/ground_handling`・`agents/mysolver`のいずれのコードも変更しない(読み取り専用)。

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

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from src.ground_handling.agent_factory import AgentFactory
from src.ground_handling.env import GroundHandlingEnv
from src.ground_handling.runner import TimedAgentRunner
from src.ground_handling.validator import PlacementValidator

from agents.mysolver import agent as agent_mod
from agents.mysolver import planner as planner_mod
from agents.mysolver import geometry as geo


def parse_args():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('--config-path', default='configs/gen/suite_*.json')
    p.add_argument('--module-path', default='agents/mysolver/')
    p.add_argument('--out', default='/tmp/phase61_transport_phase.json')
    return p.parse_args()


def _install_move_item_probe():
    """PlacementValidator._move_item をラップし、呼び出しごとの is_packable と、
    衝突相手・位置・最短距離(getClosestPointsの生データ)を記録する。
    元のメソッドは呼ぶだけで変更しない。"""
    original = PlacementValidator._move_item

    def wrapped(self, container, item, item_id, start_pos, target_pos, target_orn, steps):
        is_packable = True
        collision = None
        current_pos = start_pos
        for i in range(steps + 1):
            fraction = i / steps
            current_pos = (
                start_pos[0] + (target_pos[0] - start_pos[0]) * fraction,
                start_pos[1] + (target_pos[1] - start_pos[1]) * fraction,
                start_pos[2] + (target_pos[2] - start_pos[2]) * fraction,
            )
            item.set_pose(self.client, current_pos, target_orn)
            self.client.performCollisionDetection()

            hit = None
            for other_item in container.packed_items:
                if other_item.pybullet_id is None:
                    continue
                pts = self.client.getClosestPoints(bodyA=item_id, bodyB=other_item.pybullet_id,
                                                    distance=self.safety_margin)
                for pt in pts:
                    if pt[2] == other_item.pybullet_id:
                        hit = {'partner': 'item', 'partner_index': other_item.index,
                               'distance': float(pt[8]), 'pos': list(current_pos), 'fraction': fraction}
                        break
                if hit:
                    break

            if hit is None and container.require_shelf:
                pts = self.client.getClosestPoints(bodyA=item_id, bodyB=container.shelf_bullet_id,
                                                    distance=self.safety_margin)
                for pt in pts:
                    if pt[2] == container.shelf_bullet_id:
                        hit = {'partner': 'shelf', 'partner_index': None,
                               'distance': float(pt[8]), 'pos': list(current_pos), 'fraction': fraction}
                        break

            if hit is None:
                pts = self.client.getClosestPoints(bodyA=item_id, bodyB=container.small_shelf_bullet_id,
                                                    distance=self.safety_margin)
                for pt in pts:
                    if pt[2] == container.small_shelf_bullet_id:
                        hit = {'partner': 'small_shelf', 'partner_index': None,
                               'distance': float(pt[8]), 'pos': list(current_pos), 'fraction': fraction}
                        break

            if hit is not None:
                is_packable = False
                collision = hit
                break

        if not hasattr(self, '_phase61_calls'):
            self._phase61_calls = []
        self._phase61_calls.append({'is_packable': is_packable, 'collision': collision,
                                     'start_pos': list(start_pos), 'target_pos': list(target_pos)})
        return is_packable, current_pos

    PlacementValidator._move_item = wrapped
    return original


def _classify(calls: list) -> str:
    if not calls:
        return 'no_move_item_call'
    if len(calls) == 1:
        return 'Y区間(phase1)' if not calls[0]['is_packable'] else 'unknown(phase1成功なのに1回のみ)'
    if len(calls) == 2:
        return 'X区間(phase2)' if not calls[1]['is_packable'] else 'unknown(2回とも成功)'
    return f'unknown(呼び出し{len(calls)}回)'


# --------------------------------------------------------------------------
# Phase62: legal1(Y区間)のAABBモデルを、死亡アクションに対して直接再現する。
# _evaluate_candidates 内のインライン計算(sweep_z/x1/y1範囲)を、n=1の1候補分だけ
# 忠実に複製する(既存の公開関数 _apply_obstacle_filters・_collect_obstacles・
# geo.transport_x_bounds はそのまま呼ぶ。複製するのはモジュール化されていない
# インライン部分のみ、行番号・定数とも planner.py と完全一致させる)。
# --------------------------------------------------------------------------
def _replicate_legal1(container_dict: dict, item: dict, orn_idx: int, world_pos: np.ndarray):
    thickness = container_dict['thickness']; height = container_dict['height']
    buffer = container_dict.get('buffer', 0.0)
    ox = container_dict['center'][0]
    half = geo.half_extent([item['length'], item['width'], item['height']], orn_idx)

    world_x, world_y, world_z = world_pos
    local_x = world_x - ox
    local_y = world_y  # width方向はcenterの影響を受けない(planner.pyと同じ前提)

    bottom_z = world_z - half[2]
    resting_values = [thickness, height / 2.0 + thickness + buffer]
    is_resting = any(0.0 <= (bottom_z - rv) <= 0.05 for rv in resting_values)

    top_z = world_z + half[2]
    effective_start = 0.0 if is_resting else geo.START_Z
    handled = is_resting
    for c_z in (height / 2.0 + buffer, height + buffer - thickness):
        if handled:
            break
        clearance = c_z - top_z
        if 0.0 <= clearance < (effective_start + geo.CEILING_MARGIN):
            effective_start = max(0.0, clearance - geo.CEILING_MARGIN - 0.0005)
            handled = True

    ceiling_sweep = height + buffer - thickness - half[2] - geo.START_MARGIN
    sweep_z = min(ceiling_sweep, world_z + effective_start)

    x_min_local, x_max_local = geo.transport_x_bounds(container_dict, half[0])
    x_min_local -= ox; x_max_local -= ox
    start_x_local = min(max(local_x, x_min_local), x_max_local)
    start_x_world = start_x_local + ox

    y_entry = -container_dict['width'] / 2.0
    y1_lo = min(y_entry, local_y); y1_hi = max(y_entry, local_y)
    x1_lo = start_x_world; x1_hi = start_x_world

    obstacles = planner_mod._collect_obstacles(container_dict)

    world_pos_arr = np.array([world_pos], dtype=np.float64)
    legal1 = planner_mod._apply_obstacle_filters(
        world_pos_arr, half, obstacles,
        np.array([x1_lo]), np.array([x1_hi]), np.array([y1_lo]), np.array([y1_hi]), np.array([sweep_z]),
    )[0]

    # 各障害物ごとの生(margin適用前)のAABB分離量(軸ごと)を求める。
    # sweep box: x=[x1_lo-half0, x1_hi+half0], y=[y1_lo-half1, y1_hi+half1], z=sweep_z±half2
    sx_lo, sx_hi = x1_lo - half[0], x1_hi + half[0]
    sy_lo, sy_hi = y1_lo - half[1], y1_hi + half[1]
    sz_lo, sz_hi = sweep_z - half[2], sweep_z + half[2]
    per_obstacle = []
    for center, oh in obstacles:
        ox_lo, ox_hi = center[0] - oh[0], center[0] + oh[0]
        oy_lo, oy_hi = center[1] - oh[1], center[1] + oh[1]
        oz_lo, oz_hi = center[2] - oh[2], center[2] + oh[2]
        gap_x = max(ox_lo - sx_hi, sx_lo - ox_hi)
        gap_y = max(oy_lo - sy_hi, sy_lo - oy_hi)
        gap_z = max(oz_lo - sz_hi, sz_lo - oz_hi)
        per_obstacle.append({'center': [float(c) for c in center], 'half': [float(h) for h in oh],
                              'gap_x': float(gap_x), 'gap_y': float(gap_y), 'gap_z': float(gap_z)})

    return {
        'legal1': bool(legal1), 'half': [float(h) for h in half],
        'sweep_box': {'x': [float(sx_lo), float(sx_hi)], 'y': [float(sy_lo), float(sy_hi)],
                      'z': [float(sz_lo), float(sz_hi)]},
        'sweep_z': float(sweep_z), 'is_resting': bool(is_resting),
        'effective_start': float(effective_start),
        'per_obstacle': per_obstacle,
        'n_obstacles': len(obstacles),
    }


def _find_item_in_pool(pool_list, item_idx):
    return pool_list[item_idx] if 0 <= item_idx < len(pool_list) else None


def _find_container(container_list, container_idx):
    for c in container_list:
        if c.get('index') == container_idx:
            return c
    return None


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

        optimize_flag = init_states.get('optimize', True)
        prepacked_ids = geo.initial_prepacked_ids(init_states.get('container_list') or [])

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
            obs_before = obs
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
                replay = None
                if cause == 'is_valid':
                    calls = getattr(env.validator, '_phase61_calls', [])
                    phase = _classify(calls)

                    # Phase62: 同一observationでplanner.plan()を直接呼び直し、
                    # 実際の死亡アクションが legal1/legal2 を通った候補か、
                    # それとも None(→ agent.py の _fallback_place_pos)だったかを判定する。
                    try:
                        plan_info = {}
                        replan_action = planner_mod.plan(
                            obs_before['container_list'], obs_before['pool_list'],
                            time_budget=agent_mod.POLICY_TIME_BUDGET,
                            hard_deadline=time.perf_counter() + agent_mod.POLICY_HARD_WALL,
                            strict_support=not optimize_flag, prepacked_ids=prepacked_ids,
                            info=plan_info,
                        )
                    except Exception:
                        replan_action = 'error: ' + traceback.format_exc().splitlines()[-1]
                        plan_info = {}

                    is_fallback_signature = (action['item_idx'] == 0 and action['orientation'] == 0)
                    matches_replan = False
                    if isinstance(replan_action, dict):
                        matches_replan = (
                            replan_action.get('item_idx') == action['item_idx']
                            and replan_action.get('container_idx') == action['container_idx']
                            and replan_action.get('orientation') == action['orientation']
                            and np.allclose(replan_action.get('place_pos'), action['place_pos'], atol=1e-6)
                        )

                    replay = {
                        'planner_plan_returned_none': replan_action is None,
                        'replan_matches_actual_action': matches_replan,
                        'action_has_fallback_signature(item0_orn0)': is_fallback_signature,
                        'plan_info_slack': plan_info.get('slack'),
                    }

                    # legal1のAABBモデルを、死亡アクションに対して直接再現する
                    # (fallback経路であってもlegal1が通るかどうか自体は参考情報として計算する)。
                    try:
                        target_item = _find_item_in_pool(obs_before['pool_list'], action['item_idx'])
                        target_container = _find_container(obs_before['container_list'], action['container_idx'])
                        if target_item is not None and target_container is not None:
                            global_pos = np.array(
                                geo.local_to_world(target_container, action['place_pos']), dtype=np.float64)
                            legal1_model = _replicate_legal1(target_container, target_item,
                                                              action['orientation'], global_pos)
                            replay['legal1_model'] = legal1_model
                    except Exception:
                        replay['legal1_model_error'] = traceback.format_exc().splitlines()[-1]

                death = {
                    'cause': cause, 'n_placed_at_death': n_step - 1,
                    'transport_phase': phase,
                    'move_item_calls': getattr(env.validator, '_phase61_calls', []),
                    'action': {'item_idx': action['item_idx'], 'container_idx': action['container_idx'],
                               'orientation': action['orientation'],
                               'place_pos': list(np.asarray(action['place_pos']).tolist())},
                    'replay': replay,
                }

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
                replay = death.get('replay') or {}
                print(f"[{label}] SUDDEN DEATH cause={death['cause']} phase={death['transport_phase']} "
                      f"plan_none={replay.get('planner_plan_returned_none')} "
                      f"fallback_sig={replay.get('action_has_fallback_signature(item0_orn0)')} "
                      f"matches_replan={replay.get('replan_matches_actual_action')} "
                      f"({r['elapsed_sec']:.1f}s)")
            else:
                tally['no_death'] = tally.get('no_death', 0) + 1
                print(f"[{label}] completed={r.get('completed_without_sudden_death')} ({r['elapsed_sec']:.1f}s)")

    print('=== tally ===')
    for k, v in sorted(tally.items(), key=lambda kv: -kv[1]):
        print(f'  {k}: {v}')

    with open(args.out, 'w') as f:
        json.dump({'results': results, 'tally': tally}, f, indent=2, default=str)
    print(f"wrote {args.out}")


if __name__ == '__main__':
    main()
