"""Phase64: planner.plan()がNoneを返す局面(Phase61-63で確認した27件の死亡直前)に、
本当に合法解が1つも存在しないかを総当たりで確かめる(読み取り専用、仮説なし)。

判定は既存の生産コードをそのまま呼ぶだけで、新しい判定式は一切書かない:
  - 内包判定: `geo.check_inclusion_batch`(既定margin=geo.INCLUSION_MARGIN、
    validator.check_inclusionと同式)
  - 搬入経路(legal1×legal2): `agents.mysolver.agent._fallback_transport_legal`
    (Phase63で追加済み。`planner._collect_obstacles`・`planner._apply_obstacle_filters`を
    そのまま呼ぶラッパー)
  - 向きの列挙: `planner._unique_orientations`

探索範囲:
  - プール内の全アイテム(item_idxを0に固定しない)
  - 全ユニーク向き
  - 全コンテナ
  - xyグリッド: 5mm刻み(planner既定のBASE_GRID_DENSITY=2相当の約30mm、
    RETRY_GRID_DENSITY=4相当の約15mmより十分に細かい)
  - z: 床、および各既配置荷物の上面(+REST_CLEARANCE)。ただしその高さを使うxyは、
    対応する既配置荷物(または床の場合は全域)とXY方向で footprint が重なる点に限定する
    (「支持可能な高さ」の列挙。plannerの合議支持比率MIN_UNION_SUPPORT_RATIOのような
    品質要求は課さない——それ自体が原因かどうかを切り分けるのが本ツールの目的)。

副次的に、`planner.plan()`に極端に大きい`SearchBudget`を渡して再実行し、
「予算さえあれば見つかる」かどうかも合わせて記録する(新しい判定式ではなく、
既存の`planner.plan()`をそのまま予算だけ変えて呼ぶだけ)。

実行方法(リポジトリルートで):
    PYTHONPATH=. .venv/bin/python tools/phase64_exhaustive.py \\
        --config-path 'configs/gen/suite_*.json' --out results/phase64_exhaustive_26.json
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

from agents.mysolver import agent as agent_mod
from agents.mysolver import planner as planner_mod
from agents.mysolver import geometry as geo

XY_STEP = 0.005  # 5mm


def parse_args():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('--config-path', default='configs/gen/suite_*.json')
    p.add_argument('--module-path', default='agents/mysolver/')
    p.add_argument('--out', default='/tmp/phase64_exhaustive.json')
    return p.parse_args()


def _reach_death(task_config: dict, module_path: str, agent_module: str):
    """Phase61-63と同じループでエピソードを進め、is_valid sudden death直前の
    observationとdeath情報を返す(見つからなければNone)。"""
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
                return None
        env.reset_item_stream()
        obs, info = env.reset(seed=42)
        terminated = truncated = False
        n_step = 0
        while not terminated and not truncated:
            obs_before = obs
            action, _ = runner.call('policy', time_out_sec=task_config['agent']['policy_timeout'],
                                     fallback=env.action_space.sample(), observation=obs)
            obs, reward, terminated, truncated, info = env.step(action)
            n_step += 1
            status = info.get('status', {})
            if terminated and not env.stream_manager.is_empty():
                if not status.get('is_valid', True) and status.get('is_included', True):
                    return {'obs_before': obs_before, 'optimize_flag': optimize_flag,
                            'prepacked_ids': prepacked_ids, 'n_placed_at_death': n_step - 1}
                return None  # is_included/is_placed_safe起因、または完走。本ツールの対象外
        return None
    finally:
        try:
            env.close()
        except Exception:
            pass


def _replan_with_big_budget(container_list, pool_list, optimize_flag, prepacked_ids):
    """planner.plan()の判定式は一切変えず、SearchBudgetだけ極端に大きくして再実行する。"""
    big_budget = planner_mod.SearchBudget(limit=1e15, hard_deadline=None)
    try:
        action = planner_mod.plan(container_list, pool_list, time_budget=1e9,
                                   strict_support=not optimize_flag, prepacked_ids=prepacked_ids,
                                   budget=big_budget)
    except Exception:
        return {'error': traceback.format_exc().splitlines()[-1]}
    return {
        'found': action is not None, 'action': action,
        'budget_used': big_budget.used, 'budget_limit': big_budget.limit,
        'budget_exhausted': big_budget.exhausted(),
    }


def _footprint_overlap_mask(cand_x, cand_y, half_x, half_y, obs_center, obs_half):
    """候補(footprint中心cand_x/cand_y、半寸half_x/half_y)とobstacle(footprint)がXYで
    重なるかを判定する(境界のみ接する場合も「重なる」とみなすtoleranceを持たせる)。"""
    ox_lo, ox_hi = obs_center[0] - obs_half[0], obs_center[0] + obs_half[0]
    oy_lo, oy_hi = obs_center[1] - obs_half[1], obs_center[1] + obs_half[1]
    x_lo, x_hi = cand_x - half_x, cand_x + half_x
    y_lo, y_hi = cand_y - half_y, cand_y + half_y
    tol = 0.003  # 3mm: ちょうど角で触れる支持も拾う
    return (x_hi > ox_lo - tol) & (x_lo < ox_hi + tol) & (y_hi > oy_lo - tol) & (y_lo < oy_hi + tol)


def _nearest_planner_grid_dist(container, half, obstacles, xy_point, density):
    """(xy_point)が、指定density(BASE/RETRY)でのplanner自身のcandidate_xy集合の
    どれかにどれだけ近いか(最小ユークリッド距離、m)を返す。extreme pointsも含む
    (`planner._candidate_xy`をそのまま呼ぶ)。"""
    cand = planner_mod._candidate_xy(container, half, obstacles, grid_density=density)
    if cand.shape[0] == 0:
        return float('inf')
    d = np.linalg.norm(cand - np.array(xy_point), axis=1)
    return float(d.min())


def exhaustive_search(container_list, pool_list, prepacked_ids, stop_after: int = 200):
    """全item×全向き×全container×5mmグリッド×支持可能高さ、で
    inclusion & legal1×legal2 を満たす候補を探す。stop_afterに達したら打ち切る
    (「1つでも存在するか」は最初の1件で確定するが、1-3の傾向報告のため複数集める)。
    """
    findings = []
    n_checked_combos = 0
    for container in container_list:
        obstacles = planner_mod._collect_obstacles(container)
        thickness = container['thickness']; height = container['height']
        buffer = container.get('buffer', 0.0)
        length = container['length']; width = container['width']
        ox = container['center'][0]

        x_lo = -length / 2.0 + thickness + geo.START_MARGIN
        x_hi = length / 2.0 - thickness - geo.START_MARGIN
        y_lo = -width / 2.0 + thickness + geo.START_MARGIN
        y_hi = width / 2.0 - thickness - geo.START_MARGIN

        for item in pool_list:
            lwh = [item['length'], item['width'], item['height']]
            for orn_idx in planner_mod._unique_orientations(lwh):
                half = geo.half_extent(lwh, orn_idx)
                xs_lo = x_lo + half[0]; xs_hi = x_hi - half[0]
                ys_lo = y_lo + half[1]; ys_hi = y_hi - half[1]
                if xs_lo > xs_hi or ys_lo > ys_hi:
                    continue
                xs = np.arange(xs_lo, xs_hi + 1e-9, XY_STEP)
                ys = np.arange(ys_lo, ys_hi + 1e-9, XY_STEP)
                if xs.size == 0 or ys.size == 0:
                    continue
                xx, yy = np.meshgrid(xs, ys, indexing='ij')
                cand_x_local = xx.ravel(); cand_y = yy.ravel()
                cand_x_world = cand_x_local + ox
                n = cand_x_world.shape[0]

                # z候補: 床 + 各obstacleの上面(+REST_CLEARANCE)。obstacle系はfootprint重なりで絞る。
                z_sources = [('floor', thickness + half[2] + geo.REST_CLEARANCE, None)]
                for oc, oh in obstacles:
                    top_z = oc[2] + oh[2] + half[2] + geo.REST_CLEARANCE
                    if top_z + half[2] <= height + buffer - thickness:  # 天井を超えない
                        z_sources.append(('item_top', top_z, (oc, oh)))

                for src_kind, z_val, src_obs in z_sources:
                    if src_kind == 'floor':
                        mask = np.ones(n, dtype=bool)
                    else:
                        oc, oh = src_obs
                        mask = _footprint_overlap_mask(cand_x_world, cand_y, half[0], half[1], oc, oh)
                    if not np.any(mask):
                        continue
                    wx = cand_x_world[mask]; wy = cand_y[mask]
                    wz = np.full(wx.shape[0], z_val)
                    world_pos = np.stack([wx, wy, wz], axis=1)
                    n_checked_combos += 1

                    included = geo.check_inclusion_batch(container, half, world_pos)
                    if not np.any(included):
                        continue
                    wp2 = world_pos[included]
                    legal = agent_mod._fallback_transport_legal(container, half, wp2)
                    if np.any(legal):
                        legal_pos = wp2[legal]
                        for lp in legal_pos[:5]:  # シーンあたり爆発しないよう1(item,orn,container,z源)につき最大5件
                            dist_base = _nearest_planner_grid_dist(container, half, obstacles,
                                                                    (lp[0], lp[1]), planner_mod.BASE_GRID_DENSITY)
                            dist_retry = _nearest_planner_grid_dist(container, half, obstacles,
                                                                     (lp[0], lp[1]), planner_mod.RETRY_GRID_DENSITY)
                            findings.append({
                                'container_index': container.get('index'),
                                'item_index': item.get('index'), 'orientation': orn_idx,
                                'pos_world': [float(v) for v in lp],
                                'z_source': src_kind,
                                'dist_to_base_grid_mm': dist_base * 1000,
                                'dist_to_retry_grid_mm': dist_retry * 1000,
                            })
                        if len(findings) >= stop_after:
                            return findings, n_checked_combos
    return findings, n_checked_combos


def run_one_scene(cp: str, tk: str, module_path='agents/mysolver/') -> dict:
    task = json.load(open(cp))[tk]
    agent_module = '.'.join(module_path.split('/')) + 'agent'
    death = _reach_death(task, module_path, agent_module)
    if death is None:
        return {'status': 'no_target_death'}

    obs = death['obs_before']
    container_list = obs['container_list']; pool_list = obs['pool_list']

    replan_big = _replan_with_big_budget(container_list, pool_list, death['optimize_flag'],
                                          death['prepacked_ids'])

    t0 = time.perf_counter()
    findings, n_combos = exhaustive_search(container_list, pool_list, death['prepacked_ids'])
    search_elapsed = time.perf_counter() - t0

    return {
        'status': 'ok',
        'n_placed_at_death': death['n_placed_at_death'],
        'n_pool_remaining': len(pool_list),
        'n_containers': len(container_list),
        'replan_big_budget': replan_big,
        'exhaustive_n_findings': len(findings),
        'exhaustive_n_combos_evaluated': n_combos,
        'exhaustive_search_elapsed_sec': search_elapsed,
        'exhaustive_findings_sample': findings[:20],
    }


def main():
    args = parse_args()
    paths = sorted(glob.glob(args.config_path))
    results = {}
    for cp in paths:
        d = json.load(open(cp))
        for tk in d:
            label = f'{os.path.basename(cp)}::{tk}'
            t0 = time.perf_counter()
            try:
                r = run_one_scene(cp, tk, args.module_path)
            except Exception:
                r = {'status': f'error: {traceback.format_exc().splitlines()[-1]}'}
            r['elapsed_sec'] = time.perf_counter() - t0
            results[label] = r
            if r['status'] == 'ok':
                print(f"[{label}] n_findings={r['exhaustive_n_findings']} "
                      f"combos={r['exhaustive_n_combos_evaluated']} "
                      f"replan_big_found={r['replan_big_budget'].get('found')} "
                      f"replan_big_budget_used={r['replan_big_budget'].get('budget_used'):.0f} "
                      f"search={r['exhaustive_search_elapsed_sec']:.1f}s total={r['elapsed_sec']:.1f}s")
            else:
                print(f"[{label}] {r['status']} ({r['elapsed_sec']:.1f}s)")

    with open(args.out, 'w') as f:
        json.dump(results, f, indent=2, default=str)
    print(f"wrote {args.out}")


if __name__ == '__main__':
    main()
