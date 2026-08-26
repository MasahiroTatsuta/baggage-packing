"""Phase69: stability_scoreの減点構造を荷物単位で測る(事実収集のみ、読み取り専用)。

`tools/diagnose_stability.py::stability_with_item_detail`(Phase52)のロジックを
**一字一句そのまま複製**し、揺らし前(initial_pos捕捉時点)の追加情報を集める:

- 荷物の位置(world x/y/z)・所属コンテナのwidth/length/height・オフセット
- 扉(搬入口)からの距離: `y_entry = -width/2`(planner.py::y_entryと同じ定義)からの
  ローカルyのずれ。`container.global_to_local()`はxのみoffset_xを引く実装なので
  ローカルy/zはworld座標と同一。
- コンテナ内の置かれた順序(`container.packed_items`のインデックス、add_item()の
  呼び出し順=実際の配置順)
- 支持の質: 揺らし開始前(create_cap前)の`getContactPoints`から、荷物底面付近
  (contact点のz座標が荷物中心z − height/2 に近い)の接触を抽出し、
  (a) 支えている他アイテムの個体数、(b) 床/棚など構造物への接触の有無、
  (c) 接地点(x,y)の凸包面積 ÷ 荷物footprint面積(length×width)を「接地面積比」の
  近似値として計算する。

`src/`・`tools/scorer.py`・`tools/diagnose_stability.py`・`configs/`は一切変更しない
(本ファイルは新規追加のみ)。stability_scoreの値自体はPhase52のロジックと同一のはず
(検証: 本スクリプトの`stability_score`とdiagnose_stability.pyの出力が一致することを
別途確認)。

実行方法:
    MYSOLVER_DIAG_SHAKE_AMPLITUDE=19.6 PYTHONPATH=. .venv/bin/python \\
        tools/phase69_stability_breakdown.py --config-path 'configs/gen/suite_*.json' \\
        --out results/phase69_breakdown.json
"""
import argparse
import glob
import json
import math
import os
import sys
import time
import traceback

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from src.ground_handling.agent_factory import AgentFactory
from src.ground_handling.env import GroundHandlingEnv
from src.ground_handling.runner import TimedAgentRunner
from tools.scorer import Scorer


def parse_args():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('--config-path', default='configs/gen/suite_*.json')
    p.add_argument('--module-path', default='agents/mysolver/')
    p.add_argument('--out', default='/tmp/phase69_breakdown.json')
    return p.parse_args()


def _convex_hull_area(points):
    """2D凸包の面積(monotone chain)。点が2個以下、または全て同一直線上なら0。"""
    pts = sorted(set(points))
    if len(pts) < 3:
        return 0.0

    def cross(o, a, b):
        return (a[0] - o[0]) * (b[1] - o[1]) - (a[1] - o[1]) * (b[0] - o[0])

    lower = []
    for p_ in pts:
        while len(lower) >= 2 and cross(lower[-2], lower[-1], p_) <= 0:
            lower.pop()
        lower.append(p_)
    upper = []
    for p_ in reversed(pts):
        while len(upper) >= 2 and cross(upper[-2], upper[-1], p_) <= 0:
            upper.pop()
        upper.append(p_)
    hull = lower[:-1] + upper[:-1]
    area = 0.0
    n = len(hull)
    for i in range(n):
        x1, y1 = hull[i]
        x2, y2 = hull[(i + 1) % n]
        area += x1 * y2 - x2 * y1
    return abs(area) / 2.0


def _support_info(client, item, other_body_to_index, container_struct_ids):
    """揺らし開始前(create_cap前)のgetContactPointsから底面支持の情報を抽出する。"""
    if item.pybullet_id is None:
        return {'n_support_items': 0, 'on_floor_or_shelf': False, 'contact_footprint_ratio': 0.0}
    pos, _ = item.get_pose(client)
    if pos is None:
        return {'n_support_items': 0, 'on_floor_or_shelf': False, 'contact_footprint_ratio': 0.0}
    bottom_z = pos[2] - item.height / 2.0
    eps = 0.03  # 3cm以内の接触点を「底面付近」とみなす(荷物寸法に対して十分小さい許容)

    supporting_item_indices = set()
    on_struct = False
    xy_points = []
    contacts = client.getContactPoints(bodyA=item.pybullet_id)
    for c in contacts:
        # getContactPoints tuple: [.., bodyA, bodyB, .., positionOnA, positionOnB, ...]
        body_a, body_b = c[1], c[2]
        other_body = body_b if body_a == item.pybullet_id else body_a
        pos_on_a = c[5]
        pos_on_b = c[6]
        contact_pos = pos_on_a if body_a == item.pybullet_id else pos_on_b
        if contact_pos[2] > bottom_z + eps:
            continue  # 側面・上面接触は支持とみなさない
        xy_points.append((contact_pos[0], contact_pos[1]))
        if other_body in container_struct_ids:
            on_struct = True
        elif other_body in other_body_to_index:
            supporting_item_indices.add(other_body_to_index[other_body])

    footprint_area = max(item.length * item.width, 1e-9)
    hull_area = _convex_hull_area(xy_points)
    return {
        'n_support_items': len(supporting_item_indices),
        'on_floor_or_shelf': on_struct,
        'contact_footprint_ratio': min(hull_area / footprint_area, 1.0),
        'n_contact_points_bottom': len(xy_points),
    }


def stability_with_item_detail_ext(scorer: Scorer, containers, shake_steps: int = 150,
                                    settle_steps: int = 180) -> dict:
    """diagnose_stability.py::stability_with_item_detail のロジックを複製し、
    位置・配置順・支持情報を追加で返す。"""
    client = scorer.client
    all_items = [item for c in containers for item in c.packed_items if item.pybullet_id is not None]
    if not all_items:
        return {'stability_score': 100.0, 'items': []}

    # 荷物pybullet_id -> index のマップ(支持元の識別用)、コンテナ構造物のbullet_idの集合
    other_body_to_index = {it.pybullet_id: it.index for it in all_items}
    container_struct_ids = set()
    for c in containers:
        for attr in ('container_bullet_id', 'top_bullet_id', 'small_shelf_bullet_id', 'shelf_bullet_id'):
            bid = getattr(c, attr, None)
            if bid is not None:
                container_struct_ids.add(bid)

    initial_pos = {}
    item_meta = {}
    for c in containers:
        n_in_container = len(c.packed_items)
        for order_idx, item in enumerate(c.packed_items):
            pos, _ = item.get_pose(client)
            if pos is None:
                continue
            initial_pos[item.index] = np.array(pos)
            support = _support_info(client, item, other_body_to_index, container_struct_ids)
            item_meta[item.index] = {
                'world_x': float(pos[0]), 'world_y': float(pos[1]), 'world_z': float(pos[2]),
                'container_index': c.index,
                'container_width': c.width, 'container_length': c.length, 'container_height': c.height,
                'container_is_prioritized': c.is_prioritized,
                'dist_to_door': float(pos[1] - (-c.width / 2.0)),  # y_entry = -width/2 (planner.pyと同定義)
                'order_in_container': order_idx,
                'order_frac_in_container': order_idx / max(n_in_container - 1, 1),
                'n_items_in_container': n_in_container,
                **support,
            }

    for container in containers:
        container.create_cap(client)

    amplitude = float(os.environ.get('MYSOLVER_DIAG_SHAKE_AMPLITUDE', '6.0'))
    for step in range(shake_steps):
        angle = 2 * math.pi * step / 30.0
        gx = amplitude * math.sin(angle)
        gy = amplitude * math.cos(angle * 0.7)
        client.setGravity(gx, gy, -9.8)
        client.stepSimulation()

    client.setGravity(0, 0, -9.8)
    for _ in range(settle_steps):
        client.stepSimulation()

    disps = []
    energies = []
    item_rows = []
    for item in all_items:
        pos, _ = item.get_pose(client)
        if pos is None or item.index not in initial_pos:
            continue
        disp = float(np.linalg.norm(np.array(pos) - initial_pos[item.index]))
        lin_v, ang_v = client.getBaseVelocity(item.pybullet_id)
        ke = 0.5 * item.mass * sum(v * v for v in lin_v) + 0.5 * sum(v * v for v in ang_v)
        disps.append(disp)
        energies.append(ke)
        row = {
            'index': item.index,
            'disp': disp,
            'ke': float(ke),
            'mass': item.mass,
            'is_soft': item.is_soft,
            'is_prioritized': item.is_prioritized,
            'length': item.length, 'width': item.width, 'height': item.height,
        }
        row.update(item_meta.get(item.index, {}))
        item_rows.append(row)

    mean_disp = float(np.mean(disps)) if disps else 0.0
    mean_energy = float(np.mean(energies)) if energies else 0.0
    disp_score = max(0.0, 100.0 * (1.0 - mean_disp / 0.3))
    energy_score = max(0.0, 100.0 * (1.0 - min(mean_energy, 1.0) / 1.0))
    score = min(max(0.7 * disp_score + 0.3 * energy_score, 0.0), 100.0)

    item_rows.sort(key=lambda r: -r['disp'])
    return {
        'stability_score': score,
        'mean_disp': mean_disp,
        'mean_energy': mean_energy,
        'disp_score': disp_score,
        'energy_score': energy_score,
        'n_items': len(item_rows),
        'items': item_rows,
    }


def run_one_scene(task_config: dict, module_path: str, agent_module: str) -> dict:
    agent_factory = AgentFactory(module_name=agent_module, class_name='Agent', module_path=module_path)
    env = GroundHandlingEnv(config=task_config, verbose=False, render_mode=None)
    runner = None
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
        while not terminated and not truncated:
            action, _ = runner.call('policy', time_out_sec=task_config['agent']['policy_timeout'],
                                     fallback=env.action_space.sample(), observation=obs)
            obs, reward, terminated, truncated, info = env.step(action)

        containers = env.container_manager.containers
        scorer = Scorer(client=env.client, config=task_config)

        fill_score, _ = scorer.calculate_fill_score(containers)
        placement_score = scorer.calculate_placement_score(containers)
        soft_item_score = scorer.calculate_soft_item_score(containers)
        cog_score = scorer.calculate_cog_score(containers)
        detail = stability_with_item_detail_ext(scorer, containers)

        return {
            'status': 'ok',
            'fill_score': fill_score,
            'placement_score': placement_score,
            'soft_item_score': soft_item_score,
            'cog_score': cog_score,
            **detail,
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
        tasks = json.load(open(cp))
        for tk, task in tasks.items():
            label = f'{os.path.basename(cp)}::{tk}' if len(tasks) > 1 else os.path.basename(cp)
            t0 = time.perf_counter()
            r = run_one_scene(task, args.module_path, agent_module)
            r['elapsed_sec'] = time.perf_counter() - t0
            results[label] = r
            print(f"[{label}] stability={r.get('stability_score')} n_items={r.get('n_items', 0)} "
                  f"cog={r.get('cog_score')} status={r.get('status')}", flush=True)

    with open(args.out, 'w') as f:
        json.dump(results, f, indent=2)
    print(f"wrote {args.out}")


if __name__ == '__main__':
    main()
