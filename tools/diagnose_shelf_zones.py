"""
tools/diagnose_shelf_zones.py

Phase15 ターゲット2診断用: gen_shelf_patternA(棚あり単一コンテナ)の停止時点の
状態を再現し、「棚下(below)」「棚上(above)」のどちらの空間が使えていないかを
体積利用率で定量化する。src/, agents/mysolver/ は変更しない(読み取り専用)。

実行例:
    PYTHONPATH=. .venv/bin/python tools/diagnose_shelf_zones.py \
        --config-path configs/gen/gen_shelf_patternA.json --optimize-budget 30
"""
import argparse
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from src.ground_handling.env import GroundHandlingEnv
from agents.mysolver import geometry as geo
from agents.mysolver import ordering
from agents.mysolver import planner as msplanner


def parse_args():
    p = argparse.ArgumentParser()
    p.add_argument('--config-path', required=True)
    p.add_argument('--task-id', default=None)
    p.add_argument('--optimize-budget', type=float, default=30.0)
    p.add_argument('--policy-budget', type=float, default=5.5)
    return p.parse_args()


def run(task_config, optimize_budget, policy_budget):
    env = GroundHandlingEnv(config=task_config, verbose=False, render_mode=None)
    try:
        env.reset_settings()
        init_states = env.get_init_states()
        full_item_list = None
        if env.optimize:
            full_item_list = env.get_info_for_optimization()
            order = ordering.build_order(full_item_list, init_states['container_list'],
                                          init_states['lookahead_k'], time_budget=optimize_budget)
            env.set_item_order(order)
        else:
            full_item_list = env.get_info_for_optimization()
        env.reset_item_stream()
        obs, info = env.reset(seed=42)

        terminated = truncated = False
        while not terminated and not truncated:
            container_list = obs['container_list']
            pool_list = obs['pool_list']
            if not pool_list:
                break
            action = msplanner.plan(container_list, pool_list, time_budget=policy_budget)
            if action is None:
                break
            obs, reward, terminated, truncated, info = env.step(action)

        container_list = obs['container_list']
        for c in container_list:
            length = c['length']; width = c['width']; height = c['height']; thickness = c['thickness']
            buffer = c.get('buffer', 0.0)
            shelf = c.get('shelf', False)
            print(f"\n=== container[{c['index']}] shelf={shelf} L={length:.2f} W={width:.2f} H={height:.2f} ===")
            if not shelf:
                continue
            shelf_center, shelf_half = geo.big_shelf_aabb(c)
            shelf_bottom = shelf_center[2] - shelf_half[2]
            shelf_top = shelf_center[2] + shelf_half[2]
            shelf_y_lo = shelf_center[1] - shelf_half[1]
            shelf_y_hi = shelf_center[1] + shelf_half[1]
            floor_z = thickness
            ceiling_z = height - thickness
            print(f"  shelf: z=[{shelf_bottom:.3f},{shelf_top:.3f}] y=[{shelf_y_lo:.3f},{shelf_y_hi:.3f}] "
                  f"(container z=[{floor_z:.3f},{ceiling_z:.3f}])")

            below_vol = 0.0
            above_vol = 0.0
            other_vol = 0.0
            below_n = above_n = other_n = 0
            below_region_footprint_vol = length * (shelf_y_hi - shelf_y_lo) * (shelf_bottom - floor_z)
            above_region_footprint_vol = length * (shelf_y_hi - shelf_y_lo) * (ceiling_z - shelf_top)
            for it in c.get('packed_items', []):
                if it.get('pos') is None:
                    continue
                vol = it['length'] * it['width'] * it['height']
                z = it['pos'][2]
                y = it['pos'][1]
                in_shelf_y = shelf_y_lo - 0.01 <= y <= shelf_y_hi + 0.01
                if in_shelf_y and z < shelf_bottom - 0.05:
                    below_vol += vol; below_n += 1
                elif in_shelf_y and z > shelf_top + 0.02:
                    above_vol += vol; above_n += 1
                else:
                    other_vol += vol; other_n += 1
            print(f"  below-shelf region footprint volume: {below_region_footprint_vol:.4f} m^3")
            print(f"  above-shelf region footprint volume: {above_region_footprint_vol:.4f} m^3")
            print(f"  items below shelf: {below_n:3d}  volume={below_vol:.4f}  "
                  f"util={100*below_vol/max(below_region_footprint_vol,1e-9):.1f}%")
            print(f"  items above shelf: {above_n:3d}  volume={above_vol:.4f}  "
                  f"util={100*above_vol/max(above_region_footprint_vol,1e-9):.1f}%")
            print(f"  items elsewhere(back strip beyond shelf y): {other_n:3d}  volume={other_vol:.4f}")

        return container_list
    finally:
        env.close()


def main():
    args = parse_args()
    import json
    with open(args.config_path) as f:
        config = json.load(f)
    task_id = args.task_id or next(iter(config.keys()))
    task_config = config[task_id]
    print(f"=== {args.config_path}::{task_id} (optimize_budget={args.optimize_budget}s) ===")
    run(task_config, args.optimize_budget, args.policy_budget)


if __name__ == '__main__':
    main()
