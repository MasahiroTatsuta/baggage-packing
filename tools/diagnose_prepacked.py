"""
tools/diagnose_prepacked.py

Phase11 ターゲット2診断: 「積付済み(prepacked)初期状態」で fill が空状態より 3.6pt 低い
原因を分類する。仮説候補:

  H1. 既積み荷物の上面を候補生成が使えていない
      -> planner._evaluate_candidates は着地面を「単一の支持体に90%以上乗る」場合しか
         採用しない。既積みが小さい箱の集まりだと、どの1個の上にも90%乗らないため
         上面が丸ごと死ぬ。
  H2. 既積み荷物が非軸平行(傾き)で、AABBが過剰に膨張し保守的になる
      -> packed_items の orn(クォータニオン)から軸平行からのズレ角と、AABB膨張率を測る。
  H3. offline最適化が既積み状態を考慮しない順序を作る
      -> simulate.clone_containers は packed_items を引き継ぐので考慮はされている。
         実測(オフライン予測 placed 数 vs 実配置数)で乖離を見る。
  H4. 層規律(Y_SLICE)が既積み荷物と整合しない
      -> 既積みは奥半分(y>=0)を占有する。level0(奥半分のみ開放)で合法手が出ず、
         毎手 level1 まで落ちるだけなら害は小さい。実際に level0 で何手取れたかを数える。

出力:
  - 既積み荷物の姿勢統計(軸平行からの最大ズレ角、AABB体積膨張率)
  - 停止時点の各コンテナの体積利用率・到達高さ・既積み層上の空き体積
  - 「現行の単一支持90%ルール」vs「union(複数支持の合計面積)ルール」で、
    未配置荷物が既積み層の上に着地できる候補XYがどれだけ増えるか(H1の定量)
  - level0(奥層)で決着した手数 / 全手数 (H4の定量)

src/ と agents/mysolver/ は変更しない(読み取り専用)。

実行例:
    PYTHONPATH=. .venv/bin/python tools/diagnose_prepacked.py \
        --config-path configs/gen/suite_P01_A_1c_pre6.json --optimize-budget 30
"""
import argparse
import json
import math
import os
import sys

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from src.ground_handling.env import GroundHandlingEnv
from agents.mysolver import geometry as geo
from agents.mysolver import ordering
from agents.mysolver import planner as msplanner


def parse_args():
    parser = argparse.ArgumentParser()
    parser.add_argument('--config-path', required=True)
    parser.add_argument('--task-id', default=None)
    parser.add_argument('--optimize-budget', type=float, default=30.0)
    parser.add_argument('--policy-budget', type=float, default=5.5)
    return parser.parse_args()


def _tilt_report(container_list):
    """H2: 既積み荷物の姿勢(軸平行からのズレ)と AABB 膨張率。"""
    rows = []
    for c in container_list:
        for it in c.get('packed_items', []):
            if it.get('orn') is None:
                continue
            absR = geo.quat_abs_rotmat(it['orn'])
            # 各列(ローカル軸)が最も近いワールド軸に対して何度ずれているか
            max_cos = np.max(absR, axis=0)          # 各ローカル軸の最良の軸整合度
            tilt_deg = float(np.degrees(np.arccos(np.clip(np.min(max_cos), -1, 1))))
            half_local = np.array([it['length'] / 2, it['width'] / 2, it['height'] / 2])
            half_world = absR @ half_local
            infl = float(np.prod(half_world) / max(np.prod(half_local), 1e-12))
            rows.append((c['index'], it['index'], tilt_deg, infl))
    return rows


def _union_support_gain(container, half, obstacles, supports, grid_density=2):
    """
    H1の定量: 候補XYごとに
      cur_top   = 現行ルール(単一支持に MIN_SUPPORT_RATIO 以上乗る場合のみ採用)の着地上面
      uni_top   = union ルール(重なる支持体すべての最大上面)の着地上面
      uni_ratio = uni_top と同じ高さ帯の支持体の重なり面積合計 / 荷物底面積
    を計算し、「既積み層の上に乗れる候補XY」がどれだけ増えるかを返す。
    """
    xy = msplanner._candidate_xy(container, half, obstacles, grid_density=grid_density)
    if xy.shape[0] == 0:
        return None
    ox = container['center'][0]
    thickness = container['thickness']
    wx = xy[:, 0] + ox
    wy = xy[:, 1]
    n = xy.shape[0]

    cur_top = np.full(n, thickness)
    uni_top = np.full(n, thickness)
    ratios = []
    tops = []
    for center, oh, _sp, _ss in supports:
        top = center[2] + oh[2]
        r = msplanner._rect_overlap_ratio_batch(wx, wy, half[0], half[1],
                                                center[0], center[1], oh[0], oh[1])
        better = (r >= msplanner.MIN_SUPPORT_RATIO) & (top > cur_top)
        cur_top = np.where(better, top, cur_top)
        uni_top = np.where((r > 1e-4) & (top > uni_top), top, uni_top)
        ratios.append(r)
        tops.append(top)

    if ratios:
        R = np.stack(ratios, axis=0)             # (M,N)
        T = np.array(tops)[:, None]              # (M,1)
        same_level = np.abs(T - uni_top[None, :]) < 0.02
        uni_ratio = np.sum(np.where(same_level, R, 0.0), axis=0)
    else:
        uni_ratio = np.ones(n)
    # 床(既積みの上でない)は常に完全支持
    on_floor = uni_top <= thickness + 1e-6
    uni_ratio = np.where(on_floor, 1.0, uni_ratio)

    stacked_cur = int(np.sum(cur_top > thickness + 1e-6))
    stacked_uni = int(np.sum((uni_top > thickness + 1e-6) & (uni_ratio >= msplanner.MIN_SUPPORT_RATIO)))
    stacked_uni60 = int(np.sum((uni_top > thickness + 1e-6) & (uni_ratio >= 0.6)))
    return {
        'n_xy': n,
        'stacked_current_rule': stacked_cur,
        'stacked_union_rule_0.9': stacked_uni,
        'stacked_union_rule_0.6': stacked_uni60,
        'mean_uni_ratio_on_stack': float(np.mean(uni_ratio[uni_top > thickness + 1e-6]))
        if np.any(uni_top > thickness + 1e-6) else float('nan'),
    }


def run_scene(task_config, optimize_budget, policy_budget):
    env = GroundHandlingEnv(config=task_config, verbose=False, render_mode=None)
    try:
        env.reset_settings()
        init_states = env.get_init_states()

        print('\n--- H2: 既積み荷物の姿勢(軸平行からのズレ・AABB膨張) ---')
        rows = _tilt_report(init_states['container_list'])
        if not rows:
            print('  (既積み荷物なし)')
        else:
            tilts = [r[2] for r in rows]
            infls = [r[3] for r in rows]
            print(f'  既積み {len(rows)} 個: tilt(deg) max={max(tilts):.2f} mean={sum(tilts)/len(tilts):.2f}, '
                  f'AABB体積膨張率 max={max(infls):.3f} mean={sum(infls)/len(infls):.3f}')
            for r in sorted(rows, key=lambda x: -x[2])[:5]:
                print(f'    c{r[0]} item{r[1]}: tilt={r[2]:.2f}deg infl={r[3]:.3f}')

        full_item_list = env.get_info_for_optimization()
        offline_placed = None
        if env.optimize:
            order = ordering.build_order(full_item_list, init_states['container_list'],
                                         init_states['lookahead_k'], time_budget=optimize_budget)
            env.set_item_order(order)

        env.reset_item_stream()
        obs, info = env.reset(seed=42)

        terminated = False
        truncated = False
        step = 0
        level0_hits = 0
        stall_container_list = None
        prev_container_list = None
        while not terminated and not truncated:
            container_list = obs['container_list']
            pool_list = obs['pool_list']
            if not pool_list:
                break
            action = msplanner.plan(container_list, pool_list, time_budget=policy_budget)
            if action is None:
                stall_container_list = container_list
                break
            prev_container_list = container_list
            obs, reward, terminated, truncated, step_info = env.step(action)
            step += 1

        final_list = stall_container_list or obs.get('container_list') or prev_container_list
        print('\n--- 停止時点のコンテナ利用状況 ---')
        for c in final_list:
            floor_z = c['center'][2] - c['height'] / 2.0
            vol = c['volume']
            pv = 0.0
            max_top = 0.0
            pre_top = 0.0
            for it in c['packed_items']:
                if it.get('pos') is None:
                    continue
                pv += it['length'] * it['width'] * it['height']
                ctr, hf = geo.item_world_aabb(it)
                top_rel = (ctr[2] + hf[2]) - floor_z
                max_top = max(max_top, top_rel)
                if it['index'] >= 1000:      # 既積み(gen_suite の採番)
                    pre_top = max(pre_top, top_rel)
            print(f'  c{c["index"]}: {len(c["packed_items"])}個 vol利用率={100 * pv / vol:5.1f}% '
                  f'到達高さ={100 * max_top / c["height"]:5.1f}% 既積み層上面={100 * pre_top / c["height"]:5.1f}%')

        placed_idx = {it['index'] for c in final_list for it in c['packed_items']}
        remaining = [it for it in full_item_list if it['index'] not in placed_idx]
        print(f'  ストリーム {len(full_item_list)} 個中 未配置 {len(remaining)} 個')

        # --- H1: 単一支持90%ルール vs union ルール ---
        print('\n--- H1: 着地面ルールの比較(未配置荷物 上位10個, 停止時点の状態) ---')
        for c in final_list:
            obstacles = msplanner._collect_obstacles(c)
            supports = msplanner._landing_supports(c)
            agg = {'n_xy': 0, 'cur': 0, 'uni9': 0, 'uni6': 0}
            for it in remaining[:10]:
                lwh = (it['length'], it['width'], it['height'])
                for orn in msplanner._unique_orientations(lwh):
                    half = geo.half_extent(lwh, orn)
                    res = _union_support_gain(c, half, obstacles, supports)
                    if res is None:
                        continue
                    agg['n_xy'] += res['n_xy']
                    agg['cur'] += res['stacked_current_rule']
                    agg['uni9'] += res['stacked_union_rule_0.9']
                    agg['uni6'] += res['stacked_union_rule_0.6']
            if agg['n_xy']:
                print(f'  c{c["index"]}: 候補XY総数={agg["n_xy"]} / '
                      f'既存物の上に着地できる候補: 現行(単一90%)={agg["cur"]} '
                      f'({100 * agg["cur"] / agg["n_xy"]:.2f}%), '
                      f'union90%={agg["uni9"]} ({100 * agg["uni9"] / agg["n_xy"]:.2f}%), '
                      f'union60%={agg["uni6"]} ({100 * agg["uni6"] / agg["n_xy"]:.2f}%)')
    finally:
        try:
            env.close()
        except Exception:
            pass


def main():
    args = parse_args()
    with open(args.config_path) as f:
        config = json.load(f)
    task_id = args.task_id or next(iter(config.keys()))
    print(f'=== {args.config_path}::{task_id} (optimize_budget={args.optimize_budget}s) ===')
    run_scene(config[task_id], args.optimize_budget, args.policy_budget)


if __name__ == '__main__':
    main()
