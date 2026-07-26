"""
tools/phase17_probe.py

Phase17 事前調査: 探索打ち切りの壁時計依存が「実際にどこで・どれくらい効いているか」を実測し、
候補数ベースの打ち切りに置き換えるための較正定数(1秒あたりに処理できる評価ユニット数)を得る。

計測するもの:
  1. planner._evaluate_candidates の呼び出しごとの (候補XY数 n_xy, supports数, obstacles数, 実時間)
     -> コストモデル units = n_xy * (n_sup + n_obs + K) が実時間に線形かを確認し、units/sec を求める。
  2. deadline による打ち切りが _evaluate_candidates の入口で何回発火したか。

TimedAgentRunner は agent を spawn した別プロセスで動かすためモンキーパッチが効かない。
そこで本ツールは runner を使わず、app.py と同じ手順を「同一プロセス内で」実行する
(optimize / policy を Agent インスタンスへ直接呼ぶ)。タイムアウト計測は行わないが、
本ツールの目的は探索コストの実測なので影響しない。

src/ agents/ は一切変更しない(実行時のモンキーパッチのみ)。
"""
import argparse
import glob
import json
import os
import sys
import time

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

import numpy as np

from agents.mysolver import planner
from agents.mysolver.agent import Agent
from src.ground_handling.env import GroundHandlingEnv

SAMPLES: list[tuple[int, int, int, float]] = []
HITS: dict[str, float] = {}
CANDIDATE_BUILD_COST = 3.0


def _install_probe():
    orig_eval = planner._evaluate_candidates
    orig_cand = planner._candidate_xy
    orig_plan = planner.plan

    def probed_eval(container, item, half, obstacles, supports, candidate_xy, budget, **kw):
        # Phase17リファクタ後は budget(SearchBudget)。リファクタ前は壁時計 deadline(float)。
        cut = budget.exhausted() if hasattr(budget, 'exhausted') else (time.perf_counter() > budget)
        if cut:
            HITS['eval_entry_cut'] = HITS.get('eval_entry_cut', 0) + 1
            return None
        t0 = time.perf_counter()
        r = orig_eval(container, item, half, obstacles, supports, candidate_xy, budget, **kw)
        dt = time.perf_counter() - t0
        SAMPLES.append((int(candidate_xy.shape[0]), len(supports), len(obstacles), dt))
        HITS['eval_calls'] = HITS.get('eval_calls', 0) + 1
        HITS['eval_time'] = HITS.get('eval_time', 0.0) + dt
        return r

    def probed_cand(container, half, obstacles, grid_density=1):
        t0 = time.perf_counter()
        r = orig_cand(container, half, obstacles, grid_density=grid_density)
        HITS['cand_time'] = HITS.get('cand_time', 0.0) + (time.perf_counter() - t0)
        HITS['cand_calls'] = HITS.get('cand_calls', 0) + 1
        # 名目コスト: グリッド点数 + 障害物あたり8点のExtreme Point生成
        HITS['cand_units'] = HITS.get('cand_units', 0.0) + (
            31 * 23 * grid_density * grid_density + 8 * len(obstacles))
        return r

    def probed_plan(*a, **kw):
        t0 = time.perf_counter()
        r = orig_plan(*a, **kw)
        HITS['plan_time'] = HITS.get('plan_time', 0.0) + (time.perf_counter() - t0)
        HITS['plan_calls'] = HITS.get('plan_calls', 0) + 1
        return r

    planner._evaluate_candidates = probed_eval
    planner._candidate_xy = probed_cand
    planner.plan = probed_plan


def run_scene(task_config, label):
    env = GroundHandlingEnv(config=task_config, verbose=False, render_mode=None)
    try:
        env.reset_settings()
        agent = Agent('agents/mysolver/')
        agent.get_init_states(env.get_init_states())
        t_opt = 0.0
        if env.optimize:
            t0 = time.perf_counter()
            order = agent.optimize(env.get_info_for_optimization())
            t_opt = time.perf_counter() - t0
            env.set_item_order(order)
        env.reset_item_stream()
        obs, info = env.reset(seed=42)
        terminated = truncated = False
        n_steps = 0
        t_pol_max = 0.0
        while not terminated and not truncated:
            t0 = time.perf_counter()
            action = agent.policy(obs)
            t_pol_max = max(t_pol_max, time.perf_counter() - t0)
            obs, reward, terminated, truncated, info = env.step(action)
            n_steps += 1
        return {'label': label, 'opt_sec': t_opt, 'policy_max_sec': t_pol_max, 'steps': n_steps}
    finally:
        try:
            env.close()
        except Exception:
            pass


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--config-path', nargs='+', required=True)
    ap.add_argument('--optimize-budget', type=float, default=None)
    ap.add_argument('--out', required=True)
    args = ap.parse_args()

    if args.optimize_budget is not None:
        os.environ['MYSOLVER_OPTIMIZE_BUDGET'] = str(args.optimize_budget)

    _install_probe()

    paths = []
    for pat in args.config_path:
        m = sorted(glob.glob(pat))
        paths.extend(m if m else [pat])

    scenes = []
    for cp in paths:
        with open(cp) as f:
            cfg = json.load(f)
        for task_id, task_config in cfg.items():
            label = f'{os.path.basename(cp)}::{task_id}'
            before = dict(HITS)
            n0 = len(SAMPLES)
            r = run_scene(task_config, label)
            for k in ('eval_calls', 'eval_entry_cut', 'eval_time', 'cand_time', 'cand_units',
                      'plan_time', 'plan_calls'):
                r[k] = HITS.get(k, 0) - before.get(k, 0)
            sub = np.array(SAMPLES[n0:], dtype=np.float64)
            if sub.shape[0]:
                u = sub[:, 0] * (sub[:, 1] + sub[:, 2] + 8.0)
                r['eval_units_K8'] = float(u.sum())
                r['units_per_sec_scene'] = float((u.sum() + r['cand_units'] * 3.0)
                                                  / max(r['plan_time'], 1e-9))
            scenes.append(r)
            print(f'  [{label}] opt={r["opt_sec"]:.1f}s policy_max={r["policy_max_sec"]:.2f}s '
                  f'steps={r["steps"]} plan_time={r["plan_time"]:.1f}s eval_time={r["eval_time"]:.1f}s '
                  f'cand_time={r["cand_time"]:.1f}s units/s={r.get("units_per_sec_scene", 0):.3g}')

    arr = np.array(SAMPLES, dtype=np.float64) if SAMPLES else np.zeros((0, 4))
    out = {'hits': HITS, 'n_samples': int(arr.shape[0]), 'scenes': scenes}
    if arr.shape[0]:
        n_xy, n_sup, n_obs, dt = arr[:, 0], arr[:, 1], arr[:, 2], arr[:, 3]
        for K in (2, 4, 8, 16, 32):
            units = n_xy * (n_sup + n_obs + K)
            rate = units / np.maximum(dt, 1e-9)
            out[f'K{K}'] = {
                'units_total': float(units.sum()),
                'units_per_sec_agg': float(units.sum() / max(dt.sum(), 1e-9)),
                'rate_p05': float(np.percentile(rate, 5)),
                'rate_p50': float(np.percentile(rate, 50)),
                'rate_p95': float(np.percentile(rate, 95)),
                'rate_cv': float(np.std(rate) / max(np.mean(rate), 1e-9)),
            }
        out['n_xy'] = {'p50': float(np.percentile(n_xy, 50)), 'p90': float(np.percentile(n_xy, 90)),
                       'max': float(n_xy.max())}
        out['dt'] = {'p50': float(np.percentile(dt, 50)), 'p90': float(np.percentile(dt, 90)),
                     'max': float(dt.max()), 'sum': float(dt.sum())}
        out['n_obs'] = {'p50': float(np.percentile(n_obs, 50)), 'max': float(n_obs.max())}
        out['n_sup'] = {'p50': float(np.percentile(n_sup, 50)), 'max': float(n_sup.max())}
        # 本命の較正値: 全ユニット(評価+候補構築) / plan() の総壁時計時間。
        for K in (8, 16, 32):
            tot = float((n_xy * (n_sup + n_obs + K)).sum()) + HITS.get('cand_units', 0.0) * CANDIDATE_BUILD_COST
            out[f'calib_K{K}'] = {
                'total_units': tot,
                'plan_time': HITS.get('plan_time', 0.0),
                'units_per_sec': tot / max(HITS.get('plan_time', 0.0), 1e-9),
            }
        out['time_share'] = {
            'eval': HITS.get('eval_time', 0.0) / max(HITS.get('plan_time', 0.0), 1e-9),
            'cand': HITS.get('cand_time', 0.0) / max(HITS.get('plan_time', 0.0), 1e-9),
        }
    with open(args.out, 'w') as f:
        json.dump(out, f, indent=2)
    print(json.dumps({k: v for k, v in out.items() if k != 'scenes'}, indent=2))


if __name__ == '__main__':
    main()
