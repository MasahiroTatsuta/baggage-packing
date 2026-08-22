"""stability_score の荷物単位の内訳を調べる診断ツール(Phase52)。

`tools/scorer.py::Scorer.calculate_stability_score()` は蓋をして重力を揺らし、
収束後の変位(displacement)と残留運動エネルギー(kinetic energy)から単一のスコアを
返すだけで、荷物単位の内訳は返さない。本ツールは同メソッドの内部ロジックを
**そのまま複製**(手順・パラメータ・閾値は一字一句変更しない)し、集計前の
荷物単位の変位・エネルギーを追加で出力する。scorer.py 自体は一切変更しない。

読み取り専用(`src/`・`tools/scorer.py`・`configs/` は一切変更しない)。

実行方法(リポジトリルートで):
    MYSOLVER_HARD_WALL_LIMIT=3000 MYSOLVER_REPLICA_SELECT=0 MYSOLVER_UNITS_PER_SEC=2.00e7 \\
        PYTHONPATH=. .venv/bin/python tools/diagnose_stability.py \\
        --config-path 'configs/gen/suite_C02*.json' --out /tmp/stability_c02.json

経緯: Phase52でstability_score(本番70.44 / ローカル98.2)の乖離調査に着手するにあたり、
「どのアイテムがどれだけ動いたか」を特定する必要があったため新規作成。

Phase61追記: 揺らしの加速度振幅を `MYSOLVER_DIAG_SHAKE_AMPLITUDE`(既定 '6.0'、単位m/s²)で
env化した。公式チュートリアルセミナーで運営が開示した範囲(最大1〜3G程度=9.8〜29.4m/s²)
での**ロバスト性確認**(この範囲全体でstabilityがどう分布するか)のために使う。
**「本番のstability_score(70.44)に最も近い水準はどれか」を特定・採用する目的では
使わない**(評価関数内の非公開パラメーターの解析に該当するため。
docs/submission_policy.md §4 Phase61追記を参照)。
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
    p.add_argument('--out', default='/tmp/diagnose_stability.json')
    return p.parse_args()


def stability_with_item_detail(scorer: Scorer, containers, shake_steps: int = 150,
                                settle_steps: int = 180) -> dict:
    """tools/scorer.py::calculate_stability_score() のロジックをそのまま複製し、
    荷物単位の変位・エネルギーを追加で返す(集計後のスコア自体は元関数と一致するはず)。"""
    client = scorer.client
    all_items = [item for c in containers for item in c.packed_items if item.pybullet_id is not None]
    if not all_items:
        return {'stability_score': 100.0, 'items': []}

    initial_pos = {}
    for item in all_items:
        pos, _ = item.get_pose(client)
        if pos is not None:
            initial_pos[item.index] = np.array(pos)

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
        item_rows.append({
            'index': item.index,
            'disp': disp,
            'ke': float(ke),
            'mass': item.mass,
            'is_soft': item.is_soft,
            'is_prioritized': item.is_prioritized,
            'length': item.length, 'width': item.width, 'height': item.height,
        })

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

        # evaluate()と同じ呼び出し順(fill->placement->soft->cog->stability)を守る
        # (Phase47で発覚した「stabilityは最後」規約に従う)。
        fill_score, _ = scorer.calculate_fill_score(containers)
        placement_score = scorer.calculate_placement_score(containers)
        soft_item_score = scorer.calculate_soft_item_score(containers)
        cog_score = scorer.calculate_cog_score(containers)
        detail = stability_with_item_detail(scorer, containers)

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
        # Phase61: 1ファイルに複数タスクを持つconfig(例: sample_config.json)でも
        # 全タスクを回すよう修正(旧実装は最初のタスクのみ処理していた。26シーンの
        # suite_*.jsonは1ファイル1タスクなのでこの修正による影響はない)。
        tasks = json.load(open(cp))
        for tk, task in tasks.items():
            label = f'{os.path.basename(cp)}::{tk}' if len(tasks) > 1 else os.path.basename(cp)
            t0 = time.perf_counter()
            r = run_one_scene(task, args.module_path, agent_module)
            r['elapsed_sec'] = time.perf_counter() - t0
            results[label] = r
            n_items = r.get('n_items', 0)
            top = r.get('items', [])[:3]
            top_str = ', '.join(f"idx{it['index']}:disp={it['disp']:.4f}m,ke={it['ke']:.4f}" for it in top)
            print(f"[{label}] stability={r.get('stability_score')} n_items={n_items} "
                  f"mean_disp={r.get('mean_disp')} mean_energy={r.get('mean_energy')} top3=[{top_str}]")

    with open(args.out, 'w') as f:
        json.dump(results, f, indent=2)
    print(f"wrote {args.out}")


if __name__ == '__main__':
    main()
