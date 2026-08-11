"""
tools/phase29_order_eval.py

Phase29: **明示的に与えた順序**を本物の env / evaluator で評価する。

`tools/phase29_repair_probe.py` が出す「修正した順序の良し悪し」は影シミュレータの
代理量(placed_volume / risk_adjusted_volume)であって、採点される fill_strict ではない。
Phase20 で代理の順位相関は Spearman 0.71 しかないと分かっているので、
**修正が本当に効いたのか**は本物で測らないと言えない。

本ツールは tools/measure_regime.py の1シーン実行から `optimize` の呼び出しだけを外し、
代わりに与えられた順序を `env.set_item_order` に流し込む(それ以外は完全に同じ経路・
同じ評価器・同じ margin)。build_order を回さないので1回あたり数十秒で済む。

実行:
    PYTHONPATH=. .venv/bin/python tools/phase29_order_eval.py --spec /path/to/spec.json
      spec: [{"label": "...", "name": "...", "order": [..]}, ...]
"""
import argparse
import json
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from src.ground_handling.agent_factory import AgentFactory
from src.ground_handling.env import GroundHandlingEnv
from src.ground_handling.evaluator import Evaluator
from src.ground_handling.runner import TimedAgentRunner
from tools.measure_regime import STRICT_MARGIN, LOOSE_MARGIN
from tools.scorer import Scorer


def run_with_order(task_config, module_path, agent_module, order):
    agent_factory = AgentFactory(module_name=agent_module, class_name='Agent',
                                 module_path=module_path)
    env = GroundHandlingEnv(config=task_config, verbose=False, render_mode=None)
    runner = None
    try:
        env.reset_settings()
        init_states = env.get_init_states()
        allowed = task_config['agent']['allowed_methods']
        runner = TimedAgentRunner(agent_factory=agent_factory, allowed_methods=allowed,
                                  max_mem=task_config['agent'].get('max_mem', 4), verbose=False)
        runner.call('get_init_states', time_out_sec=task_config['agent']['init_timeout'],
                    fallback=None, init_states=init_states)
        if order is not None:
            if not env.set_item_order(list(order)):
                return {'status': 'set_item_order failed'}
        env.reset_item_stream()
        obs, info = env.reset(seed=42)
        terminated = truncated = False
        while not terminated and not truncated:
            action, _ = runner.call('policy',
                                    time_out_sec=task_config['agent']['policy_timeout'],
                                    fallback=env.action_space.sample(), observation=obs)
            obs, reward, terminated, truncated, info = env.step(action)
        containers = env.container_manager.containers
        fill_strict, _ = Evaluator(client=env.client,
                                   config={'inclusion_margin': STRICT_MARGIN}
                                   ).calculate_fill_rate(containers)
        fill_loose, _ = Evaluator(client=env.client,
                                  config={'inclusion_margin': LOOSE_MARGIN}
                                  ).calculate_fill_rate(containers)
        scorer = Scorer(client=env.client, config=task_config)
        out = {
            # calculate_fill_rate は既に百分率(%)を返す(tools/measure_regime.py と同じ扱い)。
            'fill_strict': fill_strict, 'fill_loose': fill_loose,
            'num_placed': sum(len(c.packed_items) for c in containers),
            'placement_score': scorer.calculate_placement_score(containers),
            'soft_item_score': scorer.calculate_soft_item_score(containers),
            'cog_score': scorer.calculate_cog_score(containers),
            'stability_score': scorer.calculate_stability_score(containers),  # 破壊的: 必ず最後
        }
        return out
    finally:
        try:
            if runner is not None:
                runner.close()
        except Exception:
            pass
        try:
            env.close()
        except Exception:
            pass


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--spec', required=True)
    ap.add_argument('--module-path', default='agents/mysolver/')
    ap.add_argument('--out', default=None)
    args = ap.parse_args()
    agent_module = '.'.join(args.module_path.split('/')) + 'agent'

    spec = json.load(open(args.spec))
    rows = []
    for s in spec:
        task = list(json.load(open(f"configs/gen/suite_{s['label']}.json")).values())[0]
        r = run_with_order(task, args.module_path, agent_module, s.get('order'))
        r.update({'label': s['label'], 'name': s['name']})
        rows.append(r)
        print(f"{s['label']:28s} {s['name']:20s} fill_strict={r.get('fill_strict', float('nan')):6.2f} "
              f"fill_loose={r.get('fill_loose', float('nan')):6.2f} "
              f"placed={r.get('num_placed')} placement={r.get('placement_score')}", flush=True)
        if args.out:
            json.dump(rows, open(args.out, 'w'), indent=1, ensure_ascii=False)


if __name__ == '__main__':
    main()
