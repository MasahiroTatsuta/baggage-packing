"""
tools/phase17_dump.py

Phase17: リファクタ前後で「同じ入力から同じ出力が出るか」を厳密に比べるための決定性ダンプ。

1シーンを app.py と同じ手順(同一プロセス内)で走らせ、
  - optimize() が返した積付順序
  - policy() が毎ステップ返した action (item_index / container / place_pos / orientation)
  - 最終的な各コンテナの packed_items (index / pos / orn)
を JSON に落とす。2つの実行結果を `--diff a.json b.json` で比較すれば、
「打ち切り位置が変わったか」「完全一致か」を機械的に判定できる。

module-path を切り替えれば旧実装(_hist/…)とも比較できる。
"""
import argparse
import glob
import hashlib
import importlib
import json
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from src.ground_handling.env import GroundHandlingEnv


def _round(v, nd=6):
    if isinstance(v, (list, tuple)):
        return [_round(x, nd) for x in v]
    try:
        return round(float(v), nd)
    except (TypeError, ValueError):
        return v


def run_scene(task_config, agent_cls, module_path, label, budget=None):
    env = GroundHandlingEnv(config=task_config, verbose=False, render_mode=None)
    try:
        env.reset_settings()
        agent = agent_cls(module_path)
        agent.get_init_states(env.get_init_states())
        order = None
        if env.optimize:
            order = list(agent.optimize(env.get_info_for_optimization()))
            env.set_item_order(order)
        env.reset_item_stream()
        obs, info = env.reset(seed=42)
        actions = []
        terminated = truncated = False
        while not terminated and not truncated:
            pool = obs.get('pool_list', [])
            action = agent.policy(obs)
            idx = action['item_idx']
            actions.append({
                'item_index': int(pool[idx]['index']) if idx < len(pool) else None,
                'container_idx': int(action['container_idx']),
                'orientation': int(action['orientation']),
                'place_pos': _round(list(action['place_pos'])),
            })
            obs, reward, terminated, truncated, info = env.step(action)
        final = []
        for c in env.container_manager.containers:
            final.append(sorted(
                [{'index': int(it.index), 'pos': _round(list(it.pos)), 'orn': _round(list(it.orn))}
                 for it in c.packed_items],
                key=lambda d: d['index']))
        return {'label': label, 'order': order, 'actions': actions, 'final': final,
                'n_placed': sum(len(f) for f in final)}
    finally:
        try:
            env.close()
        except Exception:
            pass


def digest(scene: dict) -> str:
    payload = json.dumps({k: scene[k] for k in ('order', 'actions', 'final')},
                          sort_keys=True, ensure_ascii=False)
    return hashlib.sha256(payload.encode()).hexdigest()[:16]


def cmd_run(args):
    module_path = args.module_path
    mod_name = '.'.join(module_path.rstrip('/').split('/')) + '.agent'
    agent_cls = importlib.import_module(mod_name).Agent

    if args.optimize_budget is not None:
        os.environ['MYSOLVER_OPTIMIZE_BUDGET'] = str(args.optimize_budget)

    paths = []
    for pat in args.config_path:
        m = sorted(glob.glob(pat))
        paths.extend(m if m else [pat])

    out = {'module_path': module_path, 'optimize_budget': args.optimize_budget, 'scenes': {}}
    for cp in paths:
        with open(cp) as f:
            cfg = json.load(f)
        for task_id, task_config in cfg.items():
            label = f'{os.path.basename(cp)}::{task_id}'
            s = run_scene(task_config, agent_cls, module_path, label)
            s['digest'] = digest(s)
            out['scenes'][label] = s
            print(f'  [{label}] placed={s["n_placed"]} steps={len(s["actions"])} digest={s["digest"]}')
    with open(args.out, 'w') as f:
        json.dump(out, f, ensure_ascii=False, indent=1)
    print(f'wrote {args.out}')


def cmd_diff(args):
    a = json.load(open(args.diff[0]))
    b = json.load(open(args.diff[1]))
    labels = sorted(set(a['scenes']) | set(b['scenes']))
    n_same = 0
    for lab in labels:
        sa, sb = a['scenes'].get(lab), b['scenes'].get(lab)
        if sa is None or sb is None:
            print(f'  [{lab}] MISSING in {"A" if sa is None else "B"}')
            continue
        same = sa['digest'] == sb['digest']
        n_same += same
        tag = 'IDENTICAL' if same else 'DIFFER'
        extra = ''
        if not same:
            parts = []
            if sa['order'] != sb['order']:
                parts.append('order')
            if sa['actions'] != sb['actions']:
                # 何手目から違うか
                k = next((i for i, (x, y) in enumerate(zip(sa['actions'], sb['actions'])) if x != y),
                         min(len(sa['actions']), len(sb['actions'])))
                parts.append(f'actions@{k}/{len(sa["actions"])}vs{len(sb["actions"])}')
            if sa['final'] != sb['final']:
                parts.append('final')
            extra = ' (' + ','.join(parts) + f'; placed {sa["n_placed"]} vs {sb["n_placed"]})'
        print(f'  [{lab}] {tag}{extra}')
    print(f'\n{n_same}/{len(labels)} scenes identical')


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--config-path', nargs='+')
    ap.add_argument('--module-path', default='agents/mysolver/')
    ap.add_argument('--optimize-budget', type=float, default=None)
    ap.add_argument('--out')
    ap.add_argument('--diff', nargs=2)
    args = ap.parse_args()
    if args.diff:
        cmd_diff(args)
    else:
        cmd_run(args)


if __name__ == '__main__':
    main()
