"""
tools/phase34_noleak.py

Phase34 の「既定(ALNS無効)では既存経路がビット単位で不変」を直接検証する。

Phase34 は `simulate.simulate_order` の内部(順序ストリームの読み出しを iter() から
リスト+カーソルへ置換)にも手を入れているため、「新しい引数を渡さなければ無変更」だけでは
不十分で、**同じ入力に対して build_order が返す順序そのものが一致すること** を確認する。

比較対象は git worktree に展開した親コミット(Phase34 適用前)の `agents/mysolver`。
同一シーン・同一予算で `build_order` を両方走らせ、返る順序を要素単位で比較する。

実行:
    MYSOLVER_UNITS_PER_SEC=1.55e7 PYTHONPATH=. .venv/bin/python tools/phase34_noleak.py \
        --old-root /tmp/p34_old --scenes A01_1c_40_plain A02_1c_80_plain A03_1c_40_shelf \
        --out results/phase34_noleak.json
"""
import argparse
import importlib.util
import io
import json
import os
import sys
import time
from contextlib import redirect_stdout

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from src.ground_handling.env import GroundHandlingEnv


def load_module_tree(root, name):
    """`root/agents/mysolver` を独立したパッケージ名で読み込む(現行版と共存させるため)。"""
    pkg_dir = os.path.join(root, 'agents', 'mysolver')
    spec = importlib.util.spec_from_file_location(
        name, os.path.join(pkg_dir, '__init__.py'),
        submodule_search_locations=[pkg_dir])
    mod = importlib.util.module_from_spec(spec)
    sys.modules[name] = mod
    spec.loader.exec_module(mod)
    ordering = importlib.import_module(f'{name}.ordering')
    return ordering


def load_scene(path):
    task = list(json.load(open(path)).values())[0]
    env = GroundHandlingEnv(config=task, verbose=False, render_mode=None)
    try:
        env.reset_settings()
        init = env.get_init_states()
        optimize = env.optimize
        items = env.get_info_for_optimization() if optimize else None
    finally:
        try:
            env.close()
        except Exception:
            pass
    return optimize, init['container_list'], init['lookahead_k'], items


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--old-root', required=True, help='Phase34適用前を展開した git worktree のパス')
    ap.add_argument('--scenes', nargs='+', required=True)
    ap.add_argument('--budget', type=float, default=120.0)
    ap.add_argument('--out', default=None)
    args = ap.parse_args()

    assert os.environ.get('MYSOLVER_ALNS', '0') != '1', 'ALNSは無効(既定)で走らせること'

    from agents.mysolver import ordering as new_ordering
    old_ordering = load_module_tree(args.old_root, 'p34old')

    rows = []
    for label in args.scenes:
        path = f'configs/gen/suite_{label}.json'
        optimize, container_list, lookahead, items = load_scene(path)
        if not optimize:
            print(f'{label:32s} optimize=False(build_orderを呼ばない)')
            rows.append({'label': label, 'optimize': False, 'match': True})
            continue
        out = {}
        for tag, mod in (('old', old_ordering), ('new', new_ordering)):
            t0 = time.perf_counter()
            with redirect_stdout(io.StringIO()):
                out[tag] = list(mod.build_order(items, container_list, lookahead,
                                                 time_budget=args.budget))
            out[tag + '_s'] = time.perf_counter() - t0
        match = out['old'] == out['new']
        n_diff = sum(1 for a, b in zip(out['old'], out['new']) if a != b)
        rows.append({'label': label, 'optimize': True, 'match': match, 'n_diff': n_diff,
                     'old_s': out['old_s'], 'new_s': out['new_s'], 'n_items': len(out['new'])})
        print(f'{label:32s} 一致={match} 差分要素={n_diff}/{len(out["new"])} '
              f'(old {out["old_s"]:.0f}s / new {out["new_s"]:.0f}s)', flush=True)
        if args.out:
            json.dump(rows, open(args.out, 'w'), indent=1, ensure_ascii=False)

    ok = sum(1 for r in rows if r['match'])
    print(f'\n{ok}/{len(rows)} シーンで build_order の出力が完全一致')


if __name__ == '__main__':
    main()
