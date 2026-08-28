"""Phase79 (1-1): Phase64/65が特定した345件のfail_support(既定閾値0.55/0.6/0.15)を、
緩2閾値(0.35/0.4/0.25)で再評価すると何件救われるかを測る(読み取り専用)。

方法論上の注意(Phase79で判明): MIN_UNION_SUPPORT_RATIO等はplanner.py import時に
環境変数から一度だけ読まれるモジュール定数のため、プロセス全体をMYSOLVER_*環境変数付きで
起動すると `_reach_death` のエピソード再現(planner.plan()を全ステップで使う)自体が
緩2挙動になり、死亡局面(obs_before・pool_list・container_listのindex)がPhase64/65の
345件と一致しなくなる(実際にcontainer_or_item_not_foundが多発することを確認)。

そこで本ツールは:
  1) 環境変数は一切設定せず(既定0.55/0.6/0.15のまま)、Phase64/65と全く同じ手順で
     `_reach_death` を呼び、345件のfindingsを生成した死亡局面を再現する
     (deterministic、seed=42固定なのでPhase64/65と同一になるはず)。
  2) 各findingについて、`_evaluate_candidates` を
       (a) 既定閾値(0.55/0.6/0.15)のまま
       (b) 呼び出し直前だけ planner_mod のモジュール変数を緩2値(0.35/0.4/0.25)に
           monkeypatchし、呼び出し後に必ず既定値へ戻す
     の両方で実行し、成否とstats(fail_support内訳)を記録する。
  3) (a)がfail_supportだったもののうち(b)がsuccessになった件数=「centroidで救われた件数」。
     (b)でもまだfail_supportのものは、その内訳(forbidden_hit有無)を集計する。

新しい判定式は書かない。既存の`planner._evaluate_candidates`をそのまま2回呼ぶだけ。
"""
import json
import os
import sys
import time
import traceback
from collections import Counter

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from agents.mysolver import planner as planner_mod
from agents.mysolver import geometry as geo  # noqa: F401

from tools.phase64_exhaustive import _reach_death
from tools.phase65_filter_trace import _config_path_for_label, _find_by_index

DEFAULT_INPUTS = [
    'results/phase64_exhaustive_26.json',
    'results/phase64_exhaustive_sampleconfig.json',
]

LOOSE2 = dict(MIN_UNION_SUPPORT_RATIO=0.35, MIN_SUPPORT_SPAN_RATIO=0.4, MAX_SUPPORT_CENTROID_OFFSET=0.25)
DEFAULTS = dict(MIN_UNION_SUPPORT_RATIO=0.55, MIN_SUPPORT_SPAN_RATIO=0.6, MAX_SUPPORT_CENTROID_OFFSET=0.15)


def _assert_defaults_active():
    for name, val in DEFAULTS.items():
        actual = getattr(planner_mod, name)
        assert abs(actual - val) < 1e-9, f'{name} is {actual}, expected default {val} (env var leaked in?)'


def _eval_once(container, item, orn_idx, pos_world, prepacked_ids, strict_support, corridor_obstacles, obstacles, supports):
    lwh = (item['length'], item['width'], item['height'])
    half = geo.half_extent(lwh, orn_idx)
    ox = container['center'][0]
    local_x = float(pos_world[0]) - ox
    local_y = float(pos_world[1])
    budget = planner_mod.SearchBudget(limit=1e15)
    stats = {}
    result = planner_mod._evaluate_candidates(
        container, item, half, obstacles, supports, np.array([[local_x, local_y]]), budget,
        stats=stats, strict_support=strict_support, corridor_obstacles=corridor_obstacles)
    return {'result': 'success' if result is not None else 'fail', 'stats': dict(stats)}


def trace_one_finding_both(container, item, orn_idx, pos_world, prepacked_ids, strict_support):
    obstacles = planner_mod._collect_obstacles(container)
    corridor_obstacles = planner_mod._collect_corridor_obstacles(container, prepacked_ids)
    supports = planner_mod._landing_supports(container)

    _assert_defaults_active()
    default_res = _eval_once(container, item, orn_idx, pos_world, prepacked_ids, strict_support,
                              corridor_obstacles, obstacles, supports)

    saved = {}
    for k, v in LOOSE2.items():
        saved[k] = getattr(planner_mod, k)
        setattr(planner_mod, k, v)
    try:
        loose2_res = _eval_once(container, item, orn_idx, pos_world, prepacked_ids, strict_support,
                                 corridor_obstacles, obstacles, supports)
    finally:
        for k, v in saved.items():
            setattr(planner_mod, k, v)
    _assert_defaults_active()

    return default_res, loose2_res


def main():
    _assert_defaults_active()
    all_scenes = {}
    for fp in DEFAULT_INPUTS:
        all_scenes.update(json.load(open(fp)))
    target_scenes = {k: v for k, v in all_scenes.items()
                      if v.get('status') == 'ok' and v.get('exhaustive_n_findings', 0) > 0}
    print(f'target scenes: {len(target_scenes)}')

    out = {}
    for label, scene_result in target_scenes.items():
        t0 = time.perf_counter()
        try:
            cp = _config_path_for_label(label)
            tk = label.split('::', 1)[1]
            task = json.load(open(cp))[tk]
            death = _reach_death(task, 'agents/mysolver/', 'agents.mysolver.agent')
            if death is None:
                out[label] = {'status': 'death_not_reproduced'}
                print(f'[{label}] death_not_reproduced')
                continue
            obs = death['obs_before']
            container_list = obs['container_list']
            pool_list = obs['pool_list']
            strict_support = not death['optimize_flag']
            prepacked_ids = death['prepacked_ids']

            traces = []
            n_not_found = 0
            for f in scene_result['exhaustive_findings_sample']:
                container = _find_by_index(container_list, f['container_index'])
                item = _find_by_index(pool_list, f['item_index'])
                if container is None or item is None:
                    n_not_found += 1
                    continue
                d_res, l_res = trace_one_finding_both(container, item, f['orientation'], f['pos_world'],
                                                        prepacked_ids, strict_support)
                traces.append({'finding': f, 'default': d_res, 'loose2': l_res})
            out[label] = {'status': 'ok', 'n_traced': len(traces), 'n_not_found': n_not_found,
                           'traces': traces}
            print(f'[{label}] ok n_traced={len(traces)} n_not_found={n_not_found} '
                  f'({time.perf_counter()-t0:.1f}s)')
        except Exception:
            out[label] = {'status': f'error: {traceback.format_exc().splitlines()[-1]}'}
            print(f'[{label}] error: {traceback.format_exc().splitlines()[-1]}')
        with open('results/phase79_centroid_recheck.json', 'w') as fh:
            json.dump(out, fh, indent=2, default=str)

    # --- summary ---
    n_default_fail_support = 0
    n_saved_by_loose2 = 0
    n_still_fail_support = 0
    n_still_threshold_only = 0
    n_still_both = 0
    n_still_forbidden_only = 0
    n_not_found_total = 0
    for label, res in out.items():
        if res.get('status') != 'ok':
            continue
        n_not_found_total += res.get('n_not_found', 0)
        for tr in res['traces']:
            d = tr['default']['stats']
            l = tr['loose2']
            if d.get('fail_support'):
                n_default_fail_support += 1
                if l['result'] == 'success':
                    n_saved_by_loose2 += 1
                else:
                    ls = l['stats']
                    if ls.get('fail_support'):
                        n_still_fail_support += 1
                        n_still_threshold_only += ls.get('fail_support_threshold_only', 0)
                        n_still_both += ls.get('fail_support_both', 0)
                        n_still_forbidden_only += ls.get('fail_support_forbidden_only', 0)
    print('\n=== summary ===')
    print(f'n_not_found_total(episode diverged from Phase64/65 sample; excluded): {n_not_found_total}')
    print(f'n_default_fail_support (matches Phase64/65 345): {n_default_fail_support}')
    print(f'n_saved_by_loose2: {n_saved_by_loose2}')
    print(f'n_still_fail_support_under_loose2: {n_still_fail_support}')
    print(f'  of which threshold_only: {n_still_threshold_only}')
    print(f'  of which both(forbidden_hit): {n_still_both}')
    print(f'  of which forbidden_only: {n_still_forbidden_only}')


if __name__ == '__main__':
    main()
