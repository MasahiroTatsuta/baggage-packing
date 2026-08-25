"""Phase65: Phase64 が見つけた合法解(18シーン・345件)を、planner.py の実際の候補評価
パイプラインへ1つずつ通し、どの段階で落ちるかを実測する(読み取り専用、仮説なし)。

判定式は一切書き起こさない。すべて既存の生産コード(planner.py / geometry.py)の関数を
そのまま呼び出し、その戻り値・診断カウンタ(`_evaluate_candidates(..., stats=...)`、
tools/diagnose_stall.py が使っているのと同じフック)をそのまま記録する。

入力: results/phase64_exhaustive_26.json, results/phase64_exhaustive_sampleconfig.json
  (`exhaustive_findings_sample` に入っている item_index/orientation/container_index/
   pos_world を使う。死亡直前の観測(obs_before)自体はこのJSONには保存されていないため、
   Phase64/Phase61-63と全く同じ手順(`tools.phase64_exhaustive._reach_death`をそのまま呼ぶ)
   で同一シーンを再実行し、同じ死亡直前局面を再現する。エピソードはseed=42固定・
   agentは決定的なので、同じ局面が再現される(Phase63のビット単位不変確認と同じ前提)。

各合法解について、planner.py の実行順に沿って以下を調べる:
  1) `planner._candidate_xy` が同一/近傍の点を生成しているか(距離つき)
  2) `planner._unique_orientations` に該当orientationが含まれるか
  3) `planner._apply_y_slice_filter`(y_active_lo、層規律)を level0 で通過するか
  4) 優先コンテナのtier分離ロジック(情報記録: enforce_priority_containerでpass1から
     除外されるか、reserve_priority_containerでtier1に格下げされるか)
  5) `planner._evaluate_candidates` 本体(支持品質 MIN_UNION_SUPPORT_RATIO(_STRICT)・
     内包判定・搬入経路 legal1/legal2 を含む合法性判定一式)を、
     (a) 解の実座標そのもの、(b) plannerが実際に生成する最近傍候補点、の両方に対して呼ぶ

実行方法(リポジトリルートで、時間がかかるため background 推奨):
    PYTHONPATH=. .venv/bin/python tools/phase65_filter_trace.py \\
        --out results/phase65_filter_trace.json
"""
import argparse
import json
import os
import sys
import time
import traceback
from collections import Counter

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from agents.mysolver import agent as agent_mod  # noqa: F401  (参照用、直接は使わない)
from agents.mysolver import planner as planner_mod
from agents.mysolver import geometry as geo

from tools.phase64_exhaustive import _reach_death

DEFAULT_INPUTS = [
    'results/phase64_exhaustive_26.json',
    'results/phase64_exhaustive_sampleconfig.json',
]
NEAR_SAME_MM = 1.0  # 「同一/近傍」とみなす距離のしきい値(報告用の目安であり合否判定には使わない)


def parse_args():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('--inputs', nargs='+', default=DEFAULT_INPUTS)
    p.add_argument('--module-path', default='agents/mysolver/')
    p.add_argument('--out', default='results/phase65_filter_trace.json')
    p.add_argument('--checkpoint-every', type=int, default=1,
                   help='何シーン処理するごとに --out へ中間保存するか')
    return p.parse_args()


def _config_path_for_label(label: str) -> str:
    basename = label.split('::', 1)[0]
    if basename == 'sample_config.json':
        return 'configs/sample_config.json'
    return f'configs/gen/{basename}'


def _find_by_index(items, index):
    for it in items:
        if it.get('index') == index:
            return it
    return None


def trace_one_finding(container, item, orn_idx, pos_world, prepacked_ids, strict_support):
    """1件の合法解(item, orientation, container, world座標)をplannerの実パイプラインへ
    順に通し、各段階の実測結果を返す。新しい判定式は書かず、既存関数の戻り値をそのまま使う。
    """
    lwh = (item['length'], item['width'], item['height'])
    half = geo.half_extent(lwh, orn_idx)
    ox = container['center'][0]
    local_x = float(pos_world[0]) - ox
    local_y = float(pos_world[1])

    obstacles = planner_mod._collect_obstacles(container)
    corridor_obstacles = planner_mod._collect_corridor_obstacles(container, prepacked_ids)
    supports = planner_mod._landing_supports(container)

    out = {}

    # --- 1) _candidate_xy: 同一/近傍の点を生成しているか ---
    nearest_base_xy = None
    for name, density in (('base', planner_mod.BASE_GRID_DENSITY),
                          ('retry', planner_mod.RETRY_GRID_DENSITY)):
        cand = planner_mod._candidate_xy(container, half, obstacles, grid_density=density)
        if cand.shape[0] == 0:
            out[f'candidate_xy_dist_mm_{name}'] = float('inf')
            continue
        d = np.linalg.norm(cand - np.array([local_x, local_y]), axis=1)
        i_min = int(np.argmin(d))
        out[f'candidate_xy_dist_mm_{name}'] = float(d[i_min]) * 1000.0
        if name == 'base':
            nearest_base_xy = cand[i_min]
    out['candidate_xy_near_same'] = out['candidate_xy_dist_mm_base'] <= NEAR_SAME_MM

    # --- 2) _unique_orientations に該当orientationが含まれるか ---
    uniq = planner_mod._unique_orientations(list(lwh))
    out['orientation_in_unique_list'] = orn_idx in uniq
    target_half = tuple(np.round(half, 5))
    out['orientation_equivalent_kept'] = any(
        tuple(np.round(geo.half_extent(lwh, k), 5)) == target_half for k in uniq)

    # --- 3) _apply_y_slice_filter (層規律, level0) ---
    n_slices = planner_mod.Y_SLICE_COUNT  # WALL_MODE既定False。有効時は_wall_slice_count依存になるため別記録
    out['wall_mode_active'] = planner_mod.WALL_MODE
    y_bounds = planner_mod._y_slice_bounds(container, n_slices)
    cand_arr = np.array([[local_x, local_y]])
    filtered0 = planner_mod._apply_y_slice_filter(cand_arr, half[1], y_bounds[0])
    out['y_slice_n_levels'] = n_slices
    out['y_slice_level0_pass'] = bool(filtered0.shape[0] > 0)
    out['y_slice_final_level_is_fully_open'] = True  # _y_slice_bounds の最終要素は常に全開放

    # --- 4) 優先コンテナのtier分離(情報記録。ハード除外にはならない) ---
    out['container_is_prioritized'] = bool(container.get('is_prioritized', False))
    out['item_is_prioritized'] = bool(item.get('is_prioritized', False))
    out['priority_clearance_could_apply'] = (
        not item.get('is_prioritized', False)
        and any(s[2] for s in supports)  # supportsタプルの3番目=sup_prioritized
    )

    # --- 5) _evaluate_candidates 本体(支持品質・内包・legal1/legal2一式) ---
    def _run_eval(xy):
        budget = planner_mod.SearchBudget(limit=1e15)
        stats = {}
        result = planner_mod._evaluate_candidates(
            container, item, half, obstacles, supports, np.array([xy]), budget,
            stats=stats, strict_support=strict_support, corridor_obstacles=corridor_obstacles)
        return {
            'result': 'success' if result is not None else 'fail',
            'stats': dict(stats),
            'score': (result['score'] if result is not None else None),
        }

    out['eval_exact_point'] = _run_eval((local_x, local_y))
    if nearest_base_xy is not None:
        out['eval_nearest_planner_candidate'] = _run_eval(tuple(nearest_base_xy))
    else:
        out['eval_nearest_planner_candidate'] = None

    return out


def _classify_first_failing_stage(trace: dict) -> str:
    """パイプライン順に見て、最初に「落選」となる段階を1つ選ぶ(報告集計用のラベル付け)。
    orientation/y_sliceは非ブロッキング(orientation_equivalent_keptがTrueなら幾何的には
    無害、y_sliceはlevel0で落ちても同一コンテナのfully-open levelで必ず再試行される)ため、
    真の合否は5)のeval_exact_pointのstatsを主に見る。あいまいさを残さないよう、
    ここでは「実測でどの関数が最初にFalseを返したか」をそのまま反映する。
    """
    if not trace['orientation_in_unique_list'] and not trace['orientation_equivalent_kept']:
        return 'unique_orientations'
    if not trace['y_slice_level0_pass']:
        # 非ブロッキング(fully-open levelで再試行される)だが、段階としては記録する
        stage = 'y_slice_level0_only'
    else:
        stage = None
    ex = trace['eval_exact_point']
    if ex['result'] == 'success':
        return stage or 'passes_all_tested_stages'
    st = ex['stats']
    for key in ('fail_support', 'fail_inclusion', 'fail_ceiling',
                'fail_inclusion_and_ceiling', 'fail_transport_y', 'fail_transport_x'):
        if st.get(key):
            return key
    return 'fail_unclassified'


def main():
    args = parse_args()
    all_scenes = {}
    for fp in args.inputs:
        all_scenes.update(json.load(open(fp)))

    target_scenes = {k: v for k, v in all_scenes.items()
                      if v.get('status') == 'ok' and v.get('exhaustive_n_findings', 0) > 0}
    print(f'target scenes with legal solutions (Phase64): {len(target_scenes)}')

    out_path = args.out
    output = {}
    if os.path.exists(out_path):
        try:
            output = json.load(open(out_path))
            print(f'resuming: {len(output)} scenes already in {out_path}')
        except Exception:
            output = {}

    agent_module = '.'.join(args.module_path.split('/')) + 'agent'
    n_done_this_run = 0
    for label, scene_result in target_scenes.items():
        if label in output and output[label].get('status') == 'ok':
            continue
        t0 = time.perf_counter()
        try:
            cp = _config_path_for_label(label)
            tk = label.split('::', 1)[1]
            task = json.load(open(cp))[tk]
            death = _reach_death(task, args.module_path, agent_module)
            if death is None:
                output[label] = {'status': 'death_not_reproduced'}
            else:
                obs = death['obs_before']
                container_list = obs['container_list']
                pool_list = obs['pool_list']
                strict_support = not death['optimize_flag']
                prepacked_ids = death['prepacked_ids']

                traces = []
                for f in scene_result['exhaustive_findings_sample']:
                    container = _find_by_index(container_list, f['container_index'])
                    item = _find_by_index(pool_list, f['item_index'])
                    if container is None or item is None:
                        traces.append({'finding': f, 'error': 'container_or_item_not_found'})
                        continue
                    tr = trace_one_finding(container, item, f['orientation'], f['pos_world'],
                                            prepacked_ids, strict_support)
                    tr['finding'] = f
                    tr['first_failing_stage'] = _classify_first_failing_stage(tr)
                    traces.append(tr)

                output[label] = {
                    'status': 'ok',
                    'strict_support': strict_support,
                    'n_placed_at_death': death['n_placed_at_death'],
                    'n_traced': len(traces),
                    'traces': traces,
                }
        except Exception:
            output[label] = {'status': f'error: {traceback.format_exc().splitlines()[-1]}'}
        elapsed = time.perf_counter() - t0
        status = output[label]['status']
        print(f'[{label}] {status} ({elapsed:.1f}s)')
        n_done_this_run += 1
        if n_done_this_run % args.checkpoint_every == 0:
            with open(out_path, 'w') as fh:
                json.dump(output, fh, indent=2, default=str)

    with open(out_path, 'w') as fh:
        json.dump(output, fh, indent=2, default=str)
    print(f'wrote {out_path}')

    # --- 集計サマリをstdoutに出す ---
    stage_counter = Counter()
    per_scene_top_stage = {}
    for label, res in output.items():
        if res.get('status') != 'ok':
            continue
        local_counter = Counter()
        for tr in res['traces']:
            if 'first_failing_stage' in tr:
                stage_counter[tr['first_failing_stage']] += 1
                local_counter[tr['first_failing_stage']] += 1
        if local_counter:
            per_scene_top_stage[label] = local_counter.most_common()

    print('\n=== 最初に落とした段階の内訳(全シーン合計) ===')
    for stage, n in stage_counter.most_common():
        print(f'  {stage}: {n}')

    print('\n=== シーンごとの内訳 ===')
    for label, counts in per_scene_top_stage.items():
        print(f'  {label}: {counts}')


if __name__ == '__main__':
    main()
