"""
tools/hist_ladder.py  (Phase10 タスク2の補助)

提出履歴の各時点のローカル5指標を「git履歴から復元」するための計測ドライバ。
各フェーズcommitの agents/mysolver/ を _hist/pN/ に取り出したものを、
phase4以降ずっと存在する固定6シーン(= sample_config の 000/001 +
gen_2containers_patternB / gen_2containers_priority / gen_shelf_patternA /
gen_manyitems_patternA)に対して**その時点のネイティブ・フル予算**で1回ずつ実行し、
fill/cog/stab/place/soft のシーン平均を出す。

これにより、背景で与えられた fill アンカー(phase7≒24.40, phase8≒25.21, phase9≒27.14)
と突き合わせて再現性を検証しつつ、cog を含む5指標の時系列を復元する。

src/ configs/ は読み取りのみ。agents/mysolver/ は変更しない(_hist/ は別ディレクトリ)。
"""
import json
import os
import sys
import time

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
from tools.local_eval import run_one_scene

METRIC_KEYS = ['fill_score', 'cog_score', 'stability_score', 'placement_score', 'soft_item_score']

# phase4以降ずっと存在する固定6シーン(= 6シーン平均の母集団)
SIX_SCENES = [
    ('configs/sample_config.json', '000'),
    ('configs/sample_config.json', '001'),
    ('configs/gen/gen_2containers_patternB.json', None),
    ('configs/gen/gen_2containers_priority.json', None),
    ('configs/gen/gen_shelf_patternA.json', None),
    ('configs/gen/gen_manyitems_patternA.json', None),
]

# 復元対象の各フェーズ agent (module_path, ラベル)
AGENTS = [
    ('_hist/p4/', 'phase4'),
    ('_hist/p6/', 'phase6'),
    ('_hist/p7/', 'phase7'),
    ('_hist/p8/', 'phase8'),
    ('_hist/p9/', 'phase9(current)'),
]


def load_scenes():
    specs = []
    for path, tid in SIX_SCENES:
        with open(path) as f:
            cfg = json.load(f)
        if tid is None:
            for k in cfg:
                specs.append((f'{os.path.basename(path)}::{k}', cfg[k]))
        else:
            specs.append((f'{os.path.basename(path)}::{tid}', cfg[tid]))
    return specs


def main():
    scenes = load_scenes()
    print(f'6シーン: {[s[0] for s in scenes]}')
    out = {'six_scenes': [s[0] for s in scenes], 'agents': {}}

    for module_path, label in AGENTS:
        agent_module = '.'.join(module_path.rstrip('/').split('/')) + '.agent'
        print(f'\n===== {label} ({module_path} -> {agent_module}) =====')
        per_scene = {}
        metric_lists = {k: [] for k in METRIC_KEYS}
        for scene_label, task_config in scenes:
            t0 = time.time()
            res = run_one_scene(task_config, module_path, agent_module, None, False)
            m = res['metrics']
            if m is None:
                print(f'  [{scene_label}] ERROR: {res["status"]}')
                per_scene[scene_label] = {'status': res['status'], 'metrics': None}
                continue
            per_scene[scene_label] = {k: m[k] for k in METRIC_KEYS}
            per_scene[scene_label]['placed'] = f'{m.get("num_placed_items_abs",0)}/{m.get("total_items",0)}'
            for k in METRIC_KEYS:
                metric_lists[k].append(m[k])
            print(f'  [{scene_label}] fill={m["fill_score"]:.2f} cog={m["cog_score"]:.2f} '
                  f'stab={m["stability_score"]:.2f} place={m["placement_score"]:.2f} '
                  f'soft={m["soft_item_score"]:.2f} ({time.time()-t0:.1f}s)')
        avg = {k: (sum(v)/len(v) if v else float('nan')) for k, v in metric_lists.items()}
        out['agents'][label] = {'module_path': module_path, 'per_scene': per_scene, 'six_scene_avg': avg}
        print(f'  -- 6シーン平均: ' + ' '.join(f'{k.split("_")[0]}={avg[k]:.2f}' for k in METRIC_KEYS))

    with open('results/phase10_hist_ladder.json', 'w') as f:
        json.dump(out, f, ensure_ascii=False, indent=2)
    print('\n=== 6シーン平均サマリ ===')
    print(f'{"agent":16s} | ' + ' '.join(f'{k.split("_")[0]:>7s}' for k in METRIC_KEYS))
    for label, d in out['agents'].items():
        a = d['six_scene_avg']
        print(f'{label:16s} | ' + ' '.join(f'{a[k]:7.2f}' for k in METRIC_KEYS))
    print('\n出力: results/phase10_hist_ladder.json')


if __name__ == '__main__':
    main()
