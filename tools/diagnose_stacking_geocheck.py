"""`tools/diagnose_stacking.py` の出力(--no-geo を付けずに実行したもの)に対して、
pybulletの接触判定(`Scorer._find_stacking_pairs`)とは独立な幾何的クロスチェックを行う。

各荷物の pos/orn/寸法から world AABB を計算し、同一コンテナ内の全ペアについて
(a) XY footprint が重なるか、(b) 下側候補の天面zと上側候補の底面zの隙間(gap)、
を求める。gapがほぼ0(実接触相当)のペアの中に is_prioritized/is_soft の「下敷き」が
無いかを、`_find_stacking_pairs` の検出結果(diagnose_stacking.py が記録した
n_stacking_pairs 等)と突き合わせるための材料を出す。

「ロジックを読んだだけの妥当に見える判断」で終わらせず、実データで検出漏れの有無を
確認するためのツール(results/phase44_report.md §2-2追記、results/phase45_report.md 参照)。

実行方法(リポジトリルートで):
    python tools/diagnose_stacking_geocheck.py results/phase45_stacking_diag_off.json
    # シーンを明示する場合
    python tools/diagnose_stacking_geocheck.py results/phase45_stacking_diag_off.json \\
        --scene suite_B04_2c_80_noprio.json::000
    # 省略時は n_placed が最大のシーン(最も荷物が密なシーン)を自動選択する
"""
import argparse
import json

import numpy as np


def parse_args():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('diag_json', help='tools/diagnose_stacking.py の出力JSON')
    p.add_argument('--scene', default=None,
                    help='突き合わせ対象シーンのラベル(既定: n_placed最大のシーンを自動選択)')
    p.add_argument('--gap-threshold', type=float, default=0.02,
                    help='「本当に接触している」とみなすgapの閾値[m](既定0.02、safety_margin=0.015の近傍)')
    return p.parse_args()


def quat_to_rotmat(q):
    x, y, z, w = q
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
    ])


def world_aabb(item):
    hl, hw, hh = item['length'] / 2, item['width'] / 2, item['height'] / 2
    corners_local = np.array([[sx * hl, sy * hw, sz * hh]
                               for sx in (-1, 1) for sy in (-1, 1) for sz in (-1, 1)])
    rot = quat_to_rotmat(item['orn']) if item['orn'] is not None else np.eye(3)
    corners_world = corners_local @ rot.T + np.array(item['pos'])
    return corners_world.min(axis=0), corners_world.max(axis=0)


def main():
    args = parse_args()
    with open(args.diag_json) as f:
        d = json.load(f)

    scene = args.scene
    if scene is None:
        candidates = [(k, v) for k, v in d.items() if v.get('status') == 'ok' and 'geo' in v]
        if not candidates:
            raise SystemExit('geoフィールドを含む有効なシーンが無い(--no-geoで実行された可能性)')
        scene = max(candidates, key=lambda kv: kv[1]['n_placed'])[0]

    if scene not in d or 'geo' not in d[scene]:
        raise SystemExit(f'{scene} に geo フィールドが無い(--no-geo で実行された可能性)')

    geo = d[scene]['geo']
    print(f'シーン: {scene}  積載数: {len(geo)}')
    print(f'_find_stacking_pairs が検出したペア数(pybullet接触判定): {d[scene]["n_stacking_pairs"]}')
    print(f'episode_status: {d[scene].get("episode_status")}')

    aabbs = {it['index']: world_aabb(it) for it in geo}

    by_container = {}
    for it in geo:
        by_container.setdefault(it['container_index'], []).append(it)

    geo_candidates = []
    for items in by_container.values():
        for a in items:
            for b in items:
                if a is b:
                    continue
                amin, amax = aabbs[a['index']]
                bmin, bmax = aabbs[b['index']]
                ox = min(amax[0], bmax[0]) - max(amin[0], bmin[0])
                oy = min(amax[1], bmax[1]) - max(amin[1], bmin[1])
                if ox <= 0 or oy <= 0:
                    continue
                gap = bmin[2] - amax[2]
                if gap < -0.001:
                    continue
                geo_candidates.append({
                    'bottom': a['index'], 'top': b['index'], 'gap': gap,
                    'xy_overlap_area': ox * oy,
                    'bottom_prio': a['is_prioritized'], 'top_prio': b['is_prioritized'],
                    'bottom_soft': a['is_soft'], 'top_soft': b['is_soft'],
                })

    print(f'\n幾何的候補(XY重なりあり かつ 上下関係が矛盾しない)ペア総数: {len(geo_candidates)}')
    for th in (0.005, 0.01, 0.02, 0.03, 0.05, 0.10):
        n = sum(1 for c in geo_candidates if c['gap'] <= th)
        print(f'  gap<={th:.3f}m: {n}件')

    close_pairs = [c for c in geo_candidates if c['gap'] <= args.gap_threshold]
    print(f'\n=== gap<={args.gap_threshold}m(「本当に接触している」とみなすペア)===')
    n_close_crushed = 0
    for c in sorted(close_pairs, key=lambda c: c['gap']):
        crushed = (c['bottom_prio'] and not c['top_prio']) or (c['bottom_soft'] and not c['top_soft'])
        if crushed:
            n_close_crushed += 1
        mark = ' <<< 下敷き候補' if crushed else ''
        print(f"  bottom=item{c['bottom']}(prio={c['bottom_prio']},soft={c['bottom_soft']}) "
              f"top=item{c['top']}(prio={c['top_prio']},soft={c['top_soft']}) "
              f"gap={c['gap']*1000:.1f}mm xy_overlap={c['xy_overlap_area']*1e4:.1f}cm^2{mark}")
    print(f'\n真に接触しているペア中、優先/ソフトの下敷きに該当するもの: {n_close_crushed}件')

    risky = [c for c in geo_candidates
             if (c['bottom_prio'] and not c['top_prio']) or (c['bottom_soft'] and not c['top_soft'])]
    print(f'\nprio/softの「下敷き」に該当しうる幾何的候補(gap不問、XY重なりのみ条件): {len(risky)}件')
    for c in sorted(risky, key=lambda c: c['gap'])[:20]:
        print(f"  bottom=item{c['bottom']}(prio={c['bottom_prio']},soft={c['bottom_soft']}) "
              f"top=item{c['top']}(prio={c['top_prio']},soft={c['top_soft']}) gap={c['gap']*1000:.1f}mm")


if __name__ == '__main__':
    main()
