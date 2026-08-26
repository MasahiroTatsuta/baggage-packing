"""Phase69: results/phase69_breakdown_2G[_sample].json から事実を集計する(読み取り専用)。

仮説検証ではなく記述統計のみ: disp/energy内訳、変位の集中度、属性別平均、
相関係数(Pearson)、シーン単位の傾向差、cog-stability相関を出す。
"""
import json
import math
import sys

import numpy as np


def load_all(paths):
    out = {}
    for p in paths:
        d = json.load(open(p))
        out.update(d)
    return out


def pearson(xs, ys):
    xs = np.array(xs, dtype=float)
    ys = np.array(ys, dtype=float)
    mask = ~(np.isnan(xs) | np.isnan(ys))
    xs, ys = xs[mask], ys[mask]
    if len(xs) < 3 or np.std(xs) == 0 or np.std(ys) == 0:
        return float('nan'), len(xs)
    r = float(np.corrcoef(xs, ys)[0, 1])
    return r, len(xs)


def main():
    paths = sys.argv[1:] or ['results/phase69_breakdown_2G.json', 'results/phase69_breakdown_2G_sample.json']
    scenes = load_all(paths)
    ok_scenes = {k: v for k, v in scenes.items() if v.get('status') == 'ok'}
    print(f'=== シーン数: {len(scenes)} (status=ok: {len(ok_scenes)}) ===')
    for k, v in scenes.items():
        if v.get('status') != 'ok':
            print(f'  NG: {k}: {v.get("status")}')

    # --- (1-1) disp/energy内訳(シーン平均) ---
    disp_scores = [v['disp_score'] for v in ok_scenes.values()]
    energy_scores = [v['energy_score'] for v in ok_scenes.values()]
    stab_scores = [v['stability_score'] for v in ok_scenes.values()]
    print('\n=== (1-1) disp/energy内訳(28シーン平均) ===')
    print(f'stability_score平均: {np.mean(stab_scores):.2f} (σ={np.std(stab_scores):.2f})')
    print(f'disp_score平均: {np.mean(disp_scores):.2f} (σ={np.std(disp_scores):.2f}) '
          f'-> 0.7倍で寄与 {0.7*np.mean(disp_scores):.2f}')
    print(f'energy_score平均: {np.mean(energy_scores):.2f} (σ={np.std(energy_scores):.2f}) '
          f'-> 0.3倍で寄与 {0.3*np.mean(energy_scores):.2f}')
    # 100からの減点をdisp/energyそれぞれの寄与に分解
    disp_deduction = [0.7 * (100 - d) for d in disp_scores]
    energy_deduction = [0.3 * (100 - e) for e in energy_scores]
    total_deduction = [d + e for d, e in zip(disp_deduction, energy_deduction)]
    disp_share = [d / t if t > 1e-9 else float('nan') for d, t in zip(disp_deduction, total_deduction)]
    print(f'減点(100-stability)平均: {np.mean(total_deduction):.2f}')
    print(f'  うちdisp由来の減点平均: {np.mean(disp_deduction):.2f} '
          f'({np.nanmean(disp_share)*100:.1f}%がdisp由来、シーン平均のシーン別比率の平均)')
    print(f'  うちenergy由来の減点平均: {np.mean(energy_deduction):.2f}')

    # --- 全アイテムをプールして荷物単位の集計 ---
    all_items = []
    for scene_label, v in ok_scenes.items():
        for it in v.get('items', []):
            row = dict(it)
            row['scene'] = scene_label
            row['scene_stability'] = v['stability_score']
            row['scene_cog'] = v.get('cog_score')
            all_items.append(row)
    print(f'\n全荷物数(プール): {len(all_items)}')

    disps = np.array([r['disp'] for r in all_items])
    order = np.argsort(-disps)
    n = len(disps)
    top10 = max(1, n // 10)
    top10_share = disps[order[:top10]].sum() / disps.sum() if disps.sum() > 0 else float('nan')
    top1pct = max(1, n // 100)
    top1_share = disps[order[:top1pct]].sum() / disps.sum() if disps.sum() > 0 else float('nan')
    median_disp = np.median(disps)
    print('\n=== (1-2) 変位の集中度 ===')
    print(f'disp平均={disps.mean():.4f}m 中央値={median_disp:.4f}m 最大={disps.max():.4f}m 標準偏差={disps.std():.4f}m')
    print(f'上位10%(n={top10})の荷物が全変位合計に占める割合: {top10_share*100:.1f}%')
    print(f'上位1%(n={top1pct})の荷物が全変位合計に占める割合: {top1_share*100:.1f}%')
    print(f'disp > 平均+2σ の荷物数: {int((disps > disps.mean()+2*disps.std()).sum())}/{n}')
    print(f'disp > 0.05m(5cm)の荷物数: {int((disps > 0.05).sum())}/{n}  '
          f'disp > 0.10m: {int((disps > 0.10).sum())}/{n}')

    # --- 属性別: 上位10% vs 残り90% ---
    top_idx = set(order[:top10].tolist())
    top_items = [all_items[i] for i in top_idx]
    rest_items = [all_items[i] for i in range(n) if i not in top_idx]

    def summarize_group(items, key, is_bool=False):
        vals = [it[key] for it in items if it.get(key) is not None]
        if not vals:
            return float('nan')
        if is_bool:
            return float(np.mean([1.0 if v else 0.0 for v in vals]))
        return float(np.mean(vals))

    print('\n=== (1-2) 大きく動いた荷物(上位10%, n={}) vs 残り(n={}) の属性比較 ==='.format(len(top_items), len(rest_items)))
    for key, is_bool in [('mass', False), ('length', False), ('width', False), ('height', False),
                         ('is_soft', True), ('is_prioritized', True),
                         ('order_frac_in_container', False), ('n_items_in_container', False),
                         ('world_z', False), ('dist_to_door', False),
                         ('n_support_items', False), ('on_floor_or_shelf', True),
                         ('contact_footprint_ratio', False), ('n_contact_points_bottom', False),
                         ('container_is_prioritized', True)]:
        top_v = summarize_group(top_items, key, is_bool)
        rest_v = summarize_group(rest_items, key, is_bool)
        print(f'  {key:28s} 上位10%平均={top_v:.4f}  残り90%平均={rest_v:.4f}  差={top_v-rest_v:+.4f}')

    # --- (1-3) 相関 ---
    print('\n=== (1-3) 変位量との相関(Pearson r, 全荷物プール) ===')
    for key, label in [('world_z', '重心高さ(z)'),
                        ('dist_to_door', '扉からの距離'),
                        ('n_support_items', '支えている荷物数'),
                        ('contact_footprint_ratio', '接地面積比(凸包近似)'),
                        ('n_contact_points_bottom', '底面接触点数'),
                        ('order_frac_in_container', 'コンテナ内配置順(0=最初,1=最後)'),
                        ('mass', '質量')]:
        xs = [it.get(key) for it in all_items]
        ys = [it['disp'] for it in all_items]
        xs2 = [x for x in xs if x is not None]
        if len(xs2) != len(xs):
            continue
        r, cnt = pearson(xs, ys)
        print(f'  disp vs {label:28s} r={r:+.3f} (n={cnt})')

    # is_soft, is_prioritized, on_floor_or_shelf (bool) は0/1相関で
    for key, label in [('is_soft', 'is_soft'), ('is_prioritized', 'is_prioritized'),
                        ('on_floor_or_shelf', '床/棚に直接接触')]:
        xs = [1.0 if it.get(key) else 0.0 for it in all_items]
        ys = [it['disp'] for it in all_items]
        r, cnt = pearson(xs, ys)
        print(f'  disp vs {label:28s} r={r:+.3f} (n={cnt})')

    # --- (1-4) シーン単位 ---
    print('\n=== (1-4) シーン単位の一覧(stability昇順) ===')
    scene_rows = []
    for label, v in ok_scenes.items():
        items = v.get('items', [])
        d = np.array([it['disp'] for it in items]) if items else np.array([0.0])
        top10n = max(1, len(d) // 10)
        ordr = np.argsort(-d)
        share = d[ordr[:top10n]].sum() / d.sum() if d.sum() > 0 else float('nan')
        scene_rows.append({
            'scene': label, 'stability': v['stability_score'], 'cog': v.get('cog_score'),
            'disp_score': v['disp_score'], 'energy_score': v['energy_score'],
            'mean_disp': v['mean_disp'], 'max_disp': float(d.max()),
            'top10_share': share, 'n_items': v.get('n_items'),
        })
    scene_rows.sort(key=lambda r: r['stability'])
    for r in scene_rows:
        print(f"  {r['scene']:45s} stability={r['stability']:6.2f} cog={r['cog']:6.2f} "
              f"disp_score={r['disp_score']:6.2f} energy_score={r['energy_score']:6.2f} "
              f"top10share={r['top10_share']*100:5.1f}% n={r['n_items']}")

    print('\n=== (1-4) cog_score と stability_score のシーン単位相関 ===')
    cogs = [r['cog'] for r in scene_rows]
    stabs = [r['stability'] for r in scene_rows]
    r, cnt = pearson(cogs, stabs)
    print(f'  r={r:+.3f} (n={cnt})')

    low = sorted(scene_rows, key=lambda r: r['stability'])[:7]
    high = sorted(scene_rows, key=lambda r: -r['stability'])[:7]
    print('\n最も低いstability 7シーン:')
    for r in low:
        print(f"  {r['scene']:45s} stability={r['stability']:6.2f} cog={r['cog']:6.2f} top10share={r['top10_share']*100:5.1f}%")
    print('最も高いstability 7シーン:')
    for r in high:
        print(f"  {r['scene']:45s} stability={r['stability']:6.2f} cog={r['cog']:6.2f} top10share={r['top10_share']*100:5.1f}%")

    with open('results/phase69_analysis_summary.json', 'w') as f:
        json.dump({'scene_rows': scene_rows}, f, indent=2, ensure_ascii=False)


if __name__ == '__main__':
    main()
