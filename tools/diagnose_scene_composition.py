"""configs/gen の26シーンについて、**アイテムプール全体**(配置された荷物ではなく
シーン定義に含まれる全荷物)の構成を集計する(Phase46 ステップ1-1)。

`tools/diagnose_stacking.py` が測る「配置された」個数(Phase44 §2-1)とは対象が異なる。
本ツールは env を一切実行せず configs/gen/*.json を読むだけなので高速(数秒で終わる)。

集計する項目:
  - is_prioritized の個数・全体比率
  - is_soft の個数・全体比率
  - is_soft かつ重い(全アイテム中の質量上位25%)アイテムの個数
  - コンテナの is_prioritized / require_shelf の有無(シーンごと)

`--placed-json` に `tools/diagnose_stacking.py` の出力を渡すと、配置されなかった
アイテムに優先/ソフトが偏っていないか(Phase46 §1-2)も合わせて報告する。

実行方法(リポジトリルートで):
    python tools/diagnose_scene_composition.py
    python tools/diagnose_scene_composition.py --placed-json results/phase45_stacking_diag_wallbound.json

読み取り専用(configs/gen は変更しない)。
"""
import argparse
import glob
import json
import os


def parse_args():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('--config-path', default='configs/gen/suite_*.json')
    p.add_argument('--placed-json', default=None,
                    help='tools/diagnose_stacking.py の出力JSON(配置済み個数との突き合わせ用、省略可)')
    return p.parse_args()


def main():
    args = parse_args()
    placed_data = {}
    if args.placed_json:
        with open(args.placed_json) as f:
            placed_data = json.load(f)

    rows = []
    for cp in sorted(glob.glob(args.config_path)):
        with open(cp) as f:
            cfg = json.load(f)
        for task_id, task in cfg.items():
            label = f'{os.path.basename(cp)}::{task_id}'
            items = task['item_stream']['item_list']
            n_total = len(items)
            n_prio = sum(1 for it in items if it.get('is_prioritized'))
            n_soft = sum(1 for it in items if it.get('is_soft'))
            masses = sorted((it['mass'] for it in items), reverse=True)
            heavy_threshold = masses[max(0, len(masses) // 4 - 1)] if masses else 0.0
            n_soft_heavy = sum(1 for it in items
                                if it.get('is_soft') and it['mass'] >= heavy_threshold)
            containers = task['containers']['container_list']
            n_prio_container = sum(1 for c in containers if c.get('is_prioritized'))
            n_shelf_container = sum(1 for c in containers if c.get('require_shelf'))

            row = {
                'scene': label, 'n_total': n_total,
                'n_prio': n_prio, 'prio_ratio': n_prio / n_total if n_total else 0.0,
                'n_soft': n_soft, 'soft_ratio': n_soft / n_total if n_total else 0.0,
                'n_soft_heavy': n_soft_heavy,
                'n_containers': len(containers),
                'n_prio_container': n_prio_container, 'n_shelf_container': n_shelf_container,
            }
            if label in placed_data:
                pd = placed_data[label]
                placed_prio = pd.get('n_prioritized_placed', 0)
                placed_soft = pd.get('n_soft_placed', 0)
                n_placed = pd.get('n_placed', 0)
                unplaced_prio = n_prio - placed_prio
                unplaced_soft = n_soft - placed_soft
                unplaced_total = n_total - n_placed
                row.update({
                    'n_placed': n_placed,
                    'placed_prio': placed_prio, 'unplaced_prio': unplaced_prio,
                    'placed_soft': placed_soft, 'unplaced_soft': unplaced_soft,
                    'unplaced_total': unplaced_total,
                    'unplaced_prio_ratio': unplaced_prio / unplaced_total if unplaced_total else None,
                    'unplaced_soft_ratio': unplaced_soft / unplaced_total if unplaced_total else None,
                })
            rows.append(row)

    has_placed = bool(placed_data)
    header = (f'{"scene":45s} {"total":6s} {"prio":5s} {"prio%":7s} {"soft":5s} {"soft%":7s} '
              f'{"softHeavy":9s} {"prioC":6s} {"shelfC":7s}')
    if has_placed:
        header += f' {"unplaced":9s} {"unplPrio%":10s} {"unplSoft%":10s}'
    print(header)

    tot_total = tot_prio = tot_soft = tot_soft_heavy = 0
    tot_unplaced = tot_unplaced_prio = tot_unplaced_soft = 0
    for r in rows:
        line = (f'{r["scene"]:45s} {r["n_total"]:<6} {r["n_prio"]:<5} {r["prio_ratio"]*100:6.1f}% '
                f'{r["n_soft"]:<5} {r["soft_ratio"]*100:6.1f}% {r["n_soft_heavy"]:<9} '
                f'{r["n_prio_container"]:<6} {r["n_shelf_container"]:<7}')
        if has_placed and 'unplaced_total' in r:
            up = r['unplaced_prio_ratio']
            us = r['unplaced_soft_ratio']
            line += (f' {r["unplaced_total"]:<9} '
                     f'{("%.1f%%" % (up*100)) if up is not None else "?":10s} '
                     f'{("%.1f%%" % (us*100)) if us is not None else "?":10s}')
            tot_unplaced += r['unplaced_total']
            tot_unplaced_prio += r['unplaced_prio']
            tot_unplaced_soft += r['unplaced_soft']
        print(line)
        tot_total += r['n_total']; tot_prio += r['n_prio']; tot_soft += r['n_soft']
        tot_soft_heavy += r['n_soft_heavy']

    print()
    print(f'合計: total={tot_total} prio={tot_prio}({100*tot_prio/tot_total:.1f}%) '
          f'soft={tot_soft}({100*tot_soft/tot_total:.1f}%) soft_heavy={tot_soft_heavy}')
    if has_placed and tot_unplaced:
        print(f'未配置合計: {tot_unplaced}件 (prio比率 {100*tot_unplaced_prio/tot_unplaced:.1f}%, '
              f'soft比率 {100*tot_unplaced_soft/tot_unplaced:.1f}%) '
              f'※プール全体の比率(prio {100*tot_prio/tot_total:.1f}% / soft {100*tot_soft/tot_total:.1f}%)と比較すること')


if __name__ == '__main__':
    main()
