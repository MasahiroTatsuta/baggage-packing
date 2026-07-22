"""
tools/cmp_regime.py

tools/measure_regime.py の出力JSONを複数並べて、シーン別・スイート平均の
fill_strict / fill_loose / stability / placement / soft を横並び比較する。

実行例:
    .venv/bin/python tools/cmp_regime.py results/phase14_bsweep_w*.json
"""
import argparse
import json
import os

KEYS = ['fill_strict', 'fill_loose', 'stability_score', 'placement_score', 'soft_item_score']


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('files', nargs='+')
    ap.add_argument('--keys', nargs='+', default=KEYS)
    args = ap.parse_args()

    data = []
    for f in args.files:
        with open(f) as fh:
            data.append((os.path.basename(f).replace('.json', ''), json.load(fh)))

    labels = data[0][1]['scene_labels']
    for key in args.keys:
        print(f'\n===== {key} =====')
        head = f'{"scene":34s}' + ''.join(f'{n[:16]:>18s}' for n, _ in data)
        print(head)
        for lab in labels:
            row = f'{lab.replace("suite_", "").replace(".json", ""):34s}'
            for _, d in data:
                s = d['per_scene'].get(lab)
                if s is None or s[key]['n'] == 0:
                    row += f'{"-":>18s}'
                else:
                    row += f'{s[key]["mean"]:>12.2f}±{s[key]["std"]:<5.2f}'
            print(row)
        row = f'{"** SUITE AVG **":34s}'
        for _, d in data:
            st = d['suite_stats'][key]
            row += f'{st["mean"]:>12.2f}±{st["std"]:<5.2f}'
        print(row)

    print('\n===== placed items (last repeat) / statuses =====')
    for n, d in data:
        print(f'  {n}: repeats={d["repeats"]} elapsed={d["elapsed_sec"]:.0f}s')


if __name__ == '__main__':
    main()
