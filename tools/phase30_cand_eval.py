"""
tools/phase30_cand_eval.py

Phase30 計測3: `tools/phase29_cand.py` が記録した全候補順序(21シーン・130件)を
`tools/phase29_order_eval.py::run_with_order`(本物の env/evaluator)に流し、
各候補の fill_strict と cog_score を得る。

新規の探索(optimize/build_order の再実行)は一切行わない —— 記録済みの `order` を
そのまま env に注入して1エピソード分の実行(setup+テストされた順序の再生)を行うだけ。
agents/mysolver/ は変更しない(hooked 呼び出しも無い素の Agent 実行)。

実行:
    PYTHONPATH=. .venv/bin/python tools/phase30_cand_eval.py \
        --cand results/phase29_cand_g1.json results/phase29_cand_g2.json \
        --out results/phase30_cand_eval.json
"""
import argparse
import json
import os
import sys
import time

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from tools.phase29_order_eval import run_with_order


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--cand', nargs='+', required=True)
    ap.add_argument('--module-path', default='agents/mysolver/')
    ap.add_argument('--out', default=None)
    ap.add_argument('--resume', action='store_true', help='--out が既にあれば続きから')
    args = ap.parse_args()
    agent_module = '.'.join(args.module_path.split('/')) + 'agent'

    cands = []
    for f in args.cand:
        cands.extend(json.load(open(f)))
    cands.sort(key=lambda c: c['label'])  # シーンID昇順(機械的な規則)

    done = {}
    if args.resume and args.out and os.path.exists(args.out):
        prev = json.load(open(args.out))
        for r in prev:
            done[(r['label'], r['cand_idx'])] = r
        print(f'resume: {len(done)}件は評価済み', flush=True)

    rows = list(done.values())
    t0 = time.perf_counter()
    n_total = sum(c['n_cand'] for c in cands)
    n_done = len(rows)
    for c in cands:
        label = c['label']
        task = list(json.load(open(f"configs/gen/suite_{label}.json")).values())[0]
        for idx, rec in enumerate(c['records']):
            if (label, idx) in done:
                continue
            t1 = time.perf_counter()
            r = run_with_order(task, args.module_path, agent_module, rec['order'])
            r.update({'label': label, 'cand_idx': idx, 'phase': rec['phase'],
                      'shadow_placed_volume': rec['placed_volume'],
                      'shadow_risk_vol': rec['risk_vol'],
                      'sec': time.perf_counter() - t1})
            rows.append(r)
            n_done += 1
            elapsed = time.perf_counter() - t0
            print(f"[{n_done}/{n_total}] {label:28s} cand{idx} "
                  f"fill_strict={r.get('fill_strict', float('nan')):6.2f} "
                  f"cog={r.get('cog_score', float('nan')):6.2f} "
                  f"({r['sec']:.1f}s, total {elapsed/60:.1f}min)", flush=True)
            if args.out:
                json.dump(rows, open(args.out, 'w'), indent=1, ensure_ascii=False)


if __name__ == '__main__':
    main()
