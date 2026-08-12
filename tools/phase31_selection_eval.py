"""
tools/phase31_selection_eval.py

Phase31 ステップ1(risk_vol の床面限定案): `tools/phase29_cand.py` が記録した全候補順序
(21シーン・130件)について、`agents.mysolver.simulate.simulate_order` を
`RISK_SLACK_FACES='floor'` で**再生**し(構築は一切やり直さない。既に確定している
`order` を流すだけ)、新しい risk_vol を得る。それを使って `ordering._better` と同じ
選択則で「勝者」を選び直し、その勝者の**本物の** fill_strict を
`results/phase30_cand_eval.json`(既存・新規ロールアウト不要)から引いて、
現行(all)選択との差を測る。

検証として RISK_SLACK_FACES='all' でも再生し、記録済みの risk_vol と一致することを
確認する(再生パイプラインの妥当性チェック)。

実行:
    MYSOLVER_UNITS_PER_SEC=1.55e7 PYTHONPATH=. .venv/bin/python tools/phase31_selection_eval.py \
        --cand results/phase29_cand_g1.json results/phase29_cand_g2.json \
        --real results/phase30_cand_eval.json \
        --out results/phase31_selection_eval.json
"""
import argparse
import io
import json
import os
import statistics
import sys
from contextlib import redirect_stdout

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from src.ground_handling.env import GroundHandlingEnv
from agents.mysolver import geometry as geo
from agents.mysolver import ordering as ordering_mod
from agents.mysolver import planner
from agents.mysolver import simulate as simulate_mod


def scene_path(label):
    return f'configs/gen/suite_{label}.json'


def load_scene(label):
    task = list(json.load(open(scene_path(label))).values())[0]
    env = GroundHandlingEnv(config=task, verbose=False, render_mode=None)
    try:
        env.reset_settings()
        init = env.get_init_states()
        container_list = init['container_list']
        lookahead = init['lookahead_k']
        items = env.get_info_for_optimization()
    finally:
        try:
            env.close()
        except Exception:
            pass
    items_by_index = {it['index']: it for it in items}
    total_vol = sum(c.get('volume', 0.0) for c in container_list)
    prepacked_ids = geo.initial_prepacked_ids(container_list)
    return container_list, items_by_index, lookahead, total_vol, prepacked_ids


def replay_one(container_list, items_by_index, order, lookahead, prepacked_ids, faces):
    simulate_mod.RISK_SLACK_FACES = faces
    budget = planner.SearchBudget.from_seconds(600.0)
    buf = io.StringIO()
    with redirect_stdout(buf):
        placed_ids, placed_volume, risk_vol, viol, srisk = simulate_mod.simulate_order(
            container_list, items_by_index, order, max(1, int(lookahead or 1)), budget,
            prepacked_ids=prepacked_ids, stability_weight=ordering_mod.STABILITY_PENALTY_WEIGHT)
    return {'n_placed': len(placed_ids), 'placed_volume': placed_volume,
            'risk_vol': risk_vol, 'violation_ratio': viol, 'stability_risk': srisk}


def score_of(rec, total_vol):
    pw = ordering_mod.PLACEMENT_PENALTY_WEIGHT
    return (rec['risk_vol'] - pw * total_vol * rec['violation_ratio'], rec['n_placed'])


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--cand', nargs='+', required=True)
    ap.add_argument('--real', required=True)
    ap.add_argument('--out', default=None)
    args = ap.parse_args()

    cands = []
    for f in args.cand:
        cands.extend(json.load(open(f)))
    cands.sort(key=lambda c: c['label'])

    real_rows = json.load(open(args.real))
    real_by_label = {}
    for r in real_rows:
        real_by_label.setdefault(r['label'], {})[r['cand_idx']] = r

    rows = []
    validation_max_diff = 0.0
    for c in cands:
        label = c['label']
        container_list, items_by_index, lookahead, total_vol, prepacked_ids = load_scene(label)

        floor_recs = []
        all_recs = []
        for rec in c['records']:
            fr = replay_one(container_list, items_by_index, rec['order'], lookahead,
                             prepacked_ids, 'floor')
            ar = replay_one(container_list, items_by_index, rec['order'], lookahead,
                             prepacked_ids, 'all')
            floor_recs.append(fr)
            all_recs.append(ar)
            diff = abs(ar['risk_vol'] - rec['risk_vol'])
            validation_max_diff = max(validation_max_diff, diff)

        # 現行(all, 記録済みrisk_vol)勝者
        old_scores = [score_of(rec, total_vol) for rec in c['records']]
        old_winner = max(range(len(old_scores)), key=lambda i: old_scores[i])
        assert old_winner == c['winner_idx'], (label, old_winner, c['winner_idx'])

        # floor勝者
        floor_scores = [score_of(rec, total_vol) for rec in floor_recs]
        floor_winner = max(range(len(floor_scores)), key=lambda i: floor_scores[i])

        real_old = real_by_label[label][old_winner]
        real_floor = real_by_label[label][floor_winner]
        gap = real_floor['fill_strict'] - real_old['fill_strict']

        row = {'label': label, 'n_cand': len(c['records']),
               'old_winner': old_winner, 'floor_winner': floor_winner,
               'changed': old_winner != floor_winner,
               'old_real_fill': real_old['fill_strict'],
               'floor_real_fill': real_floor['fill_strict'],
               'gap': gap,
               'floor_real_fill_loose': real_floor['fill_loose'],
               'old_real_fill_loose': real_old['fill_loose'],
               'floor_placement': real_floor['placement_score'],
               'floor_soft': real_floor['soft_item_score'],
               'floor_stability': real_floor['stability_score']}
        rows.append(row)
        flag = '  <-- changed' if row['changed'] else ''
        print(f"{label:28s} old=c{old_winner} floor=c{floor_winner} "
              f"real_fill {real_old['fill_strict']:6.2f} -> {real_floor['fill_strict']:6.2f} "
              f"({gap:+6.2f}) placement={real_floor['placement_score']:5.1f}{flag}", flush=True)
        if args.out:
            json.dump(rows, open(args.out, 'w'), indent=1, ensure_ascii=False)

    print(f"\n再生パイプライン検証: all再生とrecorded risk_volの最大差 = {validation_max_diff:.2e}"
          " (浮動小数点丸めの範囲内なら妥当)")

    gaps = [r['gap'] for r in rows]
    n_changed = sum(1 for r in rows if r['changed'])
    n_pos = sum(1 for r in rows if r['gap'] > 1e-9)
    n_neg = sum(1 for r in rows if r['gap'] < -1e-9)
    print(f"\n変わったシーン: {n_changed}/{len(rows)}  (gap>0: {n_pos} / gap<0: {n_neg})")
    print(f"21シーン合計gap = {sum(gaps):.3f}")
    print(f"26シーン換算 = {sum(gaps)/26:.4f}")
    print(f"σ(21シーン) = {statistics.pstdev(gaps):.3f}  SE(/sqrt(26)) = {statistics.pstdev(gaps)/(26**0.5):.3f}")
    for r in rows:
        if r['changed']:
            print(f"  {r['label']:28s} gap={r['gap']:+6.2f}  placement={r['floor_placement']:5.1f} "
                  f"soft={r['floor_soft']:5.1f} stability={r['floor_stability']:6.2f}")


if __name__ == '__main__':
    main()
