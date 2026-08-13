"""
tools/phase32_borda_eval.py

Phase32 ステップ2: risk_vol argmax(単一指標)を、4指標のBorda順位和
{floor限定risk_vol, 全面risk_vol, placed_volume, n_placed} に置き換えた場合の
26シーンA/B。新しい大域スカラー重みは導入しない(等重み順位和のみ)。

既存の記録済み候補順序(`results/phase29_cand_g1/g2.json`, 130件)を
`simulate_order(..., RISK_SLACK_FACES='floor')` で**再生**して floor risk_vol を得る
(再ロールアウトではない。Phase31の`tools/phase31_selection_eval.py`と同じ手法)。
全面risk_vol/placed_volume/n_placed は既存の記録値をそのまま使う(再生不要)。

勝者の実評価は `results/phase30_cand_eval.json`(既存、新規ロールアウト不要)から引く。

実行:
    MYSOLVER_UNITS_PER_SEC=1.55e7 PYTHONPATH=. .venv/bin/python tools/phase32_borda_eval.py \
        --cand results/phase29_cand_g1.json results/phase29_cand_g2.json \
        --real results/phase30_cand_eval.json \
        --out results/phase32_borda_eval.json
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
    prepacked_ids = geo.initial_prepacked_ids(container_list)
    return container_list, items_by_index, lookahead, prepacked_ids


def replay_floor(container_list, items_by_index, order, lookahead, prepacked_ids):
    simulate_mod.RISK_SLACK_FACES = 'floor'
    budget = planner.SearchBudget.from_seconds(600.0)
    buf = io.StringIO()
    with redirect_stdout(buf):
        placed_ids, placed_volume, risk_vol, viol, srisk = simulate_mod.simulate_order(
            container_list, items_by_index, order, max(1, int(lookahead or 1)), budget,
            prepacked_ids=prepacked_ids, stability_weight=ordering_mod.STABILITY_PENALTY_WEIGHT)
    return risk_vol


def dense_rank_desc(vals):
    """rank 1 = 最大値。同値は平均順位。"""
    order = sorted(range(len(vals)), key=lambda i: -vals[i])
    ranks = [0.0] * len(vals)
    i = 0
    while i < len(order):
        j = i
        while j + 1 < len(order) and vals[order[j + 1]] == vals[order[i]]:
            j += 1
        avg_rank = (i + j) / 2.0 + 1
        for k in range(i, j + 1):
            ranks[order[k]] = avg_rank
        i = j + 1
    return ranks


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
    for c in cands:
        label = c['label']
        container_list, items_by_index, lookahead, prepacked_ids = load_scene(label)

        floor_risk = []
        for rec in c['records']:
            fr = replay_floor(container_list, items_by_index, rec['order'], lookahead, prepacked_ids)
            floor_risk.append(fr)

        n = len(c['records'])
        # 違反ゼロ候補のみをBorda対象にする(既存の合法性フィルタと同じ趣旨。新規の重みではない)
        elig = [i for i in range(n) if c['records'][i]['violation_ratio'] == 0.0]
        if not elig:
            elig = list(range(n))

        all_risk = [c['records'][i]['risk_vol'] for i in elig]
        pv = [c['records'][i]['placed_volume'] for i in elig]
        np_ = [c['records'][i]['n_placed'] for i in elig]
        fr = [floor_risk[i] for i in elig]

        r_floor = dense_rank_desc(fr)
        r_all = dense_rank_desc(all_risk)
        r_pv = dense_rank_desc(pv)
        r_np = dense_rank_desc(np_)

        borda_sum = [r_floor[k] + r_all[k] + r_pv[k] + r_np[k] for k in range(len(elig))]
        # tie-break: 合計順位最小 -> placed_volume大 -> n_placed大 -> all_risk大 -> cand_idx小
        best_k = min(range(len(elig)),
                     key=lambda k: (borda_sum[k], -pv[k], -np_[k], -all_risk[k], elig[k]))
        borda_winner = elig[best_k]

        old_winner = c['winner_idx']

        real_old = real_by_label[label][old_winner]
        real_borda = real_by_label[label][borda_winner]
        gap = real_borda['fill_strict'] - real_old['fill_strict']

        row = {
            'label': label, 'n_cand': n,
            'old_winner': old_winner, 'borda_winner': borda_winner,
            'changed': old_winner != borda_winner,
            'old_real_fill': real_old['fill_strict'],
            'borda_real_fill': real_borda['fill_strict'],
            'gap': gap,
            'old_real_fill_loose': real_old['fill_loose'],
            'borda_real_fill_loose': real_borda['fill_loose'],
            'borda_placement': real_borda['placement_score'],
            'borda_soft': real_borda['soft_item_score'],
            'borda_stability': real_borda['stability_score'],
            'floor_risk_all_elig': fr,
            'elig_idx': elig,
            'borda_sum_all_elig': borda_sum,
        }
        rows.append(row)
        flag = '  <-- changed' if row['changed'] else ''
        print(f"{label:28s} old=c{old_winner} borda=c{borda_winner} "
              f"real_fill {real_old['fill_strict']:6.2f} -> {real_borda['fill_strict']:6.2f} "
              f"({gap:+6.2f}) placement={real_borda['placement_score']:5.1f}{flag}", flush=True)
        if args.out:
            json.dump(rows, open(args.out, 'w'), indent=1, ensure_ascii=False)

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
            print(f"  {r['label']:28s} gap={r['gap']:+6.2f}  placement={r['borda_placement']:5.1f} "
                  f"soft={r['borda_soft']:5.1f} stability={r['borda_stability']:6.2f}")


if __name__ == '__main__':
    main()
