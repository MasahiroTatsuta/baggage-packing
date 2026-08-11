"""
tools/phase29_repair_probe.py

Phase29 ステップ1の**高速な事前検証**。build_order を丸ごと回す(1シーン90秒)必要は無い:
修正フェーズがやることは「記録済みの採用順序 → 行き詰まり → ブロッカー同定 → 順序を作り直して
validate 1回」だけなので、それだけを取り出して回す(1シーン数秒)。

これで「どの修正の作り方が、どのシーンで、どれだけ効くか」を26シーン測定の前に見切る。

実行:
    MYSOLVER_UNITS_PER_SEC=1.55e7 PYTHONPATH=. .venv/bin/python tools/phase29_repair_probe.py \
        --cand results/phase29_cand_g1.json results/phase29_cand_g2.json \
        --out results/phase29_repair_probe.json
"""
import argparse
import io
import json
import os
import sys
import time
from contextlib import redirect_stdout

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from agents.mysolver import geometry as geo
from agents.mysolver import ordering as O
from agents.mysolver import planner
from agents.mysolver import reach as R
from agents.mysolver import simulate as S
from src.ground_handling.env import GroundHandlingEnv
from tools.phase29_blockers import winner_order


def _advance_to_front(order, x, blockers):
    """X を先頭へ(全ブロッカーより前、の最も強い形)。"""
    if x not in order or order[0] == x:
        return None
    rest = [v for v in order if v != x]
    return [x] + rest


def _advance_before_each(order, x, blockers):
    """ブロッカーを1個ずつしか越えない弱い修正(単発スワップ相当)を、遅い側から順に返す。"""
    pos = {v: i for i, v in enumerate(order)}
    outs = []
    for b in sorted((b for b in blockers if b in pos), key=lambda b: -pos[b]):
        if pos[b] >= pos[x]:
            continue
        rest = [v for v in order if v != x]
        outs.append((f'advance_before_{b}', rest[:pos[b]] + [x] + rest[pos[b]:]))
    return outs


def setup(label):
    task = list(json.load(open(f'configs/gen/suite_{label}.json')).values())[0]
    env = GroundHandlingEnv(config=task, verbose=False, render_mode=None)
    try:
        env.reset_settings()
        init = env.get_init_states()
        cl = init['container_list']
        la = max(1, int(init['lookahead_k'] or 1))
        items = env.get_info_for_optimization()
    finally:
        try:
            env.close()
        except Exception:
            pass
    return task, cl, la, items


def run_order(cl, items_by_index, order, la, prepacked_ids, want_stall=True):
    stall = {} if want_stall else None
    with redirect_stdout(io.StringIO()):
        # stability_weight は **必ず build_order と同じ値**(既定 0.0)を渡すこと。
        # simulate_order の既定は 1.0 なので、省略すると build_order が使っていない
        # 積み上げリスク割引が入り、risk_adjusted_volume が別物になる(配置そのものは
        # 変わらないので placed/fill は正しいままだが、目的関数の順位が入れ替わる)。
        out = S.simulate_order(cl, items_by_index, order, la,
                               planner.SearchBudget.from_seconds(600.0),
                               prepacked_ids=prepacked_ids,
                               stability_weight=O.STABILITY_PENALTY_WEIGHT,
                               stall_info=stall)
    return out, stall


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--cand', nargs='+', required=True)
    ap.add_argument('--labels', nargs='+', default=None)
    ap.add_argument('--voxel', type=float, default=0.05)
    ap.add_argument('--out', default=None)
    args = ap.parse_args()

    cands = []
    for f in args.cand:
        cands.extend(json.load(open(f)))
    cands.sort(key=lambda c: c['label'])
    if args.labels:
        cands = [c for c in cands if c['label'] in args.labels]

    rows = []
    for c in cands:
        label = c['label']
        t0 = time.perf_counter()
        task, cl, la, items = setup(label)
        ibx = {it['index']: it for it in items}
        prepacked = geo.initial_prepacked_ids(cl)
        tv = sum(cc.get('volume', 0.0) for cc in cl)
        order = winner_order(c, tv)['order']
        (pids, pvol, rv, viol, sr), stall = run_order(cl, ibx, order, la, prepacked)
        base = (rv - O.PLACEMENT_PENALTY_WEIGHT * tv * viol, len(pids))
        row = {'label': label, 'base_score': base[0], 'base_placed': base[1],
               'base_fill': 100 * pvol / tv, 'variants': []}
        if not stall.get('stalled'):
            print(f'{label:28s} 行き詰まらず'); rows.append(row); continue
        sb = R.stall_blockers(stall['containers'], stall['pool'], voxel=args.voxel)
        cand = sb['candidate']
        if cand is None:
            row['repairable'] = False
            print(f'{label:28s} placed={base[1]:3d} fill={row["base_fill"]:6.2f} '
                  f'順序修正で開ける衝突なし')
            rows.append(row)
            continue
        row['repairable'] = True
        x = cand['item_index']
        blockers = cand['result']['blockers']
        row['x'] = x
        row['blockers'] = blockers
        makers = [('advance_before', O._advance_before(order, x, blockers)),
                  ('delay_blockers', O._delay_blockers(order, x, blockers)),
                  ('advance_to_front', _advance_to_front(order, x, blockers))]
        makers += _advance_before_each(order, x, blockers)
        print(f'{label:28s} placed={base[1]:3d} fill={row["base_fill"]:6.2f} '
              f'X={x} blockers={blockers}')
        for name, o2 in makers:
            if o2 is None:
                continue
            (p2, v2, rv2, vi2, _), _ = run_order(cl, ibx, o2, la, prepacked, want_stall=False)
            s2 = rv2 - O.PLACEMENT_PENALTY_WEIGHT * tv * vi2
            better = bool((s2, len(p2)) > base)
            row['variants'].append({'name': name, 'score': float(s2), 'placed': int(len(p2)),
                                    'fill': float(100 * v2 / tv), 'better': better})
            print(f'    {name:22s} placed={len(p2):3d} fill={100 * v2 / tv:6.2f} '
                  f'(Δfill {100 * v2 / tv - row["base_fill"]:+6.2f}) '
                  f'{"**改善**" if better else ""}')
        row['sec'] = time.perf_counter() - t0
        rows.append(row)
        if args.out:
            json.dump(rows, open(args.out, 'w'), indent=1, ensure_ascii=False)

    rep = [r for r in rows if r.get('repairable')]
    imp = [r for r in rep if any(v['better'] for v in r['variants'])]
    print(f'\n===== 集計 =====')
    print(f'  修正可能な衝突があったシーン: {len(rep)}/{len(rows)}')
    print(f'  そのうち1つでも改善した修正があったシーン: {len(imp)}')
    for r in imp:
        b = max((v for v in r['variants'] if v['better']), key=lambda v: v['fill'])
        print(f"    {r['label']:28s} {b['name']:22s} fill {r['base_fill']:.2f} -> {b['fill']:.2f}")
    if args.out:
        json.dump(rows, open(args.out, 'w'), indent=1, ensure_ascii=False)


if __name__ == '__main__':
    main()
