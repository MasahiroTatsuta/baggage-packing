"""
tools/phase34_probe.py

Phase34 の**正しさ**を実装前提として確認する専用ツール(A/B測定ではない)。

確認するのは2点:

  (1) 破壊オペレータ `reach.item_occupiers` が (i)「幾何でそもそも入らない」シーンで
      実際に占有者を同定できるか(できなければゲート1に届かない = 設計が (i) に
      届いていない証拠なので、効果量を測る前に報告する)。

  (2) **接頭辞再開の等価性**: ALNS が作った順序を
        RESUME: スナップショットから末尾だけ流し直した結果
        FULL  : その順序を最初から全部流した結果
      で比較し、ビット単位で一致することを確認する。
      build_order が返すのは順序そのものであり、本番環境はそれを最初から流し直すので、
      この一致が崩れていると「再開では良く見えたが実際は違う順序」を採用してしまう。
      alns.py が「remaining_order しか並べ替えない」設計にしてある根拠がこれである。

実行:
    MYSOLVER_UNITS_PER_SEC=1.55e7 PYTHONPATH=. .venv/bin/python tools/phase34_probe.py \
        --cand results/phase29_cand_g1.json results/phase29_cand_g2.json \
        --out results/phase34_probe.json
"""
import argparse
import io
import json
import os
import sys
import time
from contextlib import redirect_stdout

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from src.ground_handling.env import GroundHandlingEnv
from agents.mysolver import alns as alns_mod
from agents.mysolver import geometry as geo
from agents.mysolver import ordering as ordering_mod
from agents.mysolver import planner
from agents.mysolver import reach as reach_mod
from agents.mysolver import simulate as simulate_mod

DEFAULT_SCENES = [
    # (i) 幾何で入らない(Phase30 分類) —— occupier removal の本命
    'A01_1c_40_plain', 'A02_1c_80_plain', 'A03_1c_40_shelf', 'A07_1c_40_bulky',
    'D02_A_1c_40_prioheavy_nocont', 'D04_A_1c_40_flat', 'P01_A_1c_pre6', 'P06_A_1c_pre12_dense',
    # (iii) 搬入経路の封鎖 —— blocker removal / lookahead_k>1 の等価性確認
    'C02_2c_55_shelfprio', 'C03_2c_80_prio', 'D01_A_1c_40_softheavy',
]


def load_scene(label):
    task = list(json.load(open(f'configs/gen/suite_{label}.json')).values())[0]
    env = GroundHandlingEnv(config=task, verbose=False, render_mode=None)
    try:
        env.reset_settings()
        init = env.get_init_states()
        container_list = init['container_list']
        lookahead = max(1, int(init['lookahead_k'] or 1))
        items = env.get_info_for_optimization()
    finally:
        try:
            env.close()
        except Exception:
            pass
    return container_list, {it['index']: it for it in items}, lookahead, \
        geo.initial_prepacked_ids(container_list)


def _sim(container_list, items_by_index, order, lookahead, prepacked_ids, **kw):
    budget = planner.SearchBudget.from_seconds(600.0)
    with redirect_stdout(io.StringIO()):
        return simulate_mod.simulate_order(
            container_list, items_by_index, order, lookahead, budget,
            prepacked_ids=prepacked_ids,
            stability_weight=ordering_mod.STABILITY_PENALTY_WEIGHT, **kw)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--cand', nargs='+', required=True)
    ap.add_argument('--scenes', nargs='*', default=None)
    ap.add_argument('--out', default=None)
    args = ap.parse_args()

    cand_by_label = {}
    for path in args.cand:
        for c in json.load(open(path)):
            cand_by_label.setdefault(c['label'], c)

    rows = []
    for label in (args.scenes or DEFAULT_SCENES):
        c = cand_by_label.get(label)
        if c is None:
            print(f'{label}: 候補記録なし(スキップ)')
            continue
        order = c['records'][c['winner_idx']]['order']
        container_list, items_by_index, lookahead, prepacked_ids = load_scene(label)

        # --- ベース: 勝者順序を全スナップショット付きで1回流す ---
        snaps: dict = {}
        stall: dict = {}
        contribs: list = []
        t0 = time.perf_counter()
        base = _sim(container_list, items_by_index, order, lookahead, prepacked_ids,
                    snapshots_out=snaps, stall_info=stall, contrib_out=contribs)
        base_s = time.perf_counter() - t0
        placed_ids, _pv, base_rv, _vr, _sr = base

        row = {'label': label, 'lookahead_k': lookahead, 'n_items': len(order),
               'n_placed': len(placed_ids), 'stalled': bool(stall.get('stalled')),
               'base_risk_vol': base_rv, 'base_s': base_s, 'n_snapshots': len(snaps)}

        if not stall.get('stalled'):
            row['note'] = '行き詰まらず(ALNS対象外)'
            rows.append(row)
            print(f'{label}: 行き詰まらず')
            continue

        # --- (1) 破壊オペレータ3種がそれぞれ何を外すと言うか ---
        for op, fn in (('occupier', lambda: alns_mod.destroy_occupier(stall, 0.10, set())),
                       ('blocker', lambda: alns_mod.destroy_blocker(stall, 0.10, set())),
                       ('worst', lambda: alns_mod.destroy_worst(stall, contribs, 3, set()))):
            t0 = time.perf_counter()
            x, removed, info = fn()
            row[f'{op}_s'] = time.perf_counter() - t0
            row[f'{op}_x'] = x
            row[f'{op}_removed'] = removed
            row[f'{op}_n'] = (len(removed) if removed else 0)

        # --- (2) occupier removal の手で、RESUME と FULL の等価性を検証 ---
        x, removed = row['occupier_x'], row['occupier_removed']
        if x is None or not removed:
            row['equiv'] = None
            row['note'] = 'occupier removal が候補を作れず(等価性検証は blocker で代替)'
            x, removed = row['blocker_x'], row['blocker_removed']
        if x is not None and removed:
            r_ids = [x] + [i for i in removed if i != x]
            k = alns_mod.choose_snapshot_k(snaps, order, r_ids)
            row['k'] = k
            if k is not None:
                snap = snaps[k]
                r_ordered = alns_mod.repair_greedy(r_ids, items_by_index)
                new_order, new_tail = alns_mod.build_new_order(order, snap, r_ordered)
                row['n_tail'] = len(new_tail)
                rs = alns_mod.make_resume_state(snap, new_tail, simulate_mod.clone_containers)
                t0 = time.perf_counter()
                res = _sim(None, items_by_index, None, lookahead, prepacked_ids, resume_state=rs)
                row['resume_s'] = time.perf_counter() - t0
                t0 = time.perf_counter()
                full = _sim(container_list, items_by_index, new_order, lookahead, prepacked_ids)
                row['full_s'] = time.perf_counter() - t0
                row['equiv'] = (res[0] == full[0] and res[1] == full[1] and res[2] == full[2]
                                and res[3] == full[3] and res[4] == full[4])
                row['resume_risk_vol'] = res[2]
                row['full_risk_vol'] = full[2]
                row['diff_risk_vol'] = res[2] - full[2]
                row['iter_cost_ratio'] = (row['resume_s'] / row['full_s']
                                          if row['full_s'] > 0 else None)
                row['delta_vs_base'] = res[2] - base_rv

        rows.append(row)
        print(f'{label:32s} k_look={lookahead} placed={row["n_placed"]}/{row["n_items"]} '
              f'occ={row.get("occupier_n")}({row.get("occupier_s", 0):.2f}s) '
              f'blk={row.get("blocker_n")} k={row.get("k")} '
              f'equiv={row.get("equiv")} '
              f'resume/full={row.get("iter_cost_ratio") and f"{row['iter_cost_ratio']:.1%}"} '
              f'Δrisk_vol={row.get("delta_vs_base", 0):+.4f}', flush=True)
        if args.out:
            json.dump(rows, open(args.out, 'w'), indent=1, ensure_ascii=False)

    checked = [r for r in rows if r.get('equiv') is not None]
    ok = [r for r in checked if r['equiv']]
    occ = [r for r in rows if r.get('occupier_n')]
    print(f'\n等価性(RESUME == FULL): {len(ok)}/{len(checked)} 件が一致')
    print(f'occupier removal が候補を作れたシーン: {len(occ)}/{len(rows)}')


if __name__ == '__main__':
    main()
