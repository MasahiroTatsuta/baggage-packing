"""
tools/phase29_blockers.py

Phase29 ステップ1の**足切り測定**(Phase28 §5.1 の教訓: 26シーン測定を回す前に
「変更が何シーンに到達するか」を数える)。

`tools/phase29_cand.py` が記録した各シーンの採用順序(best_order)を1回だけ再生し、
行き詰まった瞬間の状態を `simulate.simulate_order(stall_info=...)` で取り出して、

  - そもそも行き詰まったのか(全件置き切ったなら衝突駆動リスタートの出番は無い)
  - 置けなかった荷物 X について、**順序で動かせるブロッカー Y1..Yn を同定できるか**
    (棚が塞いでいる/そもそも収まる位置が無い場合は順序修正では開かない)
  - ブロッカーの個数分布(1個なら単発スワップでも届くが、3個以上は多手移動が要る)

を数える。ここで「修正可能な衝突を持つシーン」が4〜5件に届かなければ、
26シーン測定に進んでも t>2 は原理的に通らないので打ち切る。

実行:
    MYSOLVER_UNITS_PER_SEC=1.55e7 PYTHONPATH=. .venv/bin/python tools/phase29_blockers.py \
        --cand results/phase29_cand_g1.json results/phase29_cand_g2.json \
        --out results/phase29_blockers.json
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
from agents.mysolver import geometry as geo
from agents.mysolver import ordering as ordering_mod
from agents.mysolver import planner
from agents.mysolver import reach as reach_mod
from agents.mysolver import simulate as simulate_mod


def scene_path(label):
    return f'configs/gen/suite_{label}.json'


def winner_order(cand, total_vol):
    """記録された候補列から、build_order が実際に採用する順序を再現する。

    build_order の更新則(`_better` による (調整体積, 配置数) の辞書式比較・**厳密改善のみ**採用・
    評価順に走査)をそのまま再現する。argmax では同点時の採用候補がずれうるので使わない。
    """
    from agents.mysolver import ordering as ordering_mod
    pw = ordering_mod.PLACEMENT_PENALTY_WEIGHT
    best = None
    best_rec = None
    for rec in cand['records']:
        score = (rec['risk_vol'] - pw * total_vol * rec['violation_ratio'], rec['n_placed'])
        if best is None or score > best:
            best, best_rec = score, rec
    return best_rec


def replay(task_config, cand):
    env = GroundHandlingEnv(config=task_config, verbose=False, render_mode=None)
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
    # 採用順序の再現には build_order と同じ総容積(env が返す 'volume')を使う。
    order = winner_order(cand, sum(c.get('volume', 0.0) for c in container_list))['order']
    prepacked_ids = geo.initial_prepacked_ids(container_list)
    budget = planner.SearchBudget.from_seconds(600.0)
    stall: dict = {}
    buf = io.StringIO()
    with redirect_stdout(buf):
        # stability_weight は build_order と同じ既定 0.0 を明示的に渡す(省略すると 1.0)。
        # 配置と行き詰まり地点は変わらないが、返る risk_adjusted_volume が別物になる。
        out = simulate_mod.simulate_order(
            container_list, items_by_index, order, max(1, int(lookahead or 1)), budget,
            prepacked_ids=prepacked_ids,
            stability_weight=ordering_mod.STABILITY_PENALTY_WEIGHT, stall_info=stall)
    return out, stall, items_by_index, order


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--cand', nargs='+', required=True)
    ap.add_argument('--out', default=None)
    args = ap.parse_args()

    cands = []
    for f in args.cand:
        cands.extend(json.load(open(f)))

    rows = []
    for c in cands:
        label = c['label']
        path = scene_path(label)
        task = list(json.load(open(path)).values())[0]
        t0 = time.perf_counter()
        (placed_ids, placed_vol, risk_vol, viol, srisk), stall, items_by_index, order = \
            replay(task, c)
        row = {'label': label, 'n_placed': len(placed_ids),
               'n_items': len(order), 'stalled': bool(stall.get('stalled'))}
        if stall.get('stalled'):
            t1 = time.perf_counter()
            sb = reach_mod.stall_blockers(stall['containers'], stall['pool'])
            row['blocker_sec'] = time.perf_counter() - t1
            row['pool_size'] = len(stall['pool'])
            row['pool'] = [int(it['index']) for it in stall['pool']]
            cand = sb['candidate']
            row['repairable'] = cand is not None
            if cand:
                row['stall_item'] = cand['item_index']
                row['n_blockers'] = cand['result']['n_blockers']
                row['blockers'] = cand['result']['blockers']
                row['n_blocked_positions'] = cand['result']['n_blocked_positions']
                # ブロッカーが順序上どこにいるか(X をどこまで前に出す必要があるか)
                pos = {idx: i for i, idx in enumerate(order)}
                row['stall_item_pos'] = pos.get(cand['item_index'])
                row['earliest_blocker_pos'] = min(pos[b] for b in cand['result']['blockers']
                                                   if b in pos) if cand['result']['blockers'] else None
            row['per_item'] = [{'item': r['item_index'],
                                'n_blockers': (r['result']['n_blockers'] if r['result'] else None)}
                               for r in sb['per_item']]
        else:
            row['repairable'] = False
        row['sec'] = time.perf_counter() - t0
        rows.append(row)
        print(f"{label:28s} placed {row['n_placed']:3d}/{row['n_items']:3d} "
              f"stall={row['stalled']} repairable={row['repairable']} "
              f"blockers={row.get('n_blockers')} X_pos={row.get('stall_item_pos')} "
              f"earliest_blocker_pos={row.get('earliest_blocker_pos')} "
              f"({row['sec']:.1f}s, blocker {row.get('blocker_sec', 0):.2f}s)", flush=True)
        if args.out:
            json.dump(rows, open(args.out, 'w'), indent=1, ensure_ascii=False)

    n_stall = sum(1 for r in rows if r['stalled'])
    n_rep = sum(1 for r in rows if r['repairable'])
    print(f'\n===== 集計 =====')
    print(f'  シーン数 {len(rows)} / 行き詰まった {n_stall} / 順序修正で開ける衝突がある {n_rep}')
    dist = {}
    for r in rows:
        if r.get('n_blockers') is not None:
            dist[r['n_blockers']] = dist.get(r['n_blockers'], 0) + 1
    print(f'  最小ブロッカー数の分布: {dict(sorted(dist.items()))}')


if __name__ == '__main__':
    main()
