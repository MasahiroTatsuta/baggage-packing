"""LDBC — Limited-Discrepancy Backtracking Construction (2026-09-07 着手)。

設計は docs/search_rewrite_design.md。要点:
- `build_order` のアイテム順序はそのまま使う。位置決定を「貪欲 + デッドロック時の巻き戻し」に置換。
- 枝を体積(代理目的関数)でランクしない(Phase86 の失敗)。一次シグナルは num_placed。
- デッドロック(次アイテムが合法位置を持たない)を検出したら直近の決定点へ巻き戻し、
  その位置を forbidden にして次善手を採る。総 discrepancy ≤ K、戻り幅 ≤ D。anytime。

既定 `MYSOLVER_LDBC=0` では本モジュールは呼ばれない(ordering.build_plan 側で分岐)。

v0(本ファイル): forbidden は「score_noise 引き上げ + rng 差し替え」の確率的ナッジ。
v1(次段): planner.plan に厳密除外 `forbidden=` を足す。
"""
import os
import time

import numpy as np

from . import planner
from . import simulate

LDBC_K = int(os.environ.get('MYSOLVER_LDBC_K', '8'))          # 総 discrepancy 上限
LDBC_D = int(os.environ.get('MYSOLVER_LDBC_D', '6'))          # 1回の巻き戻し幅の上限(決定点数)
LDBC_STEP_S = float(os.environ.get('MYSOLVER_LDBC_STEP_S', '1.5'))    # 前進1手のユニット予算(名目秒)
LDBC_PROBE_S = float(os.environ.get('MYSOLVER_LDBC_PROBE_S', '0.6'))  # 先読み1手の合法性チェック予算
LDBC_LOOKAHEAD = int(os.environ.get('MYSOLVER_LDBC_LOOKAHEAD', '1'))  # 何手先までデッドロック判定するか


def _replay(container_list, items_by_index, entries):
    """位置確定済み entries を空コンテナへ置き直した clone を返す(simulate._place で再沈降)。"""
    conts = simulate.clone_containers(container_list)
    for p in entries:
        ci = p['container_idx']
        if ci >= len(conts):
            continue
        it = dict(items_by_index[p['index']])
        act = {'place_pos': np.asarray(p['place_pos'], dtype=float),
               'orientation': p['orientation'], 'container_idx': ci}
        conts[ci]['packed_items'].append(simulate._place(conts[ci], it, act))
    return conts


def _forbid_key(entry):
    px, py, _ = entry['place_pos']
    return (round(px, 2), round(py, 2), entry['orientation'])


def _place_one(conts, item, budget, forbidden_for_item):
    """item を1個置く。forbidden_for_item(このアイテムで過去に外した (x_q,y_q,orn) 集合)を
    planner.plan の厳密除外 forbidden= に渡し、その手を候補から外して次善手を採らせる(v1)。"""
    idx = int(item['index'])
    fb = {(idx, fx, fy, fo) for (fx, fy, fo) in forbidden_for_item} or None
    act = planner.plan(conts, [dict(item)], max_pool_items=None,
                       budget=budget.child_seconds(LDBC_STEP_S), forbidden=fb)
    if act is None:
        return None
    pp = act['place_pos']
    entry = {'index': idx, 'container_idx': int(act['container_idx']),
             'place_pos': (float(pp[0]), float(pp[1]), float(pp[2])),
             'orientation': int(act['orientation'])}
    return entry, act


def _has_legal_spot(conts, item, budget):
    return planner.plan(conts, [dict(item)], max_pool_items=None,
                        budget=budget.child_seconds(LDBC_PROBE_S)) is not None


def ldbc_plan(container_list, items_by_index, order, lookahead_k, budget, wall_deadline=None):
    """戻り値: plan_entries のリスト([{index, container_idx, place_pos, orientation}, ...])。
    anytime。budget(planner.SearchBudget)/ wall_deadline のどちらか枯渇で best を返す。"""
    if not container_list or not order:
        return []
    if wall_deadline is None:
        wall_deadline = time.perf_counter() + 1e9
    seq = [items_by_index[x] for x in order if x in items_by_index]
    conts = simulate.clone_containers(container_list)
    placed: list[dict] = []
    marks: list[tuple[int, int]] = []           # (order_i, len(placed)) 決定点ごと
    forbidden: dict[int, set] = {}
    best_plan: list[dict] = []
    discrepancies = 0
    rng_bt = np.random.default_rng(0)
    i = 0

    def _backtrack() -> bool:
        nonlocal conts, placed, i, discrepancies, marks
        if discrepancies >= LDBC_K or not marks:
            return False
        back = int(rng_bt.integers(1, LDBC_D + 1))
        tgt = max(0, len(marks) - back)
        order_i, plen = marks[tgt]
        if plen < 1:
            return False
        removed = placed[plen - 1]
        forbidden.setdefault(removed['index'], set()).add(_forbid_key(removed))
        placed[:] = placed[:plen - 1]
        conts = _replay(container_list, items_by_index, placed)
        marks[:] = marks[:tgt]
        i = order_i
        discrepancies += 1
        return True

    while i < len(seq) and not budget.exhausted() and time.perf_counter() < wall_deadline:
        it = seq[i]
        res = _place_one(conts, it, budget, forbidden.get(it['index'], set()))
        if res is None:
            if _backtrack():
                continue
            break
        entry, act = res
        ci = entry['container_idx']
        conts[ci]['packed_items'].append(
            simulate._place(conts[ci], dict(it), {'place_pos': np.asarray(entry['place_pos'], dtype=float),
                                                  'orientation': entry['orientation'], 'container_idx': ci}))
        placed.append(entry)
        marks.append((i, len(placed)))
        if len(placed) > len(best_plan):
            best_plan = list(placed)
        # デッドロック先読み: 次の LDBC_LOOKAHEAD 手が合法位置を持つか
        dead = False
        for j in range(i + 1, min(len(seq), i + 1 + LDBC_LOOKAHEAD)):
            if budget.exhausted():
                break
            if not _has_legal_spot(conts, seq[j], budget):
                dead = True
                break
        if dead:
            if _backtrack():
                continue
            break
        i += 1

    out = best_plan if len(best_plan) >= len(placed) else placed
    if os.environ.get('MYSOLVER_LDBC_DEBUG', '0') == '1':
        import sys
        print(f'[LDBC] n_items={len(seq)} plan={len(out)} placed_final={len(placed)} '
              f'discrepancies={discrepancies} forbidden_items={len(forbidden)} '
              f'budget_exhausted={budget.exhausted()}', file=sys.stderr)
    return out
