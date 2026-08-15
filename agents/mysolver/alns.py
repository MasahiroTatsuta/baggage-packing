"""Phase34: ALNS(Adaptive Large Neighborhood Search の破壊→修復)を接頭辞再開の上に載せる。

本モジュールは **純粋な部品** だけを持つ(予算管理・受理判定・ループ本体は
`ordering.build_order` のフェーズ4にある)。ここに書いてあるのは
「どの荷物を壊すか(destroy)」「壊した荷物をどの順で戻すか(repair)」と、
それを **接頭辞再開できる形の順序** に落とすための変換だけである。

---------------------------------------------------------------------------
なぜ ALNS なのか(Phase29 との違い)
---------------------------------------------------------------------------
Phase29 の衝突駆動リスタートは「X を1個だけ前に出す/ブロッカーを後ろへ回す」の
1手移動で、しかも毎回 **順序を最初から全部** 評価し直していた。結果、到達したのは
26シーン中2シーンだけだった(results/phase29_report.md)。原因は2つある:

  (a) 対象が (iii) 搬入経路の封鎖だけだった。Phase30 の分類では (iii) は 6/21 シーンで、
      最大区分は **(i) 幾何でそもそも入らない = 10/21 シーン**(残体積の54.8%)である。
  (b) 1試行のコストが全構築1回と同じだったため、端数予算に数回しか入らなかった。

Phase34 は両方に手を当てる:

  (a) → **occupier removal**(`reach.item_occupiers`)。「X が入れたはずの空間を
        占有している荷物」を同定する新しい破壊オペレータで、(i) に直接届く。
  (b) → **接頭辞再開**(Phase33 で 9/9 件ビット単位一致を確認済み)。壊す位置より前は
        再計算せず、末尾だけを流し直す。実測で1反復は全構築1回の 1.6〜4.8%。

---------------------------------------------------------------------------
適応化(adaptive)は入れていない
---------------------------------------------------------------------------
本家 ALNS はオペレータの選択確率を成績で適応させるが、**その重みは実質的に
新しい調整パラメータ** であり、Phase18/26/28/31/32 が繰り返し踏んだ
「シーン横断で使える値が存在しない」問題を持ち込む。ここは **決定的なラウンドロビン**
で回す。効くと分かってから適応化を検討すること。

---------------------------------------------------------------------------
接頭辞再開の正しさ(重要)
---------------------------------------------------------------------------
評価は「スナップショットからの再開」で行うが、`build_order` が最終的に返すのは
**順序そのもの** であり、本番環境はそれを最初から流し直す。したがって
「再開して得た評価」と「返した順序を最初から流した結果」は一致していなければならない。

一致を保証するために、本モジュールは **`remaining_order`(まだプールに引き込まれて
いないストリームの残り)しか並べ替えない**。プールに既に入っている荷物には触らない。
こうすると、新しい順序の先頭 `n_consumed` 個は元の順序と完全に同一なので、
最初から流し直したときもスナップショットと同じ状態に必ず到達する
(lookahead_k>1 でプールの中身が途中で混ざるシーンでも成立する)。

この条件を満たす **最大の k**(=再開位置をできるだけ後ろに取る=1反復を最も安くする)を
`choose_snapshot_k` が選ぶ。tools/phase34_probe.py がこの一致を実測で検証する。
"""

from . import reach

# 破壊オペレータの識別子(決定的ラウンドロビンの順番そのもの)。
OP_OCCUPIER = 'occupier'   # (i) 幾何で入らない ← 本命、Phase34 の新規部品
OP_BLOCKER = 'blocker'     # (iii) 搬入経路の封鎖 ← Phase29 の既存部品を再利用
OP_WORST = 'worst'         # 最も割引の大きい(=無駄な空間を作った)配置を後回しにする
OPS = (OP_OCCUPIER, OP_BLOCKER, OP_WORST)


# ---------------------------------------------------------------------------
# 破壊(destroy): 「どの荷物を順序から外して考え直すか」を決める
# ---------------------------------------------------------------------------
def destroy_occupier(stall, voxel, skip_items):
    """行き詰まったプールの各 X について、X の居場所を奪っている荷物を返す。

    戻り値 (x, removed, info)。プール順(=決定的)に見て最初に同定できたものを採用する。
    """
    cache: dict = {}
    for item in stall.get('pool', []):
        x = int(item['index'])
        if x in skip_items:
            continue
        r = reach.item_occupiers(stall['containers'], item, voxel=voxel, masks_cache=cache)
        if r:
            cells = sum(int(m['empty'].size) for m in cache.values())
            return x, list(r['occupiers']), {'op': OP_OCCUPIER, 'n_removed': r['n_occupiers'],
                                             'n_positions': r['n_positions'],
                                             'grid_cells': cells,
                                             'n_shapes': max(1, len(stall.get('pool', [])))}
    cells = sum(int(m['empty'].size) for m in cache.values())
    return None, None, {'op': OP_OCCUPIER, 'grid_cells': cells,
                        'n_shapes': max(1, len(stall.get('pool', [])))}


def destroy_blocker(stall, voxel, skip_items):
    """Phase29 の `reach.item_blockers` をそのまま使う((iii) 向けの既存部品)。"""
    cache: dict = {}
    for item in stall.get('pool', []):
        x = int(item['index'])
        if x in skip_items:
            continue
        r = reach.item_blockers(stall['containers'], item, voxel=voxel, masks_cache=cache)
        if r:
            cells = sum(int(m['empty'].size) for m in cache.values())
            return x, list(r['blockers']), {'op': OP_BLOCKER, 'n_removed': r['n_blockers'],
                                            'n_positions': r['n_blocked_positions'],
                                            'grid_cells': cells,
                                            'n_shapes': max(1, len(stall.get('pool', [])))}
    cells = sum(int(m['empty'].size) for m in cache.values())
    return None, None, {'op': OP_BLOCKER, 'grid_cells': cells,
                        'n_shapes': max(1, len(stall.get('pool', [])))}


def destroy_worst(stall, contribs, q, skip_items):
    """risk割引が最も大きかった(=壁ぎわ・不安定で体積が目減りした)配置を q 個外す。

    「無駄な空間を作った荷物」の代理として、目的関数が既に各荷物へ与えている
    割引率(`simulate_order` の contrib_out)をそのまま使う。**新しい重みも閾値も
    導入しない**(目的関数の内訳を読み替えるだけ)のが要点。
    幾何・格子の計算を一切しないので、3オペレータの中で最も安い。
    """
    pool = stall.get('pool', [])
    x = None
    for item in pool:
        if int(item['index']) not in skip_items:
            x = int(item['index'])
            break
    if x is None or not contribs:
        return None, None, {'op': OP_WORST}
    order_pos = {idx: i for i, (idx, _) in enumerate(contribs)}
    ranked = sorted(contribs, key=lambda t: (t[1], -order_pos[t[0]], t[0]))
    removed = [idx for idx, _ in ranked[:max(1, q)]]
    return x, removed, {'op': OP_WORST, 'n_removed': len(removed),
                        'worst_discount': ranked[0][1]}


# ---------------------------------------------------------------------------
# 修復(repair): 外した荷物を「どの順で」ストリームの先頭へ戻すか
# ---------------------------------------------------------------------------
def repair_greedy(r_ids, items_by_index):
    """体積の大きい順に戻す(積付けの古典的な貪欲。大物ほど置き場所を選ぶため先に確保する)。"""
    def vol(i):
        it = items_by_index[i]
        return it.get('volume', it['length'] * it['width'] * it['height'])
    return sorted(r_ids, key=lambda i: (-vol(i), i))


def repair_regret(r_ids, fit_counts):
    """置き場所の選択肢が少ないものから戻す(most-constrained-first = regret 的な順序)。

    regret 系の発想は「今やらないと後で一番損をするものから決める」である。ここでは
    「除去後の状態で収まる位置が少ない荷物ほど、後回しにすると置けなくなる」とみなし、
    位置数の昇順で戻す。位置数は `reach.fit_position_counts` が実測する。
    """
    return sorted(r_ids, key=lambda i: (fit_counts.get(i, 0), i))


# ---------------------------------------------------------------------------
# 順序への落とし込み(接頭辞再開と厳密に整合させる)
# ---------------------------------------------------------------------------
def choose_snapshot_k(snapshots, order, r_ids):
    """R の全員がまだ `remaining_order` 側に居る **最大の k** を返す(無ければ None)。

    「プールに入っている荷物は動かさない」という制約(モジュール docstring 参照)を
    満たしつつ、再開位置をできるだけ後ろに取ることで1反復を最も安くする。
    """
    pos = {v: i for i, v in enumerate(order)}
    ps = [pos[i] for i in r_ids if i in pos]
    if not ps:
        return None
    p = min(ps)
    best_k = None
    for k, snap in snapshots.items():
        n_consumed = len(order) - len(snap['remaining_order'])
        if n_consumed <= p and (best_k is None or k > best_k):
            best_k = k
    return best_k


def build_new_order(order, snap, r_ordered):
    """外した荷物を `remaining_order` の先頭へ戻した「完全な順序」と「新しい末尾」を返す。"""
    base_tail = list(snap['remaining_order'])
    rset = set(r_ordered)
    new_tail = list(r_ordered) + [i for i in base_tail if i not in rset]
    n_consumed = len(order) - len(base_tail)
    return list(order[:n_consumed]) + new_tail, new_tail


def make_resume_state(snap, new_tail, clone_containers):
    """スナップショットから再開用の状態を作る。

    `simulate_order` は containers を **その場で書き換える** ので、反復ごとに必ず複製する
    (複製しないとスナップショットが1回で壊れ、2反復目以降が別物になる)。
    """
    st = dict(snap)
    st['containers'] = clone_containers(snap['containers'])
    st['pool'] = [dict(it) for it in snap['pool']]
    st['placed_ids'] = list(snap['placed_ids'])
    st['remaining_order'] = list(new_tail)
    return st


def refresh_snapshots(snapshots, k_used, old_order, new_order, resume_snaps):
    """採用後にスナップショット表を作り直す(再計算せずに使い回す)。

    `k <= k_used` のスナップショットは **状態そのものは新旧の順序で完全に同一** である
    (接頭辞が一致しているため)。ただし各スナップショットが持つ `remaining_order` は
    旧順序の末尾なので、新順序の末尾へ差し替える必要がある。
    `k >= k_used` の分は、採用された再開ロールアウトが記録したものをそのまま使う。
    """
    out = {}
    for k, snap in snapshots.items():
        if k > k_used:
            continue
        n_consumed = len(old_order) - len(snap['remaining_order'])
        s = dict(snap)
        s['remaining_order'] = list(new_order[n_consumed:])
        out[k] = s
    out.update(resume_snaps)
    return out
