"""
optimize() 用のオフライン・シャドウシミュレータ。

pybullet の物理演算(重力・接触)は使わず、planner.py と全く同じ幾何・合法性・スコアリング
ロジックだけを使って「この順序/この貪欲戦略で実際に詰めたら何個(どれだけの体積)置けるか」を
高速に見積もる。online の policy() は毎ステップ observation から状態を再構築するだけの
純関数であり、内部的には geometry.py の保守的判定だけで合否を決めているため、この
シャドウシミュレータで得られる配置結果は実際の policy() ループの挙動を高い精度で再現する
(実機の物理沈み込みや衝撃までは再現しないが、順序の善し悪しを比較する探索の評価関数としては
十分な精度)。

container dict / item dict は get_init_states() / get_info_for_optimization() が返す
生の dict をそのまま複製して使う。既配置荷物の姿勢は ORNS[orn_idx] に対応する
クオータニオンを直接組み立てて付与するため、pybullet の物理クライアントは一切不要。
"""
import time

import pybullet as p

from . import planner
from src.ground_handling.utils import ORNS

_ORN_QUATS = [p.getQuaternionFromEuler(e) for e in ORNS]


def clone_containers(container_list: list[dict]) -> list[dict]:
    """container dict のリストを、packed_items も含めて浅くない複製にする。"""
    cloned = []
    for c in container_list:
        cc = dict(c)
        cc['packed_items'] = [dict(item) for item in c.get('packed_items', [])]
        cloned.append(cc)
    return cloned


def _place(container: dict, item: dict, action: dict) -> dict:
    """action(planner.plan の戻り値)に従い item を仮想的に配置し、packed_items に追加できる
    dict を返す(pos/orn を実配置相当の値で埋める)。"""
    ox = container['center'][0]
    lp = action['place_pos']
    placed = dict(item)
    placed['pos'] = (float(lp[0]) + ox, float(lp[1]), float(lp[2]))
    placed['orn'] = _ORN_QUATS[action['orientation']]
    return placed


def simulate_order(container_list: list[dict], items_by_index: dict[int, dict], order: list[int],
                    lookahead_k: int, deadline: float, per_step_time_budget: float = 0.35
                    ) -> tuple[list[int], float]:
    """
    online の ItemStreamManager(lookahead_k個のプールを毎ステップ最大まで補充)と同じ
    プール管理則で、順序 order 通りに荷物を流し込みながら planner.plan を毎ステップ呼ぶ。
    実際の policy() 呼び出しと同じ既定(max_pool_items=既定値)で呼ぶことで、
    「このorderを実機に渡したら何個置けるか」の妥当な見積もりになる。

    戻り値: (配置できた item index のリスト(配置順), 配置できた体積の合計)
    """
    containers = clone_containers(container_list)
    idx_iter = iter(order)
    pool: list[dict] = []

    def refill():
        while len(pool) < lookahead_k:
            try:
                pool.append(dict(items_by_index[next(idx_iter)]))
            except StopIteration:
                return

    refill()
    placed_ids: list[int] = []
    placed_volume = 0.0

    while pool:
        now = time.perf_counter()
        if now > deadline:
            break
        budget = min(per_step_time_budget, max(0.02, deadline - now))
        action = planner.plan(containers, pool, time_budget=budget)
        if action is None:
            break
        item = pool.pop(action['item_idx'])
        container = containers[action['container_idx']]
        container['packed_items'].append(_place(container, item, action))
        placed_ids.append(item['index'])
        placed_volume += item['length'] * item['width'] * item['height']
        refill()

    return placed_ids, placed_volume


def greedy_construct_order(container_list: list[dict], item_list: list[dict], deadline: float,
                            per_step_time_budget: float = 3.0, rng=None, score_noise: float = 0.0,
                            shuffle_ties: bool = False, window: int | None = None) -> list[int]:
    """
    「今置ける中で一番良い荷物・向き・位置」を毎回選び直す貪欲構築(オフライン限定, フル情報)。
    lookahead=1(パターンA)ではこの構築順そのものが online の実行結果と一致する
    (pool=1個の planner.plan は、pool内に他の候補が無いだけで同じ探索・同じ位置を返すため)。
    lookahead>1(パターンB)でも「常に最良の1手から詰める」近似順序として有効に働く。

    時間切れ、または合法手が尽きた場合は、残りの荷物インデックスを(必要なら shuffle して)
    そのまま末尾に付け足し、必ず全 item index を過不足なく含む順列を返す。
    """
    containers = clone_containers(container_list)
    remaining = {item['index']: dict(item) for item in item_list}
    order: list[int] = []

    if shuffle_ties and rng is not None:
        keys = list(remaining.keys())
        rng.shuffle(keys)
        remaining = {k: remaining[k] for k in keys}

    while remaining:
        now = time.perf_counter()
        if now > deadline:
            break
        pool = list(remaining.values())
        if window is not None:
            pool = pool[:window]
        budget = min(per_step_time_budget, max(0.05, deadline - now))
        action = planner.plan(containers, pool, time_budget=budget, max_pool_items=None,
                               rng=rng, score_noise=score_noise)
        if action is None:
            break
        item = pool[action['item_idx']]
        del remaining[item['index']]
        container = containers[action['container_idx']]
        container['packed_items'].append(_place(container, item, action))
        order.append(item['index'])

    if remaining:
        order.extend(remaining.keys())

    return order
