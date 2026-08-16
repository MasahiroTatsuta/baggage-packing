"""Phase37(ステップ1-3): ρ-test の勝者決定を「実fillの argmax」から「5成分の合成スコアの
argmax」へ拡張するための、cog/placement/soft_item/stability の近似計算。

なぜここに複製が要るか
---------------------------------------------------------------------------
`tools/scorer.py` の `Scorer` クラスに全く同じロジックが既に存在するが、`tools/` は
提出zip(`mysolver/`配下の9ファイルのみ)に含まれない。`replica.py` から `tools.scorer` を
importすると、本番では**必ず** ImportError になり、`ordering.py` 冒頭の
`try: from . import replica ... except Exception: _replica_mod = None` に引っかかって
**複製評価器そのものが恒久的に無効化される**(Phase37ステップ0が調べているH2そのもの)。
つまり本番で動かすには、このファイルのように `agents/mysolver/` 配下に複製を置く必要がある。

計算式は `tools/scorer.py` と完全に同一(コピー元のコメントも参照)。**本番評価基盤の
非公開ロジックの近似**であり、重み・正規化定数は本番と厳密には一致しない
(README「評価指標」節の定義に沿った近似)。fill_score だけは本物の
`src.ground_handling.evaluator.Evaluator.calculate_fill_rate` をそのまま使う
(replica.py側で既に実施済み、ここでは扱わない)。

呼び出し順の制約: `calculate_stability_score` は荷物位置を破壊的に変える。
fill / placement / soft_item / cog を全て計算し終えてから最後に呼ぶこと
(replica.py の呼び出し側がこの順序を守る)。
"""
import math

import numpy as np
from pybullet_utils.bullet_client import BulletClient

from src.ground_handling.containers import Container

# public = (2*fill + 1.5*cog + 1.5*stability + 1*placement + 1*soft_item) / 7
# CLAUDE_CODE_指示書.md §1.1(全16提出で小数第2位まで検証済み)。
COMPOSITE_WEIGHTS = {'fill': 2.0, 'cog_score': 1.5, 'stability_score': 1.5,
                     'placement_score': 1.0, 'soft_item_score': 1.0}
COMPOSITE_DENOM = sum(COMPOSITE_WEIGHTS.values())  # 7.0


def composite_score(fill: float, cog: float, stability: float, placement: float, soft: float) -> float:
    return (COMPOSITE_WEIGHTS['fill'] * fill + COMPOSITE_WEIGHTS['cog_score'] * cog
            + COMPOSITE_WEIGHTS['stability_score'] * stability
            + COMPOSITE_WEIGHTS['placement_score'] * placement
            + COMPOSITE_WEIGHTS['soft_item_score'] * soft) / COMPOSITE_DENOM


def _get_floor_ceil_z(container: Container) -> tuple[float, float]:
    floor_z = container.center[2] - container.height / 2.0
    ceil_z = container.center[2] + container.height / 2.0
    for pt, nv in zip(container.points, container.n_vecs):
        if abs(nv[0]) < 1e-6 and abs(nv[1]) < 1e-6:
            if nv[2] < -0.5:
                floor_z = pt[2]
            elif nv[2] > 0.5:
                ceil_z = pt[2]
    return floor_z, ceil_z


def calculate_cog_score(client: BulletClient, containers: list[Container]) -> float:
    total_mass = 0.0
    weighted_h = 0.0
    for container in containers:
        if not container.packed_items:
            continue
        floor_z, ceil_z = _get_floor_ceil_z(container)
        effective_height = max(ceil_z - floor_z, 1e-6)
        for item in container.packed_items:
            pos, _ = item.get_pose(client)
            if pos is None:
                continue
            normalized_h = min(max((pos[2] - floor_z) / effective_height, 0.0), 1.0)
            weighted_h += item.mass * normalized_h
            total_mass += item.mass
    if total_mass == 0:
        return 100.0
    avg_h = weighted_h / total_mass
    return min(max(100.0 * (1.0 - avg_h), 0.0), 100.0)


def _find_stacking_pairs(client: BulletClient, containers: list[Container]) -> list[tuple]:
    client.performCollisionDetection()
    pairs = []
    for container in containers:
        items = container.packed_items
        for i in range(len(items)):
            for j in range(i + 1, len(items)):
                item_a, item_b = items[i], items[j]
                if item_a.pybullet_id is None or item_b.pybullet_id is None:
                    continue
                contacts = client.getContactPoints(bodyA=item_a.pybullet_id, bodyB=item_b.pybullet_id)
                if not contacts:
                    continue
                if not any(abs(c[7][2]) > 0.7 for c in contacts):
                    continue
                pos_a, _ = item_a.get_pose(client)
                pos_b, _ = item_b.get_pose(client)
                if pos_a is None or pos_b is None:
                    continue
                if pos_a[2] > pos_b[2]:
                    pairs.append((item_b, item_a))
                else:
                    pairs.append((item_a, item_b))
    return pairs


def calculate_placement_score(client: BulletClient, containers: list[Container]) -> float:
    priority_items = [item for c in containers for item in c.packed_items if item.is_prioritized]
    if not priority_items:
        return 100.0
    has_prioritized_container = any(c.is_prioritized for c in containers)
    crushed_ids = set()
    for bottom, top in _find_stacking_pairs(client, containers):
        if bottom.is_prioritized and not top.is_prioritized:
            crushed_ids.add(bottom.index)
    container_of = {item.index: c for c in containers for item in c.packed_items}
    violated = 0
    for item in priority_items:
        is_crushed = item.index in crushed_ids
        is_wrong_container = has_prioritized_container and not container_of[item.index].is_prioritized
        if is_crushed or is_wrong_container:
            violated += 1
    return min(max(100.0 * (1.0 - violated / len(priority_items)), 0.0), 100.0)


def calculate_soft_item_score(client: BulletClient, containers: list[Container]) -> float:
    soft_items = [item for c in containers for item in c.packed_items if item.is_soft]
    if not soft_items:
        return 100.0
    crushed_ids = set()
    for bottom, top in _find_stacking_pairs(client, containers):
        if bottom.is_soft and not top.is_soft:
            crushed_ids.add(bottom.index)
    violated = sum(1 for item in soft_items if item.index in crushed_ids)
    return min(max(100.0 * (1.0 - violated / len(soft_items)), 0.0), 100.0)


def calculate_stability_score(client: BulletClient, containers: list[Container],
                               shake_steps: int = 150, settle_steps: int = 180) -> float:
    """破壊的(荷物を実際に動かす)。他の指標を全て算出した後、必ず最後に呼ぶこと。"""
    all_items = [item for c in containers for item in c.packed_items if item.pybullet_id is not None]
    if not all_items:
        return 100.0

    initial_pos = {}
    for item in all_items:
        pos, _ = item.get_pose(client)
        if pos is not None:
            initial_pos[item.index] = np.array(pos)

    for container in containers:
        container.create_cap(client)

    amplitude = 6.0
    for step in range(shake_steps):
        angle = 2 * math.pi * step / 30.0
        gx = amplitude * math.sin(angle)
        gy = amplitude * math.cos(angle * 0.7)
        client.setGravity(gx, gy, -9.8)
        client.stepSimulation()

    client.setGravity(0, 0, -9.8)
    for _ in range(settle_steps):
        client.stepSimulation()

    disps = []
    energies = []
    for item in all_items:
        pos, _ = item.get_pose(client)
        if pos is None or item.index not in initial_pos:
            continue
        disps.append(float(np.linalg.norm(np.array(pos) - initial_pos[item.index])))
        lin_v, ang_v = client.getBaseVelocity(item.pybullet_id)
        ke = 0.5 * item.mass * sum(v * v for v in lin_v) + 0.5 * sum(v * v for v in ang_v)
        energies.append(ke)

    mean_disp = float(np.mean(disps)) if disps else 0.0
    mean_energy = float(np.mean(energies)) if energies else 0.0

    disp_score = max(0.0, 100.0 * (1.0 - mean_disp / 0.3))
    energy_score = max(0.0, 100.0 * (1.0 - min(mean_energy, 1.0) / 1.0))
    score = 0.7 * disp_score + 0.3 * energy_score
    return min(max(score, 0.0), 100.0)


def evaluate_extra_metrics(client: BulletClient, containers: list[Container]) -> dict:
    """fill以外の4指標+合成スコアの元になる値を返す。stabilityは破壊的なので必ず最後に呼ぶ。"""
    placement = calculate_placement_score(client, containers)
    soft = calculate_soft_item_score(client, containers)
    cog = calculate_cog_score(client, containers)
    stability = calculate_stability_score(client, containers)  # 必ず最後
    return {'placement_score': placement, 'soft_item_score': soft,
            'cog_score': cog, 'stability_score': stability}
