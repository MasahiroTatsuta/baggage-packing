"""
tools/scorer.py

配布されている `src/ground_handling/evaluator.py` は fill_score しか算出しないため、
本番評価基盤と同じ5指標(fill_score, cog_score, stability_score, placement_score,
soft_item_score)をローカルで概算するためのスコアラ。

- fill_score は本家 `Evaluator.calculate_fill_rate` をそのまま利用する(改変しない)。
- cog / placement / soft_item / stability の4指標は README「評価指標」節の定義に
  忠実に沿うよう実装した近似ロジックであり、重みや正規化定数は本番と厳密には一致しない。
  目的はあくまで「今の mysolver がどの指標で弱いか」を相対比較できるようにすることである。

src/ 配下のコードは一切変更せず、Container / Item / Evaluator をそのまま読み取り専用で利用する。
"""
import math

import numpy as np
from pybullet_utils.bullet_client import BulletClient

from src.ground_handling.containers import Container
from src.ground_handling.evaluator import Evaluator
from src.ground_handling.items import Item


class Scorer:
    """本番評価基盤の5指標をローカルで近似算出するクラス"""

    def __init__(self, client: BulletClient, config: dict):
        self.client = client
        self.config = config
        inclusion_margin = config.get('validator', {}).get('inclusion_margin', 0.01)
        # fill_score の算出は本家 Evaluator にそのまま委譲する
        self._fill_evaluator = Evaluator(client=client, config={'inclusion_margin': inclusion_margin})

    # ------------------------------------------------------------------
    # fill_score : 本家ロジックをそのまま再利用
    # ------------------------------------------------------------------
    def calculate_fill_score(self, containers: list[Container]) -> tuple[float, int]:
        """戻り値: (fill_score, fill集計から漏れた(境界超え)荷物数)"""
        fill_score, out_items = self._fill_evaluator.calculate_fill_rate(containers)
        return fill_score, len(out_items)

    # ------------------------------------------------------------------
    # cog_score : 質量加重重心の高さ(低いほど高スコア)
    # ------------------------------------------------------------------
    @staticmethod
    def _get_floor_ceil_z(container: Container) -> tuple[float, float]:
        """
        コンテナ生成時(Container.create)に計算済みの内壁代表点(points)と
        法線ベクトル(n_vecs)から、床面(法線が(0,0,-1)方向)と天井面(法線が(0,0,1)方向)
        のワールドZ座標を取得する。見つからない場合は height からの概算にフォールバックする。
        """
        floor_z = container.center[2] - container.height / 2.0
        ceil_z = container.center[2] + container.height / 2.0
        for pt, nv in zip(container.points, container.n_vecs):
            if abs(nv[0]) < 1e-6 and abs(nv[1]) < 1e-6:
                if nv[2] < -0.5:
                    floor_z = pt[2]
                elif nv[2] > 0.5:
                    ceil_z = pt[2]
        return floor_z, ceil_z

    def calculate_cog_score(self, containers: list[Container]) -> float:
        """
        積載済み全荷物の質量加重重心の高さを, 各コンテナの有効高さ(床面〜天井面)で
        0(天井付近)〜1(床付近)に正規化し, 低いほど高スコアになるようにする.

            normalized_h_i = clip((z_i - floor_z) / (ceil_z - floor_z), 0, 1)
            avg_h          = Σ(mass_i * normalized_h_i) / Σ(mass_i)
            cog_score      = 100 * (1 - avg_h)

        複数コンテナがある場合は全コンテナの荷物をまとめて質量加重する。
        """
        total_mass = 0.0
        weighted_h = 0.0
        for container in containers:
            if not container.packed_items:
                continue
            floor_z, ceil_z = self._get_floor_ceil_z(container)
            effective_height = max(ceil_z - floor_z, 1e-6)
            for item in container.packed_items:
                pos, _ = item.get_pose(self.client)
                if pos is None:
                    continue
                normalized_h = (pos[2] - floor_z) / effective_height
                normalized_h = min(max(normalized_h, 0.0), 1.0)
                weighted_h += item.mass * normalized_h
                total_mass += item.mass

        if total_mass == 0:
            return 100.0  # 荷物が積まれていなければ重心の問題も生じないため満点扱い

        avg_h = weighted_h / total_mass
        return min(max(100.0 * (1.0 - avg_h), 0.0), 100.0)

    # ------------------------------------------------------------------
    # placement_score / soft_item_score : 上方向からの接触(下敷き)判定
    # ------------------------------------------------------------------
    def _find_stacking_pairs(self, containers: list[Container]) -> list[tuple[Item, Item]]:
        """
        同一コンテナ内の荷物ペアについてpybulletの接触点(接触法線)を調べ、
        ほぼ鉛直方向の接触(=積み重なり)があるペアを (下側のitem, 上側のitem) の順で返す。
        """
        self.client.performCollisionDetection()
        pairs: list[tuple[Item, Item]] = []
        for container in containers:
            items = container.packed_items
            for i in range(len(items)):
                for j in range(i + 1, len(items)):
                    item_a, item_b = items[i], items[j]
                    if item_a.pybullet_id is None or item_b.pybullet_id is None:
                        continue
                    contacts = self.client.getContactPoints(bodyA=item_a.pybullet_id, bodyB=item_b.pybullet_id)
                    if not contacts:
                        continue
                    # contactNormalOnB(index 7) のZ成分が大きい接触のみ「積み重なり」とみなす
                    if not any(abs(c[7][2]) > 0.7 for c in contacts):
                        continue
                    pos_a, _ = item_a.get_pose(self.client)
                    pos_b, _ = item_b.get_pose(self.client)
                    if pos_a is None or pos_b is None:
                        continue
                    if pos_a[2] > pos_b[2]:
                        pairs.append((item_b, item_a))  # (bottom, top)
                    else:
                        pairs.append((item_a, item_b))
        return pairs

    def calculate_placement_score(self, containers: list[Container]) -> float:
        """
        優先手荷物(is_prioritized)の配置を評価する。以下の場合に該当荷物を減点対象とする。
          (a) 優先手荷物ではない荷物の下敷きになっている(上方向からの接触がある)
          (b) 優先手荷物用コンテナ(is_prioritized=True)が存在するにもかかわらず、
              そうではないコンテナに配置されている
        優先手荷物同士が積み重なっている場合は減点しない。
        """
        priority_items = [item for c in containers for item in c.packed_items if item.is_prioritized]
        if not priority_items:
            return 100.0

        has_prioritized_container = any(c.is_prioritized for c in containers)

        crushed_ids = set()
        for bottom, top in self._find_stacking_pairs(containers):
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

    def calculate_soft_item_score(self, containers: list[Container]) -> float:
        """
        ソフト貨物(is_soft)が非ソフト貨物の下敷きになっていないかを評価する。
        優先手荷物とは独立に判定する(ソフト貨物にはコンテナの縛りはない)。
        """
        soft_items = [item for c in containers for item in c.packed_items if item.is_soft]
        if not soft_items:
            return 100.0

        crushed_ids = set()
        for bottom, top in self._find_stacking_pairs(containers):
            if bottom.is_soft and not top.is_soft:
                crushed_ids.add(bottom.index)

        violated = sum(1 for item in soft_items if item.index in crushed_ids)
        return min(max(100.0 * (1.0 - violated / len(soft_items)), 0.0), 100.0)

    # ------------------------------------------------------------------
    # stability_score : 蓋をして重力を揺らし、収束後のズレ・運動エネルギーで評価
    # ------------------------------------------------------------------
    def calculate_stability_score(self, containers: list[Container], shake_steps: int = 150,
                                   settle_steps: int = 180) -> float:
        """
        Container.create_cap で蓋をした上で、重力ベクトルの水平成分を周期的に振動させて
        「揺れ」を再現する。揺らし終わったあと通常重力に戻して沈静化させ、

          - 揺らす前の位置からの変位(displacement)
          - 沈静化後の残留運動エネルギー(kinetic energy, 簡易的に角速度項は対角慣性近似)

        の2つを組み合わせて安定性スコアとする(値が小さいほど安定 = 高スコア)。
        この処理は荷物を実際に動かす破壊的な計算のため、他の指標を算出した後に最後に呼ぶこと。
        """
        all_items = [item for c in containers for item in c.packed_items if item.pybullet_id is not None]
        if not all_items:
            return 100.0

        initial_pos = {}
        for item in all_items:
            pos, _ = item.get_pose(self.client)
            if pos is not None:
                initial_pos[item.index] = np.array(pos)

        for container in containers:
            container.create_cap(self.client)

        # 重力の水平成分を円運動的に振動させ、輸送中の揺れ・衝撃を模擬する
        amplitude = 6.0
        for step in range(shake_steps):
            angle = 2 * math.pi * step / 30.0
            gx = amplitude * math.sin(angle)
            gy = amplitude * math.cos(angle * 0.7)
            self.client.setGravity(gx, gy, -9.8)
            self.client.stepSimulation()

        # 通常重力に戻して沈静化させる
        self.client.setGravity(0, 0, -9.8)
        for _ in range(settle_steps):
            self.client.stepSimulation()

        disps = []
        energies = []
        for item in all_items:
            pos, _ = item.get_pose(self.client)
            if pos is None or item.index not in initial_pos:
                continue
            disps.append(float(np.linalg.norm(np.array(pos) - initial_pos[item.index])))
            lin_v, ang_v = self.client.getBaseVelocity(item.pybullet_id)
            # 簡易運動エネルギー(角部分は慣性テンソルを厳密に使わずスカラー近似)
            ke = 0.5 * item.mass * sum(v * v for v in lin_v) + 0.5 * sum(v * v for v in ang_v)
            energies.append(ke)

        mean_disp = float(np.mean(disps)) if disps else 0.0
        mean_energy = float(np.mean(energies)) if energies else 0.0

        # displacement_threshold(本番デフォルト0.3m)相当をゼロ点の目安として使う
        disp_score = max(0.0, 100.0 * (1.0 - mean_disp / 0.3))
        energy_score = max(0.0, 100.0 * (1.0 - min(mean_energy, 1.0) / 1.0))
        score = 0.7 * disp_score + 0.3 * energy_score
        return min(max(score, 0.0), 100.0)

    # ------------------------------------------------------------------
    # aggregate
    # ------------------------------------------------------------------
    def evaluate(self, containers: list[Container], total_items: int) -> dict[str, float]:
        """5指標 + num_placed_items をまとめて算出する"""
        num_packed_items = sum(len(c.packed_items) for c in containers)
        packed_items_percent = num_packed_items / total_items if total_items else 0.0

        fill_score, num_out_items = self.calculate_fill_score(containers)
        fill_counted_ratio = ((num_packed_items - num_out_items) / num_packed_items
                               if num_packed_items else 1.0)
        placement_score = self.calculate_placement_score(containers)
        soft_item_score = self.calculate_soft_item_score(containers)
        cog_score = self.calculate_cog_score(containers)
        # stability は荷物位置を破壊的に変えるため必ず最後に計算する
        stability_score = self.calculate_stability_score(containers)

        return {
            'fill_score': fill_score,
            'cog_score': cog_score,
            'stability_score': stability_score,
            'placement_score': placement_score,
            'soft_item_score': soft_item_score,
            'num_placed_items': packed_items_percent,
            'fill_counted_ratio': fill_counted_ratio,
        }
