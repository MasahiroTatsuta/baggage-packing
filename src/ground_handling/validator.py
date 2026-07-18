import pybullet as p
import numpy as np
import time
import math
from pybullet_utils.bullet_client import BulletClient

from .utils import ORNS, get_half_ext
from .items import Item
from .containers import Container


class BaseValidator:
    def __init__(self, client: BulletClient, config: dict, render_mode: str | None = None):
        self.client = client
        self.config = config
        self.render_mode = render_mode
        self.safety_margin: float = self.config['safety_margin']
        self.start_margin: float = self.config.get('start_margin', 0.01)
        self.inclusion_margin: float = self.config.get('inclusion_margin', 0.02)
        self.start_z: float = self.config['start_z']
        self.ceiling_margin: float = self.config.get('ceiling_margin', 0.01)


    def check_action(self, action, action_config: dict, num_containers: int, num_visible_items: int):
        valid = True
        truncated = False
        info = {}

        # キーの確認
        if set(action.keys()) != set(action_config['keys'].keys()):
            valid = False
            truncated = True
            info = {'status': {'has_valid_action_keys': False}}

        # 置き場所の座標の確認
        if len(action['place_pos'])==3:
            for v in action['place_pos']:
                if not action_config['pos_lim']['low'] <= v <= action_config['pos_lim']['high']:
                    valid = False
                    truncated = True
                    info = {'status': {f'has_valid_action_pos (all elements must be in the range of {action_config['pos_lim']['low']} to {action_config['pos_lim']['high']}': False}}
        else:
            valid = False
            truncated = True
            info = {'status': {f'has_valid_action_pos (length must be 3)': False}}

        # コンテナや物体や物体に対して指定する向きに関するインデックスの確認
        container_idx_max = num_containers - 1
        visible_pool_max = num_visible_items - 1
        orientation_max = len(action_config['orientations']) - 1
        if not (0 <= action['container_idx'] <= container_idx_max):
            valid = False
            truncated = True
            info = {'status': {f'has_valid_action_index (index in container_idx must be in the range of {0} to {container_idx_max})': False}}
        if not (0 <= action['item_idx'] <= visible_pool_max):
            valid = False
            truncated = True
            info = {'status': {f'has_valid_action_index (index in item_idx must be in the range of {0} to {visible_pool_max})': False}}
        if not (0 <= action['orientation'] <= orientation_max):
            valid = False
            truncated = True
            info = {'status': {f'has_valid_action_index (index in orientation must be in the range of {0} to {orientation_max})': False}}
        
        return valid, truncated, info


class PlacementValidator(BaseValidator):
    def check_inclusion(self, container: Container, item: Item | None, target_pos: tuple[float, float, float], target_orn_idx: int) -> bool:
        """
        目標位置において手荷物がコンテナの内部に含まれるかを判定する
        """
        if item is None:
            return False
        original_lwh =[item.length, item.width, item.height]
        half_lwh = get_half_ext(original_lwh, target_orn_idx)

        dots = (np.array(container.n_vecs)*(target_pos - np.array(container.points))).sum(axis=1) + np.dot(np.abs(np.array(container.n_vecs)), half_lwh)
        isin_all = np.all(dots<=self.inclusion_margin)
        if not bool(isin_all):
            print(f'not included. {dots.tolist()}')

        return bool(isin_all)


    def check_transport_path(self, container: Container, item: Item | None, target_pos: tuple[float, float, float], target_orn_idx: int, step_len: float = 0.01) -> bool:
        """
        目標位置(target_pos: 世界座標系)へ運ぶ軌道上で衝突がないか判定する
        座標は絶対座標
        """
        if item is None:
            return False

        # 荷物を運ぶ前の現在の状態をメモリに保存する
        original_lwh =[item.length, item.width, item.height]
        half_lwh = get_half_ext(original_lwh=original_lwh, orn_idx=target_orn_idx)
        x_min = -container.length/2 + container.thickness + container.cut_x + half_lwh[0] + self.start_margin
        x_max = container.length/2 - container.thickness - half_lwh[0] - self.start_margin
        rel_target = container.global_to_local(target_pos)    # 相対座標に直す
        rel_x = min(max(rel_target[0], x_min), x_max)

        # 1. 基準となる「面」の定義
        # 直置き面（床面、棚の上面）
        resting_surfaces = [
            container.thickness, 
            (container.height / 2.0) + container.thickness + container.buffer
        ]
        
        # 天井面（棚の下面、コンテナの天井）
        ceiling_surfaces = [
            (container.height / 2.0) + container.buffer,
            container.height + container.buffer - container.thickness
        ]

        # デフォルトの浮遊・落下量
        effective_start_z = self.start_z

        # 2. 下の判定（直置き判定）
        bottom_z = target_pos[2] - half_lwh[2]
        for r_z in resting_surfaces:
            if 0 <= (bottom_z - r_z) <= 0.05:
                effective_start_z = 0.0
                break

        # 3. 上の判定（頭打ち回避）
        top_z = target_pos[2] + half_lwh[2]
        if effective_start_z > 0.0: # 直置きでない場合のみチェック
            for c_z in ceiling_surfaces:
                clearance = c_z - top_z
                # 天井までの隙間が「予定の浮遊量 + マージン」より少ない場合
                if 0 <= clearance < (effective_start_z + self.ceiling_margin):
                    # ぶつからないギリギリの高さにクリッピングする
                    effective_start_z = max(0.0, clearance - self.ceiling_margin - 0.0005)
                    break

        rel_z = min(container.height + container.buffer - container.thickness - half_lwh[2] - self.start_margin, rel_target[2] + effective_start_z) # 少し上から落とす
        rel_start = (rel_x, -container.width/2, rel_z)
        start_pos = container.local_to_global(rel_start)

        # 手荷物の生成
        target_orn = p.getQuaternionFromEuler(ORNS[target_orn_idx])
        item_id = item.spawn(self.client, initial_pos=start_pos, initial_orn=target_orn)
        state_id = self.client.saveState()

        print(f'transport item {item.index}')

        # Y軸方向に動かす(最初に目標とz座標は合わせている)
        target_0 = (start_pos[0], target_pos[1], start_pos[2])
        steps = max(math.ceil(abs(target_0[1]-start_pos[1])/step_len), 1)
        is_packable, current_pos = self._move_item(container=container, item=item, item_id=item_id, start_pos=start_pos,
                                                   target_pos=target_0, target_orn=target_orn, steps=steps)
        if not is_packable:
            print(f'item {item.index} not packable. removed.')
            self.client.restoreState(stateId=state_id)
            self.client.removeState(state_id)
            item.remove(client=self.client)
            self.client.stepSimulation()

            return is_packable

        # X軸方向に動かす
        target_1 = (target_pos[0], target_pos[1], start_pos[2])
        steps = max(math.ceil(abs(target_1[0]-current_pos[0])/step_len), 1)
        is_packable, current_pos = self._move_item(container=container, item=item, item_id=item_id, start_pos=current_pos,
                                                   target_pos=target_1, target_orn=target_orn, steps=steps)


        # 検証が終わったら状態を元に戻し、メモリを解放する
        self.client.restoreState(stateId=state_id)
        self.client.removeState(state_id)
        if not is_packable:
            print(f'item {item.index} not packable. removed.')
            item.remove(client=self.client)
            self.client.stepSimulation()

        return is_packable


    def place_item(self, item: Item | None, target_pos: tuple[float, float, float], target_orn_idx: int) -> bool:
        if item is None:
            return False
        target_orn = p.getQuaternionFromEuler(ORNS[target_orn_idx])
        displacement_threshold = self.config['displacement_threshold']
        angle_displacement_threshold = self.config.get('angle_displacement_threshold', 30)

        # 1. 干渉チェックがOKだった目標位置へいきなりワープさせる
        item.set_pose(self.client, target_pos, target_orn)

        # 2. ワープ直後の「定着（Settling）」のための短い物理演算
        for _ in range(self.config['settle_wait_step']):
            self.client.stepSimulation()
            if self.render_mode == "human": 
                time.sleep(1./240.)


        # 3. 定着後に荷物が大きく崩れ落ちていないか（安定性）を最終チェックする
        final_pos, final_orn = item.get_pose(self.client)

        # 目標位置と最終位置が大きくズレていれば「配置後に崩れた」と判定
        displacement = np.linalg.norm(np.array(final_pos) - np.array(target_pos))
        dot_product = abs(sum(a*b for a, b in zip(target_orn, final_orn))) # type: ignore
        dot_product = min(1.0, dot_product)
        angle_diff = 2 * math.acos(dot_product)
        if displacement > displacement_threshold or angle_diff > math.radians(angle_displacement_threshold):
            item.remove(client=self.client)
            self.client.stepSimulation()
            print(f'Not safe place (disp: {displacement:.2f}m, angle: {math.degrees(angle_diff):.1f}deg). Item {item.index} removed.')
            return False
        print(f'place item {item.index}')
        return True


    def _move_item(self, container: Container, item: Item, item_id: int | None, start_pos: tuple[float,float, float],
                   target_pos: tuple[float, float, float], target_orn: tuple[float, float, float, float],
                   steps: int) -> tuple[bool, tuple[float, float, float]]:
        is_packable = True
        # スタートから目標位置までを分割し少しずつ動かしながら判定
        for i in range(steps + 1):
            # 線形補間で経路上の現在座標を計算
            fraction = i / steps
            current_pos = (
                start_pos[0] + (target_pos[0] - start_pos[0]) * fraction,
                start_pos[1] + (target_pos[1] - start_pos[1]) * fraction,
                start_pos[2] + (target_pos[2] - start_pos[2]) * fraction
            )
            # 物理時間を進めず、荷物を経路上の座標に強制配置
            item.set_pose(self.client, current_pos, target_orn)
            self.client.performCollisionDetection()
            
            # getClosestPoints で指定距離(safety_margin)内にあるオブジェクトを検出
            for other_item in container.packed_items:
                if other_item.pybullet_id is not None:
                    closest_points = self.client.getClosestPoints(
                        bodyA=item_id,
                        bodyB=other_item.pybullet_id,
                        distance=self.safety_margin
                    )

                    # 戻り値がある＝マージン内に何かがある
                    for pt in closest_points:
                        body_b = pt[2] # 接触相手のオブジェクトID
                        if body_b == other_item.pybullet_id:
                            print(f'collision: {item.index} and {other_item.index} at {current_pos}, distance {pt[8]}')
                            is_packable = False
                            break


            # もしあれば棚の衝突判定
            if container.require_shelf:
                closest_points = self.client.getClosestPoints(
                    bodyA=item_id,
                    bodyB=container.shelf_bullet_id,
                    distance=self.safety_margin
                )
        
                for pt in closest_points:
                    body_b = pt[2] # 接触相手のオブジェクトID
                    if body_b == container.shelf_bullet_id:
                        print(f'collision: {item.index} and {body_b}(shelf) at {current_pos}')
                        is_packable = False
                        break

            # 左側の小さい棚との衝突判定
            closest_points = self.client.getClosestPoints(
                bodyA=item_id,
                bodyB=container.small_shelf_bullet_id,
                distance=self.safety_margin
            )

            for pt in closest_points:
                body_b = pt[2] # 接触相手のオブジェクトID
                if body_b == container.small_shelf_bullet_id:
                    print(f'collision: {item.index} and {body_b}(small shelf) at {current_pos}, distance {pt[8]}')
                    is_packable = False
                    break


            if not is_packable:
                break

        return is_packable, current_pos