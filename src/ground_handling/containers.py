import pybullet as p
import math
import os
import tempfile
from pybullet_utils.bullet_client import BulletClient
from dataclasses import dataclass

from .utils import write_open_cut_corner_cup_obj, write_corner_lid_obj, aff
from .items import Item


@dataclass
class Container:
    """単一のコンテナの情報と状態を保持するデータクラス"""

    index: int       # 0から始まる整数値(デプスマップ用)
    offset_x: float  # X軸上の配置オフセット
    thickness: float
    length: float
    width: float
    height: float
    packed_items: list[Item]
    require_shelf: bool = False
    buffer: float = 0.01
    cut_x: float = 0.3
    cut_y: float = 0.3
    is_prioritized: bool = False


    def __post_init__(self):
        # PyBulletのボディIDを保持
        self.container_bullet_id: int
        self.top_bullet_id: int
        self.small_shelf_bullet_id: int
        self.shelf_bullet_id: int

        # 状態管理用
        self.center: tuple[float, float, float]  # 中心座標
        self.points: list[tuple]                 # 正位置にした後の各面の代表点
        self.n_vecs: list[tuple]                 # 正位置にした後の各面の法線ベクトル
        self.volume: float                       # コンテナの有効体積


    def create(self, client: BulletClient, save_obj: bool=False) -> None:
        """PyBullet上で1つの横空きコンテナを生成"""
        hz = self.height / 2
        color = [0.2, 0.7, 0.9, 0.3]

        with tempfile.TemporaryDirectory() as tmp_dir:
            # コンテナ本体の生成
            fname = os.path.join(tmp_dir, f'container_{self.index}.obj')

            points, n_vecs = write_open_cut_corner_cup_obj(fname,
                                                           width = self.length, height = self.height,
                                                           cut_x = self.cut_x, cut_y = self.cut_y, depth=self.width,
                                                           wall = self.thickness, bottom = self.thickness)
            rot = [[1, 0, 0],
                   [0, math.cos(math.pi/2), -math.sin(math.pi/2)],
                   [0, math.sin(math.pi/2), math.cos(math.pi/2)]]
            orn = p.getQuaternionFromEuler([math.pi/2, 0, 0])
            pos = (0.0, 0.0, hz + self.buffer) # 相対座標
            aff_points = aff(points, rot, pos) # 各面の代表点の原点から見た座標
            aff_n_vecs = aff(n_vecs, rot, intercept = (0, 0, 0)) # 各面の法線ベクトル

            ## オフセット分移動する
            self.center = self.local_to_global(pos)                              # コンテナ本体の中心座標(世界座標系)
            self.points = [self.local_to_global(point) for point in aff_points]  # 各面の代表点の座標(世界座標系)
            self.n_vecs = aff_n_vecs                                             # 法線ベクトルは平行移動に対して不変

            self.container_bullet_id = self._create_body(client = client, fname = fname, pos = self.center, orn = orn, color = color)

            # 蓋の生成
            top_fname = os.path.join(tmp_dir, f'top_{self.index}.obj')
            write_corner_lid_obj(top_fname,
                                 width = self.length, height = self.height,
                                 cut_x = self.cut_x, cut_y = self.cut_y, depth = self.width,
                                 lid_thickness = self.thickness)        
            self.top_bullet_id = self._create_body(client = client, fname = top_fname, pos = self.center, orn = orn, color = color)


            if self.require_shelf:
                # 棚の生成
                orn = p.getQuaternionFromEuler([math.pi/2, 0, 0])
                global_pos = self.local_to_global((0, self.width/4, self.height/2 + self.thickness/2 + self.buffer))
                color = [0.0, 0.0, 1.0, 1.0]
                self.shelf_bullet_id = self._create_shelf(client = client, pos = global_pos, orn = orn, color = color)

                # 小さい棚生成
                global_pos = self.local_to_global((-self.length/2 + self.cut_x/2 + self.thickness, 0, self.height/2 + self.thickness/2 + self.buffer))
                color = [1.0, 0, 0, 1.0]
                self.small_shelf_bullet_id = self._create_small_shelf(client = client, pos = global_pos, orn = orn, color = color)

            else:
                # 脇部分の小さい棚を生成
                orn = p.getQuaternionFromEuler([math.pi/2, 0, 0])
                global_pos = self.local_to_global((-self.length/2 + self.cut_x/2 + self.thickness, 0, self.height/2 + self.thickness/2 + self.buffer))
                color = [1.0, 0, 0, 1.0]
                self.small_shelf_bullet_id = self._create_small_shelf(client = client, pos = global_pos, orn = orn, color = color)


        # 有効体積の計算
        inner_length = self.length - 2 * self.thickness
        inner_width  = self.width - 2 * self.thickness
        inner_height = self.height - self.thickness - self.buffer

        base_volume = inner_length * inner_width * inner_height
        cut_volume = 0.5 * (self.cut_x - self.thickness) * (self.cut_y - self.thickness) * inner_width
        small_shelf_volume = self.cut_x * self.thickness * inner_width
        shelf_volume = 0.0
        if self.require_shelf:
            shelf_width = (self.width / 2.0) - (2 * self.thickness)
            shelf_volume = inner_length * self.thickness * shelf_width

        self.volume = base_volume - cut_volume - small_shelf_volume - shelf_volume

        if save_obj:
            print('save obj.')
            os.makedirs('./_tmp', exist_ok=True)
            fname = os.path.join('./_tmp', f'container_{self.index}.obj')
            points, n_vecs = write_open_cut_corner_cup_obj(fname,
                                                           width = self.length, height = self.height,
                                                           cut_x = self.cut_x, cut_y = self.cut_y, depth=self.width,
                                                           wall = self.thickness, bottom = self.thickness)
            top_fname = os.path.join('./_tmp', f'top_{self.index}.obj')
            write_corner_lid_obj(top_fname,
                                 width = self.length, height = self.height,
                                 cut_x = self.cut_x, cut_y = self.cut_y, depth = self.width,
                                 lid_thickness = self.thickness)        


    def create_cap(self, client: BulletClient) -> int:
        """蓋を生成する"""
        color = [0.2, 0.7, 0.9, 0.3]
        orn = p.getQuaternionFromEuler([math.pi/2, 0, 0])
        pos = self.local_to_global((self.cut_x/2, -self.width/2, self.height/2+self.buffer))
        half_ext = [self.length/2-self.cut_x/2, self.height/2, self.thickness/2]


        box_vis = client.createVisualShape(
            shapeType=p.GEOM_BOX,
            halfExtents=half_ext,
            rgbaColor=color
        )
        box_col = client.createCollisionShape(
            shapeType=p.GEOM_BOX,
            halfExtents=half_ext
        )
        cap_id = client.createMultiBody(
            baseMass=0,
            baseVisualShapeIndex = box_vis,
            baseCollisionShapeIndex=box_col,
            basePosition=pos,
            baseOrientation=orn
        )

        return cap_id


    def _create_body(self, client: BulletClient, fname: str, pos: tuple[float, float, float], orn: tuple[float, float, float, float], color: list[float]) -> int:
        visual_id = client.createVisualShape(
            shapeType=p.GEOM_MESH,
            fileName=fname,
            meshScale=[1, 1, 1],
            rgbaColor=color
        )

        collision_id = client.createCollisionShape(
            shapeType=p.GEOM_MESH,
            fileName=fname,
            meshScale=[1, 1, 1],
            flags=p.GEOM_FORCE_CONCAVE_TRIMESH
        )


        bullet_id = client.createMultiBody(
            baseMass=0,
            baseCollisionShapeIndex=collision_id,
            baseVisualShapeIndex=visual_id,
            basePosition=pos,
            baseOrientation=orn
        )

        client.changeDynamics(
            bullet_id, 
            -1, 
            lateralFriction=0.8,
            rollingFriction=0.01,
            spinningFriction=0.01
        )

        return bullet_id


    def _create_small_shelf(self, client: BulletClient, pos: tuple[float, float, float], orn: tuple[float, float, float, float], color: list[float]):
        box_vis = client.createVisualShape(
            shapeType=p.GEOM_BOX,
            halfExtents=[self.cut_x/2, self.thickness/2, self.width/2-self.thickness],
            rgbaColor=color
        )

        box_col = client.createCollisionShape(
            shapeType=p.GEOM_BOX,
            halfExtents=[self.cut_x/2, self.thickness/2, self.width/2-self.thickness]
        )
        shelf_id = client.createMultiBody(
            baseMass=0,
            baseVisualShapeIndex=box_vis,
            baseCollisionShapeIndex=box_col,
            basePosition=pos,
            baseOrientation=orn
        )

        return shelf_id


    def _create_shelf(self, client: BulletClient, pos: tuple[float, float, float], orn: tuple[float, float, float, float], color: list[float]):
        box_vis = client.createVisualShape(
            shapeType=p.GEOM_BOX,
            halfExtents=[self.length/2-self.thickness/2, self.thickness/2, self.width/4-self.thickness],
            rgbaColor=color
        )

        box_col = client.createCollisionShape(
            shapeType=p.GEOM_BOX,
            halfExtents=[self.length/2-self.thickness/2, self.thickness/2, self.width/4-self.thickness]
        )
        shelf_id = client.createMultiBody(
            baseMass=0,
            baseVisualShapeIndex=box_vis,
            baseCollisionShapeIndex=box_col,
            basePosition=pos,
            baseOrientation=orn
        )

        return shelf_id


    def local_to_global(self, local_pos: tuple[float, float, float]):
        """相対座標をPyBullet上の世界座標に変換"""
        return (local_pos[0] + self.offset_x, local_pos[1], local_pos[2])


    def global_to_local(self, global_pos: tuple[float, float, float]):
        """世界座標から相対座標に変換"""
        return (global_pos[0] - self.offset_x, global_pos[1], global_pos[2])


    def add_item(self, item: Item):
        """コンテナに手荷物を追加する"""
        self.packed_items.append(item)


    def update_packed_items(self, client: BulletClient):
        for item in self.packed_items:
            pos, orn = item.get_pose(client=client)
            if pos is not None and orn is not None:
                item.register_pos_orn(container_index=self.index, pos=pos, orn=orn)


class MultiContainerManager:
    """複数コンテナの生成と全体管理を行うマネージャクラス"""
    def __init__(self, client: BulletClient, config: dict):
        self.client = client
        self.config = config
        self.containers: list[Container] = []   # Containerインスタンスのリスト


    def build(self):
        """指定された数の横空きコンテナをX軸に並べて構築する"""
        spacing = self.config['spacing']
        for i, container_config in enumerate(self.config['container_list']):
            # X軸方向にspacing分ずつずらして配置
            offset_x = i * spacing
            container = Container(offset_x=offset_x, **container_config)

            # PyBullet上に実体を生成
            container.create(self.client)
            self.containers.append(container)

            # 手荷物があれば合わせて生成
            container.packed_items = [Item(**item_config) for item_config in container_config['packed_items']]
            for item in container.packed_items:
                if item.pos is not None and item.orn is not None:
                    item.spawn(self.client, initial_pos=item.pos, initial_orn=item.orn)

        # 物理演算を進めておく
        self.client.setGravity(0, 0, -9.8)
        max_steps = 480
        for step in range(max_steps):
            self.client.stepSimulation()
            if step > 60: # 最初の自由落下が終わった頃からチェック開始
                all_sleeping = True
                for container in self.containers:
                    for item in container.packed_items:
                        if item.pybullet_id is not None:
                            linear_v, angular_v = self.client.getBaseVelocity(item.pybullet_id)
                            # 速度が微小(1mm/s以下)でなければまだ動いていると判定
                            if sum(abs(v) for v in linear_v) > 0.001 or sum(abs(v) for v in angular_v) > 0.001:
                                all_sleeping = False
                                break
                    if not all_sleeping: break
                
                if all_sleeping:
                    break # 完全に安定したため早期にシミュレーション停止

        # 着地して安定した「真の座標」を取得し、エージェント用に登録
        for container in self.containers:
            for item in container.packed_items:
                pos, orn = item.get_pose(self.client)
                if pos is not None and orn is not None:
                    item.register_pos_orn(container.index, pos, orn)


    def get_item_info_in_containers(self) -> list[dict[str, int | float | list[dict]]]:
        """policy側に渡す観測情報の生成"""
        item_list_in_containers = [
            {
                'index': container.index,
                'length': container.length,
                'width': container.width,
                'cut_x': container.cut_x,
                'cut_y': container.cut_y,
                'height': container.height,
                'thickness': container.thickness,
                'n_vecs': container.n_vecs,
                'points': container.points,
                'volume': container.volume,
                'center': container.center,
                'shelf': container.require_shelf,
                'is_prioritized': container.is_prioritized,
                'packed_items': [item.get_info() for item in container.packed_items]

            } for container in self.containers
        ]

        return item_list_in_containers


    def get_container(self, index: int) -> Container:
        """指定されたインデックスのコンテナオブジェクトを取得"""
        return self.containers[index]


    def update_and_add_item_to_container(self, container_id: int, item: Item | None):
        """コンテナの中身の更新と安全に配置された手荷物を座標とともに登録"""
        # コンテナの中身の情報を更新
        for container in self.containers:
            container.update_packed_items(self.client)

        # 新しい手荷物情報の追加
        if item is not None:
            pos, orn = item.get_pose(client=self.client)
            if pos is not None and orn is not None:
                item.register_pos_orn(self.get_container(container_id).index, pos, orn)
                self.get_container(container_id).add_item(item)


    def clear(self):
        self.containers = []
