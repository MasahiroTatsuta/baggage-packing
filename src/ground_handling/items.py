import pybullet as p
from dataclasses import dataclass
from pybullet_utils.bullet_client import BulletClient


@dataclass
class Item:
    """単一の荷物(直方体)の物理プロパティとPyBullet上の実体を管理するクラス"""
    index: int
    length: float
    width: float
    height: float
    mass: float = 1.0
    is_prioritized: bool = False
    is_soft: bool = False
    belongs_to: int | None = None          # 置かれているコンテナ

    pos: tuple[float, float, float] | None = None           # 世界座標系による位置座標
    orn: tuple[float, float, float, float] | None = None    # 姿勢を表すクオータニオン

    lateralFriction: float = 0.8      # 横摩擦：横滑りを防ぐ（段ボールなどに近い値）
    rollingFriction: float = 0.01     # 転がり摩擦：角を支点にして不自然に転がるのを防ぐ
    spinningFriction: float = 0.01    # ねじれ摩擦：接触面での回転ズレを防ぐ
    restitution: float = 0.0          # 反発係数：落下時にバウンドさせず、エネルギーを吸収させる
    angularDamping: float = 0.8

    # is_soft=Trueの時に有効になる
    contactStiffness: float = 5000    # 接触剛性（低いほどクッションのようにめり込む）
    contactDamping: float = 500       # 接触減衰（めり込んだ後の揺れを抑える）
    linearDamping: float = 0.8        # 並進運動の減衰（高いほどすぐに滑り止まる）


    def __post_init__(self):
        self.color: list[float] = [0.8, 0.6, 0.4, 1.0] if not self.is_prioritized else [0.0, 1.0, 0.0, 1.0]
        if self.is_soft:
            self.color = [0, 0, 1, 1]

        self.volume: float = self.length * self.width * self.height
        self.pybullet_id: int | None = None


    def get_info(self) -> dict[str, int | float | tuple ]:
        """物理空間における最新の位置や姿勢情報を取得する"""
        info = {
            'index': self.index,
            'length': self.length,
            'width': self.width,
            'height': self.height,
            'mass': self.mass,
            'is_prioritized': self.is_prioritized,
            'is_soft': self.is_soft,
            'belongs_to': self.belongs_to,
            'pos': self.pos,
            'orn': self.orn,
            'lateralFriction': self.lateralFriction,
            'rollingFriction': self.rollingFriction,
            'spinningFriction': self.spinningFriction,
            'restitution': self.restitution,
            'angularDamping': self.angularDamping
        }
        if self.is_soft:
            info.update({
                'contactStiffness': self.contactStiffness,
                'contactDamping': self.contactDamping,
                'linearDamping': self.linearDamping
            })

        return info


    def spawn(self, client: BulletClient, initial_pos: tuple[float, float, float] = (0.0, 0.0, 5.0), initial_orn: tuple[float, float, float, float] = (0,0,0,1)) -> int | None:
        """PyBulletの物理空間に荷物を生成(スポーン)する. 物理空間上のIDが付与される"""
        
        # PyBulletのボックス生成は「サイズ（長さ）」ではなく「中心からのハーフサイズ」を指定する
        half_extents = [self.length / 2, self.width / 2, self.height / 2]
        
        # 1. 衝突判定用の形状（Collision Shape）を作成
        col_id = client.createCollisionShape(p.GEOM_BOX, halfExtents=half_extents)
        
        # 2. 描画用の形状（Visual Shape）を作成
        vis_id = client.createVisualShape(p.GEOM_BOX, halfExtents=half_extents, rgbaColor=self.color)
        
        # 3. 質量と形状を結びつけて物理空間に実体を生成
        self.pybullet_id = client.createMultiBody(
            baseMass=self.mass,
            baseCollisionShapeIndex=col_id,
            baseVisualShapeIndex=vis_id,
            basePosition=initial_pos,
            baseOrientation=initial_orn
        )
        
        # 4. 安定した積載のための物理パラメータのチューニング
        dynamics_params = {
            'lateralFriction': self.lateralFriction,
            'rollingFriction': self.rollingFriction,
            'spinningFriction': self.spinningFriction,
            'restitution': self.restitution,
            'angularDamping': self.angularDamping
        }
        if self.is_soft:
            dynamics_params.update(
                {
                    'contactStiffness': self.contactStiffness,
                    'contactDamping': self.contactDamping,
                    'linearDamping': self.linearDamping  # ベースの値を上書き
                }
            )

        client.changeDynamics(
            self.pybullet_id, 
            -1, # -1はベースリンク（荷物そのもの）を指す
            **dynamics_params
        )

        text_label = f"Item {self.index}"
        
        # 荷物の中心からZ軸方向（上）に少しずらした位置をローカル座標で指定
        # 荷物の高さの半分 + 5cm 上に表示
        local_pos = [0, 0, (self.height / 2) + 0.05]
        
        self.debug_text_id = client.addUserDebugText(
            text=text_label,
            textPosition=local_pos,
            textColorRGB=[1.0, 1.0, 1.0],  # 白文字
            textSize=1.0,                  # 文字のサイズ
            parentObjectUniqueId=self.pybullet_id, # 荷物に追従させる
            parentLinkIndex=-1             # ベースリンクに固定
        )

        return self.pybullet_id


    def remove(self, client: BulletClient):
        """物理空間から生成した手荷物を削除"""
        if self.pybullet_id is not None:
            client.removeBody(self.pybullet_id)
            self.pybullet_id = None
            self.pos = None
            self.orn = None


    def set_pose(self, client: BulletClient, pos: tuple[float, float, float], orn: tuple[float, float, float ,float]):
        """シミュレーションの時間を進めずに指定された座標・姿勢へ瞬時にワープさせる"""
        if self.pybullet_id is not None:
            client.resetBasePositionAndOrientation(self.pybullet_id, pos, orn)


    def get_pose(self, client: BulletClient) -> tuple[tuple[float, float, float], tuple[float, float, float, float]] | tuple[None, None]:
        """現在の座標(pos)と姿勢(orn: クォータニオン)を取得する"""
        if self.pybullet_id is not None:
            pos, orn = client.getBasePositionAndOrientation(self.pybullet_id)
            return pos, orn
        return None, None


    def register_pos_orn(self, container_index: int, pos: tuple[float, float, float], orn: tuple[float, float, float, float]):
        self.belongs_to = container_index
        self.pos = pos
        self.orn = orn # クオータニオン


class ItemStreamManager:
    """あらかじめ全荷物情報を保持し, コンベア上の k 個の荷物をプールとして管理・供給するクラス"""
    
    def __init__(self, config: dict):
        """
        config:
            - item_list: list[Item], 全手荷物のリスト
            - lookahead: int,  見える手荷物の数の上限
            - visible_pool: list[Item], プールされていて見えている手荷物一覧リスト
        """
        self.all_items: list[Item] = [Item(**item_config) for item_config in config['item_list']]
        self.total_items: int = len(self.all_items)
        self.all_indices: set = {item.index for item in self.all_items}
        self.lookahead_k: int = config['look_ahead']
        self.max_space: int = config['max_space'] # 補填するタイミングを決める残りの手荷物の数
        assert self.lookahead_k >= self.max_space

        self.visible_pool: list[Item] = [Item(**item_config) for item_config in config['visible_pool']]
        self.current_index: int = 0 # 次にプールへ補充する all_items のインデックス


    def set_order(self, order: list[int]) -> None:
        order_map = {v: i for i, v in enumerate(order)}
        self.all_items.sort(key=lambda x: order_map[x.index])


    def get_items_of_visible_pool(self) -> list[dict]:
        info_list = [
            item.get_info() for item in self.visible_pool
        ]
        return info_list


    def get_all_items(self) -> list[dict]:
        info_list = [
            item.get_info() for item in self.all_items
        ]
        return info_list


    def reset(self):
        """エピソード開始時にストリームをリセットし, 最初のk個をプールに設定する"""
        self.current_index = 0
        self.visible_pool = []
        for _ in range(self.lookahead_k):
            self._refill_item()


    def _refill_item(self):
        """全荷物リストから新しい荷物をプールに補充する"""
        if self.current_index < self.total_items:
            item = self.all_items[self.current_index]
            self.visible_pool.append(item)
            self.current_index += 1


    def get_item(self, pool_index: int) -> Item | None:
        """プール内の指定インデックスのアイテム実体を取得する"""
        if 0 <= pool_index < len(self.visible_pool):
            return self.visible_pool[pool_index]
        return None
        

    def pop_and_refill(self, pool_index: int) -> None:
        """
        プールの状態を更新（取られたときや補填するとき）
        荷物を選択後, プールから取り除き奥から新しい荷物を補充する
        """
        if 0 <= pool_index < len(self.visible_pool):
            self.visible_pool.pop(pool_index)
            # 1つ減ったので新しい荷物をプールに追加
            num_space = self.lookahead_k - len(self.visible_pool)
            if num_space >= self.max_space:
                for _ in range(num_space):
                    self._refill_item()


    def is_empty(self) -> bool:
        """プール内のすべての荷物が処理され空になったか"""
        return len(self.visible_pool) == 0
