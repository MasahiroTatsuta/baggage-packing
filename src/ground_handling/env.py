import numpy as np
import pybullet_data
import pybullet as p
import gymnasium as gym
import time
import os
import random
import atexit
from datetime import datetime
from gymnasium import spaces
from pybullet_utils import bullet_client
from PIL import Image
from multiprocessing import shared_memory

from .containers import MultiContainerManager
from .items import ItemStreamManager
from .validator import PlacementValidator
from .camera import Camera
from .evaluator import Evaluator


class GroundHandlingEnv(gym.Env):
    """
    PyBulletを用いた物理シミュレーションベースの3Dパッキング環境
    横空きコンテナへの「押し込み配置」と強化学習用インターフェースを提供する
    """
    metadata = {'render_modes': ['human', 'rgb_array'], 'render_fps': 30}

    def __init__(self, config: dict[str, dict], verbose: bool, render_mode=None):
        super().__init__()
        self.render_mode = render_mode
        self.config = config
        self.optimize = config['agent']['optimize']
        self.verbose = verbose


        # 1. PyBulletへの接続
        if render_mode == 'human':
            self.client = bullet_client.BulletClient(connection_mode=p.GUI)
            self.client.configureDebugVisualizer(p.COV_ENABLE_GUI, 0) # GUIの余計なメニューを消す [6]
        else:
            self.client = bullet_client.BulletClient(connection_mode=p.DIRECT)
            
        self.client.setAdditionalSearchPath(pybullet_data.getDataPath())
        
        # 2. 各サブモジュールの初期化
        self.container_manager = MultiContainerManager(client = self.client, config = self.config['containers'])
        self.stream_manager = ItemStreamManager(config = self.config['item_stream'])
        self.validator = PlacementValidator(client=self.client, config=self.config['validator'])
        self.camera = Camera(client = self.client, config = self.config['camera'])
        self.evaluator = Evaluator(client = self.client, config = {'inclusion_margin': self.config['validator']['inclusion_margin']})
        self.visualize = self.config['visualizer']['vis']
        if self.visualize:
            self.visualizer = Camera(client=self.client, config=self.config['visualizer']['camera'])
            self.image_dir = os.path.join('images', 'steps', datetime.now().strftime("%Y%m%d_%H%M%S"))
            os.makedirs(self.image_dir, exist_ok=True)


        # 3. Action Space (行動空間) の定義
        self.num_containers = len(self.container_manager.config['container_list'])
        self.action_space = spaces.Dict({
            'item_idx': spaces.Discrete(self.stream_manager.lookahead_k), # プール内のどの荷物を選ぶか
            'container_idx': spaces.Discrete(self.num_containers),
            'place_pos': spaces.Box( # ローカル座標 (x, y, z) のみ指定
                low=np.array([-0.2, -0.2, 0.4], dtype=np.float32),
                high=np.array([0.2, 0.2, 0.8], dtype=np.float32),
                dtype=np.float32
            ),
            'orientation': spaces.Discrete(len(self.config['action']['orientations'])) # 荷物の向き
        })
        self.pos_lim = config['action']['pos_lim']

        # 4. Observation Space (観測空間) の定義(テスト用)
        img_h, img_w = 64, 64
        self.observation_space = spaces.Dict({
            "depth_map": spaces.Box(low=0, high=3.0, shape=(img_h, img_w), dtype=np.float32),
            "item_features": spaces.Box(low=0, high=5.0, shape=(self.stream_manager.lookahead_k, 3), dtype=np.float32)
        })

        self.max_num_containers=config['camera']['num_containers']
        self.shm_shape = (self.max_num_containers, self.camera.height, self.camera.width)
        self.shm_dtype = np.float32
        shm_size = self.shm_shape[0] * self.shm_shape[1] * self.shm_shape[2] * np.dtype(self.shm_dtype).itemsize

        # 共有メモリ領域の作成
        self.shm = shared_memory.SharedMemory(create=True, size=shm_size)
        # そのバッファを直接参照するNumPy配列を作成
        self.shm_depth_map = np.ndarray(self.shm_shape, dtype=self.shm_dtype, buffer=self.shm.buf)
        atexit.register(self._cleanup_shared_memory_at_exit)

        self.num_step: int = 0
        self.num_total_items: int = 0


    def _cleanup_shared_memory_at_exit(self):
        """プロセス終了時にOSから確実に共有メモリを消し去るメソッド"""
        try:
            self.shm.close()
            self.shm.unlink()
            if self.verbose:
                print("[atexit] 共有メモリを安全に解放しました。")
        except FileNotFoundError:
            pass
        except Exception as e:
            pass



    def reset_settings(self):
        # PyBulletのリセット
        self.client.resetSimulation()
        self.client.setPhysicsEngineParameter(deterministicOverlappingPairs=1)
        self.client.setGravity(0, 0, -9.8) # 重力設定
        self.client.loadURDF('plane.urdf') # 床面の生成

        # コンテナの再構築
        self.container_manager.clear()
        self.container_manager.build()

        self.num_total_items = self.stream_manager.total_items + sum(len(c.packed_items) for c in self.container_manager.containers)

        # デバッグ用のカメラパラメータの調整
        target_position = [0.0, 0.0, 0.0]
        distance = 0
        for c in self.container_manager.containers:
            target_position[0] += c.center[0]
            target_position[1] += c.center[1]
            target_position[2] += c.center[2]
            distance += c.width
        
        target_position[0] /= len(self.container_manager.containers)
        target_position[1] /= len(self.container_manager.containers)
        target_position[2] /= len(self.container_manager.containers)
        distance = 1.8*distance/len(self.container_manager.containers)

        self.client.resetDebugVisualizerCamera(
            cameraDistance=distance,
            cameraYaw=self.config['visualizer']['camera']['yaw'],
            cameraPitch=self.config['visualizer']['camera']['pitch'], # -90で真上から見た図になる
            cameraTargetPosition=target_position
        )


    def reset_item_stream(self):
        # 荷物ストリームの生成
        self.stream_manager.reset()


    def reset(self, seed=None, options=None):
        super().reset(seed=seed)

        if seed is not None:
            random.seed(seed)
            np.random.seed(seed)
        
        observation = self._get_obs()
        if self.verbose:
            print(f'env: {len(self.stream_manager.all_items)}, {len(self.stream_manager.visible_pool)}')
        info = {}

        # 可視化フラグがtrueならスナップショットを保存
        if self.visualize:
            img = Image.fromarray(self.visualizer.get_rgb_image())
            img.save(os.path.join(self.image_dir, f"{self.num_step:04}.png"))
        self.num_step += 1

        return observation, info


    def get_init_states(self) -> dict[str, list | int | None]:

        return {
            'optimize': self.optimize,
            'lookahead_k': self.stream_manager.lookahead_k,
            'container_list': self.container_manager.get_item_info_in_containers()
        }


    def get_info_for_optimization(self) -> list[dict]:
        return self.stream_manager.get_all_items()


    def set_item_order(self, order) -> bool:
        if isinstance(order, list) and set(order) == self.stream_manager.all_indices:
            self.stream_manager.set_order(order)
            return True
        return False


    def step(self, action):
        """
        1ステップの実行: 行動解析 -> 目標位置検証 -> 物理配置 -> 観測更新
        """
        reward = 0
        start = time.time()
        # ------ 0. 行動の型などを確認 ------
        num_visible_items = len(self.stream_manager.visible_pool)
        valid, truncated, info = self.validator.check_action(action, self.config['action'], self.num_containers, num_visible_items)
        if not valid: # Falseの場合必ずtruncated=True
            return {}, 0, True, truncated, info

        # ------ 1. 行動の解析 ------
        local_xyz = action['place_pos'] # [x, y, z]
        item_idx = action['item_idx']
        container_idx = action['container_idx']
        target_orn_idx = action['orientation']

        ## 選択された荷物データの取得（まだ物理空間には出さない）
        target_item = self.stream_manager.get_item(item_idx)
        target_container = self.container_manager.get_container(container_idx)

        ## 世界座標へ変換
        global_pos = target_container.local_to_global(local_xyz)

        # ------ 2. 事前チェック (めり込み等がないか) ------
        terminated = False
        truncated = False
        is_placed_safe = False
        is_valid = False

        is_included = self.validator.check_inclusion(target_container, target_item, global_pos, target_orn_idx)

        if not is_included:
            terminated = True
        else:
            is_valid = self.validator.check_transport_path(target_container, target_item, global_pos, target_orn_idx)
            if is_valid:
                # ------ 3. 物理空間へのスポーンと配置実行 ------
                is_placed_safe = self.validator.place_item(target_item, global_pos, target_orn_idx)
                if is_placed_safe: # 成功: ストリームから消費し補充する
                    self.container_manager.update_and_add_item_to_container(container_id=container_idx, item=target_item)
                    self.stream_manager.pop_and_refill(item_idx)

                else:
                    ## 失敗: エピソード終了
                    terminated = True
            else:
                terminated = True

        ## すべての荷物を積み終わったら終了
        if self.stream_manager.is_empty():
            terminated = True

        # -------- 4. 観測情報の出力 ---------
        observation = self._get_obs()
        if self.verbose:
            print(f'env : {len(self.stream_manager.all_items)}, {len(self.stream_manager.visible_pool)}(elapsed time {round(time.time()-start,3)} sec)')
        if self.render_mode=='human':
            time.sleep(1.0)

        # 可視化フラグがtrueならスナップショットを保存
        if self.visualize:
            img = Image.fromarray(self.visualizer.get_rgb_image())
            img.save(os.path.join(self.image_dir, f"{self.num_step:04}.png"))
        
        self.num_step += 1

        return observation, reward, terminated, truncated, {'status': {'is_included': is_included, 'is_valid': is_valid, 'is_placed_safe': is_placed_safe}}


    def evaluate(self):
        evaluation_report = self.evaluator.evaluate(self.container_manager.containers, self.num_total_items)

        return evaluation_report


    def _get_obs(self):
        """現在の観測(荷物情報)を取得してDictで返す"""

        for container in self.container_manager.containers:
            # カメラを対象のコンテナの前に移動させる
            lwh = container.length, container.width, container.height
            
            # 画像を取得して、共有メモリの i 番目のチャンネルに直接上書き
            depth_map = self.camera.get_heightmap(container.center, lwh)
            np.copyto(self.shm_depth_map[container.index], depth_map)

        return {
            'shm_name': self.shm.name,
            'shm_shape': self.shm_shape,
            'shm_dtype': 'float32',
            'optimize': self.optimize,
            'lookahead_k': self.stream_manager.lookahead_k,
            'container_list': self.container_manager.get_item_info_in_containers(),
            'pool_list': self.stream_manager.get_items_of_visible_pool()
        }


    def close(self):
        print('close env.')
        self.client.disconnect()
        if hasattr(self, 'shm'):
            self.shm.close()
            self.shm.unlink()
