import pybullet as p
import numpy as np
from pybullet_utils.bullet_client import BulletClient


class Camera:
    """
    コンテナの状態を撮影し, ハイトマップ(高さ画像)を生成するクラス
    """
    def __init__(self, client: BulletClient, config: dict):
        """
        config:
            target_pos (list): カメラが注視する中心座標 [x, y, z]
            distance (float): 注視点からの距離
            yaw, pitch, roll (float): カメラの回転角度（度）。
                                      pitch=-90で真下(トップダウン)を見る。
                                      横から見る場合は pitch=0 などに調整。
            img_width, img_height (int): 出力画像の解像度
            fov (float): 視野角
            near_val, far_val (float): 撮影範囲の最小・最大距離
        """
        self.client = client
        self.width = config['img_width']
        self.height = config['img_height']
        self.near = config['near_val']
        self.far = config['far_val']
        self.fov = config['fov']


        # カメラの位置・姿勢（View Matrix）と 投影設定（Projection Matrix）を計算
        self.distance = config['distance']
        self.yaw = config['yaw']
        self.pitch = config['pitch']
        self.roll = config['roll']
        self.view_matrix = self.client.computeViewMatrixFromYawPitchRoll(
            cameraTargetPosition=config['target_pos'],
            distance=self.distance,
            yaw=self.yaw,
            pitch=self.pitch,
            roll=self.roll,
            upAxisIndex=2  # Z軸を上とする
        )
        
        self.proj_matrix = self.client.computeProjectionMatrixFOV(
            fov=self.fov,
            aspect=float(self.width) / self.height,
            nearVal=self.near,
            farVal=self.far
        )

        self.camera_y = config['target_pos'][1] + config['distance']


    def get_heightmap(self, center: tuple[float, float, float], lwh: tuple[float, float, float]):
        """
        現在のコンテナ状態を撮影し、デプスマップを返す
        
        Returns:
            np.array: 形状 (height, width) の2次元配列。値はメートル単位の高さ。
        """
        # カメラの位置・姿勢（View Matrix）と 投影設定（Projection Matrix）を計算
        # コンテナの奥行き
        c_depth = lwh[1]
        self.door_y = center[1] - (c_depth / 2.0)
        distance = 0.15*c_depth # カメラ視点は余裕を持っておく

        # カメラ座標を「扉側(Y軸マイナス側)」に直接固定する
        camera_eye = [center[0], self.door_y - distance, center[2]]
        self.view_matrix = self.client.computeViewMatrix(
            cameraEyePosition=camera_eye,
            cameraTargetPosition=center,
            cameraUpVector=[0, 0, 1] # Z軸が上
        )

        # --- 歪み防止パディングロジック ---
        # 1. 共有メモリで固定されている画像の縦横比
        target_aspect = self.width / self.height
        
        # 2. 実際のコンテナの縦横比
        container_aspect = lwh[0] / lwh[2]

        # 3. 歪まないように、撮影する物理的な枠(phys_w, phys_h)を広げる
        if container_aspect > target_aspect:
            # コンテナの方が横長な場合 -> 幅に合わせて、高さを余分に映す（上下に余白）
            phys_w = lwh[0]
            phys_h = lwh[0] / target_aspect
        else:
            # コンテナの方が縦長(または同じ)場合 -> 高さに合わせて、幅を余分に映す（左右に余白）
            phys_h = lwh[2]
            phys_w = lwh[2] * target_aspect

        half_w = phys_w / 2.0
        half_h = phys_h / 2.0
        
        left = -half_w
        right = half_w
        bottom = -half_h
        top = half_h
        
        self.near = distance - 0.01 # 撮影始点をドアの少し手前にする
        self.far  = distance + c_depth + 0.1

        # 直交投影マトリクス
        self.proj_matrix = [
            2.0 / (right - left), 0.0, 0.0, 0.0,
            0.0, 2.0 / (top - bottom), 0.0, 0.0,
            0.0, 0.0, -2.0 / (self.far - self.near), 0.0,
            -(right + left) / (right - left), -(top + bottom) / (top - bottom), -(self.far + self.near) / (self.far - self.near), 1.0
        ]

        # PyBulletから画像データを取得
        # p.DIRECTモードでも動作するように ER_TINY_RENDERER を指定するのが重要
        _, _, _, depth_buffer, _ = self.client.getCameraImage(
            width=self.width,
            height=self.height,
            viewMatrix=self.view_matrix,
            projectionMatrix=self.proj_matrix,
            renderer=p.ER_TINY_RENDERER
        )

        # デプスバッファをNumPy配列に変換 reshape: (H, W)
        depth_buffer = np.array(depth_buffer).reshape((self.height, self.width))
        depth_linear = self.near + (self.far - self.near) * depth_buffer

        available_depth = depth_linear - distance
        available_depth = np.where(available_depth > c_depth + 0.05, 0.0, available_depth)
        available_depth = np.maximum(available_depth, 0.0)


        return available_depth.astype(np.float32)


    def get_rgb_image(self):
        """デバッグ確認用のRGB画像取得(人間が見る用)"""
        w, h, rgb, _, _ = self.client.getCameraImage(
            width=self.width,
            height=self.height,
            viewMatrix=self.view_matrix,
            projectionMatrix=self.proj_matrix,
            renderer=p.ER_TINY_RENDERER
        )
        # RGB配列 (H, W, 4) -> RGBA
        return np.array(rgb, dtype=np.uint8).reshape((self.height, self.width, 4))

