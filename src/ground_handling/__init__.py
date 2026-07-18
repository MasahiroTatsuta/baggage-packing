from abc import ABC, abstractmethod
from typing import Any


class Agent(ABC):
    """
    荷物配置エージェントの抽象基底クラス。
    独自のエージェントを作成する際は、このクラスを継承して各メソッドを実装してください。
    """
    def __init__(self, module_path: str):
        self.module_path = module_path

    @abstractmethod
    def get_init_states(self, init_states: dict[str, Any]) -> None:
        """
        環境の初期状態を受け取る。
        
        Args:
            init_states: コンテナ情報や lookahead_k などの初期設定
        """
        pass

    @abstractmethod
    def optimize(self, item_list: list[dict[str, Any]]) -> list[int]:
        """
        [オフライン最適化] 事前に全ての荷物情報を受け取り、積み込む最適な順序を計算する。
        
        Args:
            item_list: 全荷物の情報リスト
        Returns:
            list[int]: 荷物のインデックスを並べ替えたリスト
        """
        pass

    @abstractmethod
    def policy(self, observation: dict[str, Any]) -> dict[str, Any]:
        """
        [オンライン最適化/逐次実行] 現在の観測状態から次の行動を決定する。
        
        Args:
            observation: 現在のプール内の荷物やコンテナの状況
        Returns:
            dict: 選択した荷物、コンテナ、座標、向きを含むアクション辞書
        """
        pass