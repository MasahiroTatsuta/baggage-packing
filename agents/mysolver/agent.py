import os

import numpy as np

from . import ordering
from . import planner

# 開発中の反復を速くするための環境変数。未設定なら本番想定の ordering.DEFAULT_TIME_BUDGET
# (165s、180sタイムアウトに対する安全マージン込み)を使う。最終計測時は未設定のまま
# (=170s相当のフル予算)で回すこと。
OPTIMIZE_BUDGET_ENV = 'MYSOLVER_OPTIMIZE_BUDGET'


class Agent:
    """
    合法貪欲ベースラインAgent。
    policy は毎回 observation から状態を再構築し、内部状態(前ステップの記憶)には依存しない。
    """

    def __init__(self, module_path: str):
        self._lookahead_k = None
        self._container_list = None

    def get_init_states(self, init_states: dict) -> None:
        self._lookahead_k = init_states.get('lookahead_k')
        self._container_list = init_states.get('container_list')

    def optimize(self, item_list: list) -> list[int]:
        try:
            budget_str = os.environ.get(OPTIMIZE_BUDGET_ENV)
            budget = float(budget_str) if budget_str else ordering.DEFAULT_TIME_BUDGET
            return ordering.build_order(item_list, self._container_list, self._lookahead_k, time_budget=budget)
        except Exception:
            # 探索中に何らかの例外が起きても、必ず有効な完全順列を返す最終フォールバック。
            return ordering.order_items(item_list)

    def policy(self, observation: dict) -> dict:
        container_list = observation.get('container_list', [])
        pool_list = observation.get('pool_list', [])

        action = None
        if pool_list and container_list:
            try:
                action = planner.plan(container_list, pool_list, time_budget=5.5)
            except Exception:
                action = None

        if action is None:
            # 合法手が見つからない場合の最終フォールバック(それ以上物理的に置けない状況を想定)
            container = container_list[0] if container_list else None
            item = pool_list[0] if pool_list else None
            if container is not None and item is not None:
                thickness = container.get('thickness', 0.05)
                z = thickness + item.get('height', 0.2) / 2.0
                action = {
                    'item_idx': 0,
                    'container_idx': container.get('index', 0),
                    'place_pos': np.array([0.0, 0.0, z], dtype=np.float32),
                    'orientation': 0,
                }
            else:
                action = {
                    'item_idx': 0,
                    'container_idx': 0,
                    'place_pos': np.array([0.0, 0.0, 0.3], dtype=np.float32),
                    'orientation': 0,
                }

        return action
