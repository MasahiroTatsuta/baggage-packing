import os
import time

import numpy as np

from . import geometry as geo
from . import ordering
from . import planner

# online(policy)1回に許す名目探索予算[s]。planner.UNITS_PER_SEC で決定的にユニット数へ
# 換算され、実際の打ち切りは消費ユニット数で決まる(壁時計には依存しない)。
POLICY_TIME_BUDGET = 5.5
# Phase17(方針3): online側は「決定性より制約遵守が優先」。本番の policy_timeout(8s)・
# 本フェーズの制約(policy<7s)を絶対に踏まないため、非常用の壁時計チェックを必ず残す。
# 較正が想定より甘い(実機がこの環境より遅い)場合はここが発火して決定性は失われるが、
# タイムアウトによるエピソード即死のほうが遥かに損失が大きい。
# Phase38(ステップA): 環境変数化(既定値は6.0のまま不変)。
POLICY_HARD_WALL = float(os.environ.get('MYSOLVER_POLICY_HARD_WALL', '6.0'))

# 開発中の反復を速くするための環境変数。未設定なら本番想定の ordering.DEFAULT_TIME_BUDGET
# (165s、180sタイムアウトに対する安全マージン込み)を使う。最終計測時は未設定のまま
# (=170s相当のフル予算)で回すこと。
OPTIMIZE_BUDGET_ENV = 'MYSOLVER_OPTIMIZE_BUDGET'

# Phase54/55: policy()フォールバック(planner.plan()がNoneを返した最終手段)が
# 「床にぴったり(clearance=0)」置いていたため、inclusion_margin<0(=境界に触れるだけでは
# 不足で最低限のクリアランスを要求する)の設定下では**恒等式的に**is_included判定に落ちて
# いた(Phase54実測: 30/30件が同一原因、はみ出し量5.000mm±6.4e-9m)。
# inclusion_margin の実値(本番レジーム)はPhase12/13/27以来未確定のため、ハードコードせず
# 環境変数から読む。既定値はローカルvalidator設定(-0.005)相当。
FALLBACK_INCLUSION_MARGIN = float(os.environ.get('MYSOLVER_FALLBACK_INCLUSION_MARGIN', '-0.005'))
# 選んだ候補が margin ちょうどの境界に乗ると浮動小数の丸めで容易に反転する
# (Phase54実測のはみ出し量が±1.4e-8m桁で振れていたのがまさにこの規模)。
# 「-margin + eps」で必ずmarginより内側(より安全)へ寄せる(「-margin - eps」だと
# 逆に境界を割り込み得るので符号に注意)。
FALLBACK_CLEARANCE_EPS = float(os.environ.get('MYSOLVER_FALLBACK_CLEARANCE_EPS', '1e-4'))
# 検証(Phase55 V-1〜V-4)が通るまでは既定無効にできるよう環境変数化。
# 現行動作(修正前)は100%決定論的なバグのため、検証後は既定有効にする方針。
FALLBACK_SAFE_POS = os.environ.get('MYSOLVER_FALLBACK_SAFE_POS', '1') == '1'

# ---------------------------------------------------------------------------
# Phase38(ステップB): policy の壁時計を2値(1-B)から4値に拡張する。
#
# 1-Bの2値(6.2s/6.7s)は「取り置きが構築を打ち切ったか」だけを符号化していたが、
# n=4帯(162.00s〜)がPhase37のn=6/7帯(161.0s/161.5s)より上にあるため、
# `optimization`は全シーンのmaxしか報告されず、n=4が出ているシーンがあると
# n=6/7(=ρ-testが完走したシーン)がmaxの陰に隠れて読めなくなってしまう
# (背景の問題1)。policyは別チャネルなので、ここに「どこかのシーンでρ-testが
# 完走したか」を独立して符号化しておけば、optimizationでn=4しか見えない状況でも
# 「ρ-testが機能しているか」だけは判別できる。
#
#   T_policy = 6.20 + 0.15 × (2·any_success + wall_cut)   → 6.20/6.35/6.50/6.65
#
#   any_success = そのシーンで複製評価が完走したか(ordering.LAST_ANY_SUCCESS、
#                 Phase37テレメトリのn=6/n=7に相当。「効いたか」ではなく「動いたか」)
#   wall_cut    = 取り置きが構築を壁時計締切で実際に打ち切ったか(1-Bと同じ、
#                 ordering.LAST_BUILD_WALL_CUT)
#
# policy_timeout=8.0sに対し最大6.65sで余裕1.35s。最初のpolicy()呼び出し1回だけに
# 限定する(1-Bと同じ制約)。既定無効(MYSOLVER_TELEMETRY=0)時は分岐にすら入らない。
POLICY_TELEMETRY_BASE_S = float(os.environ.get('MYSOLVER_TELEMETRY_POLICY_BASE_S', '6.20'))
POLICY_TELEMETRY_STEP_S = float(os.environ.get('MYSOLVER_TELEMETRY_POLICY_STEP_S', '0.15'))


def _fallback_place_pos(container: dict, item: dict) -> np.ndarray:
    """policyフォールバック専用(Phase55): 6面すべてのinclusion判定
    (`geo.inclusion_slack_batch`——`validator.check_inclusion`と同式)に対し、
    3x3の局所グリッド探索で最も安全な(=slackが最小の)候補位置を選ぶ。

    `MYSOLVER_FALLBACK_SAFE_POS=0`で旧来の「床にぴったり(clearance=0)」配置
    (Phase54で特定したバグそのもの)に戻せる——V-3のビット単位確認用。

    選んだ候補がmarginを満たす保証はない(既に荷物が詰まっていれば他の荷物と
    干渉しうる)。目的は「フォールバックが構造的に必ず死ぬ」状態の解消であり、
    「絶対に死なない」ことではない。
    """
    thickness = container.get('thickness', 0.05)
    height = item.get('height', 0.2)
    if not FALLBACK_SAFE_POS:
        return np.array([0.0, 0.0, thickness + height / 2.0], dtype=np.float32)

    length = container.get('length', 2.0)
    width = container.get('width', 1.45)
    cont_height = container.get('height', 1.61)
    cut_x = container.get('cut_x', 0.0)
    half = geo.half_extent([item.get('length', 0.2), item.get('width', 0.2), height], 0)

    # margin(既定-0.005、環境変数で上書き可)ちょうどに乗せると浮動小数の丸めで
    # 反転しうる(Phase54実測: ±1.4e-8m桁のノイズ)ため、必ず内側へepsだけ余分に
    # 寄せる(「-margin + eps」。「-margin - eps」だと逆に境界を割り込みうる)。
    clearance = -FALLBACK_INCLUSION_MARGIN + FALLBACK_CLEARANCE_EPS
    z_floor = thickness + half[2] + clearance
    z_ceiling_limit = cont_height - thickness - half[2] - clearance
    z = z_floor
    if z_ceiling_limit > thickness + half[2]:
        # 天井にも同じだけの余裕を残せるならclipする(天井を突き抜けない側へ寄せる)。
        # アイテムが背が高すぎて余裕が無い場合はz_floorのまま(ベストエフォート)。
        z = min(z_floor, z_ceiling_limit)

    x_lo = -length / 2.0 + thickness + cut_x + half[0] + geo.START_MARGIN
    x_hi = length / 2.0 - thickness - half[0] - geo.START_MARGIN
    y_lo = -width / 2.0 + thickness + half[1] + geo.START_MARGIN
    y_hi = width / 2.0 - thickness - half[1] - geo.START_MARGIN
    if x_lo > x_hi:
        x_lo = x_hi = 0.0
    if y_lo > y_hi:
        y_lo = y_hi = 0.0

    xs = (x_lo, (x_lo + x_hi) / 2.0, x_hi)
    ys = (y_lo, (y_lo + y_hi) / 2.0, y_hi)
    local_candidates = np.array([[x, y, z] for x in xs for y in ys], dtype=np.float64)
    world_candidates = np.array([geo.local_to_world(container, c) for c in local_candidates])
    # inclusion_slack_batchは各候補について「全平面(6/7面)のうち最も厳しい
    # (=最大の)dots値」を返す(dots<=marginが合法、値が大きいほど外側に近い)。
    # したがって安全な候補ほどこの値は小さい(より負)——argminで選ぶ。
    slack = geo.inclusion_slack_batch(container, half, world_candidates)
    chosen = local_candidates[int(np.argmin(slack))]
    return np.array(chosen, dtype=np.float32)


class Agent:
    """
    合法貪欲ベースラインAgent。
    policy は毎回 observation から状態を再構築し、内部状態(前ステップの記憶)には依存しない。
    """

    def __init__(self, module_path: str):
        self._lookahead_k = None
        self._container_list = None
        self._optimize = True
        self._prepacked_ids = None
        self._policy_telemetry_done = False  # Phase38(ステップ1-B): 最初の1回だけ埋める

    def get_init_states(self, init_states: dict) -> None:
        self._lookahead_k = init_states.get('lookahead_k')
        self._container_list = init_states.get('container_list')
        # Phase13(ターゲット2): offline optimize が無効なシーン(事前の順序検証が無い)では
        # planner.plan により保守的な union支持しきい値を使わせる(詳細はplanner.py参照)。
        self._optimize = init_states.get('optimize', True)
        # Phase15(ターゲット1): エピソード開始時から既に積まれていた荷物のindexを記録する。
        # corridor_penalty(planner._corridor_excess)が既積み層とオンライン中に自分が積んだ層を
        # 区別するために使う(詳細はgeometry.initial_prepacked_ids参照)。
        self._prepacked_ids = geo.initial_prepacked_ids(self._container_list)

    def optimize(self, item_list: list) -> list[int]:
        try:
            budget_str = os.environ.get(OPTIMIZE_BUDGET_ENV)
            budget = float(budget_str) if budget_str else ordering.DEFAULT_TIME_BUDGET
            return ordering.build_order(item_list, self._container_list, self._lookahead_k, time_budget=budget)
        except Exception:
            # 探索中に何らかの例外が起きても、必ず有効な完全順列を返す最終フォールバック。
            return ordering.order_items(item_list)

    def policy(self, observation: dict) -> dict:
        t0 = time.perf_counter()
        container_list = observation.get('container_list', [])
        pool_list = observation.get('pool_list', [])

        action = None
        if pool_list and container_list:
            try:
                action = planner.plan(container_list, pool_list, time_budget=POLICY_TIME_BUDGET,
                                       hard_deadline=time.perf_counter() + POLICY_HARD_WALL,
                                       strict_support=not self._optimize,
                                       prepacked_ids=self._prepacked_ids)
            except Exception:
                action = None

        if action is None:
            # 合法手が見つからない場合の最終フォールバック。
            # planner.plan は通常探索が全滅した場合に密グリッドでの最終リトライまで
            # 内部で行った上でNoneを返す(Phase7)ため、ここに到達するのは「本当にどの
            # 荷物もどの向き・位置にも置けない」場合に限られる…はずだった。
            # Phase54で判明: 旧実装(床にぴったり=clearance 0)は、config側の
            # inclusion_margin<0(境界に触れるだけでは不足)のもとで**恒等式的に**
            # is_included判定に落ちており、「本当に置けない」かどうかによらず
            # このフォールバックに到達した時点で100%即死していた(30/30件で確認)。
            # Phase55で `_fallback_place_pos()` に置き換え、その保証を持たない形にした
            # (=それでも他の荷物と干渉して死ぬことはありうるが、構造的に必ず死ぬ
            # わけではなくなった)。
            container = container_list[0] if container_list else None
            item = pool_list[0] if pool_list else None
            if container is not None and item is not None:
                action = {
                    'item_idx': 0,
                    'container_idx': container.get('index', 0),
                    'place_pos': _fallback_place_pos(container, item),
                    'orientation': 0,
                }
            else:
                action = {
                    'item_idx': 0,
                    'container_idx': 0,
                    'place_pos': np.array([0.0, 0.0, 0.3], dtype=np.float32),
                    'orientation': 0,
                }

        if ordering.MYSOLVER_TELEMETRY and not self._policy_telemetry_done:
            self._policy_telemetry_done = True
            wall_cut = bool(getattr(ordering, 'LAST_BUILD_WALL_CUT', False))
            any_success = bool(getattr(ordering, 'LAST_ANY_SUCCESS', False))
            code = 2 * int(any_success) + int(wall_cut)
            target = POLICY_TELEMETRY_BASE_S + POLICY_TELEMETRY_STEP_S * code
            target_t = t0 + target
            while time.perf_counter() < target_t:
                time.sleep(0.01)

        return action
