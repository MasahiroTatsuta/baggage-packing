"""Phase35: 複製評価器 —— 代理関数を信じずに「本物と同じ判定」で候補順序を選ぶ。

---------------------------------------------------------------------------
なぜ必要か(Phase34 の帰結)
---------------------------------------------------------------------------
Phase34 の ALNS は受理則が厳密改善のみの山登りなので、**採用された手は定義上すべて
代理目的関数(risk調整済み体積)を改善している**。にもかかわらず実fillの改善は
7シーン中4シーンにとどまり、**代理gainと実fill差の順位相関は Spearman ρ = −0.321(負)**
だった。代理 = 実 + ノイズ なら、代理gainが大きい手を選ぶことは
**ノイズが正に大きい手を選ぶこと**に等しい(optimizer's curse の純粋形)。

つまり Phase24(候補側)・Phase28(目的関数側)・Phase29(1手移動)・Phase34(ALNS)の
4連敗は、すべて「現在の代理関数を山登りしている」という一つの原因で説明される。
本モジュールは代理を良くするのではなく、**代理を信じないための仕組み**である:
候補順序を本物と同じ pybullet・同じ validator・同じ evaluator で実際に走らせて選ぶ。

---------------------------------------------------------------------------
何から復元しているか(重要)
---------------------------------------------------------------------------
エージェントが受け取れるのは `get_init_states` の
`{optimize, lookahead_k, container_list}` だけである。ここから:

  offset_x      = center[0]                (local_to_global が x にだけ offset を足す)
  buffer        = center[2] - height/2     (center = (0,0,height/2+buffer) の平行移動)
  require_shelf = 観測の 'shelf'
  その他(length/width/height/thickness/cut_x/cut_y/is_prioritized/packed_items)は観測にそのまま

を復元して `src.ground_handling` の **本物のクラスをそのまま使って** 環境を組み直す。
再構築した幾何(n_vecs/points/volume/center)が観測値と完全一致することは
`tools/phase35_replica.py` が21シーンで検査済み(最大差分 0.000e+00)。

**観測から復元できない設定値** (`config['validator']`、エージェントには渡らない):
  inclusion_margin / start_z / safety_margin / ceiling_margin /
  displacement_threshold / angle_displacement_threshold / settle_wait_step
これらは決め打ちするしかない。値は `geometry.py` が既に前提にしているものと同一で、
ローカル26シーンの config 全件で同一だった。本番の値が違えばその分だけ狂う ——
Phase12/13/27 から続く「本番の inclusion_margin レジームが未確定」という
既知の未解決課題そのものであり、本モジュール固有の新しいリスクではない。

---------------------------------------------------------------------------
既積み荷物があるシーンでは使わない(実測に基づく適用範囲の限定)
---------------------------------------------------------------------------
`tools/phase35_replica.py` が記録済み130候補(21シーン)で本物と突き合わせた結果:

  既積みなし 16シーン(99候補): **99/99 が完全一致(σ=0.000)、シーン内順位 ρ=1.000**
  既積みあり  5シーン(31候補): 25/31 一致(σ=4.372)、ρ=0.835(最悪 P06 で 0.455)

原因は情報の欠落である。`container_list` は既積み荷物の **落ち着いた姿勢** しか伝えず、
接触状態やソルバの内部状態(どの接触が有効か、スリープしているか)は伝わらない。
そのため復元した既積み荷物は、配置ごとの定着(settle_wait_step=300 ステップ)で
**1回あたり約1.0mm ずつ本物と違う方向へ動き**、これが safety_margin=0.015m /
inclusion_margin=0.005m という判定閾値に対して無視できない大きさまで蓄積する。
実際 P06 では先頭10手までは本物と完全に同一で、そこから先で判定が食い違っていた。

したがって **既積み荷物が1つでもあるシーンでは複製評価器を使わない**(`is_applicable`)。
これは「精度が出ない領域を実測で特定して避ける」ことであり、
Phase12 の「根拠が循環参照だと気づいて撤回した」教訓と同じ姿勢である。
"""
import os
import time

import numpy as np
import pybullet as p
import pybullet_data
from pybullet_utils import bullet_client

# 本物の評価系をそのまま使う(判定式を書き写すと、それは「もう一つの代理関数」になる)。
from src.ground_handling.containers import MultiContainerManager
from src.ground_handling.evaluator import Evaluator
from src.ground_handling.items import Item
from src.ground_handling.validator import PlacementValidator

from . import planner

# config['validator'] 相当(観測から取れないため決め打ち。モジュール docstring 参照)。
ASSUMED_VALIDATOR = {
    'inclusion_margin': float(os.environ.get('MYSOLVER_REPLICA_INCL_MARGIN', '-0.005')),
    'start_z': 0.08,
    'safety_margin': 0.015,
    'ceiling_margin': 0.018,
    'displacement_threshold': 0.3,
    'angle_displacement_threshold': 45,
    'settle_wait_step': 300,
}
# ItemStreamManager.max_space(補充タイミング)。ローカル全シーンで 1。
ASSUMED_MAX_SPACE = 1
# 採点に使う inclusion_margin。Phase27 が「採点と合法性判定は常に同じ margin を共有する」
# (env.py:51 の配線)ことを確定させているので、validator と同じ値を使う。
SCORE_MARGIN = ASSUMED_VALIDATOR['inclusion_margin']


# ---------------------------------------------------------------------------
# Phase36(タスク1-4): 障害注入フック。**既定無効**(空文字)。
# {import, init, runtime, deadline} のいずれかを指定すると、その段階で失敗を強制する。
# 本番経路には一切影響しない(この定数が空なら分岐にすら入らない)。
# ---------------------------------------------------------------------------
FORCE_FAIL = os.environ.get('MYSOLVER_REPLICA_FORCE_FAIL', '')

# Phase38(ステップA): run_order() 内の planner.plan 呼び出しに掛ける壁時計安全弁。
# 環境変数化(既定値は6.0のまま不変)。agent.py の POLICY_HARD_WALL と同じ性質の値。
RUN_ORDER_HARD_WALL = float(os.environ.get('MYSOLVER_REPLICA_RUN_ORDER_HARD_WALL', '6.0'))

# ---------------------------------------------------------------------------
# Phase37(ステップ0-1): n=4(evaluate()内の実行時例外)がメモリ(RLIMIT_AS)由来かを
# 切り分けるためのテレメトリ。**既定無効**(MYSOLVER_REPLICA_VMLOG=1で有効)。
# 無効時は /proc を一切読まない・分岐にも入らないため本番経路への影響はゼロ。
#
# RSS(Phase36が測ったpeak 102→103MB)ではなく VmPeak/VmSize(仮想アドレス空間)を見る。
# runner.py:14-15 が子プロセスに課すのは RLIMIT_AS(=VA)であり、RSSが小さいことは
# 安全性の根拠にならない。open()直後・run_order の10配置ごと・終了時の3点で記録する。
# ---------------------------------------------------------------------------
VMLOG = os.environ.get('MYSOLVER_REPLICA_VMLOG', '0') == '1'


def _vmlog(tag: str) -> None:
    if not VMLOG:
        return
    try:
        with open('/proc/self/status') as f:
            status = f.read()
        vals = {}
        for line in status.splitlines():
            for key in ('VmPeak', 'VmSize', 'VmRSS', 'VmHWM'):
                if line.startswith(key + ':'):
                    vals[key] = line.split(':', 1)[1].strip()
        print(f'[replica-vmlog] {tag}: '
              f'VmPeak={vals.get("VmPeak", "?")} VmSize={vals.get("VmSize", "?")} '
              f'VmHWM={vals.get("VmHWM", "?")} VmRSS={vals.get("VmRSS", "?")}', flush=True)
    except Exception as e:
        print(f'[replica-vmlog] {tag}: read failed ({e!r})', flush=True)


# ---------------------------------------------------------------------------
# Phase38(ステップ1-D検証用): FORCE_FAIL=='runtime' のとき、どの例外クラスを
# 何回目の evaluate() から投げるかを指定できるようにする(ordering.py の
# _classify_exception / a(成功候補数カウント) の分岐を個別に検証するため)。
# 既定は 'RuntimeError' を1回目から(=a=0)。本番経路には影響しない
# (FORCE_FAIL自体が空文字既定で、この分岐に入るのはテスト時だけ)。
# ---------------------------------------------------------------------------
FORCE_FAIL_EXC = os.environ.get('MYSOLVER_REPLICA_FORCE_EXC', 'RuntimeError')
FORCE_FAIL_AFTER = int(os.environ.get('MYSOLVER_REPLICA_FORCE_FAIL_AFTER', '0'))
_FORCE_EXC_MAP = {
    'MemoryError': MemoryError,
    'pybullet.error': p.error,
    'KeyError': KeyError,
    'IndexError': IndexError,
    'AttributeError': AttributeError,
    'ValueError': ValueError,
    'TypeError': TypeError,
    'TimeoutError': TimeoutError,
    'RuntimeError': RuntimeError,
    'OSError': OSError,
    'AssertionError': AssertionError,
    'ZeroDivisionError': ZeroDivisionError,
    'RecursionError': RecursionError,
    'OverflowError': OverflowError,
}


def is_applicable(container_list: list[dict]) -> bool:
    """複製評価器を使ってよいシーンか(既積み荷物が1つも無いこと)。"""
    if FORCE_FAIL == 'import':
        # import 失敗と同じ「構築前に判明する失敗」として扱う(ordering 側で
        # 取り置きが解除され、ρ-test 無効時とビット単位一致の経路へ落ちる)。
        return False
    for c in container_list:
        if c.get('packed_items'):
            return False
    return True


def preflight() -> bool:
    """**構築を始める前に** pybullet を実際に初期化できるか確かめる(タスク1-2a)。

    ここで False を返すと `build_order` は取り置き(REPLICA_RESERVE_S)を 0 に戻すので、
    構築予算は満額のまま = ρ-test 無効時と完全に同じ経路になる。
    逆にこの確認を省くと、「複製評価はできないのに構築の締切だけ45秒早い」という
    **一番損な状態**になる。所要は数ミリ秒なので構築予算への影響は無視できる。
    """
    if FORCE_FAIL in ('import', 'init'):
        return False
    try:
        c = bullet_client.BulletClient(connection_mode=p.DIRECT)
        try:
            c.disconnect()
        except Exception:
            pass
        return True
    except Exception:
        return False


class ReplicaEvaluator:
    """container_list から本物と同じ環境を組み直し、順序を実際に走らせて fill を返す。"""

    def __init__(self, container_list: list[dict], lookahead_k: int, prepacked_ids=None):
        self.given = container_list
        self.lookahead_k = max(1, int(lookahead_k or 1))
        self.prepacked_ids = prepacked_ids
        self.client = None
        self.cm = None
        self._eval_calls = 0  # Phase38(ステップ1-D検証用): FORCE_FAIL_AFTER のカウンタ

    # -- 生成/破棄 ------------------------------------------------------
    def open(self):
        if self.client is None:
            self.client = bullet_client.BulletClient(connection_mode=p.DIRECT)
            self.client.setAdditionalSearchPath(pybullet_data.getDataPath())
        _vmlog('open() 直後')
        return self

    def close(self):
        if self.client is not None:
            try:
                self.client.disconnect()
            except Exception:
                pass
            self.client = None

    def __enter__(self):
        return self.open()

    def __exit__(self, *exc):
        self.close()

    # -- 環境構築 -------------------------------------------------------
    def _containers_config(self) -> dict:
        cl = []
        for c in self.given:
            cl.append({
                'index': int(c['index']),
                'thickness': float(c['thickness']),
                'length': float(c['length']),
                'width': float(c['width']),
                'height': float(c['height']),
                'cut_x': float(c['cut_x']),
                'cut_y': float(c['cut_y']),
                'require_shelf': bool(c['shelf']),
                'is_prioritized': bool(c['is_prioritized']),
                'buffer': float(c['center'][2]) - float(c['height']) / 2.0,
                'packed_items': [dict(it) for it in c.get('packed_items', [])],
            })
        offs = [float(c['center'][0]) for c in self.given]
        spacing = (offs[1] - offs[0]) if len(offs) > 1 else 0.0
        return {'spacing': spacing, 'container_list': cl}

    def reset(self):
        """毎回まっさらな状態から組み直す(前の候補の配置を持ち越さない)。"""
        self.client.resetSimulation()
        self.client.setPhysicsEngineParameter(deterministicOverlappingPairs=1)
        self.client.setGravity(0, 0, -9.8)
        self.client.loadURDF('plane.urdf')
        self.cm = MultiContainerManager(client=self.client, config=self._containers_config())
        self.cm.build()
        self.validator = PlacementValidator(client=self.client, config=dict(ASSUMED_VALIDATOR))

    # -- 評価 -----------------------------------------------------------
    def run_order(self, all_item_infos: list[dict], order: list[int],
                  policy_budget: float = 5.5, hard_wall: float = RUN_ORDER_HARD_WALL,
                  deadline: float | None = None, compute_composite: bool = False) -> dict | None:
        """order を実際に走らせて fill(+ compute_composite指定時は5成分の合成スコア)を返す。
        deadline(壁時計)を超えたら None。"""
        by_idx = {int(it['index']): it for it in all_item_infos}
        try:
            stream = [by_idx[i] for i in order]
        except KeyError:
            return None
        cursor = 0
        pool: list[Item] = []
        while len(pool) < self.lookahead_k and cursor < len(stream):
            pool.append(Item(**stream[cursor]))
            cursor += 1

        while pool:
            if deadline is not None and time.perf_counter() > deadline:
                return None          # 時間切れ: この候補は「評価できなかった」扱い
            action = planner.plan(self.cm.get_item_info_in_containers(),
                                  [it.get_info() for it in pool],
                                  time_budget=policy_budget,
                                  hard_deadline=time.perf_counter() + hard_wall,
                                  strict_support=False,
                                  prepacked_ids=self.prepacked_ids)
            if action is None:
                break
            item_idx = int(action['item_idx'])
            ci = int(action['container_idx'])
            if not (0 <= item_idx < len(pool)) or not (0 <= ci < len(self.cm.containers)):
                break
            item = pool[item_idx]
            container = self.cm.get_container(ci)
            gpos = container.local_to_global(action['place_pos'])
            oi = int(action['orientation'])
            # 本物の env.step と同じ順序・同じ判定
            if not self.validator.check_inclusion(container, item, gpos, oi):
                break
            if not self.validator.check_transport_path(container, item, gpos, oi):
                break
            if not self.validator.place_item(item, gpos, oi):
                break
            self.cm.update_and_add_item_to_container(container_id=ci, item=item)
            pool.pop(item_idx)
            n_placed = sum(len(c.packed_items) for c in self.cm.containers)
            if VMLOG and n_placed % 10 == 0:
                _vmlog(f'run_order {n_placed}配置')
            n_space = self.lookahead_k - len(pool)
            if n_space >= ASSUMED_MAX_SPACE:
                for _ in range(n_space):
                    if cursor < len(stream):
                        pool.append(Item(**stream[cursor]))
                        cursor += 1

        _vmlog('run_order 終了時')
        containers = self.cm.containers
        fill, _out = Evaluator(client=self.client,
                               config={'inclusion_margin': SCORE_MARGIN}
                               ).calculate_fill_rate(containers)
        result = {'fill': fill,
                  'num_placed': sum(len(c.packed_items) for c in containers)}
        # Phase37(ステップ1-3): 合成スコア用の残り4指標。**既定では計算しない**(ordering.py の
        # MYSOLVER_REPLICA_METRIC=fill が既定であり、そちらでは呼ばれないため常にFalse)。
        # tools/scorer.py と同じ近似ロジックの複製(agents/mysolver/replica_scorer.py、
        # tools/は提出zipに含まれないため複製が必要な事情はそちらのdocstring参照)。
        # 失敗しても fill 単体の結果は握りつぶさず返す(呼び出し側がcompositeの欠落を見て
        # fillへフォールバックする)。stability計算は破壊的なので必ず最後に呼ぶ。
        if compute_composite:
            try:
                from . import replica_scorer
                extra = replica_scorer.evaluate_extra_metrics(self.client, containers)
                result.update(extra)
                result['composite'] = replica_scorer.composite_score(
                    fill, extra['cog_score'], extra['stability_score'],
                    extra['placement_score'], extra['soft_item_score'])
            except Exception:
                pass
        return result

    def evaluate(self, all_item_infos, order, deadline=None, compute_composite=False):
        if FORCE_FAIL == 'runtime':
            if self._eval_calls >= FORCE_FAIL_AFTER:
                exc_cls = _FORCE_EXC_MAP.get(FORCE_FAIL_EXC, RuntimeError)
                self._eval_calls += 1
                raise exc_cls(f'MYSOLVER_REPLICA_FORCE_FAIL=runtime exc={FORCE_FAIL_EXC} '
                              f'after={FORCE_FAIL_AFTER} (障害注入)')
        if FORCE_FAIL == 'deadline':
            return None          # 壁時計 deadline 超過と同じ扱い(評価できなかった)
        self._eval_calls += 1
        self.reset()
        return self.run_order(all_item_infos, order, deadline=deadline,
                              compute_composite=compute_composite)
