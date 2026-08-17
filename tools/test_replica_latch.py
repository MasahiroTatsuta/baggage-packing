"""ordering.py の複製評価器ラッチ制御(Phase38 候補単位ラッチ / Phase42 None多義性解消)
の回帰テスト。

`ReplicaEvaluator.evaluate()` は `(status, payload)` を返す(Phase42、
`agents/mysolver/replica.py` の `ReplicaEvaluator` docstring参照)。
`ordering.build_order()` はこれを次のように扱うことになっている:

  - status=='ok'       : 候補を採用検討する(通常経路)。
  - status=='deadline' : 壁時計超過。即座にシーン全体をラッチする(候補単位の猶予は無い)。
  - status=='data_error': 観測データ欠損・型異常。`except Exception` 経路(実際に例外が
                          飛んできた場合)と全く同じ**候補単位ラッチ**
                          (`MYSOLVER_REPLICA_LATCH_MODE=per_candidate` 既定:
                          1候補の失敗は飛ばして続行、2回連続失敗でシーンラッチ)を適用する。

Phase41では data_error も deadline も同じ `None` として区別なく扱われており、
観測データが1候補目で欠損すると即座にシーン全体がラッチされる(=候補単位ラッチが
機能しない)という退行があった。本テストはこの3分岐が正しく動くことを、
`_replica_mod` をフェイクに差し替えて確認する(実データでは欠損が起きないため
ローカル26シーンでは再現できない。README/results/phase41_report.md 参照)。

実行方法(リポジトリルートで):
    MYSOLVER_HARD_WALL_LIMIT=3000 PYTHONPATH=. python tools/test_replica_latch.py

戻り値: 全ケースOKなら終了コード0、1件でもNGなら1。副作用なし(results/には書かない)。
"""
import io
import json
import sys
from contextlib import redirect_stdout

sys.path.insert(0, '.')

from src.ground_handling.env import GroundHandlingEnv
from agents.mysolver import ordering as ordering_mod

SCENE = 'configs/gen/suite_B01_1c_40_plain.json'
N_ITEMS = 8          # 各 construct+validate を軽くして複数候補を早く確保するため縮小する
TIME_BUDGET = 10.0


def load_scene(cp, n_items=None):
    task = list(json.load(open(cp)).values())[0]
    env = GroundHandlingEnv(config=task, verbose=False, render_mode=None)
    try:
        env.reset_settings()
        init = env.get_init_states()
        items = env.get_info_for_optimization()
        if n_items is not None:
            items = items[:n_items]
        return init['container_list'], items, init['lookahead_k']
    finally:
        env.close()


class FakeReplicaModule:
    """_replica_mod の代わりに ordering.py へ差し込むフェイク。"""
    def __init__(self, eval_sequence_fn):
        self.eval_sequence_fn = eval_sequence_fn  # rank(0-indexed) -> (status, payload)
        self.calls = []

    def is_applicable(self, container_list):
        return True

    def preflight(self):
        return True

    def ReplicaEvaluator(self, container_list, lookahead_k, prepacked_ids=None):
        return FakeEvaluator(self)


class FakeEvaluator:
    def __init__(self, mod):
        self.mod = mod

    def open(self):
        return self

    def close(self):
        pass

    def evaluate(self, all_item_infos, order, deadline=None, compute_composite=False):
        rank = len(self.mod.calls)
        self.mod.calls.append(order)
        return self.mod.eval_sequence_fn(rank)


def run_case(label, container_list, items, lk, eval_sequence_fn):
    fake_mod = FakeReplicaModule(eval_sequence_fn)
    orig_mod = ordering_mod._replica_mod
    orig_select = ordering_mod.REPLICA_SELECT
    ordering_mod._replica_mod = fake_mod
    ordering_mod.REPLICA_SELECT = True
    try:
        with redirect_stdout(io.StringIO()):
            ordering_mod.build_order(items, container_list, lk, time_budget=TIME_BUDGET)
        stats = dict(ordering_mod.REPLICA_STATS)
    finally:
        ordering_mod._replica_mod = orig_mod
        ordering_mod.REPLICA_SELECT = orig_select
    n_calls = len(fake_mod.calls)
    print(f'[{label}] n_ranked={stats.get("n_ranked")} n_calls={n_calls} '
          f'stopped={stats.get("stopped")} latched={stats.get("latched")} '
          f'evaluated={stats.get("evaluated")} exc_code={stats.get("exc_code")}')
    return n_calls, stats


def main():
    with redirect_stdout(io.StringIO()):
        container_list, items, lk = load_scene(SCENE, n_items=N_ITEMS)

    n_ng = 0

    # ケース1: 1候補目 data_error(KeyError)、以降は全部 ok。
    # 期待: 全ランク候補が呼ばれ(候補単位で飛ばして継続)、シーンはラッチされない。
    # これが Phase41 §4 の退行(「1候補目の失敗で2候補目以降が評価されなくなる」)が
    # 解消したことを直接示すケースである。
    n_calls, stats = run_case(
        'case1_1st_data_error_then_all_ok', container_list, items, lk,
        lambda rank: ('data_error', KeyError('cut_x')) if rank == 0
                     else ('ok', {'fill': 10.0 + rank, 'num_placed': 5 + rank}))
    n_ranked = stats['n_ranked']
    assert n_ranked >= 2, f'テスト前提が崩れている: n_ranked={n_ranked} (>=2 が必要)'
    ok = (n_calls == n_ranked and stats['latched'] is False and stats['stopped'] == 'done'
          and stats['evaluated'] == n_ranked - 1 and stats.get('exc_code') == 16 * 0 + 2)
    print(f'  -> {"OK" if ok else "NG"}')
    n_ng += 0 if ok else 1

    # ケース2: 先頭2候補が連続 data_error → 2回連続でシーンラッチ、3候補目は呼ばれない。
    n_calls, stats = run_case(
        'case2_two_consecutive_data_error_then_latch', container_list, items, lk,
        lambda rank: ('data_error', TypeError('height')) if rank < 2
                     else ('ok', {'fill': 99.9, 'num_placed': 99}))
    ok = (n_calls == 2 and stats['stopped'] == 'runtime_error' and stats['latched'] is True
          and stats.get('exc_code') == 16 * 0 + 6)
    print(f'  -> {"OK" if ok else "NG"}')
    n_ng += 0 if ok else 1

    # ケース3: 1候補目が deadline → 即座にシーン全体ラッチ、2候補目は呼ばれない
    # (data_errorとは違い、壁時計超過には候補単位の猶予を与えない。従来どおりの挙動)。
    n_calls, stats = run_case(
        'case3_deadline_immediate_latch', container_list, items, lk,
        lambda rank: ('deadline', None) if rank == 0 else ('ok', {'fill': 99.9, 'num_placed': 99}))
    ok = (n_calls == 1 and stats['stopped'] == 'wall_deadline' and stats['latched'] is True
          and stats['evaluated'] == 0)
    print(f'  -> {"OK" if ok else "NG"}')
    n_ng += 0 if ok else 1

    # ケース4: 1,3候補目が data_error・2候補目は ok(非連続)。consecutive_fail は
    # 成功のたびに0へ戻るため、非連続な失敗ではラッチされないはず。
    if n_ranked >= 3:
        n_calls, stats = run_case(
            'case4_nonconsecutive_data_error_no_latch', container_list, items, lk,
            lambda rank: ('data_error', AttributeError('x')) if rank in (0, 2)
                         else ('ok', {'fill': 10.0 + rank, 'num_placed': 5 + rank}))
        ok = (n_calls == n_ranked and stats['latched'] is False and stats['stopped'] == 'done')
        print(f'  -> {"OK" if ok else "NG"}')
        n_ng += 0 if ok else 1
    else:
        print(f'[case4] スキップ(n_ranked={n_ranked} < 3、このシーン/予算では'
              '非連続パターンを再現できず。ケース1〜3で主要分岐は網羅済み)')

    print(f'\n== NG {n_ng} 件 ==')
    return 1 if n_ng else 0


if __name__ == '__main__':
    sys.exit(main())
