"""agents/mysolver/simulate.py の cog代理(Phase49 作業1)の回帰テスト。

1. `cog_proxy_score()` が `tools/scorer.py::Scorer.calculate_cog_score` と同一の
   正規化式で計算していることを、手計算した既知の入力で確認する。
2. `simulate_order()` の `compute_cog_proxy` 既定(False)で戻り値が従来どおり
   **5要素タプル**のままであること(既存呼び出し側のビット単位不変を壊さない)、
   `compute_cog_proxy=True` のときだけ6要素タプルになり、6番目がcog代理値である
   ことを、実シーン(B01)の短い候補順序で確認する。

実行方法(リポジトリルートで):
    MYSOLVER_HARD_WALL_LIMIT=3000 PYTHONPATH=. python tools/test_cog_proxy.py

戻り値: 全ケースOKなら終了コード0、1件でもNGなら1。副作用なし(results/には書かない)。
"""
import io
import json
import sys
from contextlib import redirect_stdout

sys.path.insert(0, '.')

from agents.mysolver import geometry as geo
from agents.mysolver import planner
from agents.mysolver import simulate as simulate_mod
from src.ground_handling.env import GroundHandlingEnv

SCENE = 'configs/gen/suite_B01_1c_40_plain.json'


def test_formula_matches_scorer_by_hand():
    """tools/scorer.py::calculate_cog_score と同じ式を手計算で再現し、
    cog_proxy_score() の結果と一致するか確認する(独自の式を作っていないことの証明)。"""
    # 1コンテナ、高さ10、床(center.z=5想定、center[2]-height/2=0, +height/2=10)。
    # points/n_vecsは与えない(fallback: floor_z=center.z-height/2, ceil_z=center.z+height/2)。
    container = {
        'center': (0.0, 0.0, 5.0), 'height': 10.0,
        'packed_items': [
            {'pos': (0.0, 0.0, 2.0), 'mass': 3.0},   # normalized_h = (2-0)/10 = 0.2
            {'pos': (0.0, 0.0, 8.0), 'mass': 1.0},   # normalized_h = (8-0)/10 = 0.8
        ],
    }
    # 手計算: weighted_h = 3*0.2 + 1*0.8 = 0.6+0.8 = 1.4, total_mass=4, avg_h=0.35
    # cog_score = 100*(1-0.35) = 65.0
    expected = 65.0
    got = simulate_mod.cog_proxy_score([container])
    ok = abs(got - expected) < 1e-9
    print(f'  formula match: expected={expected} got={got} -> {"OK" if ok else "NG"}')
    return ok


def test_empty_containers_returns_100():
    got = simulate_mod.cog_proxy_score([{'center': (0, 0, 5), 'height': 10, 'packed_items': []}])
    ok = (got == 100.0)
    print(f'  empty containers -> 100.0: got={got} -> {"OK" if ok else "NG"}')
    return ok


def test_clip_to_0_100():
    """normalized_hは[0,1]にクリップされる(壁外に飛び出た極端な値でも領域外に出ない)。"""
    container = {
        'center': (0.0, 0.0, 5.0), 'height': 10.0,
        'packed_items': [{'pos': (0.0, 0.0, -100.0), 'mass': 1.0}],  # floor_zより大幅に下
    }
    got = simulate_mod.cog_proxy_score([container])
    ok = (got == 100.0)  # normalized_h=0にクリップ -> cog=100
    print(f'  clip test (極端に低い位置->cog=100): got={got} -> {"OK" if ok else "NG"}')
    return ok


def load_scene(cp):
    task = list(json.load(open(cp)).values())[0]
    env = GroundHandlingEnv(config=task, verbose=False, render_mode=None)
    try:
        env.reset_settings()
        init = env.get_init_states()
        items = env.get_info_for_optimization()
        return init['container_list'], items, init['lookahead_k']
    finally:
        env.close()


def test_simulate_order_tuple_length():
    """既定(compute_cog_proxy=False)は5要素、True指定時のみ6要素。"""
    with redirect_stdout(io.StringIO()):
        container_list, items, lookahead = load_scene(SCENE)
    items_by_index = {it['index']: it for it in items}
    order = [it['index'] for it in items][:8]  # 短い順序で十分(戻り値の形だけ見る)
    prepacked_ids = geo.initial_prepacked_ids(container_list)
    budget = planner.SearchBudget.from_seconds(60.0)

    with redirect_stdout(io.StringIO()):
        result_default = simulate_mod.simulate_order(
            container_list, items_by_index, order, max(1, int(lookahead or 1)), budget,
            prepacked_ids=prepacked_ids)
    ok1 = (len(result_default) == 5)
    print(f'  compute_cog_proxy未指定(既定False): len={len(result_default)} -> '
          f'{"OK(5要素)" if ok1 else "NG"}')

    budget2 = planner.SearchBudget.from_seconds(60.0)
    with redirect_stdout(io.StringIO()):
        result_cog = simulate_mod.simulate_order(
            container_list, items_by_index, order, max(1, int(lookahead or 1)), budget2,
            prepacked_ids=prepacked_ids, compute_cog_proxy=True)
    ok2 = (len(result_cog) == 6)
    ok3 = ok2 and isinstance(result_cog[5], float)
    # 既定Falseと同じ入力・同じ乱数状態でのplaced_ids等(先頭5要素)が一致することも確認
    ok4 = ok2 and (result_cog[:5] == result_default)
    print(f'  compute_cog_proxy=True: len={len(result_cog)} cog={result_cog[5] if ok2 else "?"} -> '
          f'{"OK(6要素)" if ok2 else "NG"}')
    print(f'  先頭5要素が既定呼び出しと一致: {"OK" if ok4 else "NG"}')
    return ok1 and ok2 and ok3 and ok4


def main():
    results = {
        'formula_match': test_formula_matches_scorer_by_hand(),
        'empty_containers': test_empty_containers_returns_100(),
        'clip_0_100': test_clip_to_0_100(),
        'simulate_order_tuple_length': test_simulate_order_tuple_length(),
    }
    n_fail = sum(1 for v in results.values() if not v)
    print(f'\n== 合計 {len(results)} 件中 NG {n_fail} 件 ==')
    return 1 if n_fail else 0


if __name__ == '__main__':
    sys.exit(main())
