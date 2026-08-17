"""replica.py の防御的書き直し(Phase41/Phase42)の回帰テスト。

`agents/mysolver/replica.py` の `ReplicaEvaluator` は観測データ(container_list /
item 情報)を辞書として読む。この辞書から必須キーを1つずつ削り、
「例外を投げずに ('data_error', 原因例外) を返し、その候補だけを諦める」ことを、
代替可能キー(is_prioritized 等)を1つずつ削り「Container/Item のフィールド既定値で
継続する(('ok', dict) を返す)」ことを確認する。

必須キー(格上げ理由込み): index/thickness/length/width/height/center(いずれも
Container のフィールドに既定値が無い)、cut_x/cut_y/shelf(Phase42: 既定値で継続すると
Container.volume が変わり fill の分母そのものが変わってしまうため、代替可から格上げした。
cut_x欠損時に旧実装が返していた fill=15.45 <実測28.17> がその実例)。
代替可能キー: is_prioritized(containers.py 内で Container.create()/volume 計算に
一切使われず、幾何・質量・物理係数への影響が無いことを確認済み)。

実行方法(リポジトリルートで):
    MYSOLVER_HARD_WALL_LIMIT=3000 PYTHONPATH=. python tools/test_replica_missing_keys.py

戻り値: 全ケースOKなら終了コード0、1件でもNGなら1(標準出力に一覧を表示)。
副作用なし(results/ には書かない)。詳細な経緯は results/phase41_report.md 参照。
"""
import copy
import io
import json
import sys
from contextlib import redirect_stdout

sys.path.insert(0, '.')

from src.ground_handling.env import GroundHandlingEnv
from agents.mysolver import replica

SCENE = 'configs/gen/suite_B01_1c_40_plain.json'


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


def run_eval(container_list, items, lookahead_k, order):
    rep = replica.ReplicaEvaluator(container_list, lookahead_k).open()
    try:
        return rep.evaluate(items, order, deadline=None, compute_composite=False)
    finally:
        rep.close()


def main():
    with redirect_stdout(io.StringIO()):
        container_list, items, lookahead_k = load_scene(SCENE)
    order = [it['index'] for it in items]

    print('== ベースライン(欠損なし) ==')
    with redirect_stdout(io.StringIO()):
        base_status, base_payload = run_eval(container_list, items, lookahead_k, order)
    print(f'  base = ({base_status!r}, fill={base_payload.get("fill") if base_payload else None})')
    assert base_status == 'ok', 'ベースライン自体が失敗している'
    base_fill = base_payload['fill']

    essential_container_keys = ['index', 'thickness', 'length', 'width', 'height', 'center',
                                 'cut_x', 'cut_y', 'shelf']
    optional_container_keys = ['is_prioritized']

    results = {}

    print('\n== 必須キー(container)を1つずつ欠落させる: 例外なく data_error を期待 ==')
    for key in essential_container_keys:
        cl2 = copy.deepcopy(container_list)
        del cl2[0][key]
        try:
            with redirect_stdout(io.StringIO()):
                status, payload = run_eval(cl2, items, lookahead_k, order)
            ok = (status == 'data_error' and isinstance(payload, KeyError))
            results[f'container.{key}'] = (
                f'OK(data_error, {type(payload).__name__})' if ok
                else f'NG: status={status!r} payload={payload!r}'
            )
        except Exception as e:
            results[f'container.{key}'] = f'NG: 例外が漏れた {type(e).__name__}: {e}'
        print(f'  {key:15s}: {results[f"container.{key}"]}')

    print('\n== 代替可能キー(container)を1つずつ欠落させる: 既定値で継続(ok)を期待 ==')
    for key in optional_container_keys:
        cl2 = copy.deepcopy(container_list)
        del cl2[0][key]
        try:
            with redirect_stdout(io.StringIO()):
                status, payload = run_eval(cl2, items, lookahead_k, order)
            ok = (status == 'ok' and 'fill' in payload)
            results[f'container.{key}'] = (
                f'OK(継続, fill={payload["fill"]:.4f})' if ok else f'NG: status={status!r} payload={payload!r}'
            )
        except Exception as e:
            results[f'container.{key}'] = f'NG: 例外が漏れた {type(e).__name__}: {e}'
        print(f'  {key:15s}: {results[f"container.{key}"]}')

    print('\n== item 側: 必須フィールド(height)欠落 → data_error を期待 ==')
    items2 = copy.deepcopy(items)
    del items2[0]['height']
    try:
        with redirect_stdout(io.StringIO()):
            status, payload = run_eval(container_list, items2, lookahead_k, order)
        ok = (status == 'data_error')
        results['item.height'] = (f'OK(data_error, {type(payload).__name__})' if ok
                                   else f'NG: status={status!r} payload={payload!r}')
    except Exception as e:
        results['item.height'] = f'NG: 例外が漏れた {type(e).__name__}: {e}'
    print(f'  {"item.height":15s}: {results["item.height"]}')

    print('\n== item 側: 代替可能フィールド(mass, dataclass既定値)欠落 → 継続(ok)を期待 ==')
    items3 = copy.deepcopy(items)
    del items3[0]['mass']
    try:
        with redirect_stdout(io.StringIO()):
            status, payload = run_eval(container_list, items3, lookahead_k, order)
        ok = (status == 'ok' and 'fill' in payload)
        results['item.mass'] = f'OK(継続, fill={payload["fill"]:.4f})' if ok else f'NG: status={status!r} payload={payload!r}'
    except Exception as e:
        results['item.mass'] = f'NG: 例外が漏れた {type(e).__name__}: {e}'
    print(f'  {"item.mass":15s}: {results["item.mass"]}')

    print('\n== cut_x 欠損時、Phase41版は fill=15.45(実測28.17)という別人の値を'
          '返していたが、Phase42以降は data_error になり argmax を汚染しないことを確認 ==')
    cl_cutx = copy.deepcopy(container_list)
    del cl_cutx[0]['cut_x']
    with redirect_stdout(io.StringIO()):
        status, payload = run_eval(cl_cutx, items, lookahead_k, order)
    ok = (status == 'data_error')
    results['cut_x_no_longer_silent'] = (
        'OK(data_error, もはや fill 値を返さない)' if ok else f'NG: status={status!r} payload={payload!r}'
    )
    print(f'  {"cut_x_regression_check":15s}: {results["cut_x_no_longer_silent"]}')

    print('\n== 候補単位の独立性: 1候補目が壊れたデータでも、'
          '同一インスタンスで2候補目(正常データ)が正しく評価できるか ==')
    rep = replica.ReplicaEvaluator(container_list, lookahead_k).open()
    try:
        cl_broken = copy.deepcopy(container_list)
        del cl_broken[0]['index']
        rep.given = cl_broken
        with redirect_stdout(io.StringIO()):
            status1, payload1 = rep.evaluate(items, order, deadline=None, compute_composite=False)
        rep.given = container_list
        with redirect_stdout(io.StringIO()):
            status2, payload2 = rep.evaluate(items, order, deadline=None, compute_composite=False)
        ok = (status1 == 'data_error' and status2 == 'ok'
              and abs(payload2['fill'] - base_fill) < 1e-9)
        results['candidate_isolation'] = (
            f'OK(1候補目data_error, 2候補目fill={payload2["fill"]:.4f}, base一致)' if ok
            else f'NG: (status1,payload1)=({status1!r},{payload1!r}) (status2,payload2)=({status2!r},{payload2!r})'
        )
    except Exception as e:
        results['candidate_isolation'] = f'NG: 例外が漏れた {type(e).__name__}: {e}'
    finally:
        rep.close()
    print(f'  {"candidate_isolation":15s}: {results["candidate_isolation"]}')

    n_fail = sum(1 for v in results.values() if v.startswith('NG'))
    print(f'\n== 合計 {len(results)} 件中 NG {n_fail} 件 ==')
    return 1 if n_fail else 0


if __name__ == '__main__':
    sys.exit(main())
