"""
tools/phase36_fallback.py

Phase36 タスク1/2: 複製評価器のフォールバック経路とリソース管理を検証する。

Phase35 でエージェント内部に pybullet を持ち込んだため、事故の形が変わった。
これまでの失敗は「点が伸びない」形だったが、今後は
**例外でそのシーンが丸ごと落ちる / OOM で全損する**形で出る。ここはその確認専用。

検証する項目:

  (1-2a) **早期失敗**(import 失敗 / pybullet 初期化失敗 = 構築を始める前に判明)
         → 取り置きが解除され、出力が ρ-test 無効時と **ビット単位で一致する**こと。
  (1-2b) **実行時失敗**(実行時例外 / deadline 超過 = 構築後に判明)
         → 取り置き45秒ぶん構築が短いのでビット単位一致は原理的に不可能。
           代わりに **run1 の劣化パターン(A07 −10.74 / D01 −10.77 / C02 −6.92)を
           再現しないこと**を確認する。run2 は取り置きを壁時計からしか引いていないので
           原理的には小さいはずで、その原理が効いているかをここで測る。
  (1-3)  **ラッチ**: 1度失敗したら以降そのシーンでは複製評価をしないこと。
  (2-1/2-2) pybullet クライアントが**例外経路でも**解放され、単調増加しないこと。

実行:
    MYSOLVER_UNITS_PER_SEC=1.55e7 PYTHONPATH=. .venv/bin/python tools/phase36_fallback.py \
        --out results/phase36_fallback.json
"""
import argparse
import io
import json
import os
import resource
import sys
import time
from contextlib import redirect_stdout

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

import pybullet as p

from src.ground_handling.env import GroundHandlingEnv

# 早期失敗の一致確認に使うシーン(決定的5シーンは optimize=False で build_order を
# 呼ばないため、ここは optimize 有効な A01-A03 + ρ-test が実際に効く3シーンで見る)
BITWISE_SCENES = ['A01_1c_40_plain', 'A02_1c_80_plain', 'A03_1c_40_shelf']
# 実行時失敗の劣化幅を測るシーン(run1 で大きく劣化した3件)
RUNTIME_SCENES = ['A07_1c_40_bulky', 'D01_A_1c_40_softheavy', 'C02_2c_55_shelfprio']
# run1(取り置きをユニット予算からも引いた版)で観測された劣化量
RUN1_DEGRADATION = {'A07_1c_40_bulky': -10.74, 'D01_A_1c_40_softheavy': -10.77,
                    'C02_2c_55_shelfprio': -6.92}


def load_scene(label):
    task = list(json.load(open(f'configs/gen/suite_{label}.json')).values())[0]
    env = GroundHandlingEnv(config=task, verbose=False, render_mode=None)
    try:
        env.reset_settings()
        init = env.get_init_states()
        items = env.get_info_for_optimization()
        return init['container_list'], items, init['lookahead_k']
    finally:
        try:
            env.close()
        except Exception:
            pass


def n_clients():
    """今つながっている pybullet クライアント数(リーク検出用)。"""
    n = 0
    for cid in range(64):
        try:
            if p.isConnected(cid):
                n += 1
        except Exception:
            pass
    return n


def peak_mb():
    return resource.getrusage(resource.RUSAGE_SELF).ru_maxrss / 1024.0


def run_build(label, scene, env_overrides, budget):
    """環境変数を差し替えて build_order を1回走らせ、順序と診断を返す。"""
    import importlib
    for k, v in env_overrides.items():
        if v is None:
            os.environ.pop(k, None)
        else:
            os.environ[k] = v
    # 定数は import 時に環境変数を読むのでモジュールを読み直す
    from agents.mysolver import replica as replica_mod
    from agents.mysolver import ordering as ordering_mod
    importlib.reload(replica_mod)
    importlib.reload(ordering_mod)

    container_list, items, lookahead = scene
    ordering_mod.REPLICA_STATS.clear()
    before = n_clients()
    t0 = time.perf_counter()
    with redirect_stdout(io.StringIO()):
        order = ordering_mod.build_order(items, container_list, lookahead, time_budget=budget)
    el = time.perf_counter() - t0
    after = n_clients()
    return {'order': list(order), 'elapsed': el,
            'stats': dict(ordering_mod.REPLICA_STATS),
            'clients_before': before, 'clients_after': after,
            'peak_mb': peak_mb()}


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--budget', type=float, default=120.0)
    ap.add_argument('--out', default=None)
    args = ap.parse_args()

    out = {'bitwise': [], 'runtime': [], 'latch': [], 'clients': [], 'memory': {}}

    # ---------------- (1-2a) 早期失敗はビット単位一致 ----------------
    print('=== (1-2a) 早期失敗(import/init)は ρ-test 無効時とビット単位一致するか ===')
    for label in BITWISE_SCENES:
        scene = load_scene(label)
        base = run_build(label, scene, {'MYSOLVER_REPLICA_SELECT': '0',
                                        'MYSOLVER_REPLICA_FORCE_FAIL': None}, args.budget)
        row = {'label': label, 'base_elapsed': base['elapsed']}
        for mode in ('import', 'init'):
            got = run_build(label, scene, {'MYSOLVER_REPLICA_SELECT': '1',
                                           'MYSOLVER_REPLICA_FORCE_FAIL': mode}, args.budget)
            same = got['order'] == base['order']
            ndiff = sum(1 for a, b in zip(base['order'], got['order']) if a != b)
            row[mode] = {'match': same, 'n_diff': ndiff, 'elapsed': got['elapsed']}
            print(f'  {label:26s} FORCE_FAIL={mode:7s} 一致={same} 差分={ndiff}/{len(base["order"])} '
                  f'({base["elapsed"]:.0f}s -> {got["elapsed"]:.0f}s)', flush=True)
        out['bitwise'].append(row)
        if args.out:
            json.dump(out, open(args.out, 'w'), indent=1, ensure_ascii=False)

    # ---------------- (1-2b) 実行時失敗の劣化幅 ----------------
    print('\n=== (1-2b) 実行時失敗で run1 の劣化(A07 -10.74 / D01 -10.77 / C02 -6.92)を再現しないか ===')
    print('    ※ 代理スコアで比較する。ここは build_order の出力そのものを見ており、')
    print('       実 fill の測定(26シーンA/B)とは別物である点に注意。')
    for label in RUNTIME_SCENES:
        scene = load_scene(label)
        base = run_build(label, scene, {'MYSOLVER_REPLICA_SELECT': '0',
                                        'MYSOLVER_REPLICA_FORCE_FAIL': None}, args.budget)
        row = {'label': label, 'run1_degradation': RUN1_DEGRADATION[label]}
        for mode in ('runtime', 'deadline'):
            got = run_build(label, scene, {'MYSOLVER_REPLICA_SELECT': '1',
                                           'MYSOLVER_REPLICA_FORCE_FAIL': mode}, args.budget)
            st = got['stats']
            row[mode] = {'order_same': got['order'] == base['order'],
                         'stopped': st.get('stopped'), 'latched': st.get('latched'),
                         'evaluated': st.get('evaluated'), 'elapsed': got['elapsed']}
            print(f'  {label:26s} FORCE_FAIL={mode:8s} 停止={st.get("stopped")} '
                  f'ラッチ={st.get("latched")} 実評価={st.get("evaluated")} '
                  f'順序一致={got["order"] == base["order"]} '
                  f'({base["elapsed"]:.0f}s -> {got["elapsed"]:.0f}s)', flush=True)
        out['runtime'].append(row)
        if args.out:
            json.dump(out, open(args.out, 'w'), indent=1, ensure_ascii=False)

    # ---------------- (1-3)/(2-2) ラッチとクライアントリーク ----------------
    print('\n=== (1-3)(2-2) ラッチ動作と pybullet クライアントのリーク ===')
    scene = load_scene('A01_1c_40_plain')
    seq = []
    for i, mode in enumerate([None, 'runtime', None, 'deadline', None]):
        got = run_build('A01_1c_40_plain', scene,
                        {'MYSOLVER_REPLICA_SELECT': '1',
                         'MYSOLVER_REPLICA_FORCE_FAIL': mode}, args.budget)
        st = got['stats']
        seq.append({'i': i, 'mode': mode, 'clients_before': got['clients_before'],
                    'clients_after': got['clients_after'],
                    'evaluated': st.get('evaluated'), 'latched': st.get('latched'),
                    'stopped': st.get('stopped'), 'peak_mb': got['peak_mb']})
        print(f'  run{i} FORCE_FAIL={str(mode):8s} clients {got["clients_before"]}->{got["clients_after"]} '
              f'実評価={st.get("evaluated")} ラッチ={st.get("latched")} '
              f'peakRSS={got["peak_mb"]:.0f}MB', flush=True)
    out['clients'] = seq

    # ---------------- (2-3) ピークメモリ ----------------
    print('\n=== (2-3) ρ-test 有効/無効のピークメモリ差 ===')
    scene = load_scene('A02_1c_80_plain')
    off = run_build('A02', scene, {'MYSOLVER_REPLICA_SELECT': '0',
                                   'MYSOLVER_REPLICA_FORCE_FAIL': None}, args.budget)
    on = run_build('A02', scene, {'MYSOLVER_REPLICA_SELECT': '1',
                                  'MYSOLVER_REPLICA_FORCE_FAIL': None}, args.budget)
    out['memory'] = {'off_peak_mb': off['peak_mb'], 'on_peak_mb': on['peak_mb'],
                     'delta_mb': on['peak_mb'] - off['peak_mb']}
    print(f'  無効時 peakRSS={off["peak_mb"]:.0f}MB / 有効時 peakRSS={on["peak_mb"]:.0f}MB '
          f'(差 {on["peak_mb"] - off["peak_mb"]:+.0f}MB)')

    if args.out:
        json.dump(out, open(args.out, 'w'), indent=1, ensure_ascii=False)

    # ---------------- まとめ ----------------
    print('\n========== まとめ ==========')
    bw = [(r['label'], m, r[m]['match']) for r in out['bitwise'] for m in ('import', 'init')]
    print(f'(1-2a) 早期失敗のビット単位一致: {sum(1 for _, _, ok in bw if ok)}/{len(bw)}')
    lat = [r[m]['latched'] for r in out['runtime'] for m in ('runtime', 'deadline')]
    print(f'(1-3)  ラッチ発動: {sum(1 for x in lat if x)}/{len(lat)}')
    leak = [s for s in seq if s['clients_after'] > s['clients_before']]
    print(f'(2-2)  クライアントのリーク: {len(leak)}件 (0であること)')
    print(f'(2-3)  ピークメモリ差: {out["memory"]["delta_mb"]:+.0f}MB')


if __name__ == '__main__':
    main()
