"""復旧コード健全性チェック(2026-09-07)。

決定的8シーンの build_order 出力を scripts/bp_baseline_8scenes.json と厳格照合する。
bp_check.sh の照合部分を、コンテナ内(conda python, cwd=/workspace)でも動くよう
ハードコードパス・.venv 依存を外して切り出したもの。副作用なし(results/ に書かない)。

実行(コンテナ内、リポジトリルート):
    MYSOLVER_HARD_WALL_LIMIT=3000 PYTHONPATH=. python tools/recover_check.py
"""
import io
import json
import os
import sys
import time
from contextlib import redirect_stdout

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from src.ground_handling.env import GroundHandlingEnv
from agents.mysolver import ordering as ordering_mod

SCENES = {
    'B01': 'configs/gen/suite_B01_1c_40_plain.json',
    'B02': 'configs/gen/suite_B02_1c_40_shelf.json',
    'B03': 'configs/gen/suite_B03_2c_80_prio.json',
    'B04': 'configs/gen/suite_B04_2c_80_noprio.json',
    'P04': 'configs/gen/suite_P04_B_1c_pre8_shelf.json',
    'A01': 'configs/gen/suite_A01_1c_40_plain.json',
    'A02': 'configs/gen/suite_A02_1c_80_plain.json',
    'A03': 'configs/gen/suite_A03_1c_40_shelf.json',
}


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


def main():
    baseline = json.load(open('scripts/bp_baseline_8scenes.json'))
    n_mismatch = 0
    for label, cp in SCENES.items():
        cl, items, lk = load_scene(cp)
        t0 = time.perf_counter()
        with redirect_stdout(io.StringIO()):
            order = ordering_mod.build_order(items, cl, lk, time_budget=30.0)
        dt = time.perf_counter() - t0
        ref = baseline.get(label, {}).get('order')
        if ref is None:
            mark = '(基準値なし)'
        elif order == ref:
            mark = 'OK'
        else:
            mark = '★MISMATCH'
            n_mismatch += 1
            # 先頭の相違位置を出す
            for i, (a, b) in enumerate(zip(order, ref)):
                if a != b:
                    mark += f' (pos {i}: got {a} want {b}; len got/want={len(order)}/{len(ref)})'
                    break
        print(f'[{label}] n={len(order)} ({dt:.1f}s) {mark}')
    print()
    if n_mismatch:
        print(f'!!! {n_mismatch}/8 mismatch — 復旧コードの build_order がベースラインと不一致。')
        sys.exit(1)
    print('8/8 一致 — 復旧コードの build_order はベースラインと同一。')


if __name__ == '__main__':
    main()
