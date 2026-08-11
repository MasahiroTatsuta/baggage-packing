"""
tools/phase28_verify_reach.py

Phase28: `agents/mysolver/reach.py` が `tools/phase24_corridor_audit.py` からの
**忠実な移植**であることを検証する。

reach.py は探索のホットパスから呼ぶために分類マスク(band_*/blk_*/obs*/reach_optimistic)を
落としてあるが、`covered` / `supported` / `reach_strict` の判定式には手を入れていない。
本ツールは実際の行き詰まり状態に対して両実装を同一 voxel で走らせ、
**3つのマスクが voxel 単位で完全一致すること**を確認する。

実行:
    PYTHONPATH=. .venv/bin/python tools/phase28_verify_reach.py \
        --config-path configs/gen/suite_B01_1c_40_plain.json --voxel 0.025 0.10
"""
import argparse
import glob
import json
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

import numpy as np

from agents.mysolver import geometry as geo
from agents.mysolver import reach as R
from tools import phase24_corridor_audit as A


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--config-path', nargs='+', required=True)
    ap.add_argument('--voxel', nargs='+', type=float, default=[0.025, 0.10])
    args = ap.parse_args()

    paths = []
    for p in args.config_path:
        paths.extend(sorted(glob.glob(p)))

    all_ok = True
    for path in paths:
        cfg = json.load(open(path))
        task = list(cfg.values())[0]
        state = A.run_to_stall(task, 'agents/mysolver/')
        print(f'\n=== {os.path.basename(path)} '
              f'(packed={state["n_packed"]}/{state["n_total"]}, '
              f'remaining={len(state["remaining"])}) ===')

        for voxel in args.voxel:
            for cdict in state['container_list']:
                m_a = A.build_masks(cdict, geo, voxel=voxel)
                o_a = A.fit_and_reach(m_a, state['remaining'], geo, cdict, voxel=voxel)
                m_r = R.build_masks(cdict, voxel=voxel)
                o_r = R.fit_and_reach(m_r, state['remaining'], cdict, voxel=voxel)

                checks = {
                    'empty': np.array_equal(m_a['empty'], m_r['empty']),
                    'occupied': np.array_equal(m_a['occupied'], m_r['occupied']),
                    'covered': np.array_equal(o_a['covered'], o_r['covered']),
                    'supported': np.array_equal(o_a['supported'], o_r['supported']),
                    'reach_strict': np.array_equal(o_a['reach_strict'], o_r['reach_strict']),
                }
                ok = all(checks.values())
                all_ok &= ok
                bad = [k for k, v in checks.items() if not v]
                v3 = voxel ** 3
                blk_a = float((o_a['supported'] & ~o_a['reach_strict']).sum()) * v3
                blk_r = float((o_r['supported'] & ~o_r['reach_strict']).sum()) * v3
                print(f'  voxel={voxel:.3f} c{cdict.get("index")} '
                      f'{"一致 ✅" if ok else "不一致 ❌ " + ",".join(bad)}  '
                      f'blocked(a): audit={blk_a:.4f} reach.py={blk_r:.4f} m^3')

    print(f'\n総合: {"全マスク一致 ✅" if all_ok else "不一致あり ❌"}')
    return 0 if all_ok else 1


if __name__ == '__main__':
    sys.exit(main())
