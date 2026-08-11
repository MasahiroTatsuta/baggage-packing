"""
tools/phase28_rerank.py

Phase28: 到達可能性の項が「なぜ順序の選択をほとんど変えないのか」を定量化する。

26シーン測定では **26シーン中1シーン(A03)しか動かなかった**。原因の仮説は:

  build_order が比較する候補順序どうしの risk_adjusted_volume の差が、
  到達可能性の割引 (1 - W*blocked_ratio) が作れる差より **桁で大きい**

である。本ツールは `simulate.simulate_order` をフックして(戻り値も副作用も変えない読み取り
専用のラッパ。`tools/phase20_surrogate.py` と同じ手法)、build_order が実際に評価した
**全候補順序**の (risk_vol, placementペナルティ, blocked_ratio) を記録し、

  - 候補間の risk_vol の相対スプレッド
  - 割引係数 (1 - W*blocked_ratio) の相対スプレッド
  - 「argmax(=採用される順序)が変わる最小の W」

を出す。3つ目が本命で、これが「崖の手前の W では再ランクが起きない」ことを直接示す。

実行:
    MYSOLVER_REACH_WEIGHT=0.25 PYTHONPATH=. .venv/bin/python tools/phase28_rerank.py \
        --config-path configs/gen/suite_A01*.json configs/gen/suite_A02*.json
"""
import argparse
import glob
import json
import os
import sys
from contextlib import redirect_stdout
import io

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

import numpy as np

from src.ground_handling.env import GroundHandlingEnv
from agents.mysolver import ordering as ordering_mod
from agents.mysolver import simulate as simulate_mod


def collect(task_config):
    """build_order を1回走らせ、評価された全候補順序の内訳を集める。"""
    records = []
    original = simulate_mod.simulate_order

    def hooked(*args, **kwargs):
        reach_info = kwargs.get('reach_info')
        out = original(*args, **kwargs)
        placed_ids, placed_volume, risk_vol, viol, srisk = out
        records.append({
            'n_placed': len(placed_ids),
            'risk_vol': float(risk_vol),
            'violation_ratio': float(viol),
            'blocked_ratio': float(reach_info.get('blocked_ratio', 0.0)) if reach_info else None,
            'blocked_volume': float(reach_info.get('blocked_volume', 0.0)) if reach_info else None,
            'supported_volume': float(reach_info.get('supported_volume', 0.0)) if reach_info else None,
        })
        return out

    env = GroundHandlingEnv(config=task_config, verbose=False, render_mode=None)
    try:
        env.reset_settings()
        init = env.get_init_states()
        container_list = init['container_list']
        lookahead = init['lookahead_k']
        items = env.get_info_for_optimization()
    finally:
        try:
            env.close()
        except Exception:
            pass

    simulate_mod.simulate_order = hooked
    try:
        buf = io.StringIO()
        with redirect_stdout(buf):
            ordering_mod.build_order(items, container_list, lookahead)
    finally:
        simulate_mod.simulate_order = original

    total_vol = sum(c.get('volume', 0.0) for c in container_list)
    return records, total_vol


def analyze(records, total_vol, label):
    if not records:
        print(f'{label}: 候補が記録されなかった(optimize無効シーン)')
        return None
    pw = ordering_mod.PLACEMENT_PENALTY_WEIGHT
    base = np.array([r['risk_vol'] - pw * total_vol * r['violation_ratio'] for r in records])
    br = np.array([r['blocked_ratio'] if r['blocked_ratio'] is not None else 0.0 for r in records])
    rv = np.array([r['risk_vol'] for r in records])

    base_spread = base.max() - base.min()
    base_rel = base_spread / max(abs(base.max()), 1e-9)
    br_spread = br.max() - br.min()

    print(f'\n=== {label} ===')
    print(f'  評価された候補順序: {len(records)}件')
    print(f'  base目的関数(risk_vol - placement penalty): '
          f'min={base.min():.4f} max={base.max():.4f} spread={base_spread:.4f} '
          f'(相対 {base_rel*100:.1f}%)')
    print(f'  blocked_ratio: min={br.min():.4f} max={br.max():.4f} spread={br_spread:.4f}')
    win0 = int(np.argmax(base))
    print(f'  W=0 での勝者: 候補#{win0} (base={base[win0]:.4f}, '
          f'placed={records[win0]["n_placed"]}, blocked_ratio={br[win0]:.4f})')

    # W を上げていったとき、勝者が最初に変わる W を二分せず直接走査する
    flip_w = None
    for W in np.arange(0.0, 4.0001, 0.01):
        score = rv * np.maximum(0.0, 1.0 - W * br) - pw * total_vol * np.array(
            [r['violation_ratio'] for r in records])
        if int(np.argmax(score)) != win0:
            flip_w = float(W)
            new = int(np.argmax(score))
            print(f'  **勝者が変わる最小の W = {flip_w:.2f}** -> 候補#{new} '
                  f'(placed={records[new]["n_placed"]}, blocked_ratio={br[new]:.4f})')
            break
    if flip_w is None:
        print('  W<=4.0 の範囲で勝者は一度も変わらない(=この項はこのシーンで完全に無効)')
    # 割引が作れる最大の相対差
    for W in (0.25, 0.5, 1.0):
        disc = np.maximum(0.0, 1.0 - W * br)
        print(f'    W={W}: 割引係数 {disc.min():.3f}〜{disc.max():.3f} '
              f'(作れる相対差 {(disc.max()-disc.min())/max(disc.max(),1e-9)*100:.1f}% '
              f'vs base相対スプレッド {base_rel*100:.1f}%)')
    return {'label': label, 'n_cand': len(records), 'base_rel_spread': base_rel,
            'br_spread': float(br_spread), 'flip_w': flip_w}


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--config-path', nargs='+', required=True)
    ap.add_argument('--out', default=None)
    args = ap.parse_args()

    paths = []
    for p in args.config_path:
        paths.extend(sorted(glob.glob(p)))

    results = []
    for path in paths:
        task = list(json.load(open(path)).values())[0]
        label = os.path.basename(path).replace('suite_', '').replace('.json', '')
        recs, tv = collect(task)
        r = analyze(recs, tv, label)
        if r:
            results.append(r)

    if results:
        flips = [r['flip_w'] for r in results if r['flip_w'] is not None]
        print(f'\n===== 集計 =====')
        print(f'  勝者が変わる W が W<=4.0 に存在したシーン: {len(flips)}/{len(results)}')
        if flips:
            print(f'  その最小W: {sorted(flips)}')
        print(f'  base相対スプレッドの中央値: '
              f'{np.median([r["base_rel_spread"] for r in results])*100:.1f}%')
    if args.out:
        json.dump(results, open(args.out, 'w'), indent=1, ensure_ascii=False)


if __name__ == '__main__':
    main()
