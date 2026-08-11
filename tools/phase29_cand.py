"""
tools/phase29_cand.py

Phase29 ステップ0: build_order が実際に比較している「候補順序の母集団」を、
**新しい採否ロジックを一切入れずに** 記録する読み取り専用ツール。

Phase28 の tools/phase28_rerank.py と同じフック手法だが、2点違う:

  (1) `MYSOLVER_REACH_WEIGHT` を立てない。フック側が `reach_info=None` を `{}` に
      差し替えて統計だけ取る。`ordering.validate()` の割引は
      `discount = 1 - REACH_WEIGHT*br = 1.0` になるので目的関数は**浮動小数点として不変**
      (x*1.0 == x)。つまり既定構成の探索そのものを観測できる。
  (2) 各候補について `placed_volume`(=その順序が最終的に到達した充填体積)も記録し、
      「行き詰まり時点の量から最終fillの順位を復元できるか」(0-2)を
      Spearman 順位相関で直接検定できるようにする。
  (3) `simulate.beam_construct_order` もフックして、各候補が
      「ヒューリスティック順 / フェーズ1(window列挙) / フェーズ2(ランダムリスタート)」
      のどれで作られたかを分類する(0-1 の内訳)。

実行:
    MYSOLVER_UNITS_PER_SEC=1.55e7 PYTHONPATH=. .venv/bin/python tools/phase29_cand.py \
        --config-path configs/gen/suite_A01*.json --out results/phase29_cand_A01.json
"""
import argparse
import glob
import io
import json
import os
import sys
import time
from contextlib import redirect_stdout

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

import numpy as np

from src.ground_handling.env import GroundHandlingEnv
from agents.mysolver import ordering as ordering_mod
from agents.mysolver import simulate as simulate_mod


def collect(task_config):
    """build_order を1回走らせ、評価された全候補順序の内訳を集める。"""
    records = []
    events = []          # 'construct_det' / 'construct_noise' / 'validate'
    orig_sim = simulate_mod.simulate_order
    orig_beam = simulate_mod.beam_construct_order

    def hooked_sim(*args, **kwargs):
        # reach_info=None のときだけ差し替える(REACH_WEIGHT>0 の運用は壊さない)。
        injected = kwargs.get('reach_info') is None
        if injected:
            kwargs['reach_info'] = {}
        info = kwargs['reach_info']
        out = orig_sim(*args, **kwargs)
        placed_ids, placed_volume, risk_vol, viol, srisk = out
        events.append('validate')
        records.append({
            'phase': None,   # 後段で events から埋める
            'n_placed': len(placed_ids),
            'placed_volume': float(placed_volume),
            'risk_vol': float(risk_vol),
            'violation_ratio': float(viol),
            'stability_risk': float(srisk),
            'blocked_ratio': float(info.get('blocked_ratio', 0.0)),
            'blocked_volume': float(info.get('blocked_volume', 0.0)),
            'supported_volume': float(info.get('supported_volume', 0.0)),
            'empty_volume': float(info.get('empty_volume', 0.0)),
            'order': [int(i) for i in args[2]] if len(args) > 2 else None,
        })
        return out

    def hooked_beam(*args, **kwargs):
        events.append('construct_noise' if kwargs.get('rng') is not None else 'construct_det')
        return orig_beam(*args, **kwargs)

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

    simulate_mod.simulate_order = hooked_sim
    simulate_mod.beam_construct_order = hooked_beam
    t0 = time.perf_counter()
    try:
        buf = io.StringIO()
        with redirect_stdout(buf):
            best_order = ordering_mod.build_order(items, container_list, lookahead)
    finally:
        simulate_mod.simulate_order = orig_sim
        simulate_mod.beam_construct_order = orig_beam
    elapsed = time.perf_counter() - t0

    # events を走査して各 validate の由来を決める。
    last_construct = None
    vi = 0
    for ev in events:
        if ev.startswith('construct'):
            last_construct = ev
        else:
            records[vi]['phase'] = {
                None: 'heuristic',
                'construct_det': 'phase1_window',
                'construct_noise': 'phase2_random',
            }[last_construct]
            vi += 1

    total_vol = sum(c.get('volume', 0.0) for c in container_list)
    n_construct_det = sum(1 for e in events if e == 'construct_det')
    n_construct_noise = sum(1 for e in events if e == 'construct_noise')
    return {
        'records': records,
        'total_container_volume': total_vol,
        'n_construct_det': n_construct_det,
        'n_construct_noise': n_construct_noise,
        'n_items': len(items),
        'elapsed': elapsed,
        'best_order': [int(i) for i in best_order],
    }


def spearman(a, b):
    """Spearman 順位相関(同順位は平均順位)。scipy 非依存。"""
    a = np.asarray(a, dtype=float)
    b = np.asarray(b, dtype=float)
    n = len(a)
    if n < 3:
        return None
    def rank(x):
        order = np.argsort(x, kind='stable')
        r = np.empty(n, dtype=float)
        r[order] = np.arange(1, n + 1, dtype=float)
        # 同順位の平均化
        for v in np.unique(x):
            m = (x == v)
            if m.sum() > 1:
                r[m] = r[m].mean()
        return r
    ra, rb = rank(a), rank(b)
    if ra.std() == 0 or rb.std() == 0:
        return None
    return float(np.corrcoef(ra, rb)[0, 1])


def analyze(res, label):
    recs = res['records']
    if not recs:
        print(f'{label}: 候補が記録されなかった(optimize無効シーン)')
        return None
    tv = res['total_container_volume']
    pw = ordering_mod.PLACEMENT_PENALTY_WEIGHT
    base = np.array([r['risk_vol'] - pw * tv * r['violation_ratio'] for r in recs])
    fill = np.array([100.0 * r['placed_volume'] / tv for r in recs])
    phases = [r['phase'] for r in recs]

    out = {
        'label': label,
        'n_cand': len(recs),
        'n_phase1': phases.count('phase1_window'),
        'n_phase2': phases.count('phase2_random'),
        'n_items': res['n_items'],
        'elapsed': res['elapsed'],
        'fill_min': float(fill.min()), 'fill_max': float(fill.max()),
        'fill_spread': float(fill.max() - fill.min()),
        'winner_idx': int(np.argmax(base)),
        'winner_fill': float(fill[int(np.argmax(base))]),
        'best_fill': float(fill.max()),
        'winner_fill_rank': int((fill > fill[int(np.argmax(base))]).sum() + 1),
        'rho_blocked_ratio': spearman(fill, [r['blocked_ratio'] for r in recs]),
        'rho_blocked_volume': spearman(fill, [r['blocked_volume'] for r in recs]),
        'rho_supported_volume': spearman(fill, [r['supported_volume'] for r in recs]),
        'rho_empty_volume': spearman(fill, [r['empty_volume'] for r in recs]),
        'rho_n_placed': spearman(fill, [r['n_placed'] for r in recs]),
        'records': recs,
    }
    print(f'\n=== {label} ===')
    print(f"  候補順序 {out['n_cand']}件 (heuristic 1 / phase1 {out['n_phase1']} / phase2 {out['n_phase2']}), "
          f"荷物{out['n_items']}個, elapsed {out['elapsed']:.1f}s")
    print(f"  候補の代理fill: {out['fill_min']:.2f}〜{out['fill_max']:.2f} "
          f"(スプレッド {out['fill_spread']:.2f}pt)")
    print(f"  勝者(=採用される候補)の代理fill {out['winner_fill']:.2f} "
          f"/ 候補中の最良 {out['best_fill']:.2f} (勝者の順位 {out['winner_fill_rank']}/{out['n_cand']})")
    for k in ('rho_blocked_ratio', 'rho_blocked_volume', 'rho_supported_volume',
              'rho_empty_volume', 'rho_n_placed'):
        v = out[k]
        print(f"  Spearman(fill, {k[4:]:>17s}) = " + ('n/a' if v is None else f'{v:+.3f}'))
    return out


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--config-path', nargs='+', required=True)
    ap.add_argument('--out', default=None)
    args = ap.parse_args()

    # 観測が探索を変えないための条件:
    #  - REACH_WEIGHT=0 -> validate の割引が 1.0 倍(浮動小数点として恒等)
    #  - REACH_UNIT_COST=0 -> 到達可能性の計算がユニット予算を消費しない
    #    (消費するとリスタート回数が変わり、観測対象の母集団そのものが変わってしまう)
    assert ordering_mod.REACH_WEIGHT == 0.0, 'MYSOLVER_REACH_WEIGHT は 0 のまま実行すること'
    assert simulate_mod.REACH_UNIT_COST == 0.0, 'MYSOLVER_REACH_UNIT_COST=0 を指定して実行すること'

    paths = []
    for p in args.config_path:
        paths.extend(sorted(glob.glob(p)))

    results = []
    for path in paths:
        task = list(json.load(open(path)).values())[0]
        label = os.path.basename(path).replace('suite_', '').replace('.json', '')
        res = collect(task)
        r = analyze(res, label)
        if r:
            results.append(r)
        if args.out:
            json.dump(results, open(args.out, 'w'), indent=1, ensure_ascii=False)


if __name__ == '__main__':
    main()
