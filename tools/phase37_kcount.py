"""
tools/phase37_kcount.py

Phase37 ステップ1-4: ρ-test の勝者決定を「実fillのargmax」から「5成分合成スコアのargmax」に
差し替えたとき、26シーン中いくつのシーンで勝者(採用される候補順序)が変わるか(=k)を数える。

MYSOLVER_REPLICA_METRIC=composite で build_order を1回走らせるだけで済む
(replica.py の evaluate() は候補ごとに real_fill と composite の両方を rows に記録するため、
 同一実行結果から「fillだけならどの候補が勝っていたか」も事後的に復元できる。2回走らせて
 比較する必要はない)。

出力: 各シーンについて
  - fill_argmax_rank : real_fill の argmax が選ぶ候補の rank
  - composite_argmax_rank : composite の argmax(=実際に採用された勝者)の rank
  - k_flag : 上記2つが異なる(=勝者が変わった)かどうか
  - is_applicable : replica が適用されたシーンかどうか(既積みありは対象外)

実行:
    MYSOLVER_UNITS_PER_SEC=2.00e7 PYTHONPATH=. .venv/bin/python tools/phase37_kcount.py \
        --out results/phase37_kcount.json
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

os.environ['MYSOLVER_REPLICA_METRIC'] = 'composite'
os.environ.setdefault('MYSOLVER_REPLICA_SELECT', '1')

from src.ground_handling.env import GroundHandlingEnv
from agents.mysolver import ordering as ordering_mod


def load_scene(config_path):
    task = list(json.load(open(config_path)).values())[0]
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


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--budget', type=float, default=None,
                    help='未指定ならordering.DEFAULT_TIME_BUDGET(=本番既定)を使う')
    ap.add_argument('--out', default='results/phase37_kcount.json')
    args = ap.parse_args()
    budget = args.budget if args.budget is not None else ordering_mod.DEFAULT_TIME_BUDGET

    config_paths = sorted(glob.glob('configs/gen/suite_*.json'))
    rows = []
    k = 0
    n_applicable = 0
    n_not_applicable = 0

    for cp in config_paths:
        label = os.path.basename(cp).replace('suite_', '').replace('.json', '')
        container_list, items, lookahead = load_scene(cp)
        ordering_mod.REPLICA_STATS.clear()
        t0 = time.perf_counter()
        with redirect_stdout(io.StringIO()):
            order = ordering_mod.build_order(items, container_list, lookahead, time_budget=budget)
        el = time.perf_counter() - t0
        stats = dict(ordering_mod.REPLICA_STATS)

        row = {'label': label, 'elapsed': el, 'enabled': stats.get('enabled', False),
              'stopped': stats.get('stopped'), 'evaluated': stats.get('evaluated')}
        if not stats.get('enabled'):
            row['applicable'] = False
            n_not_applicable += 1
        else:
            n_applicable += 1
            eval_rows = stats.get('rows', [])
            row['applicable'] = True
            row['n_evaluated'] = len(eval_rows)
            if eval_rows:
                fill_best = max(eval_rows, key=lambda r: r['real_fill'])
                comp_rows = [r for r in eval_rows if r.get('composite') is not None]
                if comp_rows:
                    comp_best = max(comp_rows, key=lambda r: r['composite'])
                    row['fill_argmax_rank'] = fill_best['rank']
                    row['composite_argmax_rank'] = comp_best['rank']
                    row['fill_at_fill_argmax'] = fill_best['real_fill']
                    row['fill_at_composite_argmax'] = comp_best['real_fill']
                    row['composite_at_composite_argmax'] = comp_best['composite']
                    row['composite_at_fill_argmax'] = next(
                        (r['composite'] for r in eval_rows if r['rank'] == fill_best['rank']), None)
                    changed = fill_best['rank'] != comp_best['rank']
                    row['k_flag'] = changed
                    if changed:
                        k += 1
                else:
                    row['k_flag'] = None
                    row['note'] = 'composite が1件も取れなかった(replica_scorer失敗の可能性)'
            else:
                row['k_flag'] = None
        rows.append(row)
        print(f'[{label:26s}] applicable={row["applicable"]:5} '
              f'k_flag={row.get("k_flag")} evaluated={row.get("evaluated")} ({el:.1f}s)', flush=True)
        json.dump({'rows': rows, 'k': k, 'n_applicable': n_applicable,
                   'n_not_applicable': n_not_applicable, 'budget': budget},
                  open(args.out, 'w'), indent=1, ensure_ascii=False)

    print(f'\n========== SUMMARY ==========')
    print(f'k(勝者が変わったシーン数) = {k} / applicable={n_applicable} '
          f'(not_applicable(既積みあり等)={n_not_applicable}, 全{len(rows)}シーン)')
    import math
    n_total = len(rows)  # 26。足切りの式は評価ルール(全26シーンA/Bのt上限)通り総数を使う
    if 0 < k < n_total:
        t_upper = 5 * math.sqrt(k) / math.sqrt(n_total - k)
        print(f't上限 5*sqrt(k)/sqrt(26-k) = {t_upper:.3f} (n_total={n_total})')
    print(f'足切り: k>=4 なら26シーンA/Bへ、k<=3なら打ち切り')


if __name__ == '__main__':
    main()
