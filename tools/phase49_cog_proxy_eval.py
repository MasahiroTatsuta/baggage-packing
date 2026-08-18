"""
tools/phase49_cog_proxy_eval.py

Phase49 作業2(最重要ゲート): `tools/phase29_cand.py` が記録した全候補順序
(21シーン・130件)を `agents.mysolver.simulate.simulate_order(compute_cog_proxy=True)`
に**再生**し(構築は一切やり直さない、既に確定している`order`を流すだけ——Phase35の
replica忠実度検証と同じ立て付け)、影シミュレータのcog代理値を得る。
本物の cog_score(`results/phase30_cand_eval.json`、Scorerによる実測)と突き合わせ、
シーン内順位のSpearman相関を全体・シーン別・既積みあり/なし層別で報告する。

新しい探索は一切行わない。agents/mysolver/ の他ファイルは変更しない。

実行:
    PYTHONPATH=. .venv/bin/python tools/phase49_cog_proxy_eval.py \
        --cand results/phase29_cand_g1.json results/phase29_cand_g2.json \
        --real results/phase30_cand_eval.json \
        --manifest tools/suite_manifest.json \
        --out results/phase49_cog_proxy_eval.json
"""
import argparse
import io
import json
import os
import sys
from contextlib import redirect_stdout

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from scipy.stats import spearmanr

from src.ground_handling.env import GroundHandlingEnv
from agents.mysolver import geometry as geo
from agents.mysolver import ordering as ordering_mod
from agents.mysolver import planner
from agents.mysolver import simulate as simulate_mod


def scene_path(label):
    return f'configs/gen/suite_{label}.json'


def load_scene(label):
    task = list(json.load(open(scene_path(label))).values())[0]
    env = GroundHandlingEnv(config=task, verbose=False, render_mode=None)
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
    items_by_index = {it['index']: it for it in items}
    prepacked_ids = geo.initial_prepacked_ids(container_list)
    return container_list, items_by_index, lookahead, prepacked_ids


def replay_cog_proxy(container_list, items_by_index, order, lookahead, prepacked_ids):
    budget = planner.SearchBudget.from_seconds(600.0)
    buf = io.StringIO()
    with redirect_stdout(buf):
        (placed_ids, placed_volume, risk_vol, viol, srisk, cog_proxy) = simulate_mod.simulate_order(
            container_list, items_by_index, order, max(1, int(lookahead or 1)), budget,
            prepacked_ids=prepacked_ids, stability_weight=ordering_mod.STABILITY_PENALTY_WEIGHT,
            compute_cog_proxy=True)
    return cog_proxy


def spearman(xs, ys):
    """外部ライブラリ非依存の簡易Spearman順位相関(タイは平均順位で処理)。"""
    n = len(xs)
    if n < 2:
        return float('nan')

    def ranks(vals):
        order_idx = sorted(range(len(vals)), key=lambda i: vals[i])
        r = [0.0] * len(vals)
        i = 0
        while i < len(order_idx):
            j = i
            while j + 1 < len(order_idx) and vals[order_idx[j + 1]] == vals[order_idx[i]]:
                j += 1
            avg_rank = (i + j) / 2.0 + 1.0
            for k in range(i, j + 1):
                r[order_idx[k]] = avg_rank
            i = j + 1
        return r

    rx = ranks(xs)
    ry = ranks(ys)
    mean_rx = sum(rx) / n
    mean_ry = sum(ry) / n
    cov = sum((a - mean_rx) * (b - mean_ry) for a, b in zip(rx, ry))
    var_x = sum((a - mean_rx) ** 2 for a in rx)
    var_y = sum((b - mean_ry) ** 2 for b in ry)
    if var_x == 0 or var_y == 0:
        return float('nan')
    return cov / (var_x ** 0.5 * var_y ** 0.5)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--cand', nargs='+', required=True)
    ap.add_argument('--real', required=True)
    ap.add_argument('--manifest', default='tools/suite_manifest.json')
    ap.add_argument('--out', default=None)
    args = ap.parse_args()

    cands = []
    for f in args.cand:
        cands.extend(json.load(open(f)))
    cands.sort(key=lambda c: c['label'])

    real_rows = json.load(open(args.real))
    real_by_key = {(r['label'], r['cand_idx']): r for r in real_rows}

    manifest = json.load(open(args.manifest)) if os.path.exists(args.manifest) else {}

    rows = []
    per_scene_cache = {}
    for c in cands:
        label = c['label']
        if label not in per_scene_cache:
            per_scene_cache[label] = load_scene(label)
        container_list, items_by_index, lookahead, prepacked_ids = per_scene_cache[label]
        manifest_key = f'suite_{label}.json::000'
        prepacked = manifest.get(manifest_key, {}).get('initial_state') == 'prepacked'
        for idx, rec in enumerate(c['records']):
            real = real_by_key.get((label, idx))
            if real is None:
                continue
            cog_proxy = replay_cog_proxy(container_list, items_by_index, rec['order'], lookahead, prepacked_ids)
            rows.append({
                'label': label, 'cand_idx': idx, 'prepacked': prepacked,
                'cog_proxy': cog_proxy, 'cog_real': real['cog_score'],
                'fill_strict': real['fill_strict'],
            })
            print(f"{label:28s} cand{idx} prepacked={prepacked} "
                  f"cog_proxy={cog_proxy:6.2f} cog_real={real['cog_score']:6.2f}", flush=True)

    if args.out:
        json.dump(rows, open(args.out, 'w'), indent=1, ensure_ascii=False)

    # --- 全体のSpearman(全130候補をまとめてプールした順位相関) ---
    proxy_all = [r['cog_proxy'] for r in rows]
    real_all = [r['cog_real'] for r in rows]
    rho_all = spearman(proxy_all, real_all)
    print(f"\n=== 全体(プール、n={len(rows)}) Spearman = {rho_all:.4f} ===")

    # --- シーン内順位のSpearman(シーンごとに計算し、平均も出す) ---
    by_label = {}
    for r in rows:
        by_label.setdefault(r['label'], []).append(r)
    print("\n=== シーン別(シーン内順位) ===")
    within_rhos = []
    for label, rs in sorted(by_label.items()):
        rho = spearman([r['cog_proxy'] for r in rs], [r['cog_real'] for r in rs])
        within_rhos.append(rho)
        print(f"  {label:28s} n={len(rs):2d} prepacked={rs[0]['prepacked']} Spearman={rho:.4f}")
    valid_within = [r for r in within_rhos if r == r]  # NaN除外
    if valid_within:
        print(f"\nシーン内Spearmanの平均: {sum(valid_within)/len(valid_within):.4f} (n_scenes={len(valid_within)})")

    # --- 既積みあり/なし層別(プール) ---
    print("\n=== 既積みあり/なし層別(プール) ===")
    for prepacked_flag in (False, True):
        sub = [r for r in rows if r['prepacked'] == prepacked_flag]
        if not sub:
            continue
        rho = spearman([r['cog_proxy'] for r in sub], [r['cog_real'] for r in sub])
        print(f"  prepacked={prepacked_flag}: n={len(sub)} Spearman={rho:.4f}")

    # --- 系統的なズレの確認(平均差・相関係数だけでなく実際のオフセットも見る) ---
    diffs = [r['cog_proxy'] - r['cog_real'] for r in rows]
    mean_diff = sum(diffs) / len(diffs)
    print(f"\n=== 系統的なズレ ===\n代理値 - 実測値 の平均: {mean_diff:+.2f}pt "
          f"(範囲 {min(diffs):+.2f}〜{max(diffs):+.2f})")


if __name__ == '__main__':
    main()
