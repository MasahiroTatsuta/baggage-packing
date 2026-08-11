"""
tools/phase29_step0.py

Phase29 ステップ0 の集計。`tools/phase29_cand.py` が出した候補順序の記録から

  (0-1) optimize有効21シーンの「build_order が実際に比較している候補順序の件数」と、
        その内訳(ヒューリスティック順 / フェーズ1の window 列挙 / フェーズ2のランダムリスタート)
  (0-2) 「最終fillでの順位を、行き詰まり時点の量から復元できるか」
        —— 各シーンで Spearman 順位相関を取り、21シーンで平均して t 検定する

を出す。新しいロールアウトは不要(記録済みJSONの再集計のみ)。
"""
import argparse
import json
import math

import numpy as np

import sys, os
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
from tools.phase29_cand import spearman


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--cand', nargs='+', required=True)
    args = ap.parse_args()

    rows = []
    for f in args.cand:
        rows.extend(json.load(open(f)))
    rows.sort(key=lambda r: r['label'])

    print('== (0-1) 候補順序の件数 ==')
    print(f"{'scene':28s} {'items':>5s} {'cand':>5s} {'heur':>5s} {'win':>5s} {'rand':>5s} "
          f"{'fill範囲':>16s} {'勝者/最良':>14s} {'順位':>6s}")
    for r in rows:
        print(f"{r['label']:28s} {r['n_items']:5d} {r['n_cand']:5d} {1:5d} {r['n_phase1']:5d} "
              f"{r['n_phase2']:5d} {r['fill_min']:7.2f}〜{r['fill_max']:7.2f} "
              f"{r['winner_fill']:6.2f}/{r['best_fill']:6.2f} "
              f"{r['winner_fill_rank']:3d}/{r['n_cand']:<3d}")
    nc = np.array([r['n_cand'] for r in rows])
    print(f"\n  候補件数: min={nc.min()} max={nc.max()} 中央値={np.median(nc):.0f} 平均={nc.mean():.2f}")
    print(f"  フェーズ2(ランダムリスタート)が1回でも回ったシーン: "
          f"{sum(1 for r in rows if r['n_phase2'] > 0)}/{len(rows)}")
    print(f"  フェーズ1(window列挙)の回数: "
          f"{sorted(set(r['n_phase1'] for r in rows))}")
    lost = [r['best_fill'] - r['winner_fill'] for r in rows]
    print(f"  代理の取りこぼし(候補中の最良fill − 実際に選ばれた候補のfill): "
          f"平均 {np.mean(lost):.2f}pt / 最大 {np.max(lost):.2f}pt / "
          f"1位を選べたシーン {sum(1 for r in rows if r['winner_fill_rank'] == 1)}/{len(rows)}")

    print('\n== (0-2) 最終fill(代理)と行き詰まり時点の量との Spearman 順位相関 ==')
    keys = [('rho_blocked_ratio', 'blocked_ratio'), ('rho_blocked_volume', 'blocked_volume'),
            ('rho_supported_volume', 'supported_volume'), ('rho_empty_volume', 'empty_volume(参考)'),
            ('rho_n_placed', 'n_placed(参考)')]
    print(f"{'scene':28s} " + ' '.join(f'{n:>20s}' for _, n in keys))
    for r in rows:
        cells = []
        for k, _ in keys:
            v = r[k]
            cells.append('   n/a' if v is None else f'{v:+.3f}')
        print(f"{r['label']:28s} " + ' '.join(f'{c:>20s}' for c in cells))

    print()
    for k, name in keys:
        vals = [r[k] for r in rows if r[k] is not None]
        if not vals:
            continue
        a = np.array(vals)
        n = len(a)
        sd = a.std(ddof=1) if n > 1 else 0.0
        se = sd / math.sqrt(n) if n > 1 else float('nan')
        t = a.mean() / se if se and se > 0 else float('nan')
        print(f'  {name:22s} 平均ρ={a.mean():+.3f} σ={sd:.3f} SE={se:.3f} '
              f't={t:+.2f} (n={n}, |ρ|>=0.5のシーン {int((np.abs(a) >= 0.5).sum())})')

    # 追加: 「代理目的関数(risk_vol - placementペナルティ)」自体の順位復元力との比較
    print('\n  参考: 代理目的関数 base = risk_vol - 0.5*V*violation の順位復元力')
    bs = []
    for r in rows:
        recs = r['records']
        fill = [rec['placed_volume'] for rec in recs]
        base = [rec['risk_vol'] for rec in recs]
        v = spearman(fill, base)
        if v is not None:
            bs.append(v)
    a = np.array(bs)
    print(f'    risk_vol               平均ρ={a.mean():+.3f} σ={a.std(ddof=1):.3f} (n={len(a)})')


if __name__ == '__main__':
    main()


def partial_spearman(y, x, z):
    """z を除いたうえでの y と x の順位相関(順位に線形回帰した残差どうしの相関)。

    (0-2) の判定には必須。blocked_ratio は「どれだけ密に詰めたか」と強く相関するので、
    素の相関が有意でも **目的関数が既に持っている情報(risk_vol)の焼き直し** かもしれない。
    それを分離する。
    """
    import numpy as np
    def rank(v):
        v = np.asarray(v, dtype=float)
        o = np.argsort(v, kind='stable')
        r = np.empty(len(v)); r[o] = np.arange(1, len(v) + 1)
        for u in np.unique(v):
            m = (v == u)
            if m.sum() > 1:
                r[m] = r[m].mean()
        return r
    ry, rx, rz = rank(y), rank(x), rank(z)
    if len(ry) < 4 or rz.std() == 0:
        return None
    def resid(a):
        b = np.polyfit(rz, a, 1)
        return a - (b[0] * rz + b[1])
    ey, ex = resid(ry), resid(rx)
    if ey.std() < 1e-12 or ex.std() < 1e-12:
        return None
    return float(np.corrcoef(ey, ex)[0, 1])
