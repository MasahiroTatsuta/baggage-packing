"""
tools/weight_fit.py

Phase10 タスク2: 提出履歴(public score)を、各時点のローカル5指標の関数として
最もよく説明するモデルを推定する。仮説B(cogとのトレードオフ)の検証が目的。

入力: --data <json>。形式:
  {
    "points": [
      {"tag": "phase8", "public": 40.18,
       "fill_score": 25.21, "cog_score": 61.9, "stability_score": 98.3,
       "placement_score": 100.0, "soft_item_score": 100.0},
      ...
    ]
  }

推定するモデル:
  M1 (fill単独)     : public = a*fill + b                        (2 params)
  M2 (fill+cog)     : public = a*fill + c*cog + b                (3 params) ← 仮説B検証の主役
  M3 (5指標フリー)  : public = Σ w_i*metric_i + b                (6 params, 7点では飽和気味・参考)
  M4 (単体重み,和1) : public = A*(Σ w_i*metric_i) + B, w>=0,Σw=1  (simplex上グリッド探索)

出力: 各モデルの係数・残差(RMSE)・R²、fill単独からの改善、
      「cogを1pt上げることのpublicスコア寄与」の見積もり。

scipy非依存(numpyのみ)。simplexはグリッド探索(step=0.05)+各点で closed-form の
affine(A,B)最小二乗。データ7点なので過信は禁物(全出力に注意書き)。
"""
import argparse
import itertools
import json

import numpy as np

METRIC_KEYS = ['fill_score', 'cog_score', 'stability_score', 'placement_score', 'soft_item_score']
SHORT = {'fill_score': 'fill', 'cog_score': 'cog', 'stability_score': 'stab',
         'placement_score': 'place', 'soft_item_score': 'soft'}


def _ols(X, y):
    """普通の最小二乗。X は (n,p) 説明変数(定数項は呼び出し側で付与)。返り値: 係数, 予測, rmse, r2"""
    coef, _, _, _ = np.linalg.lstsq(X, y, rcond=None)
    pred = X @ coef
    resid = y - pred
    rmse = float(np.sqrt(np.mean(resid ** 2)))
    ss_res = float(np.sum(resid ** 2))
    ss_tot = float(np.sum((y - y.mean()) ** 2))
    r2 = 1.0 - ss_res / ss_tot if ss_tot > 0 else float('nan')
    return coef, pred, rmse, r2


def _simplex_grid(dim, step):
    """和=1・非負のグリッド点(各成分は step の整数倍)を列挙。"""
    n = int(round(1.0 / step))
    for combo in itertools.combinations_with_replacement(range(dim), 0):
        pass
    # star-and-bars: dim 個の非負整数で和 n
    for cuts in itertools.combinations(range(n + dim - 1), dim - 1):
        prev = -1
        parts = []
        for c in cuts:
            parts.append(c - prev - 1)
            prev = c
        parts.append(n + dim - 1 - prev - 1)
        yield np.array(parts, dtype=float) / n


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--data', required=True)
    ap.add_argument('--simplex-step', type=float, default=0.05)
    args = ap.parse_args()

    with open(args.data) as f:
        data = json.load(f)
    points = data['points']
    n = len(points)
    tags = [p['tag'] for p in points]
    y = np.array([p['public'] for p in points], dtype=float)
    M = np.array([[p[k] for k in METRIC_KEYS] for p in points], dtype=float)  # (n,5)

    print(f'=== データ {n} 点 ===')
    hdr = f'{"tag":14s} {"public":>7s} | ' + ' '.join(f'{SHORT[k]:>6s}' for k in METRIC_KEYS)
    print(hdr)
    for i in range(n):
        print(f'{tags[i]:14s} {y[i]:7.2f} | ' + ' '.join(f'{M[i,j]:6.2f}' for j in range(5)))
    print(f'\n注意: n={n}点と極めて少なく、stab/place/softはほぼ一定のため重みは不定に近い。'
          f'\n以下は「傾向」把握用であり、係数の絶対値を過信しないこと。\n')

    ones = np.ones((n, 1))
    fill = M[:, [0]]
    cog = M[:, [1]]

    # M1: fill単独
    X1 = np.hstack([fill, ones])
    c1, pred1, rmse1, r2_1 = _ols(X1, y)
    print('--- M1: public = a*fill + b ---')
    print(f'  a(fill)={c1[0]:.3f}  b={c1[1]:.3f}   RMSE={rmse1:.2f}  R2={r2_1:.3f}')

    # M2: fill + cog
    X2 = np.hstack([fill, cog, ones])
    c2, pred2, rmse2, r2_2 = _ols(X2, y)
    print('\n--- M2: public = a*fill + c*cog + b  (仮説B検証) ---')
    print(f'  a(fill)={c2[0]:.3f}  c(cog)={c2[1]:.3f}  b={c2[2]:.3f}   RMSE={rmse2:.2f}  R2={r2_2:.3f}')
    print(f'  fill単独からのRMSE改善: {rmse1 - rmse2:+.2f} pt  (R2 {r2_1:.3f}->{r2_2:.3f})')
    print(f'  → cogを1pt上げるときのpublic寄与の推定: {c2[1]:+.3f} pt/pt')
    if c2[1] > 0 and (rmse1 - rmse2) > 0.1:
        print('  → cogの係数は正かつ当てはまり改善あり: 仮説B(cog低下がスコアを相殺)を支持する向き')
    elif abs(c2[1]) < 0.05 or (rmse1 - rmse2) <= 0.05:
        print('  → cogの寄与はほぼゼロ/当てはまり改善なし: 仮説Bは弱い(fillでほぼ説明できる)')
    else:
        print('  → cogの係数は負: 仮説Bとは逆(cogは説明変数として効いていない可能性)')

    # M3: 5指標フリー(参考・飽和気味)
    X3 = np.hstack([M, ones])
    c3, pred3, rmse3, r2_3 = _ols(X3, y)
    print('\n--- M3: public = Σ w_i*metric_i + b  (5指標フリー, 参考: 6params/7点で飽和気味) ---')
    for j, k in enumerate(METRIC_KEYS):
        print(f'    {SHORT[k]:>6s}: {c3[j]:+.3f}')
    print(f'    b={c3[5]:+.3f}   RMSE={rmse3:.2f}  R2={r2_3:.3f}')

    # M4: simplex重み(和1・非負) + affine
    best = None
    for w in _simplex_grid(5, args.simplex_step):
        comp = M @ w                       # (n,)
        Xc = np.hstack([comp[:, None], ones])
        c, pred, rmse, r2 = _ols(Xc, y)
        if best is None or rmse < best['rmse']:
            best = {'w': w, 'A': c[0], 'B': c[1], 'rmse': rmse, 'r2': r2}
    print(f'\n--- M4: public = A*(Σ w_i*metric_i) + B,  w>=0, Σw=1 (simplex grid step={args.simplex_step}) ---')
    print('  最良の重み w:')
    for j, k in enumerate(METRIC_KEYS):
        print(f'    {SHORT[k]:>6s}: {best["w"][j]:.2f}')
    print(f'  A={best["A"]:.3f}  B={best["B"]:.3f}   RMSE={best["rmse"]:.2f}  R2={best["r2"]:.3f}')
    print(f'  fill単独(M1)からのRMSE改善: {rmse1 - best["rmse"]:+.2f} pt')
    # 単体重みモデルでの「cog 1pt」の寄与 = A * w_cog
    print(f'  → このモデルでの cog 1pt の寄与: {best["A"] * best["w"][1]:+.3f} pt/pt'
          f'  (fill 1pt: {best["A"] * best["w"][0]:+.3f} pt/pt)')

    print('\n=== まとめ ===')
    print(f'  fill単独 RMSE={rmse1:.2f} / +cog RMSE={rmse2:.2f} / simplex RMSE={best["rmse"]:.2f}')
    print('  (RMSEが小さいほど良い当てはまり。ただしパラメータ数増による過学習に注意)')


if __name__ == '__main__':
    main()
