"""Phase10 タスク2: weight_fit の結果を results/phase10_weights.json に構造化保存する。"""
import json
import numpy as np

METRIC_KEYS = ['fill_score', 'cog_score', 'stability_score', 'placement_score', 'soft_item_score']


def ols(X, y):
    coef, _, _, _ = np.linalg.lstsq(X, y, rcond=None)
    pred = X @ coef
    resid = y - pred
    rmse = float(np.sqrt(np.mean(resid ** 2)))
    ss_res = float(np.sum(resid ** 2)); ss_tot = float(np.sum((y - y.mean()) ** 2))
    r2 = 1.0 - ss_res / ss_tot if ss_tot > 0 else float('nan')
    return coef, rmse, r2


def fit_set(points):
    y = np.array([p['public'] for p in points], float)
    M = np.array([[p[k] for k in METRIC_KEYS] for p in points], float)
    ones = np.ones((len(points), 1))
    fill = M[:, [0]]; cog = M[:, [1]]
    c1, r1, R1 = ols(np.hstack([fill, ones]), y)
    c2, r2, R2 = ols(np.hstack([fill, cog, ones]), y)
    return {
        'n': len(points),
        'M1_fill_only': {'a_fill': round(float(c1[0]), 3), 'b': round(float(c1[1]), 2),
                          'rmse': round(r1, 3), 'r2': round(R1, 3)},
        'M2_fill_cog': {'a_fill': round(float(c2[0]), 3), 'c_cog': round(float(c2[1]), 3),
                        'b': round(float(c2[2]), 2), 'rmse': round(r2, 3), 'r2': round(R2, 3),
                        'rmse_improve_vs_M1': round(r1 - r2, 3),
                        'cog_1pt_public_contribution': round(float(c2[1]), 3)},
    }


def main():
    data = json.load(open('results/phase10_weights_input.json'))
    pts = data['points']
    pts_nb = [p for p in pts if p['tag'] != 'baseline']

    out = {
        'input_points': pts,
        'excluded_submissions': data['excluded'],
        'method': ('固定6シーン平均5指標(各agentをネイティブoptimize予算で再計測)を各提出publicに突き合わせ、'
                   '非負・和1制約は weight_fit.py M4 で別途探索。ここでは仮説B検証の主役 M1/M2 を保存。'),
        'fit_full_6pt': fit_set(pts),
        'fit_robust_5pt_no_baseline': fit_set(pts_nb),
        'M4_simplex_note': ('weight_fit M4(非負・和1)は stab に w=0.95 を置くが、stabは全提出で'
                            '~98.1でほぼ一定のため係数は退化的アーティファクト。解釈不能。'),
        'key_findings': {
            'fill_dominates': 'publicはfillでほぼ説明可能(M1 R2=0.61[6pt]/0.80[5pt])。',
            'cog_contribution_not_robust': ('M2のcog係数はbaseline1点の有無で +3.98→+0.48 と激変。'
                                            '信頼できる5点ではcog寄与≈0(+0.48pt/pt, RMSE改善+0.02)。'),
            'hypothesis_B_verdict': ('弱く支持されない。cog低下がpublicを有意に相殺している証拠は'
                                     '再構成データからは得られない(fill支配)。'),
            'slope_normalization': ('local fill→public の傾きは初期3.5から最近~1へ低下したが、'
                                    'これは「劣化」ではなく、序盤の破綻シーン修復による超線形ゲインが'
                                    '尽き、fill(local)≈fill(public)の自然な~1:1転移レジームに収束したと解釈できる。'),
            'baseline_anomaly': ('baselineはlocal fill=22.25とphase4(20.53)より高いのにpublic=16.94と低い。'
                                 'ローカル改善が本番へ転移しない典型例=仮説A(過適合/非転移)の傍証。'),
        },
        'caveats': 'n=6(有効5)、stab/place/softはほぼ一定。係数の絶対値は指標傾向の把握用で過信禁物。',
    }
    json.dump(out, open('results/phase10_weights.json', 'w'), ensure_ascii=False, indent=2)
    print('saved results/phase10_weights.json')
    print('full6:', out['fit_full_6pt'])
    print('robust5:', out['fit_robust_5pt_no_baseline'])


if __name__ == '__main__':
    main()
