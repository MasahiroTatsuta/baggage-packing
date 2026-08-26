"""Phase67: 支持閾値スイープの結果比較(対照 vs 各水準)。

`tools/phase67_suite_metrics.py` が出力した結果JSON(26シーン分+sample_config分、
計28件)を対照(baseline)と各水準で突き合わせ、シーン単位の対応のある差分(paired diff)
から mean/σ/SE/t を算出する。

グループ分け(Phase65/66で確定した「100%純粋」なシーン群):
  threshold_only(forbidden_hit 0件): suite_A02, suite_P01, suite_P02,
      sample_config::000, sample_config::001
  forbidden_hit(100%): suite_B03, suite_C03, suite_D02, suite_P04, suite_P03
  それ以外(残り18シーン): 上記どちらにも属さない全シーン(悪化していないかの確認用)

読み取り専用。tools/scorer.py・configs/は変更しない。

実行方法:
    PYTHONPATH=. python tools/phase67_analyze.py \\
        --baseline results/phase67_baseline_2G.json results/phase67_baseline_2G_sample.json \\
        --variant results/phase67_loose1_2G.json results/phase67_loose1_2G_sample.json \\
        --label "緩1(0.45/0.5/0.20)"
"""
import argparse
import json
import math


THRESHOLD_ONLY_SCENES = {
    'suite_A02_1c_80_plain.json', 'suite_P01_A_1c_pre6.json', 'suite_P02_A_1c_pre10.json',
    'sample_config.json::000', 'sample_config.json::001',
}
FORBIDDEN_HIT_SCENES = {
    'suite_B03_2c_80_prio.json', 'suite_C03_2c_80_prio.json',
    'suite_D02_A_1c_40_prioheavy_nocont.json', 'suite_P03_A_2c_pre8_prio.json',
    'suite_P04_B_1c_pre8_shelf.json',
}

METRICS = ['fill_score', 'cog_score', 'stability_score', 'placement_A', 'soft_A',
           'placement_B', 'soft_B', 'composite_A', 'composite_B', 'num_placed_items']


def load_merged(paths):
    merged = {}
    for p in paths:
        d = json.load(open(p))
        merged.update(d['results'])
    return merged


def mean_std_se_t(diffs):
    n = len(diffs)
    if n == 0:
        return {'mean': float('nan'), 'std': float('nan'), 'se': float('nan'), 't': float('nan'), 'n': 0}
    mean = sum(diffs) / n
    if n > 1:
        var = sum((d - mean) ** 2 for d in diffs) / (n - 1)
        std = math.sqrt(var)
    else:
        std = 0.0
    se = std / math.sqrt(n) if n > 0 else float('nan')
    t = mean / se if se > 0 else float('nan')
    return {'mean': mean, 'std': std, 'se': se, 't': t, 'n': n}


def group_report(label, base, var, scenes):
    print(f"\n--- {label} (n={len(scenes)}) ---")
    out = {}
    for metric in METRICS:
        diffs = []
        for s in scenes:
            bv = base.get(s, {}).get(metric)
            vv = var.get(s, {}).get(metric)
            if bv is None or vv is None:
                continue
            diffs.append(vv - bv)
        stat = mean_std_se_t(diffs)
        out[metric] = stat
        print(f"  {metric:18s}: mean={stat['mean']:+.3f}  sigma={stat['std']:.3f}  "
              f"SE={stat['se']:.3f}  t={stat['t']:+.3f}  n={stat['n']}")
    return out


def worsened_scenes(base, var, scenes, metric='composite_B', eps=0.01):
    rows = []
    for s in scenes:
        bv = base.get(s, {}).get(metric)
        vv = var.get(s, {}).get(metric)
        if bv is None or vv is None:
            continue
        d = vv - bv
        if d < -eps:
            rows.append((s, d))
    rows.sort(key=lambda r: r[1])
    return rows


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument('--baseline', nargs='+', required=True)
    ap.add_argument('--variant', nargs='+', required=True)
    ap.add_argument('--label', default='variant')
    ap.add_argument('--out', default=None)
    args = ap.parse_args()

    base = load_merged(args.baseline)
    var = load_merged(args.variant)
    all_scenes = sorted(set(base.keys()) & set(var.keys()))
    missing = (set(base.keys()) | set(var.keys())) - set(all_scenes)
    if missing:
        print(f"WARNING: baseline/variantで欠けているシーン: {sorted(missing)}")

    rest_scenes = [s for s in all_scenes if s not in THRESHOLD_ONLY_SCENES and s not in FORBIDDEN_HIT_SCENES]

    print(f"===== {args.label} vs baseline =====")
    report = {}
    report['all'] = group_report('全28シーン', base, var, all_scenes)
    report['threshold_only'] = group_report('threshold_only (A02/P01/P02/sample x2)', base, var,
                                             sorted(THRESHOLD_ONLY_SCENES & set(all_scenes)))
    report['forbidden_hit'] = group_report('forbidden_hit (B03/C03/D02/P03/P04)', base, var,
                                            sorted(FORBIDDEN_HIT_SCENES & set(all_scenes)))
    report['rest'] = group_report(f'残り{len(rest_scenes)}シーン', base, var, rest_scenes)

    print("\n--- 悪化シーン(composite_B, -0.01超の悪化) ---")
    worse = worsened_scenes(base, var, all_scenes)
    for s, d in worse:
        print(f"  {s}: {d:+.3f}")
    if not worse:
        print("  なし")

    # fill/stability トレードオフ比(全シーン平均)
    fill_stat = report['all']['fill_score']
    stab_stat = report['all']['stability_score']
    d_fill = fill_stat['mean']
    d_stab = stab_stat['mean']
    net_true = (2 * d_fill + 1.5 * d_stab) / 7
    print(f"\nΔfill(mean)={d_fill:+.3f}  Δstability(mean)={d_stab:+.3f}  "
          f"net(2Δfill+1.5Δstability)/7 = {net_true:+.4f}")
    if d_stab < 0:
        threshold = 0.75 * abs(d_stab)
        verdict = "プラス" if d_fill > threshold else "マイナスまたは拮抗"
        print(f"  基準: Δfill > 0.75*|Δstability| = {threshold:.3f} -> net {verdict}")

    result = {
        'label': args.label,
        'report': report,
        'worsened_scenes': worse,
        'n_scenes': len(all_scenes),
        'missing_scenes': sorted(missing),
    }
    if args.out:
        with open(args.out, 'w') as f:
            json.dump(result, f, ensure_ascii=False, indent=2)
        print(f"\nwrote {args.out}")


if __name__ == '__main__':
    main()
