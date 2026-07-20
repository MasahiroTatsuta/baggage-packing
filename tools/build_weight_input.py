"""
tools/build_weight_input.py  (Phase10 タスク2)

復元した各フェーズの6シーン平均5指標(results/phase10_hist_ladder.json + baseline)を、
各提出のpublicスコアと突き合わせて weight_fit.py の入力(points)を作る。

マッピングの根拠:
  - 提出public(時系列): 16.94, 24.01, 31.26, 37.31, 37.65, 40.18, 42.47
  - 各フェーズcommitの6シーン平均fillの昇順・時系列と、背景で与えられたfillアンカー
    (phase8 fill=25.21 -> public 40.18 は完全一致、phase9 -> 42.47)で対応付け。
  - baseline(95de6e1) = 最初期の提出 16.94。
  - 37.65 は distinct な commit が無い phase7/8 間の微調整提出のため、数値フィットからは除外
    (metrics を復元できる確かなコード状態が無い。レポートに明記)。

出力: results/phase10_weights_input.json
"""
import json

# public への対応 (metricsソース, public, メモ)
MAPPING = [
    ('baseline', 16.94, '95de6e1 初期ベースライン'),
    ('phase4',   24.01, '467ae45'),
    ('phase6',   31.26, 'd145aa3'),
    ('phase7',   37.31, '4d8a1d7 (37.65は同系の微調整提出=フィット除外)'),
    ('phase8',   40.18, 'fb8d80a  fill=25.21 アンカー一致'),
    ('phase9(current)', 42.47, '1ed528b  最新'),
]
METRIC_KEYS = ['fill_score', 'cog_score', 'stability_score', 'placement_score', 'soft_item_score']


def main():
    ladder = json.load(open('results/phase10_hist_ladder.json'))['agents']
    # baseline は別ファイル(measure.py出力形式: per_scene の mean)
    base = json.load(open('results/phase10_baseline.json'))
    base_avg = {k: base['suite_stats'][k]['mean'] for k in METRIC_KEYS}

    def metrics_for(src):
        if src == 'baseline':
            return base_avg
        return ladder[src]['six_scene_avg']

    points = []
    for src, public, note in MAPPING:
        m = metrics_for(src)
        pt = {'tag': src.replace('(current)', ''), 'public': public, 'note': note}
        pt.update({k: round(m[k], 3) for k in METRIC_KEYS})
        points.append(pt)

    out = {'points': points,
           'excluded': [{'public': 37.65, 'reason': 'distinctなcommitが無い微調整提出。metrics復元不可のため数値フィット除外'}],
           'note': ('各点のmetricsは固定6シーン(sample_config 000/001 + gen_2containers_patternB/'
                    'priority + gen_shelf + gen_manyitems)平均。各agentは提出時のネイティブoptimize予算で再計測。')}
    with open('results/phase10_weights_input.json', 'w') as f:
        json.dump(out, f, ensure_ascii=False, indent=2)

    print('=== weight_fit 入力 (results/phase10_weights_input.json) ===')
    hdr = f'{"tag":10s} {"public":>7s} | ' + ' '.join(f'{k.split("_")[0]:>6s}' for k in METRIC_KEYS)
    print(hdr)
    for p in points:
        print(f'{p["tag"]:10s} {p["public"]:7.2f} | ' + ' '.join(f'{p[k]:6.2f}' for k in METRIC_KEYS))


if __name__ == '__main__':
    main()
