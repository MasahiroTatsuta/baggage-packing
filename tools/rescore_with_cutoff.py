"""Phase61 §1: 足切り(apply_cutoff)と定義B(Phase59)を、既存の計測結果JSONに対して
事後に適用し、感度を確認するツール。

`tools/scorer.py` は変更しない。新規ロールアウトも行わない。使うのは:
  - `results/phase60_diag_xy022.json`  (26シーン、現行既定 SAFETY_MARGIN_XY=0.022 での
    診断結果。fill/cog/stability/placement/soft_item の5指標すべてを含む)
  - `results/phase47_sample_config_diag.json` (sample_config 2タスク、同様の5指標。
    Phase55修正前の計測だが、Phase55の本番差分は fill/placement/soft_item 完全不変・
    cog/stability微小[+0.00003/-0.0128]なので影響は無視できる)
  - `results/phase61_container_volumes.json` (`src/ground_handling/containers.py` の
    公式 volume 計算をそのまま呼び出して抽出した、コンテナの有効体積。改変なし)
  - `configs/gen/suite_*.json` / `configs/sample_config.json` (プール全体の対象荷物数、
    定義B用)

【重要: このツールが「しないこと」】
`tools/scorer.py::CUTOFF_CANDIDATES` の7閾値それぞれについて、シーンごとの
cleared/not と合成スコアへの影響は出力する。しかし、**「本番の集計値
(fill 38.09 / cog 59.27 / stability 70.44 / placement 57.60 / soft_item 47.15)に
最も近い候補はどれか」という比較・選定は行わない。** これは公式チュートリアル
セミナーで運営が明言した禁止事項(評価関数内の非公開パラメーターの解析)に
該当するため。理由の詳細は `docs/submission_policy.md` §4 Phase61追記、
`docs/official_spec.md` を参照。

実行方法(リポジトリルートで):
    PYTHONPATH=. python tools/rescore_with_cutoff.py
"""
import json
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from tools.scorer import CUTOFF_CANDIDATES, METRIC_KEYS, apply_cutoff, composite_score
from tools.rescore_placement_soft import CONFIG_MAP, _pool_counts, _wrong_container_count, _score


def _pool_total(config_path: str, task_key: str) -> int:
    d = json.load(open(config_path))
    task = d[task_key] if task_key in d else list(d.values())[0]
    return len(task['item_stream']['item_list'])


def _definition_b_metrics(diag: dict, config_path: str, task_key: str) -> dict:
    """Phase59定義B(分母=プール全体、未配置も違反)でplacement/soft_itemを再計算した
    metricsセットを返す(cog/stability/fillはそのまま)。"""
    n_prio_pool, n_soft_pool, container_is_prio = _pool_counts(config_path, task_key)
    n_wrong = _wrong_container_count(diag.get('geo', []), container_is_prio)
    violated_p = diag['n_prio_crushed_pairs'] + n_wrong
    violated_s = diag['n_soft_crushed_pairs']
    placement_B = _score(violated_p + (n_prio_pool - diag['n_prioritized_placed']), n_prio_pool)
    soft_B = _score(violated_s + (n_soft_pool - diag['n_soft_placed']), n_soft_pool)
    m = dict(diag)
    m['placement_score'] = placement_B
    m['soft_item_score'] = soft_B
    return m


def _row(diag: dict, config_path: str, task_key: str, container_volume: float) -> dict:
    n = diag['n_placed']
    t = diag['total_items']
    pv = sum(g['length'] * g['width'] * g['height'] for g in diag.get('geo', []))
    cv = container_volume

    base_metrics = {
        'fill_score': diag['fill_score'], 'cog_score': diag['cog_score'],
        'stability_score': diag['stability_score'], 'placement_score': diag['placement_score'],
        'soft_item_score': diag['soft_item_score'],
    }
    defb_metrics = _definition_b_metrics(diag, config_path, task_key)
    defb_metrics = {k: defb_metrics[k] for k in METRIC_KEYS}

    out = {
        'composite_baseline': composite_score(base_metrics),
        'composite_defB_only': composite_score(defb_metrics),
    }
    for name, fn in CUTOFF_CANDIDATES:
        cleared = bool(fn(n, t, pv, cv))
        out[f'cutoff[{name}]_cleared'] = cleared
        out[f'cutoff[{name}]_only'] = composite_score(apply_cutoff(base_metrics, cleared))
        out[f'cutoff[{name}]_and_defB'] = composite_score(apply_cutoff(defb_metrics, cleared))
    out['n_placed'] = n
    out['total_items'] = t
    out['fill_score'] = diag['fill_score']
    return out


def main():
    diag26 = json.load(open('results/phase60_diag_xy022.json'))
    diag_sample = json.load(open('results/phase47_sample_config_diag.json'))
    volumes = json.load(open('results/phase61_container_volumes.json'))

    rows26 = {}
    for label, diag in diag26.items():
        fname, tk = label.split('::')
        cp = CONFIG_MAP[fname]
        rows26[label] = _row(diag, cp, tk, volumes[label])

    rows_sample = {}
    for label, diag in diag_sample.items():
        fname, tk = label.split('::')
        rows_sample[label] = _row(diag, 'configs/sample_config.json', tk, volumes[label])

    def mean(key, rows):
        vals = [r[key] for r in rows.values()]
        return sum(vals) / len(vals)

    print('=== 26シーン平均(合成スコア = 5指標単純平均) ===')
    print(f"  baseline(足切りなし・定義A):        {mean('composite_baseline', rows26):.2f}")
    print(f"  定義Bのみ(未配置も違反に算入):      {mean('composite_defB_only', rows26):.2f}")
    for name, _ in CUTOFF_CANDIDATES:
        print(f"  足切り[{name}]のみ:               "
              f"{mean(f'cutoff[{name}]_only', rows26):.2f}  "
              f"(clear={sum(r[f'cutoff[{name}]_cleared'] for r in rows26.values())}/26)")
    for name, _ in CUTOFF_CANDIDATES:
        print(f"  足切り[{name}]+定義B:            {mean(f'cutoff[{name}]_and_defB', rows26):.2f}")

    print()
    print('=== sample_config(2タスク) ===')
    print(f"  baseline: {mean('composite_baseline', rows_sample):.2f}")
    print(f"  定義Bのみ: {mean('composite_defB_only', rows_sample):.2f}")
    for name, _ in CUTOFF_CANDIDATES:
        print(f"  足切り[{name}]のみ: {mean(f'cutoff[{name}]_only', rows_sample):.2f}  "
              f"(clear={sum(r[f'cutoff[{name}]_cleared'] for r in rows_sample.values())}/2)")
    for name, _ in CUTOFF_CANDIDATES:
        print(f"  足切り[{name}]+定義B: {mean(f'cutoff[{name}]_and_defB', rows_sample):.2f}")

    with open('results/phase61_cutoff_rescore.json', 'w') as f:
        json.dump({'scenes_26': rows26, 'sample_config': rows_sample}, f, indent=2, ensure_ascii=False)
    print('\nwrote results/phase61_cutoff_rescore.json')
    print('\n注記: 本番値との「最も近い候補」比較はPhase61方針(docs/submission_policy.md §4)'
          'により意図的に行っていません。')


if __name__ == '__main__':
    main()
