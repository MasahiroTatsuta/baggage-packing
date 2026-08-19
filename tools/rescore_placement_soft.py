"""placement_score/soft_item_scoreの「分母仮説」を検証する(Phase59)。

`tools/scorer.py::calculate_placement_score/calculate_soft_item_score`は変更しない。
既存の診断結果(`results/phase45_stacking_diag_wallbound.json`——26シーン、geo付き、
`results/phase47_sample_config_diag.json`——sample_config 2タスク、geo付き)と
シーンconfig(`configs/gen/suite_*.json`/`configs/sample_config.json`)だけを使い、
4つの分母/分子定義で再計算する。新規ロールアウトは行わない。

  定義A(現行): 分母=配置済みの対象荷物、違反=下敷き or 誤コンテナ
  定義B: 分母=プール全体の対象荷物、違反=下敷き or 誤コンテナ or 未配置
  定義C: 分母=プール全体、違反=下敷き or 誤コンテナのみ(未配置は違反にしない)
  定義D: 分母=配置済み、違反に未配置を加算(分母と分子の扱いを分離)

  soft_itemは「誤コンテナ」条件が無いため、下敷き and/or 未配置のみで組む。

読み取り専用(`tools/scorer.py`・`configs/`・診断結果jsonはいずれも変更しない)。

実行方法(リポジトリルートで):
    PYTHONPATH=. python tools/rescore_placement_soft.py
"""
import json
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))


def _pool_counts(config_path: str, task_key: str) -> tuple[int, int, dict]:
    """シーンconfigからプール全体のprioritized/soft個数と、コンテナのis_prioritized写像を返す。"""
    d = json.load(open(config_path))
    task = d[task_key] if task_key in d else list(d.values())[0]
    items = task['item_stream']['item_list']
    n_prio_pool = sum(1 for it in items if it.get('is_prioritized'))
    n_soft_pool = sum(1 for it in items if it.get('is_soft'))
    containers = task['containers']['container_list']
    container_is_prio = {c['index']: c.get('is_prioritized', False) for c in containers}
    return n_prio_pool, n_soft_pool, container_is_prio


def _wrong_container_count(geo: list, container_is_prio: dict) -> int:
    """配置済み優先手荷物のうち、優先コンテナが存在するのに非優先コンテナへ
    置かれてしまった個数(tools/scorer.py::calculate_placement_scoreの(b)条件と同式)。"""
    has_prioritized_container = any(container_is_prio.values())
    if not has_prioritized_container:
        return 0
    n_wrong = 0
    for g in geo:
        if g.get('is_prioritized') and not container_is_prio.get(g['container_index'], False):
            n_wrong += 1
    return n_wrong


def _score(violated: int, denom: int) -> float:
    if denom <= 0:
        return 100.0
    return min(max(100.0 * (1.0 - violated / denom), 0.0), 100.0)


def rescore_scene(diag: dict, config_path: str, task_key: str) -> dict:
    n_prio_pool, n_soft_pool, container_is_prio = _pool_counts(config_path, task_key)
    n_prio_placed = diag['n_prioritized_placed']
    n_soft_placed = diag['n_soft_placed']
    n_prio_crushed = diag['n_prio_crushed_pairs']
    n_soft_crushed = diag['n_soft_crushed_pairs']
    n_wrong = _wrong_container_count(diag.get('geo', []), container_is_prio)

    # placement(優先手荷物)
    violated_A_p = n_prio_crushed + n_wrong
    result = {
        'n_prio_pool': n_prio_pool, 'n_prio_placed': n_prio_placed,
        'n_prio_crushed': n_prio_crushed, 'n_prio_wrong_container': n_wrong,
        'n_soft_pool': n_soft_pool, 'n_soft_placed': n_soft_placed,
        'n_soft_crushed': n_soft_crushed,
        'placement_A': _score(violated_A_p, n_prio_placed),
        'placement_B': _score(violated_A_p + (n_prio_pool - n_prio_placed), n_prio_pool),
        'placement_C': _score(violated_A_p, n_prio_pool),
        'placement_D': _score(violated_A_p + (n_prio_pool - n_prio_placed), n_prio_placed),
        'soft_A': _score(n_soft_crushed, n_soft_placed),
        'soft_B': _score(n_soft_crushed + (n_soft_pool - n_soft_placed), n_soft_pool),
        'soft_C': _score(n_soft_crushed, n_soft_pool),
        'soft_D': _score(n_soft_crushed + (n_soft_pool - n_soft_placed), n_soft_placed),
    }
    # 元の診断値(定義Aと一致するはずの検算)
    result['placement_orig'] = diag['placement_score']
    result['soft_orig'] = diag['soft_item_score']
    return result


CONFIG_MAP = {
    'suite_A01_1c_40_plain.json': 'configs/gen/suite_A01_1c_40_plain.json',
    'suite_A02_1c_80_plain.json': 'configs/gen/suite_A02_1c_80_plain.json',
    'suite_A03_1c_40_shelf.json': 'configs/gen/suite_A03_1c_40_shelf.json',
    'suite_A04_2c_80_noprio.json': 'configs/gen/suite_A04_2c_80_noprio.json',
    'suite_A05_2c_80_prio.json': 'configs/gen/suite_A05_2c_80_prio.json',
    'suite_A06_1c_40_small.json': 'configs/gen/suite_A06_1c_40_small.json',
    'suite_A07_1c_40_bulky.json': 'configs/gen/suite_A07_1c_40_bulky.json',
    'suite_A08_2c_140_extreme.json': 'configs/gen/suite_A08_2c_140_extreme.json',
    'suite_B01_1c_40_plain.json': 'configs/gen/suite_B01_1c_40_plain.json',
    'suite_B02_1c_40_shelf.json': 'configs/gen/suite_B02_1c_40_shelf.json',
    'suite_B03_2c_80_prio.json': 'configs/gen/suite_B03_2c_80_prio.json',
    'suite_B04_2c_80_noprio.json': 'configs/gen/suite_B04_2c_80_noprio.json',
    'suite_C01_1c_40_shelf.json': 'configs/gen/suite_C01_1c_40_shelf.json',
    'suite_C02_2c_55_shelfprio.json': 'configs/gen/suite_C02_2c_55_shelfprio.json',
    'suite_C03_2c_80_prio.json': 'configs/gen/suite_C03_2c_80_prio.json',
    'suite_D01_A_1c_40_softheavy.json': 'configs/gen/suite_D01_A_1c_40_softheavy.json',
    'suite_D02_A_1c_40_prioheavy_nocont.json': 'configs/gen/suite_D02_A_1c_40_prioheavy_nocont.json',
    'suite_D03_A_2c_60_prioheavy_cont.json': 'configs/gen/suite_D03_A_2c_60_prioheavy_cont.json',
    'suite_D04_A_1c_40_flat.json': 'configs/gen/suite_D04_A_1c_40_flat.json',
    'suite_D05_A_1c_40_tall.json': 'configs/gen/suite_D05_A_1c_40_tall.json',
    'suite_P01_A_1c_pre6.json': 'configs/gen/suite_P01_A_1c_pre6.json',
    'suite_P02_A_1c_pre10.json': 'configs/gen/suite_P02_A_1c_pre10.json',
    'suite_P03_A_2c_pre8_prio.json': 'configs/gen/suite_P03_A_2c_pre8_prio.json',
    'suite_P04_B_1c_pre8_shelf.json': 'configs/gen/suite_P04_B_1c_pre8_shelf.json',
    'suite_P05_C_2c_pre8_shelfprio.json': 'configs/gen/suite_P05_C_2c_pre8_shelfprio.json',
    'suite_P06_A_1c_pre12_dense.json': 'configs/gen/suite_P06_A_1c_pre12_dense.json',
}


def main():
    diag26 = json.load(open('results/phase45_stacking_diag_wallbound.json'))
    diag_sample = json.load(open('results/phase47_sample_config_diag.json'))

    rows = []
    for label, diag in diag26.items():
        fname, task_key = label.split('::')
        cp = CONFIG_MAP[fname]
        r = rescore_scene(diag, cp, task_key)
        r['label'] = fname.replace('suite_', '').replace('.json', '')
        rows.append(r)

    sample_rows = []
    for label, diag in diag_sample.items():
        fname, task_key = label.split('::')
        r = rescore_scene(diag, 'configs/sample_config.json', task_key)
        r['label'] = label
        sample_rows.append(r)

    def mean(key, rs):
        return sum(r[key] for r in rs) / len(rs)

    print(f"{'label':32s} {'A(現行)':>8s} {'B':>8s} {'C':>8s} {'D':>8s}   | soft "
          f"{'A':>8s} {'B':>8s} {'C':>8s} {'D':>8s}")
    for r in rows:
        print(f"{r['label']:32s} {r['placement_A']:8.2f} {r['placement_B']:8.2f} "
              f"{r['placement_C']:8.2f} {r['placement_D']:8.2f}   |      "
              f"{r['soft_A']:8.2f} {r['soft_B']:8.2f} {r['soft_C']:8.2f} {r['soft_D']:8.2f}")

    print()
    print("=== 26シーン平均 ===")
    for k in ['placement_A', 'placement_B', 'placement_C', 'placement_D']:
        print(f"  {k}: {mean(k, rows):.4f}")
    for k in ['soft_A', 'soft_B', 'soft_C', 'soft_D']:
        print(f"  {k}: {mean(k, rows):.4f}")

    print()
    print("=== sample_config (2タスク) ===")
    for r in sample_rows:
        print(f"{r['label']:32s} placement A/B/C/D = {r['placement_A']:.2f}/{r['placement_B']:.2f}/"
              f"{r['placement_C']:.2f}/{r['placement_D']:.2f}  "
              f"soft A/B/C/D = {r['soft_A']:.2f}/{r['soft_B']:.2f}/{r['soft_C']:.2f}/{r['soft_D']:.2f}")
    for k in ['placement_A', 'placement_B', 'placement_C', 'placement_D', 'soft_A', 'soft_B', 'soft_C', 'soft_D']:
        print(f"  mean {k}: {mean(k, sample_rows):.4f}")

    with open('results/phase59_rescore.json', 'w') as f:
        json.dump({'scenes_26': rows, 'sample_config': sample_rows}, f, indent=2, ensure_ascii=False)
    print('\nwrote results/phase59_rescore.json')


if __name__ == '__main__':
    main()
