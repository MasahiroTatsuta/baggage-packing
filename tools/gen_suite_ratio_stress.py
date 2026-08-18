"""
tools/gen_suite_ratio_stress.py

Phase46 ステップ2: 「plannerが本当に違反を回避できているのか、それとも構成上
違反しようがないのか」を切り分けるための検証用シーンを生成する。

`tools/gen_suite.py` の生成規則(`_container`/`_gen_items`/`_build_scene`/
`PATTERN_CFG`)をそのまま再利用し、ソフト貨物比率・優先手荷物比率だけを
段階的に変えた6シーンを作る。既存の `configs/gen/suite_*.json`(26シーン)は
**一切変更しない**(新規追加のみ)。

**ファイル名は `suite_` プレフィックスを使わない**(`ratiostress_` にする)——
`bp_ab.sh`/`tools/measure_regime.py` は `configs/gen/suite_*.json` を glob で
拾うため、同じプレフィックスにすると既存26シーンA/Bのベースラインに紛れ込み、
比較可能性が壊れる(指示の禁止事項)。本スクリプトが生成するシーンは
既存スイートとは**別スイート**として扱うこと。

生成するシーン(いずれも 1container・pattern A・40アイテム・prepackなし。
既存スイートの 'mid' 分布(size_lo=0.2, size_hi=0.9)を踏襲し、比率だけ変える):
  - ratiostress_soft30 / soft50 / soft70  : soft_ratio を 0.3/0.5/0.7 に上げる
                                            (prioritized_ratio は既定0.15のまま)
  - ratiostress_prio30 / prio50 / prio70  : prioritized_ratio を 0.3/0.5/0.7 に上げる
                                            (soft_ratio は既定0.2のまま)

実行方法(リポジトリルートで):
    PYTHONPATH=. python tools/gen_suite_ratio_stress.py

読み取り専用ではない(configs/gen へ新規ファイルを書き込む)が、既存ファイルは
一切上書きしない。tools/scorer.py は変更しない。
"""
import json
import os
import sys

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from tools.gen_suite import _build_scene, _container, _gen_items, _load_template

OUT_DIR = 'configs/gen'
MANIFEST_PATH = 'tools/suite_manifest_ratio_stress.json'
N_ITEMS = 40
BASE_DIST = dict(size_lo=0.2, size_hi=0.9)  # 既存 'mid' 分布と同一(比率だけ変える)


def _scene_specs():
    specs = []
    for level in (0.3, 0.5, 0.7):
        specs.append({
            'name': f'soft{int(level * 100)}', 'soft_ratio': level, 'prioritized_ratio': 0.15,
        })
    for level in (0.3, 0.5, 0.7):
        specs.append({
            'name': f'prio{int(level * 100)}', 'soft_ratio': 0.2, 'prioritized_ratio': level,
        })
    # Phase47 ステップ2: sample_config.json の実測比率(prio 5〜10%、soft 26〜32%)を
    # 踏まえ、prio を「下げる」方向も検証する(分母が小さいほど1件の違反が効きやすい)。
    # soft比率は sample_config に合わせて 0.3 に固定。
    # prio=5%・n=40 だと期待値2個で二項乱数のばらつきにより0個になる回もある(既定seedの
    # 5000+si=5006 が実際にそうだった)。プールに優先手荷物が2個以上存在するseedを
    # 明示的に選ぶ(seed_override。5014は事前探索でn_prio=2を確認済み)。
    specs.append({'name': 'lowprio5', 'soft_ratio': 0.3, 'prioritized_ratio': 0.05,
                  'seed_override': 5014})
    specs.append({'name': 'lowprio10', 'soft_ratio': 0.3, 'prioritized_ratio': 0.10})
    return specs


def main():
    base = _load_template()
    specs = _scene_specs()
    manifest = {}

    for si, spec in enumerate(specs):
        seed = spec.get('seed_override', 5000 + si)  # gen_suite.py(1000+si)と衝突しない専用シード帯
        rng = np.random.default_rng(seed)
        dist = dict(BASE_DIST, soft_ratio=spec['soft_ratio'],
                    prioritized_ratio=spec['prioritized_ratio'])
        items = _gen_items(rng, N_ITEMS, start_index=0, **dist)
        containers = [_container(0, is_prioritized=False, require_shelf=False, packed_items=[])]
        scene = _build_scene(base, containers, items, pattern='A')

        fname = f"ratiostress_{spec['name']}.json"
        path = os.path.join(OUT_DIR, fname)
        with open(path, 'w') as f:
            json.dump({'000': scene}, f, ensure_ascii=False, indent=2)

        n_prio_actual = sum(1 for it in items if it['is_prioritized'])
        n_soft_actual = sum(1 for it in items if it['is_soft'])
        label = f'{fname}::000'
        manifest[label] = {
            'path': path, 'n_items': N_ITEMS,
            'target_soft_ratio': spec['soft_ratio'], 'target_prioritized_ratio': spec['prioritized_ratio'],
            'actual_n_prioritized': n_prio_actual, 'actual_n_soft': n_soft_actual,
            'actual_prio_ratio': n_prio_actual / N_ITEMS, 'actual_soft_ratio': n_soft_actual / N_ITEMS,
        }
        print(f'  wrote {path}  (target soft={spec["soft_ratio"]:.0%} prio={spec["prioritized_ratio"]:.0%}, '
              f'actual soft={n_soft_actual}/{N_ITEMS}({n_soft_actual/N_ITEMS:.0%}) '
              f'prio={n_prio_actual}/{N_ITEMS}({n_prio_actual/N_ITEMS:.0%}))')

    with open(MANIFEST_PATH, 'w') as f:
        json.dump(manifest, f, ensure_ascii=False, indent=2)
    print(f'\n{len(specs)} scenes generated (別スイート、既存26シーンには混ぜない). manifest -> {MANIFEST_PATH}')


if __name__ == '__main__':
    main()
