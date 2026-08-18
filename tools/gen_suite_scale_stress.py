"""
tools/gen_suite_scale_stress.py

Phase53 ステップ2: 「本番とローカルの乖離はシーン規模(アイテム総数)の違いで
説明できるか」を検証するための、アイテム数を段階的に増やした検証シーンを生成する。

`tools/gen_suite.py` の生成規則(`_container`/`_gen_items`/`_build_scene`)を
そのまま再利用し、アイテム総数だけを 200/300/500 に増やす(既存26シーンの最大は
A08の140個)。比率は `configs/sample_config.json` の実測値相当
(soft_ratio=0.30, prioritized_ratio=0.08、Phase47実測)に固定する。

**ファイル名は `suite_` プレフィックスを使わない**(`scalestress_` にする)——
`bp_ab.sh`/`tools/measure_regime.py` が `configs/gen/suite_*.json` を glob で
拾う既存26シーンA/Bのベースラインに紛れ込まないようにするため(Phase46の
ratiostress_と同じ理由・同じ命名規約)。既存 `configs/gen/suite_*.json`(26シーン)
は一切変更しない。

生成するシーン(いずれも 2container・pattern A・prepackなし。
既存スイートの 'mid' 分布 size_lo=0.2, size_hi=0.9 を踏襲):
  - scalestress_200 : 200アイテム
  - scalestress_300 : 300アイテム
  - scalestress_500 : 500アイテム

実行方法(リポジトリルートで):
    PYTHONPATH=. python tools/gen_suite_scale_stress.py

読み取り専用ではない(configs/gen へ新規ファイルを書き込む)が、既存ファイルは
一切上書きしない。tools/scorer.py・既存26シーン・sample_config.json は変更しない。
"""
import json
import os
import sys

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from tools.gen_suite import _build_scene, _container, _gen_items, _load_template

OUT_DIR = 'configs/gen'
MANIFEST_PATH = 'tools/suite_manifest_scale_stress.json'
BASE_DIST = dict(size_lo=0.2, size_hi=0.9)  # 既存 'mid' 分布と同一
SOFT_RATIO = 0.30       # sample_config実測相当(Phase47: 26〜32%)
PRIORITIZED_RATIO = 0.08  # sample_config実測相当(Phase47: 5〜10%)


def _scene_specs():
    return [
        {'name': '200', 'n_items': 200},
        {'name': '300', 'n_items': 300},
        {'name': '500', 'n_items': 500},
    ]


def main():
    base = _load_template()
    specs = _scene_specs()
    manifest = {}

    for si, spec in enumerate(specs):
        seed = 6000 + si  # gen_suite.py(1000+si)/ratio_stress(5000+si)と衝突しない専用シード帯
        rng = np.random.default_rng(seed)
        n_items = spec['n_items']
        dist = dict(BASE_DIST, soft_ratio=SOFT_RATIO, prioritized_ratio=PRIORITIZED_RATIO)
        items = _gen_items(rng, n_items, start_index=0, **dist)
        containers = [
            _container(0, is_prioritized=False, require_shelf=False, packed_items=[]),
            _container(1, is_prioritized=False, require_shelf=False, packed_items=[]),
        ]
        scene = _build_scene(base, containers, items, pattern='A')

        fname = f"scalestress_{spec['name']}.json"
        path = os.path.join(OUT_DIR, fname)
        with open(path, 'w') as f:
            json.dump({'000': scene}, f, ensure_ascii=False, indent=2)

        n_prio_actual = sum(1 for it in items if it['is_prioritized'])
        n_soft_actual = sum(1 for it in items if it['is_soft'])
        label = f'{fname}::000'
        manifest[label] = {
            'path': path, 'n_items': n_items,
            'target_soft_ratio': SOFT_RATIO, 'target_prioritized_ratio': PRIORITIZED_RATIO,
            'actual_n_prioritized': n_prio_actual, 'actual_n_soft': n_soft_actual,
            'actual_prio_ratio': n_prio_actual / n_items, 'actual_soft_ratio': n_soft_actual / n_items,
        }
        print(f'  wrote {path}  (n_items={n_items}, '
              f'actual soft={n_soft_actual}/{n_items}({n_soft_actual/n_items:.1%}) '
              f'prio={n_prio_actual}/{n_items}({n_prio_actual/n_items:.1%}))')

    with open(MANIFEST_PATH, 'w') as f:
        json.dump(manifest, f, ensure_ascii=False, indent=2)
    print(f'\n{len(specs)} scenes generated (別スイート、既存26シーンには混ぜない). manifest -> {MANIFEST_PATH}')


if __name__ == '__main__':
    main()
