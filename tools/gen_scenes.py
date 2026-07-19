"""
tools/gen_scenes.py

Phase7: シーンの多様性を増やす(過学習回避)ための追加configジェネレータ。
既存 configs/gen/*.json (荷物サイズ分布・コンテナ仕様・優先/ソフト比率) を参考に、
"validator/action/agent/camera/visualizer" のボイラープレートは既存configと同一のまま、
containers と item_stream.item_list だけを乱数(固定seed、再現可能)で生成する。

生成するシーン:
  - gen_fewitems_patternA.json      : 極端に荷物が少ない(6個)。足切りを容易に越えられるはずの
                                       sanity-check用シーン(越えられなければ根本バグ)。
  - gen_extrememany_patternA.json   : gen_manyitems_patternA(83個/1container)より更に荷物が多い
                                       (140個/2container)、サイズもより広く分布させ、量で押す
                                       ケースを増やす。
  - gen_sizevariety_patternC.json   : shelf+priority+2container+lookahead>1(パターンC相当)を
                                       同時に持つ複合シーン。荷物サイズは極小〜極大まで広く分布。

自前生成のため本番の隠しシーンとは分布が一致しないが、「得意な既存4シーンだけに過学習しない」
ための検証用途として使う。
"""
import copy
import json

import numpy as np

TEMPLATE_PATH = 'configs/gen/gen_2containers_priority.json'
CONTAINER_LWH = (2.0, 1.45, 1.61)


def _load_template() -> dict:
    with open(TEMPLATE_PATH) as f:
        d = json.load(f)
    return d[next(iter(d.keys()))]


def _container(index: int, is_prioritized: bool = False, require_shelf: bool = False) -> dict:
    length, width, height = CONTAINER_LWH
    return {
        'index': index, 'length': length, 'width': width, 'height': height,
        'thickness': 0.04, 'buffer': 0.0, 'cut_x': 0.44, 'cut_y': 0.4,
        'packed_items': [], 'require_shelf': require_shelf, 'is_prioritized': is_prioritized,
    }


def _gen_items(rng: np.random.Generator, n: int, size_lo=0.2, size_hi=0.9,
               prioritized_ratio=0.15, soft_ratio=0.2) -> list[dict]:
    items = []
    for i in range(n):
        length = float(rng.uniform(size_lo, size_hi))
        width = float(rng.uniform(size_lo, size_hi))
        height = float(rng.uniform(size_lo * 0.6, size_hi * 0.7))
        volume = length * width * height
        mass = float(np.clip(rng.normal(loc=volume * 25.0, scale=3.0), 1.0, 25.0))
        is_soft = bool(rng.random() < soft_ratio)
        item = {
            'index': i,
            'length': round(length, 3), 'width': round(width, 3), 'height': round(height, 3),
            'mass': round(mass, 2),
            'is_prioritized': bool(rng.random() < prioritized_ratio),
            'is_soft': is_soft,
            'lateralFriction': 0.4, 'rollingFriction': 0.01, 'spinningFriction': 0.01,
            'restitution': 0.2,
        }
        if is_soft:
            item.update({'contactStiffness': 2500, 'contactDamping': 800, 'linearDamping': 0.8})
        items.append(item)
    return items


def _build_scene(base: dict, containers: list[dict], items: list[dict], look_ahead: int) -> dict:
    scene = copy.deepcopy(base)
    scene['containers'] = {'spacing': 2.5, 'container_list': containers}
    scene['item_stream'] = {
        'item_list': items, 'look_ahead': look_ahead, 'max_space': 1, 'visible_pool': [],
    }
    scene['camera']['num_containers'] = len(containers)
    return scene


def main():
    base = _load_template()

    # --- 1. gen_fewitems: 極端に荷物が少ない(sanity-check) ---
    rng = np.random.default_rng(101)
    items = _gen_items(rng, n=6, size_lo=0.3, size_hi=0.6, prioritized_ratio=0.2, soft_ratio=0.15)
    containers = [_container(0)]
    scene = _build_scene(base, containers, items, look_ahead=1)
    with open('configs/gen/gen_fewitems_patternA.json', 'w') as f:
        json.dump({'000': scene}, f, ensure_ascii=False, indent=2)

    # --- 2. gen_extrememany: manyitemsより更に多い・サイズも広く分布・2container ---
    rng = np.random.default_rng(102)
    items = _gen_items(rng, n=140, size_lo=0.15, size_hi=0.85, prioritized_ratio=0.1, soft_ratio=0.2)
    containers = [_container(0), _container(1)]
    scene = _build_scene(base, containers, items, look_ahead=1)
    with open('configs/gen/gen_extrememany_patternA.json', 'w') as f:
        json.dump({'000': scene}, f, ensure_ascii=False, indent=2)

    # --- 3. gen_sizevariety_patternC: shelf+priority+2container+lookahead>1の複合 ---
    rng = np.random.default_rng(103)
    items = _gen_items(rng, n=55, size_lo=0.15, size_hi=0.95, prioritized_ratio=0.25, soft_ratio=0.3)
    containers = [_container(0, is_prioritized=True, require_shelf=True), _container(1)]
    scene = _build_scene(base, containers, items, look_ahead=5)
    with open('configs/gen/gen_sizevariety_patternC.json', 'w') as f:
        json.dump({'000': scene}, f, ensure_ascii=False, indent=2)

    print('generated: gen_fewitems_patternA.json / gen_extrememany_patternA.json / gen_sizevariety_patternC.json')


if __name__ == '__main__':
    main()
