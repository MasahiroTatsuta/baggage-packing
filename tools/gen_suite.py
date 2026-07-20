"""
tools/gen_suite.py

Phase10 タスク3: 本番分布に近い評価スイートを生成する。既存 configs/gen/*.json は一切
上書きせず、新規に `suite_*.json` を configs/gen/ へ追加する(既存configs改変禁止のため)。

網羅する軸:
  - コンテナ台数: 1 / 2
  - 棚: あり / なし (require_shelf)
  - 初期状態: 空 / 積付済み(packed_items入り = 現状未カバーの重大な盲点)
  - 優先コンテナ: あり / なし (is_prioritized)
  - パターン: A(optimize=True, look_ahead=1) / B(optimize=False, look_ahead>1) /
             C(optimize=True, look_ahead>1)
  - 荷物数: 6 / 40 / 80 / 140
  - 寸法・質量分布・ソフト/優先比率の変化

「積付済み初期状態」は、床面に非重複グリッドで箱を並べた packed_items を用意して表現する
(env.build() が物理で着地・静定させるため、多少の誤差は吸収される)。pre-packed の index は
ストリーム item と衝突しないよう 1000 番台の別空間を使う(optimize が並べ替えるのはストリーム
item の index のみで、pre-packed は対象外)。

各シーンには条件タグを付け、tools/suite_manifest.json に書き出す(層別集計・ワースト特定用)。

src/ と agents/mysolver/ は一切変更しない。
実行: PYTHONPATH=. .venv/bin/python tools/gen_suite.py
"""
import copy
import json
import os

import numpy as np

TEMPLATE_PATH = 'configs/gen/gen_2containers_priority.json'
OUT_DIR = 'configs/gen'
MANIFEST_PATH = 'tools/suite_manifest.json'
SPACING = 2.5
CONTAINER_LWH = (2.0, 1.45, 1.61)
THICKNESS = 0.04
CUT_X = 0.44


def _load_template() -> dict:
    with open(TEMPLATE_PATH) as f:
        d = json.load(f)
    return d[next(iter(d.keys()))]


def _container(index: int, is_prioritized: bool = False, require_shelf: bool = False,
               packed_items: list | None = None) -> dict:
    length, width, height = CONTAINER_LWH
    return {
        'index': index, 'length': length, 'width': width, 'height': height,
        'thickness': THICKNESS, 'buffer': 0.0, 'cut_x': CUT_X, 'cut_y': 0.4,
        'packed_items': packed_items or [], 'require_shelf': require_shelf,
        'is_prioritized': is_prioritized,
    }


def _gen_items(rng, n, start_index=0, size_lo=0.2, size_hi=0.9,
               height_lo_scale=0.6, height_hi_scale=0.7,
               prioritized_ratio=0.15, soft_ratio=0.2, mass_scale=25.0):
    """ストリーム用の手荷物リスト。start_index からの連番 index を振る。"""
    items = []
    for i in range(n):
        length = float(rng.uniform(size_lo, size_hi))
        width = float(rng.uniform(size_lo, size_hi))
        height = float(rng.uniform(size_lo * height_lo_scale, size_hi * height_hi_scale))
        volume = length * width * height
        mass = float(np.clip(rng.normal(loc=volume * mass_scale, scale=3.0), 1.0, 25.0))
        is_soft = bool(rng.random() < soft_ratio)
        item = {
            'index': start_index + i,
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


def _prepack_floor(rng, container_index: int, n: int, start_index: int,
                   fp=0.26, h=0.26, max_layers=2, soft_ratio=0.0, prioritized_ratio=0.0):
    """
    床面の非重複グリッドに n 個の箱を並べた packed_items を返す(不足なら段積みで補う)。
    pos は world 座標(x は container_index*SPACING だけシフト、y はそのまま、
    z は床上面 thickness からの段数で決まる)。orn は単位クオータニオン。
    グリッド間隔は footprint+余裕を確保し、コンテナ壁の内側・cut corner を避けて収める。
    """
    length, width, _ = CONTAINER_LWH
    offset_x = container_index * SPACING
    floor_top = THICKNESS  # geometry.py と同じく床上面 z ≈ thickness

    margin = 0.05
    step = fp + 0.08
    x_lo = -length / 2.0 + THICKNESS + CUT_X + fp / 2.0 + margin   # cut corner 側を避ける
    x_hi = length / 2.0 - THICKNESS - fp / 2.0 - margin
    # 手前(搬入口)側は空けておくのが現実的な「途中まで積付」状態。奥半分(y>=0)だけを占有し、
    # 手前半分を新規荷物の搬入経路・着地スペースとして残す。
    y_lo = 0.0 + fp / 2.0
    y_hi = width / 2.0 - THICKNESS - fp / 2.0 - margin

    xs = list(np.arange(x_lo, x_hi + 1e-9, step))
    ys = list(np.arange(y_lo, y_hi + 1e-9, step))
    # 現実の「途中まで積付済み」は奥(+width/2)から手前へ埋まり、搬入口側(手前=-width/2)が
    # 空いている状態。奥の行から先に占有し、手前の搬入経路を空けておく(=リアルな初期状態)。
    grid = [(x, y) for y in sorted(ys, reverse=True) for x in xs]
    capacity = len(grid) * max_layers
    n = min(n, capacity)

    packed = []
    for k in range(n):
        layer = k // len(grid)
        lx, ly = grid[k % len(grid)]
        # 段積み分は着地の反発を避けるため僅かに隙間(0.005)を空ける
        cz = floor_top + h / 2.0 + layer * (h + 0.005)
        is_soft = bool(rng.random() < soft_ratio)
        item = {
            'index': start_index + k,
            'length': round(fp, 3), 'width': round(fp, 3), 'height': round(h, 3),
            'mass': round(float(np.clip(rng.normal(6.0, 1.5), 1.0, 15.0)), 2),
            'is_prioritized': bool(rng.random() < prioritized_ratio),
            'is_soft': is_soft,
            'lateralFriction': 0.4, 'rollingFriction': 0.01, 'spinningFriction': 0.01,
            'restitution': 0.2,
            'belongs_to': container_index,
            'pos': [round(offset_x + lx, 4), round(ly, 4), round(cz, 4)],
            'orn': [0.0, 0.0, 0.0, 1.0],
        }
        if is_soft:
            item.update({'contactStiffness': 2500, 'contactDamping': 800, 'linearDamping': 0.8})
        packed.append(item)
    return packed


PATTERN_CFG = {
    'A': {'optimize': True, 'look_ahead': 1, 'max_space': 1},
    'B': {'optimize': False, 'look_ahead': 10, 'max_space': 1},
    'C': {'optimize': True, 'look_ahead': 5, 'max_space': 1},
}


def _build_scene(base, containers, items, pattern):
    pc = PATTERN_CFG[pattern]
    scene = copy.deepcopy(base)
    scene['containers'] = {'spacing': SPACING, 'container_list': containers}
    scene['item_stream'] = {
        'item_list': items, 'look_ahead': pc['look_ahead'],
        'max_space': pc['max_space'], 'visible_pool': [],
    }
    scene['agent'] = dict(scene['agent'])
    scene['agent']['optimize'] = pc['optimize']
    scene['camera']['num_containers'] = len(containers)
    return scene


# =====================================================================
# スイート定義。各 spec は (name, pattern, builder(rng)->(containers, items), tags)
# tags は層別集計用。
# =====================================================================
def _scene_specs():
    specs = []

    def add(name, pattern, n_items, n_cont, shelf, prio, prepack, dist, tags_extra=None):
        specs.append({
            'name': name, 'pattern': pattern, 'n_items': n_items, 'n_cont': n_cont,
            'shelf': shelf, 'prio': prio, 'prepack': prepack, 'dist': dist,
            'tags_extra': tags_extra or {},
        })

    # ---- Pattern A (optimize, look_ahead=1) : empty ----
    add('A01_1c_40_plain', 'A', 40, 1, False, False, 0, 'mid')
    add('A02_1c_80_plain', 'A', 80, 1, False, False, 0, 'mid')
    add('A03_1c_40_shelf', 'A', 40, 1, True, False, 0, 'mid')
    add('A04_2c_80_noprio', 'A', 80, 2, False, False, 0, 'mid')
    add('A05_2c_80_prio', 'A', 80, 2, False, True, 0, 'mid')
    add('A06_1c_40_small', 'A', 40, 1, False, False, 0, 'small')     # count-heavy
    add('A07_1c_40_bulky', 'A', 40, 1, False, False, 0, 'bulky')     # large items
    add('A08_2c_140_extreme', 'A', 140, 2, False, False, 0, 'wide')

    # ---- Pattern B (no optimize, look_ahead=10) : empty ----
    add('B01_1c_40_plain', 'B', 40, 1, False, False, 0, 'mid')
    add('B02_1c_40_shelf', 'B', 40, 1, True, False, 0, 'mid')
    add('B03_2c_80_prio', 'B', 80, 2, False, True, 0, 'mid')
    add('B04_2c_80_noprio', 'B', 80, 2, False, False, 0, 'mid')

    # ---- Pattern C (optimize, look_ahead=5) : empty ----
    add('C01_1c_40_shelf', 'C', 40, 1, True, False, 0, 'mid')
    add('C02_2c_55_shelfprio', 'C', 55, 2, True, True, 0, 'wide')
    add('C03_2c_80_prio', 'C', 80, 2, False, True, 0, 'mid')

    # ---- Pre-packed initial state (blind spot) ----
    add('P01_A_1c_pre6', 'A', 34, 1, False, False, 6, 'mid')
    add('P02_A_1c_pre10', 'A', 40, 1, False, False, 10, 'mid')
    add('P03_A_2c_pre8_prio', 'A', 60, 2, False, True, 8, 'mid')
    add('P04_B_1c_pre8_shelf', 'B', 34, 1, True, False, 8, 'mid')
    add('P05_C_2c_pre8_shelfprio', 'C', 50, 2, True, True, 8, 'wide')
    add('P06_A_1c_pre12_dense', 'A', 30, 1, False, False, 12, 'mid')

    # ---- Distribution stress ----
    add('D01_A_1c_40_softheavy', 'A', 40, 1, False, False, 0, 'softheavy')
    add('D02_A_1c_40_prioheavy_nocont', 'A', 40, 1, False, False, 0, 'prioheavy')
    add('D03_A_2c_60_prioheavy_cont', 'A', 60, 2, False, True, 0, 'prioheavy')
    add('D04_A_1c_40_flat', 'A', 40, 1, False, False, 0, 'flat')
    add('D05_A_1c_40_tall', 'A', 40, 1, False, False, 0, 'tall')

    return specs


DIST_PARAMS = {
    'mid':       dict(size_lo=0.2, size_hi=0.9, soft_ratio=0.2, prioritized_ratio=0.15),
    'small':     dict(size_lo=0.15, size_hi=0.45, soft_ratio=0.2, prioritized_ratio=0.15),
    'bulky':     dict(size_lo=0.5, size_hi=1.0, soft_ratio=0.15, prioritized_ratio=0.15),
    'wide':      dict(size_lo=0.15, size_hi=0.95, soft_ratio=0.25, prioritized_ratio=0.2),
    'softheavy': dict(size_lo=0.2, size_hi=0.85, soft_ratio=0.5, prioritized_ratio=0.1),
    'prioheavy': dict(size_lo=0.2, size_hi=0.85, soft_ratio=0.15, prioritized_ratio=0.4),
    'flat':      dict(size_lo=0.3, size_hi=0.95, soft_ratio=0.15, prioritized_ratio=0.15,
                      height_lo_scale=0.25, height_hi_scale=0.35),
    'tall':      dict(size_lo=0.2, size_hi=0.6, soft_ratio=0.15, prioritized_ratio=0.15,
                      height_lo_scale=1.1, height_hi_scale=1.3),
}


def main():
    base = _load_template()
    specs = _scene_specs()
    manifest = {}

    for si, spec in enumerate(specs):
        rng = np.random.default_rng(1000 + si)
        dist = dict(DIST_PARAMS[spec['dist']])

        # ストリーム item(index 0..n-1)
        items = _gen_items(rng, spec['n_items'], start_index=0, **dist)

        # pre-packed(あれば。index は 1000 番台)
        containers = []
        prepack_total = 0
        for ci in range(spec['n_cont']):
            packed = []
            if spec['prepack'] > 0:
                per_c = spec['prepack'] // spec['n_cont']
                if per_c > 0:
                    packed = _prepack_floor(rng, ci, per_c, start_index=1000 + ci * 100,
                                            soft_ratio=0.1, prioritized_ratio=0.0)
                    prepack_total += len(packed)
            is_prio = spec['prio'] and (ci == 0)   # 優先コンテナは先頭のみ
            containers.append(_container(ci, is_prioritized=is_prio,
                                         require_shelf=spec['shelf'], packed_items=packed))

        scene = _build_scene(base, containers, items, spec['pattern'])
        fname = f"suite_{spec['name']}.json"
        path = os.path.join(OUT_DIR, fname)
        with open(path, 'w') as f:
            json.dump({'000': scene}, f, ensure_ascii=False, indent=2)

        label = f'{fname}::000'
        manifest[label] = {
            'path': path,
            'pattern': spec['pattern'],
            'n_stream_items': spec['n_items'],
            'n_prepacked': prepack_total,
            'total_items': spec['n_items'] + prepack_total,
            'n_containers': spec['n_cont'],
            'shelf': spec['shelf'],
            'prio_container': spec['prio'],
            'initial_state': 'prepacked' if prepack_total > 0 else 'empty',
            'dist': spec['dist'],
        }
        print(f'  wrote {path}  (pattern {spec["pattern"]}, stream {spec["n_items"]}, '
              f'prepacked {prepack_total}, {spec["n_cont"]}c, shelf={spec["shelf"]}, prio={spec["prio"]})')

    with open(MANIFEST_PATH, 'w') as f:
        json.dump(manifest, f, ensure_ascii=False, indent=2)
    print(f'\n{len(specs)} scenes generated. manifest -> {MANIFEST_PATH}')


if __name__ == '__main__':
    main()
