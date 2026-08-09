"""
tools/phase24_analyze.py

tools/phase24_corridor_audit.py の出力 JSON を集計し、
(a) 搬入経路の封鎖の分解表(帯域 / 障害物種別 / 経路上の障害物個数 / X レーン)と
空隙の位置(領域別シェア・軸別分布・連結成分の重心)を出力する。

    .venv/bin/python tools/phase24_analyze.py results/phase24_void.json
"""
import argparse
import json
import statistics


def short(label):
    return label.replace('suite_', '').replace('.json::000', '')


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('path')
    ap.add_argument('--fill-per-m3', type=float, default=None,
                    help='体積1m³あたりの fill_strict 換算[pt]。未指定なら 0.63/総容積 から算出')
    args = ap.parse_args()

    d = json.load(open(args.path))['scenes']
    n = len(d)
    tot_container = sum(s['container_volume'] for s in d.values())
    tot_placed = sum(s['placed_volume'] for s in d.values())
    tot_c = sum(s['c_volume'] for s in d.values())
    eff_geom = sum(s['eff_geom_volume'] for s in d.values())

    keys = ['in_container', 'empty', 'covered', 'supported', 'reach_strict', 'reach_optimistic',
            'a_strict', 'a_optimistic', 'a_band_floor', 'a_band_shelf', 'a_band_high',
            'a_item_only', 'a_shelf', 'a_xshift', 'a_x_only',
            'a_obs0', 'a_obs1', 'a_obs2', 'a_obs3plus']
    V = {k: sum(s['voxel'][k] for s in d.values()) for k in keys}

    # fill_strict 換算: Phase21 で fill_strict ≒ 0.63 × (置けた体積/総容積)
    fpm = args.fill_per_m3 if args.fill_per_m3 else 0.63 * 100.0 / tot_container

    print(f'=== phase24 corridor audit ({n} scenes) ===')
    print(f'container.volume 合計      : {tot_container:8.3f} m^3')
    print(f'voxel 実効幾何体積         : {eff_geom:8.3f} m^3 ({eff_geom/tot_container*100:.1f}%)')
    print(f'置けた体積(production)     : {tot_placed:8.3f} m^3 ({tot_placed/tot_container*100:.1f}%)')
    print(f'(c) 強い探索の追加配置     : {tot_c:8.3f} m^3 (fill換算 +{tot_c*fpm:.2f}pt)')
    print(f'(c)適用後の空き            : {V["empty"]:8.3f} m^3')
    print(f'1 m^3 = {fpm:.3f} pt (fill_strict 換算)')

    print('\n--- (a) 搬入経路の封鎖: 楽観近似 vs 厳密 ---')
    print(f'{"":34s}{"体積[m^3]":>12s}{"空きの%":>10s}{"fill換算pt":>12s}')
    for name, v in (('Phase22 楽観近似 (a)', V['a_optimistic']),
                    ('Phase24 厳密 (a)', V['a_strict'])):
        print(f'{name:34s}{v:12.3f}{v/V["empty"]*100:9.1f}%{v*fpm:12.2f}')
    print(f'{"到達可能(厳密)":34s}{V["reach_strict"]:12.3f}'
          f'{V["reach_strict"]/V["empty"]*100:9.1f}%{V["reach_strict"]*fpm:12.2f}')
    print(f'{"支持あり(=(a)+到達可能)":34s}{V["supported"]:12.3f}'
          f'{V["supported"]/V["empty"]*100:9.1f}%')

    a = V['a_strict']
    def row(name, v, denom=a):
        print(f'{name:34s}{v:12.3f}{v/denom*100:9.1f}%{v*fpm:12.2f}')

    print('\n--- (i) 帯域別 ((a) を排他分割) ---')
    print(f'{"":34s}{"体積[m^3]":>12s}{"(a)の%":>10s}{"fill換算pt":>12s}')
    row('床直置き帯 (is_resting/床)', V['a_band_floor'])
    row('棚直置き帯 (is_resting/棚上)', V['a_band_shelf'])
    row('浮上帯 (非直置き, START_Z上げ)', V['a_band_high'])
    rest = V['a_band_floor'] + V['a_band_shelf']
    print(f'  → 直置き帯(不感帯が実質0の帯)合計: {rest:.3f} m^3 = {rest/a*100:.1f}% of (a)')

    print('\n--- (ii) 塞いでいる障害物の種類 ((a) を排他分割) ---')
    print(f'{"":34s}{"体積[m^3]":>12s}{"(a)の%":>10s}{"fill換算pt":>12s}')
    row('既配置荷物のみ', V['a_item_only'])
    row('棚(構造物)が絡む', V['a_shelf'])
    print(f'{"(参考) 切り欠きでx迂回が必要":34s}{V["a_xshift"]:12.3f}'
          f'{V["a_xshift"]/a*100:9.1f}%')
    row('うち x掃引でだけ塞がれる(=切り欠き由来)', V['a_x_only'])

    print('\n--- (iii) 経路上の障害物の個数 ((a) を排他分割・少ない方優先) ---')
    print(f'{"":34s}{"体積[m^3]":>12s}{"(a)の%":>10s}{"fill換算pt":>12s}')
    row('1個だけ', V['a_obs1'])
    row('2個', V['a_obs2'])
    row('3個以上', V['a_obs3plus'])
    row('0個(=開口部の幅・切り欠きで不可)', V['a_obs0'])

    print('\n--- (iii-b) X レーン別 (0=左端/切り欠き側, 9=右端) ---')
    lane_a = [0.0] * 10; lane_e = [0.0] * 10; lane_o1 = [0.0] * 10
    for s in d.values():
        for i in range(10):
            lane_a[i] += s['lane_a'][i]; lane_e[i] += s['lane_empty'][i]
            lane_o1[i] += s['lane_obs1'][i]
    print(f'{"lane":>5s}{"空き":>10s}{"(a)":>10s}{"(a)/空き":>10s}{"1個で封鎖":>12s}')
    for i in range(10):
        r = lane_a[i] / lane_e[i] * 100 if lane_e[i] else 0.0
        print(f'{i:5d}{lane_e[i]:10.3f}{lane_a[i]:10.3f}{r:9.1f}%{lane_o1[i]:12.3f}')

    print('\n--- (iv) 空隙の位置: 領域別 ---')
    reg = {}; reg_a = {}
    for s in d.values():
        for k, v in s['region'].items():
            reg[k] = reg.get(k, 0.0) + v
        for k, v in s['region_a'].items():
            reg_a[k] = reg_a.get(k, 0.0) + v
    names = {'upper': '上部 (z>2/3)', 'back': '奥 (y>2/3, 上部以外)',
             'front': '手前 (y<1/3, 上部以外)', 'under_cut': '切り欠き下',
             'other': 'その他(中間)'}
    print(f'{"":26s}{"空き[m^3]":>12s}{"空きの%":>10s}{"(a)[m^3]":>12s}{"(a)の%":>10s}')
    for k in ('upper', 'back', 'front', 'under_cut', 'other'):
        print(f'{names[k]:26s}{reg[k]:12.3f}{reg[k]/V["empty"]*100:9.1f}%'
              f'{reg_a[k]:12.3f}{reg_a[k]/a*100:9.1f}%')

    print('\n--- (iv-b) 軸別の空き体積分布 (正規化座標を10分割) ---')
    for ax, lab in (('x', 'X (0=左/切り欠き側)'), ('y', 'Y (0=手前/開口部)'), ('z', 'Z (0=床)')):
        hs = [0.0] * 10; ha = [0.0] * 10
        for s in d.values():
            for i in range(10):
                hs[i] += s['axis_hist'][ax][i]; ha[i] += s['axis_hist_a'][ax][i]
        print(f'{lab}')
        print('  空き %: ' + ' '.join(f'{v/V["empty"]*100:5.1f}' for v in hs))
        print('  (a)  %: ' + ' '.join(f'{v/a*100:5.1f}' for v in ha))

    print('\n--- (iv-c) 空き連結成分 ---')
    comps = []
    for s in d.values():
        comps.extend(s['components'])
    comps.sort(key=lambda c: -c['volume'])
    vols = [c['volume'] for c in comps]
    print(f'成分数: {len(comps)}  中央値: {statistics.median(vols):.3f} m^3  '
          f'最大: {vols[0]:.3f} m^3')
    big = [c for c in comps if c['volume'] >= 1.0]
    print(f'1.0 m^3 以上の成分: {len(big)} 個 / 体積シェア {sum(c["volume"] for c in big)/sum(vols)*100:.1f}%')
    if big:
        print(f'  重心(体積加重, 正規化): x={sum(c["cx"]*c["volume"] for c in big)/sum(c["volume"] for c in big):.3f} '
              f'y={sum(c["cy"]*c["volume"] for c in big)/sum(c["volume"] for c in big):.3f} '
              f'z={sum(c["cz"]*c["volume"] for c in big)/sum(c["volume"] for c in big):.3f}')

    print('\n--- シーン別 (a) 上位 ---')
    rows = sorted(d.items(), key=lambda kv: -kv[1]['voxel']['a_strict'])
    print(f'{"scene":34s}{"a_strict":>10s}{"a_opt":>9s}{"床":>8s}{"棚上":>8s}{"浮上":>8s}'
          f'{"荷物のみ":>10s}{"1個封鎖":>9s}')
    for lab, s in rows:
        v = s['voxel']
        print(f'{short(lab):34s}{v["a_strict"]:10.3f}{v["a_optimistic"]:9.3f}'
              f'{v["a_band_floor"]:8.3f}{v["a_band_shelf"]:8.3f}{v["a_band_high"]:8.3f}'
              f'{v["a_item_only"]:10.3f}{v["a_obs1"]:9.3f}')


if __name__ == '__main__':
    main()
