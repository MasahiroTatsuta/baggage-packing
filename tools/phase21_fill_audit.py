"""
tools/phase21_fill_audit.py

Phase21 ターゲット1: fill計上漏れの原因分解。

背景:
    Phase20 の実測で「配置に成功した体積のうち fill_strict に計上されるのは 0.638」、
    つまり **置けた体積の36%が計上されていない**ことが分かった。これを 0.85 まで
    上げられれば fill_strict は 23.74 -> 約31.6 に相当する。本ツールはその36%が
    「どの面をどれだけ突き抜けたせいで落ちているのか」を体積ベースで分解する。

判定式(src/ground_handling/evaluator.py calculate_fill_rate と同一):
    荷物の8角点それぞれについて、コンテナの各面 f で
        dot_f = n_vec_f ・ (corner - point_f)
    を計算し、**どれか1つでも** dot_f > inclusion_margin なら「内包していない」として
    fill から除外される。したがって除外の原因は「どの面の、どの角が、どれだけ超えたか」で
    完全に特定できる。

コンテナの面(実測で確認、7面):
    face0: n=(0,0,-1)   内床面        -> dot = -(corner.z - thickness)
    face1: n=(+1,0,0)   +X側壁
    face2: n=(0,0,+1)   天井
    face3: n=(-1,0,0)   -X側壁
    face4: 斜め         開口部脇の切り欠き斜面
    face5: n=(0,-1,0)   手前(開口部)面
    face6: n=(0,+1,0)   奥面

分類の考え方:
    面ごとの「超過量 excess_f = max_corner(dot_f) - margin」を求め、
    **最大の超過を出した面**を primary としてボリューム按分する(相互排他)。
    そのうえで、傾き角・床からの高さ・目標位置からのずれを属性として併記し、
    「なぜその面を突き抜けたのか」(構造的な床接地 / 沈降時の傾き / 押されて移動)を
    切り分けられるようにする。件数ではなく**体積シェア**で集計する(fillは体積ベース)。

使い方:
    PYTHONPATH=. .venv/bin/python tools/phase21_fill_audit.py \
        --config-path 'configs/gen/suite_*.json' --out results/phase21_audit.json

src/ と agents/ は一切変更しない(読み取りのみ)。
"""
import argparse
import glob
import importlib
import json
import math
import os
import sys
import time
import traceback
from contextlib import redirect_stdout

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

import numpy as np

from src.ground_handling.env import GroundHandlingEnv

STRICT_MARGIN = -0.005
LOOSE_MARGIN = 0.01

FACE_NAMES = ['floor', 'wall_+X', 'ceiling', 'wall_-X', 'cut_corner', 'front_open', 'back']

# 「床に接地している」とみなす、内床面からの底面高さの許容[m]
FLOOR_CONTACT_TOL = 0.005
# 「傾いている」とみなす軸ずれ角[deg]。物理沈降で数度は普通に生じるため、
# 明確に姿勢が崩れたものだけを拾う水準に置く。
TILT_DEG_THRESHOLD = 5.0
# 「押されて動いた」とみなす水平移動量[m]
DISPLACE_XY_THRESHOLD = 0.05


def _face_name(i):
    return FACE_NAMES[i] if i < len(FACE_NAMES) else f'face{i}'


def _corners(pos, orn, lwh, client):
    rot = np.array(client.getMatrixFromQuaternion(orn)).reshape(3, 3)
    hl, hw, hh = lwh[0] / 2.0, lwh[1] / 2.0, lwh[2] / 2.0
    local = np.array([[sx * hl, sy * hw, sz * hh]
                      for sx in (1, -1) for sy in (1, -1) for sz in (1, -1)])
    return local @ rot.T + np.array(pos), rot


def _tilt_deg(rot):
    """各ローカル軸が最も近いワールド軸から何度ずれているかの最大値。"""
    worst = 0.0
    for i in range(3):
        c = float(np.max(np.abs(rot[:, i])))
        worst = max(worst, math.degrees(math.acos(min(1.0, c))))
    return worst


def audit_scene(task_config, module_path, label):
    mod_prefix = '.'.join(module_path.rstrip('/').split('/'))
    agent_mod = importlib.import_module(mod_prefix + '.agent')

    env = GroundHandlingEnv(config=task_config, verbose=False, render_mode=None)
    try:
        env.reset_settings()
        agent = agent_mod.Agent(module_path)
        agent.get_init_states(env.get_init_states())
        prepacked = set()
        for c in env.container_manager.containers:
            for it in c.packed_items:
                prepacked.add(int(it.index))
        if env.optimize:
            order = list(agent.optimize(env.get_info_for_optimization()))
            env.set_item_order(order)
        env.reset_item_stream()
        obs, info = env.reset(seed=42)

        targets = {}
        terminated = truncated = False
        while not terminated and not truncated:
            pool = obs.get('pool_list', [])
            action = agent.policy(obs)
            idx = action['item_idx']
            if idx < len(pool):
                cidx = int(action['container_idx'])
                cont = env.container_manager.containers[cidx]
                ox = float(cont.center[0])
                lp = action['place_pos']
                targets[int(pool[idx]['index'])] = (float(lp[0]) + ox, float(lp[1]), float(lp[2]))
            obs, reward, terminated, truncated, info = env.step(action)

        rows = []
        for cont in env.container_manager.containers:
            n_vecs = np.array(cont.n_vecs)
            points = np.array(cont.points)
            thickness = float(cont.thickness)
            shelf_top = (float(cont.height) / 2.0 + thickness / 2.0 + float(cont.buffer)
                         + thickness / 2.0) if cont.require_shelf else None
            for it in cont.packed_items:
                pos, orn = it.get_pose(env.client)
                if pos is None or orn is None:
                    continue
                lwh = (float(it.length), float(it.width), float(it.height))
                corners, rot = _corners(pos, orn, lwh, env.client)
                # (8, F) の dot 行列 -> 面ごとの最大値
                diff = corners[:, None, :] - points[None, :, :]
                dots = np.einsum('cfk,fk->cf', diff, n_vecs)
                per_face_max = dots.max(axis=0)

                vol = lwh[0] * lwh[1] * lwh[2]
                bottom_z = float(corners[:, 2].min())
                tgt = targets.get(int(it.index))
                disp_xy = (math.hypot(pos[0] - tgt[0], pos[1] - tgt[1]) if tgt else None)
                disp_z = (float(pos[2] - tgt[2]) if tgt else None)

                ext = corners.max(axis=0) - corners.min(axis=0)   # 実姿勢のAABB寸法
                rec = {
                    'index': int(it.index),
                    'container': int(cont.index),
                    'volume': vol,
                    'aabb_ext': [float(x) for x in ext],
                    'footprint_xy': float(ext[0] * ext[1]),
                    'top_above_floor': float(corners[:, 2].max()) - thickness,
                    'floor_area': float((cont.length - 2 * thickness) * (cont.width - 2 * thickness)),
                    'prepacked': int(it.index) in prepacked,
                    'per_face_max': [float(x) for x in per_face_max],
                    'bottom_above_floor': bottom_z - thickness,
                    'tilt_deg': _tilt_deg(rot),
                    'disp_xy': disp_xy,
                    'disp_z': disp_z,
                    'on_shelf': (shelf_top is not None
                                 and abs(bottom_z - shelf_top) <= 0.03),
                    'is_soft': bool(getattr(it, 'is_soft', False)),
                }
                for tag, margin in (('strict', STRICT_MARGIN), ('loose', LOOSE_MARGIN)):
                    excess = per_face_max - margin
                    violated = [i for i in range(len(excess)) if excess[i] > 0]
                    rec[f'{tag}_excluded'] = bool(violated)
                    rec[f'{tag}_violated'] = [_face_name(i) for i in violated]
                    if violated:
                        pf = max(violated, key=lambda i: excess[i])
                        rec[f'{tag}_primary'] = _face_name(pf)
                        rec[f'{tag}_excess'] = float(excess[pf])
                    else:
                        rec[f'{tag}_primary'] = None
                        rec[f'{tag}_excess'] = 0.0
                rows.append(rec)

        return {'label': label, 'container_volume': sum(float(c.volume) for c in env.container_manager.containers),
                'items': rows}
    except Exception:
        return {'label': label, 'error': traceback.format_exc().splitlines()[-1], 'items': []}
    finally:
        try:
            env.close()
        except Exception:
            pass


def classify(rec, tag):
    """primary face と属性から、原因ラベルを1つ返す(相互排他)。"""
    if not rec[f'{tag}_excluded']:
        return 'counted'
    pf = rec[f'{tag}_primary']
    tilt = rec['tilt_deg']
    if pf == 'floor':
        # 床面を「超える」= 底面が内床面より下、または一致。軸平行のまま接地していれば
        # dot=0 となり strict(-0.005) では構造的に必ず落ちる。
        if tilt < TILT_DEG_THRESHOLD and abs(rec['bottom_above_floor']) <= FLOOR_CONTACT_TOL:
            return '(a) 床直置き(構造的)'
        if tilt >= TILT_DEG_THRESHOLD:
            return '(b) 傾き -> 床面'
        return '(e) その他(床面)'
    if tilt >= TILT_DEG_THRESHOLD:
        return f'(b) 傾き -> {pf}'
    if rec['disp_xy'] is not None and rec['disp_xy'] >= DISPLACE_XY_THRESHOLD:
        return f'(d) 押されて移動 -> {pf}'
    return f'(e) その他 -> {pf}'


def summarize(scenes, tag):
    tot_placed = tot_counted = 0.0
    by_cause = {}
    by_face = {}
    floor_layer_vol = 0.0
    for s in scenes.values():
        for r in s['items']:
            v = r['volume']
            tot_placed += v
            if not r[f'{tag}_excluded']:
                tot_counted += v
            if r[f'{tag}_excluded']:
                by_cause[classify(r, tag)] = by_cause.get(classify(r, tag), 0.0) + v
            if r[f'{tag}_excluded']:
                by_face[r[f'{tag}_primary']] = by_face.get(r[f'{tag}_primary'], 0.0) + v
            if abs(r['bottom_above_floor']) <= FLOOR_CONTACT_TOL:
                floor_layer_vol += v
    return {
        'placed_volume': tot_placed,
        'counted_volume': tot_counted,
        'counted_ratio': tot_counted / tot_placed if tot_placed else float('nan'),
        'missed_volume': tot_placed - tot_counted,
        'by_cause': dict(sorted(by_cause.items(), key=lambda kv: -kv[1])),
        'by_primary_face': dict(sorted(by_face.items(), key=lambda kv: -kv[1])),
        'floor_layer_volume': floor_layer_vol,
        'floor_layer_share_of_placed': floor_layer_vol / tot_placed if tot_placed else float('nan'),
    }


def _print_summary(name, s):
    print(f'\n===== {name} =====')
    print(f'  置けた体積        : {s["placed_volume"]:.3f} m^3')
    print(f'  計上された体積    : {s["counted_volume"]:.3f} m^3  (計上率 {s["counted_ratio"]:.3f})')
    print(f'  計上漏れ体積      : {s["missed_volume"]:.3f} m^3')
    print(f'  床接地層の体積    : {s["floor_layer_volume"]:.3f} m^3 '
          f'(置けた体積の {s["floor_layer_share_of_placed"]*100:.1f}%)')
    miss = s['missed_volume'] or 1.0
    print(f'  --- 原因別 体積シェア(計上漏れ体積に対する%) ---')
    for k, v in s['by_cause'].items():
        print(f'    {k:34s} {v:8.3f} m^3  {v/miss*100:6.1f}%  '
              f'(置けた体積の {v/s["placed_volume"]*100:5.1f}%)')
    print(f'  --- 突き抜けた面別 体積シェア ---')
    for k, v in s['by_primary_face'].items():
        print(f'    {str(k):34s} {v:8.3f} m^3  {v/miss*100:6.1f}%')


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--config-path', nargs='+', required=True)
    ap.add_argument('--module-path', default='agents/mysolver/')
    ap.add_argument('--optimize-budget', type=float, default=None)
    ap.add_argument('--out', required=True)
    args = ap.parse_args()

    if args.optimize_budget is not None:
        os.environ['MYSOLVER_OPTIMIZE_BUDGET'] = str(args.optimize_budget)

    paths = []
    for pat in args.config_path:
        m = sorted(glob.glob(pat))
        paths.extend(m if m else [pat])

    scenes = {}
    for cp in paths:
        with open(cp) as f:
            cfg = json.load(f)
        for task_id, task_config in cfg.items():
            label = f'{os.path.basename(cp)}::{task_id}'
            t0 = time.time()
            # evaluator/env の大量の print を抑止する
            with open(os.devnull, 'w') as devnull, redirect_stdout(devnull):
                res = audit_scene(task_config, args.module_path, label)
            scenes[label] = res
            n_ex = sum(1 for r in res['items'] if r.get('strict_excluded'))
            print(f'[{label}] items={len(res["items"])} strict除外={n_ex} '
                  f'({time.time()-t0:.1f}s){" ERROR:"+res["error"] if "error" in res else ""}', flush=True)

    out = {'scenes': scenes,
           'strict': summarize(scenes, 'strict'),
           'loose': summarize(scenes, 'loose')}
    with open(args.out, 'w') as f:
        json.dump(out, f, indent=1)

    _print_summary('STRICT (inclusion_margin = -0.005)', out['strict'])
    _print_summary('LOOSE  (inclusion_margin = +0.01)', out['loose'])
    print(f'\nwrote {args.out}')


if __name__ == '__main__':
    main()
