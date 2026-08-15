"""
tools/phase35_replica.py

Phase35 ステップ0: **複製評価器**が作れるか、作れたとして本物とどれだけ一致するかを検証する。

狙い(Phase34 の帰結):
  Phase34 は「ALNS が採用した手は定義上すべて代理目的関数を改善しているのに、実fillの
  改善は 7シーン中4シーン、代理gainと実fill差の順位相関は ρ=−0.321(負)」を実測した。
  代理 = 実 + ノイズ なら、代理gainが大きい手を選ぶことは **ノイズが正に大きい手を選ぶこと**
  であり、現在の代理関数を山登りするあらゆる手法が失敗する。
  そこで「代理を良くする」のではなく「**代理を信じないための仕組み**」——受理判定を
  本物と同じ pybullet で行う複製評価器——が作れるかを先に確かめる。

本ツールがやること:
  (0-1) エージェントが `get_init_states` で受け取る情報だけからコンテナを再構築し、
        再構築した幾何(n_vecs/points/volume/center)が渡された値と一致するかを検査する。
  (0-2) 記録済みの 21シーン・130候補(results/phase30_cand_eval.json、本物の env で評価済み)を
        複製評価器に流し、fill_strict の差と **シーン内順位の Spearman** を測る。

**新しい探索は一切行わない**(記録済みの order を流すだけ)。

実行:
    MYSOLVER_UNITS_PER_SEC=1.55e7 PYTHONPATH=. .venv/bin/python tools/phase35_replica.py \
        --cand results/phase29_cand_g1.json results/phase29_cand_g2.json \
        --truth results/phase30_cand_eval.json --out results/phase35_replica.json
"""
import argparse
import io
import json
import math
import os
import sys
import time
from contextlib import redirect_stdout

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

import numpy as np
import pybullet as p
from pybullet_utils import bullet_client
import pybullet_data

from src.ground_handling.containers import MultiContainerManager
from src.ground_handling.evaluator import Evaluator
from src.ground_handling.items import Item
from src.ground_handling.validator import PlacementValidator
from src.ground_handling.env import GroundHandlingEnv

from agents.mysolver import planner

# ---------------------------------------------------------------------------
# エージェントが get_init_states では受け取れない設定値。
# **ここが本フェーズ最大の前提**: これらは config['validator'] にあり、エージェントには
# 渡らない。したがって複製評価器はこれらを「決め打ち」するしかない。
# 値は agents/mysolver/geometry.py が既に前提にしているものと同一で、
# 26シーンの config 全件で同じ値だった(configs/gen/suite_*.json を全数確認)。
# 本番の値が違えば複製評価器はその分だけ狂う —— これは Phase12/13/27 から続く
# 「本番の inclusion_margin レジームが未確定」という既知の未解決課題そのものである。
# ---------------------------------------------------------------------------
ASSUMED_VALIDATOR = {
    'inclusion_margin': -0.005,
    'start_z': 0.08,
    'safety_margin': 0.015,
    'ceiling_margin': 0.018,
    'displacement_threshold': 0.3,
    'angle_displacement_threshold': 45,
    'settle_wait_step': 300,
}
ASSUMED_MAX_SPACE = 1     # ItemStreamManager.max_space(補充タイミング)
STRICT_MARGIN = -0.005
LOOSE_MARGIN = 0.01


class ReplicaEvaluator:
    """`get_init_states` の container_list だけからコンテナを再構築し、
    本物と同じ src.ground_handling のクラスで順序を評価する複製評価器。"""

    def __init__(self, container_list: list[dict], lookahead_k: int,
                 validator_config: dict | None = None):
        self.given = container_list
        self.lookahead_k = max(1, int(lookahead_k or 1))
        self.vcfg = dict(validator_config or ASSUMED_VALIDATOR)
        self.client = bullet_client.BulletClient(connection_mode=p.DIRECT)
        self.client.setAdditionalSearchPath(pybullet_data.getDataPath())
        self.cm = None

    def _containers_config(self) -> dict:
        """container_list(観測)から MultiContainerManager 用の config を復元する。

        復元できる根拠:
          offset_x = center[0]                 (local_to_global が x にだけ offset を足すため)
          buffer   = center[2] - height/2      (center = (0,0,height/2+buffer) を平行移動した点)
          require_shelf = 観測の 'shelf'
        残りは観測にそのまま入っている。
        """
        cl = []
        for c in self.given:
            cl.append({
                'index': int(c['index']),
                'thickness': float(c['thickness']),
                'length': float(c['length']),
                'width': float(c['width']),
                'height': float(c['height']),
                'cut_x': float(c['cut_x']),
                'cut_y': float(c['cut_y']),
                'require_shelf': bool(c['shelf']),
                'is_prioritized': bool(c['is_prioritized']),
                'buffer': float(c['center'][2]) - float(c['height']) / 2.0,
                'packed_items': [dict(it) for it in c.get('packed_items', [])],
            })
        offs = [float(c['center'][0]) for c in self.given]
        spacing = (offs[1] - offs[0]) if len(offs) > 1 else 0.0
        return {'spacing': spacing, 'container_list': cl}

    def build(self) -> dict:
        """物理環境を構築し、再構築した幾何が観測と一致するかの検査結果を返す。"""
        self.client.resetSimulation()
        self.client.setPhysicsEngineParameter(deterministicOverlappingPairs=1)
        self.client.setGravity(0, 0, -9.8)
        self.client.loadURDF('plane.urdf')
        cfg = self._containers_config()
        self.cm = MultiContainerManager(client=self.client, config=cfg)
        self.cm.build()
        self._pin_prepacked()
        self.validator = PlacementValidator(client=self.client, config=self.vcfg)

        # --- (0-1) 再構築した幾何が観測と一致するか ---
        diffs = {'volume': 0.0, 'center': 0.0, 'n_vecs': 0.0, 'points': 0.0}
        for got, want in zip(self.cm.containers, self.given):
            diffs['volume'] = max(diffs['volume'], abs(got.volume - float(want['volume'])))
            diffs['center'] = max(diffs['center'],
                                  float(np.max(np.abs(np.array(got.center) - np.array(want['center'])))))
            diffs['n_vecs'] = max(diffs['n_vecs'],
                                  float(np.max(np.abs(np.array(got.n_vecs) - np.array(want['n_vecs'])))))
            diffs['points'] = max(diffs['points'],
                                  float(np.max(np.abs(np.array(got.points) - np.array(want['points'])))))
        return diffs

    def _pin_prepacked(self):
        """既積み荷物を **観測された姿勢そのもの** に固定し直す。

        `MultiContainerManager.build()` は既積み荷物をスポーンしたあと最大480ステップの
        物理演算で「落ち着かせる」。本物の env は *未定着の初期姿勢* から落ち着かせるので
        これが正しいが、複製側は **既に落ち着いた姿勢**(container_list の値)から始めるため、
        同じ480ステップを踏むと**さらに沈み込んで**しまう。
        実測でこのズレは位置 0.74mm・姿勢 0.0045(P06)に達し、safety_margin 0.015 /
        inclusion_margin −0.005 という判定閾値に対して十分大きく、エピソード後半の
        際どい合否をひっくり返していた(ステップ0の初回計測で P 系5シーンだけ
        Spearman が 1.000 を割った原因)。
        本物の env が optimize を呼ぶ時点の状態はまさに container_list の値そのものなので、
        そこへ戻すのが正しい復元である。
        """
        for container, want in zip(self.cm.containers, self.given):
            for item, wi in zip(container.packed_items, want.get('packed_items', [])):
                if wi.get('pos') is None or wi.get('orn') is None:
                    continue
                item.set_pose(self.client, tuple(wi['pos']), tuple(wi['orn']))
                item.register_pos_orn(container.index, tuple(wi['pos']), tuple(wi['orn']))

    def _obs_container_list(self):
        return self.cm.get_item_info_in_containers()

    def run_order(self, all_item_infos: list[dict], order: list[int],
                  policy_budget: float, hard_wall: float) -> dict:
        """order の通りに荷物を流し、本物と同じ検証・物理でエピソードを回す。

        方策は agents/mysolver/planner.plan(=policy と同一)。
        """
        by_idx = {int(it['index']): it for it in all_item_infos}
        stream = [by_idx[i] for i in order]
        cursor = 0
        pool: list[Item] = []
        while len(pool) < self.lookahead_k and cursor < len(stream):
            pool.append(Item(**stream[cursor]))
            cursor += 1

        n_steps = 0
        while pool:
            obs_containers = self._obs_container_list()
            pool_infos = [it.get_info() for it in pool]
            action = planner.plan(obs_containers, pool_infos,
                                  time_budget=policy_budget,
                                  hard_deadline=time.perf_counter() + hard_wall,
                                  strict_support=False,
                                  prepacked_ids=self._prepacked_ids)
            if action is None:
                break
            item_idx = int(action['item_idx'])
            ci = int(action['container_idx'])
            if not (0 <= item_idx < len(pool)) or not (0 <= ci < len(self.cm.containers)):
                break
            target_item = pool[item_idx]
            container = self.cm.get_container(ci)
            global_pos = container.local_to_global(action['place_pos'])
            orn_idx = int(action['orientation'])

            if not self.validator.check_inclusion(container, target_item, global_pos, orn_idx):
                break
            if not self.validator.check_transport_path(container, target_item, global_pos, orn_idx):
                break
            if not self.validator.place_item(target_item, global_pos, orn_idx):
                break
            self.cm.update_and_add_item_to_container(container_id=ci, item=target_item)
            pool.pop(item_idx)
            n_space = self.lookahead_k - len(pool)
            if n_space >= ASSUMED_MAX_SPACE:
                for _ in range(n_space):
                    if cursor < len(stream):
                        pool.append(Item(**stream[cursor]))
                        cursor += 1
            n_steps += 1

        containers = self.cm.containers
        fill_strict, out_s = Evaluator(client=self.client,
                                       config={'inclusion_margin': STRICT_MARGIN}
                                       ).calculate_fill_rate(containers)
        fill_loose, out_l = Evaluator(client=self.client,
                                      config={'inclusion_margin': LOOSE_MARGIN}
                                      ).calculate_fill_rate(containers)
        return {'fill_strict': fill_strict, 'fill_loose': fill_loose,
                'num_placed': sum(len(c.packed_items) for c in containers),
                'n_steps': n_steps}

    def set_prepacked(self, prepacked_ids):
        self._prepacked_ids = prepacked_ids

    def close(self):
        try:
            self.client.disconnect()
        except Exception:
            pass


def load_scene(label):
    task = list(json.load(open(f'configs/gen/suite_{label}.json')).values())[0]
    env = GroundHandlingEnv(config=task, verbose=False, render_mode=None)
    try:
        env.reset_settings()
        init = env.get_init_states()
        items = env.get_info_for_optimization()
        container_list = init['container_list']
        lookahead = init['lookahead_k']
    finally:
        try:
            env.close()
        except Exception:
            pass
    return container_list, items, lookahead


def spearman(xs, ys):
    n = len(xs)
    if n < 2:
        return float('nan')

    def rank(v):
        order = sorted(range(n), key=lambda i: v[i])
        r = [0.0] * n
        i = 0
        while i < n:
            j = i
            while j + 1 < n and v[order[j + 1]] == v[order[i]]:
                j += 1
            avg = (i + j) / 2.0 + 1.0
            for k in range(i, j + 1):
                r[order[k]] = avg
            i = j + 1
        return r
    rx, ry = rank(xs), rank(ys)
    mx, my = sum(rx) / n, sum(ry) / n
    num = sum((a - mx) * (b - my) for a, b in zip(rx, ry))
    dx = math.sqrt(sum((a - mx) ** 2 for a in rx))
    dy = math.sqrt(sum((b - my) ** 2 for b in ry))
    return num / (dx * dy) if dx > 0 and dy > 0 else float('nan')


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--cand', nargs='+', required=True)
    ap.add_argument('--truth', default='results/phase30_cand_eval.json')
    ap.add_argument('--out', default=None)
    ap.add_argument('--scenes', nargs='*', default=None)
    ap.add_argument('--policy-budget', type=float, default=5.5)
    ap.add_argument('--hard-wall', type=float, default=6.0)
    args = ap.parse_args()

    from agents.mysolver import geometry as geo

    cands = []
    for f in args.cand:
        cands.extend(json.load(open(f)))
    cands.sort(key=lambda c: c['label'])
    truth = {(r['label'], r['cand_idx']): r for r in json.load(open(args.truth))}

    rows = []
    geom_checks = []
    t_all = time.perf_counter()
    for c in cands:
        label = c['label']
        if args.scenes and label not in args.scenes:
            continue
        container_list, items, lookahead = load_scene(label)
        rep = ReplicaEvaluator(container_list, lookahead)
        with redirect_stdout(io.StringIO()):
            diffs = rep.build()
            rep.set_prepacked(geo.initial_prepacked_ids(container_list))
        geom_checks.append({'label': label, **diffs})
        print(f'--- {label}: 幾何再構築の最大差分 '
              f"volume={diffs['volume']:.3e} center={diffs['center']:.3e} "
              f"n_vecs={diffs['n_vecs']:.3e} points={diffs['points']:.3e}", flush=True)

        for idx, rec in enumerate(c['records']):
            key = (label, idx)
            if key not in truth:
                continue
            with redirect_stdout(io.StringIO()):
                rep.build()
                rep.set_prepacked(geo.initial_prepacked_ids(container_list))
                t0 = time.perf_counter()
                got = rep.run_order(items, rec['order'], args.policy_budget, args.hard_wall)
            sec = time.perf_counter() - t0
            tr = truth[key]
            row = {'label': label, 'cand_idx': idx,
                   'replica_fill_strict': got['fill_strict'],
                   'truth_fill_strict': tr['fill_strict'],
                   'diff_fill_strict': got['fill_strict'] - tr['fill_strict'],
                   'replica_fill_loose': got['fill_loose'],
                   'truth_fill_loose': tr['fill_loose'],
                   'replica_num_placed': got['num_placed'],
                   'truth_num_placed': tr['num_placed'],
                   'replica_sec': sec, 'truth_sec': tr.get('sec')}
            rows.append(row)
            print(f"  cand{idx}: replica={got['fill_strict']:6.2f} truth={tr['fill_strict']:6.2f} "
                  f"diff={row['diff_fill_strict']:+6.2f} "
                  f"placed={got['num_placed']}/{tr['num_placed']} ({sec:.1f}s)", flush=True)
            if args.out:
                json.dump({'geom_checks': geom_checks, 'rows': rows},
                          open(args.out, 'w'), indent=1, ensure_ascii=False)
        rep.close()

    # ---- 集計 ----
    diffs = [r['diff_fill_strict'] for r in rows]
    secs = [r['replica_sec'] for r in rows]
    n = len(diffs)
    mean = sum(diffs) / n if n else float('nan')
    sd = math.sqrt(sum((d - mean) ** 2 for d in diffs) / (n - 1)) if n > 1 else 0.0
    per_scene = {}
    for r in rows:
        per_scene.setdefault(r['label'], []).append(r)
    rhos = []
    for lab, rs in sorted(per_scene.items()):
        if len(rs) < 3:
            continue
        rho = spearman([x['replica_fill_strict'] for x in rs],
                       [x['truth_fill_strict'] for x in rs])
        rhos.append((lab, rho, len(rs)))
    exact = sum(1 for r in rows if abs(r['diff_fill_strict']) < 1e-9)
    within1 = sum(1 for r in rows if abs(r['diff_fill_strict']) <= 1.0)

    print('\n========== ステップ0 集計 ==========')
    print(f'候補数: {n}')
    print(f'fill_strict の差: mean={mean:+.3f} σ={sd:.3f} '
          f'max|diff|={max((abs(d) for d in diffs), default=float("nan")):.3f}')
    print(f'完全一致(<1e-9): {exact}/{n} / |差|<=1.0: {within1}/{n}')
    valid = [r for _, r, _ in rhos if not math.isnan(r)]
    print(f'シーン内順位 Spearman: 平均={sum(valid)/len(valid) if valid else float("nan"):+.3f} '
          f'(シーン数 {len(valid)})')
    for lab, rho, k in rhos:
        print(f'    {lab:30s} rho={rho:+.3f} (n={k})')
    print(f'1候補あたり実行時間: mean={sum(secs)/len(secs) if secs else float("nan"):.1f}s '
          f'max={max(secs, default=float("nan")):.1f}s '
          f'(本物 env: mean={sum(r["truth_sec"] for r in rows if r.get("truth_sec"))/max(1,n):.1f}s)')
    print(f'総所要 {(time.perf_counter() - t_all)/60:.1f}min')

    if args.out:
        summary = {'n': n, 'diff_mean': mean, 'diff_sigma': sd,
                   'exact': exact, 'within1': within1,
                   'rho_mean': (sum(valid) / len(valid)) if valid else None,
                   'rho_per_scene': [{'label': l, 'rho': r, 'n': k} for l, r, k in rhos],
                   'replica_sec_mean': (sum(secs) / len(secs)) if secs else None,
                   'assumed_validator': ASSUMED_VALIDATOR}
        json.dump({'geom_checks': geom_checks, 'rows': rows, 'summary': summary},
                  open(args.out, 'w'), indent=1, ensure_ascii=False)


if __name__ == '__main__':
    main()
