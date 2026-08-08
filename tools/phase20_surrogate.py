"""
tools/phase20_surrogate.py

Phase20 ターゲット1: 影シミュレータ(simulate.simulate_order)の**代理誤差**を直接測定する。

背景:
    Phase16(予算4倍)・Phase17(決定性)・Phase19(29%高速化)と探索側の改善を3フェーズ
    続けたが fill はほぼ動かなかった(+1.15 / -0.14 / -0.63)。探索は速く・決定的で・予算
    単調になったのに結果が伸びないなら、**最適化している目的関数そのものが実評価とずれて
    いる**というのが最も自然な説明である。Phase17で決定性が確保されたため、この「代理誤差」
    は初めて厳密に(乱数に埋もれずに)測れる。

測定の設計:
    `ordering.build_order` は候補順序 order ごとに `simulate.simulate_order` を呼び、
    その戻り値から目的関数
        objective = risk調整済み体積 - PLACEMENT_PENALTY_WEIGHT * 総容積 * placement違反率
    を作って argmax を選ぶ。つまり build_order が実際に使っているのは **候補間の順位** だけ
    である。したがって代理誤差の本質は「予測値と実測値の順位が一致するか」= Spearman順位相関
    であり、絶対値のずれ(系統バイアス)は全候補に共通なら順位を変えないので二次的である。

    そこで:
      1. `collect`     : simulate_order をフックして、build_order が評価した全候補順序と
                         その予測値を記録する(探索の挙動自体は一切変えない)。
      2. `groundtruth` : 記録した各順序を **実際の pybullet エピソードに固定順序として流し込み**、
                         真の fill_strict / fill_loose / 配置個数 / fill計上率を測る。
      3. `analyze`     : 予測 vs 実測の Spearman / Pearson、系統バイアス、そして
                         **regret(argmax(予測)を選んだことで失った実 fill)** を出す。

    regret こそが「代理誤差が実際にいくらの fill を捨てているか」の直接の金額表示であり、
    本フェーズの主KPI(順位相関)を fill に換算した値になる。

誤差の分解:
    影シミュレータの予測が外れる経路は原理的に2つある。
      (A) 配置予測誤差 : 「この順序でどの荷物が置けるか」の予測が外れる
                         (影シミュレータは幾何・合法性のみで、実機の物理沈降/傾き/衝撃は
                          モデル化していない)。 pred_placed_volume vs actual_placed_volume。
      (B) 計上予測誤差 : 置けた荷物のうち「実際に fill に計上される」割合の予測が外れる
                         (geo.fill_risk_factor が担当している部分)。
                         pred_risk_adjusted / pred_placed vs actual_counted / actual_placed。
    この2つを分けて出すことで、次にどちらを直すべきかが決まる。

使い方:
    # 1シーンの候補順序と予測値を収集(探索を1回走らせるだけ、physics不要)
    PYTHONPATH=. .venv/bin/python tools/phase20_surrogate.py collect \
        --config-path configs/gen/suite_P02_A_1c_pre10.json \
        --out results/phase20_pred_P02.json

    # 収集した各順序を実エピソードで走らせて真値を測る(重い)
    PYTHONPATH=. .venv/bin/python tools/phase20_surrogate.py groundtruth \
        --pred results/phase20_pred_P02.json --out results/phase20_gt_P02.json

    # 相関・バイアス・regret を集計
    PYTHONPATH=. .venv/bin/python tools/phase20_surrogate.py analyze \
        --gt results/phase20_gt_*.json

src/ は一切変更しない。agents/ もこのツールからは変更せず、フックのみで観測する。
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

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from src.ground_handling.env import GroundHandlingEnv
from src.ground_handling.evaluator import Evaluator

STRICT_MARGIN = -0.005
LOOSE_MARGIN = 0.01


# ----------------------------------------------------------------------------
# 統計ヘルパ(scipy に依存しない: agents/ 側と同じ「追加依存を増やさない」方針に揃える)
# ----------------------------------------------------------------------------
def _ranks(values):
    """同順位は平均順位(midrank)にする。Spearman の標準的な扱い。"""
    n = len(values)
    order = sorted(range(n), key=lambda i: values[i])
    ranks = [0.0] * n
    i = 0
    while i < n:
        j = i
        while j + 1 < n and values[order[j + 1]] == values[order[i]]:
            j += 1
        avg = (i + j) / 2.0 + 1.0
        for k in range(i, j + 1):
            ranks[order[k]] = avg
        i = j + 1
    return ranks


def _pearson(xs, ys):
    n = len(xs)
    if n < 2:
        return float('nan')
    mx = sum(xs) / n
    my = sum(ys) / n
    sxy = sum((x - mx) * (y - my) for x, y in zip(xs, ys))
    sxx = sum((x - mx) ** 2 for x in xs)
    syy = sum((y - my) ** 2 for y in ys)
    if sxx <= 0 or syy <= 0:
        return float('nan')
    return sxy / math.sqrt(sxx * syy)


def _spearman(xs, ys):
    if len(xs) < 2:
        return float('nan')
    return _pearson(_ranks(xs), _ranks(ys))


def _mean(vals):
    vals = [v for v in vals if not (isinstance(v, float) and math.isnan(v))]
    return sum(vals) / len(vals) if vals else float('nan')


# ----------------------------------------------------------------------------
# collect: build_order が評価した候補順序と予測値を記録する
# ----------------------------------------------------------------------------
def collect_scene(task_config, module_path, label, optimize_budget=None, extra_restarts=0,
                   clean_validate_sec=60.0):
    """1シーンについて build_order を1回走らせ、評価された全候補順序と予測値を返す。

    `simulate.simulate_order` をラップして観測するだけで、戻り値も副作用も一切変えない
    (=探索の軌跡は無改造時と完全に同一。Phase17の決定性によりこれは再現可能)。

    extra_restarts > 0 のとき、build_order が実際に評価した候補(既定予算120sでは
    フェーズ1の数本しか回らない)に加えて、**フェーズ2と同じレシピ**(ランダムwindow・
    ランダム戦略シード・score_noise・shuffle_ties)で追加の候補順序を生成する。
    順位相関を N=4〜6 で測ると推定量の分散が大きすぎるため、「予算をもっと積んだら
    build_order が実際に見たであろう候補」で N を増やす。無関係なランダム置換を混ぜると
    「明らかに悪い順序を下位に置けるだけ」で相関が過大評価されるので、候補は必ず
    貪欲構築器が生成したもの(=実際に競合しうる、同程度の品質の順序)だけにする。

    clean_validate_sec: 全候補を**同一の潤沢な予算**で評価し直すための枠。build_order 本来の
    検証枠(MAX_VALIDATE_SLICE=12s)は途中打ち切りされうるため、打ち切られた予測と
    打ち切られていない予測が混在したまま相関を測ると、代理誤差と予算不足が交絡する。
    そこで相関測定には全候補を打ち切りなしで測り直した値を使い、本来の as-used 値
    (`objective_pred_asused`)は意思決定への影響(regret)の再現用に別途保持する。
    """
    mod_prefix = '.'.join(module_path.rstrip('/').split('/'))
    agent_mod = importlib.import_module(mod_prefix + '.agent')
    simulate_mod = importlib.import_module(mod_prefix + '.simulate')
    ordering_mod = importlib.import_module(mod_prefix + '.ordering')
    planner_mod = importlib.import_module(mod_prefix + '.planner')
    geo_mod = importlib.import_module(mod_prefix + '.geometry')
    import numpy as np

    records = []
    original = simulate_mod.simulate_order

    def hooked(container_list, items_by_index, order, lookahead_k, budget, **kwargs):
        result = original(container_list, items_by_index, order, lookahead_k, budget, **kwargs)
        placed_ids, placed_volume, risk_adjusted_volume, violation_ratio, stability_risk_ratio = result
        # この検証呼び出し自身の枠(max_validate_slice)を使い切ったか = 予測が途中で
        # 打ち切られたか。打ち切られた予測は「置けるはずの残りを評価していない」ので
        # 系統的に過小評価になる(代理誤差の候補要因の1つとして必ず記録する)。
        truncated = bool(budget.used >= budget.limit) if hasattr(budget, 'used') else None
        records.append({
            'order': list(order),
            'n_placed_pred': len(placed_ids),
            'placed_ids_pred': list(placed_ids),
            'placed_volume_pred': float(placed_volume),
            'risk_adjusted_volume_pred': float(risk_adjusted_volume),
            'violation_ratio_pred': float(violation_ratio),
            'stability_risk_ratio_pred': float(stability_risk_ratio),
            'budget_exhausted': truncated,
            'budget_used': float(budget.used) if hasattr(budget, 'used') else None,
            'budget_limit': float(budget.limit) if hasattr(budget, 'limit') else None,
        })
        return result

    env = GroundHandlingEnv(config=task_config, verbose=False, render_mode=None)
    try:
        env.reset_settings()
        if not env.optimize:
            return {'label': label, 'skipped': 'optimize=False (build_orderを通らないシーン)',
                    'candidates': []}
        init_states = env.get_init_states()
        item_list = env.get_info_for_optimization()

        agent = agent_mod.Agent(module_path)
        agent.get_init_states(init_states)

        simulate_mod.simulate_order = hooked
        try:
            t0 = time.time()
            chosen_order = list(agent.optimize(item_list))
            elapsed = time.time() - t0
        finally:
            simulate_mod.simulate_order = original

        container_list = init_states.get('container_list', [])
        total_container_volume = sum(c.get('volume', 0.0) for c in container_list)
        placement_w = getattr(ordering_mod, 'PLACEMENT_PENALTY_WEIGHT', 0.5)
        k = max(1, int(init_states.get('lookahead_k') or 1))
        items_by_index = {it['index']: it for it in item_list}
        all_indices = set(items_by_index)
        prepacked_ids = geo_mod.initial_prepacked_ids(container_list)

        # 重複順序(同じ order が2回評価された場合)は最初の1件に潰す。
        seen = {}
        uniq = []
        for rec in records:
            key = tuple(rec['order'])
            if key in seen:
                continue
            seen[key] = True
            rec['source'] = 'build_order'
            rec['objective_pred_asused'] = (rec['risk_adjusted_volume_pred']
                                             - placement_w * total_container_volume * rec['violation_ratio_pred'])
            rec['is_chosen'] = (rec['order'] == chosen_order)
            uniq.append(rec)
        n_native = len(uniq)

        # --- 追加候補の生成(フェーズ2と同じレシピ) ---
        if extra_restarts > 0:
            rng = np.random.default_rng(12345)   # build_order 内の rng(seed=0)とは独立
            windows = list(getattr(ordering_mod, 'WINDOW_CANDIDATES', [None]))
            strategies = [fn(item_list) for fn in getattr(ordering_mod, 'STRATEGIES', [])]
            for _ in range(extra_restarts):
                budget = planner_mod.SearchBudget.from_seconds(clean_validate_sec)
                window = windows[int(rng.integers(0, len(windows)))]
                seed_items = strategies[int(rng.integers(0, len(strategies)))] if strategies else item_list
                try:
                    order = simulate_mod.greedy_construct_order(
                        container_list, seed_items, budget,
                        per_step_time_budget=getattr(ordering_mod, 'PER_STEP_TIME_BUDGET', 3.0),
                        rng=rng, score_noise=0.35, shuffle_ties=True, window=window,
                        prepacked_ids=prepacked_ids)
                except Exception:
                    continue
                if set(order) != all_indices:
                    continue
                key = tuple(order)
                if key in seen:
                    continue
                seen[key] = True
                uniq.append({'order': list(order), 'source': 'extra_restart', 'is_chosen': False})

        # --- 全候補を同一の潤沢な予算で評価し直す(打ち切りと代理誤差の交絡を除去) ---
        for rec in uniq:
            budget = planner_mod.SearchBudget.from_seconds(clean_validate_sec)
            placed_ids, placed_volume, risk_adjusted_volume, violation_ratio, stability_risk = original(
                container_list, items_by_index, rec['order'], k, budget,
                prepacked_ids=prepacked_ids,
                stability_weight=getattr(ordering_mod, 'STABILITY_PENALTY_WEIGHT', 0.0))
            rec['n_placed_pred'] = len(placed_ids)
            rec['placed_ids_pred'] = list(placed_ids)
            rec['placed_volume_pred'] = float(placed_volume)
            rec['risk_adjusted_volume_pred'] = float(risk_adjusted_volume)
            rec['violation_ratio_pred'] = float(violation_ratio)
            rec['stability_risk_ratio_pred'] = float(stability_risk)
            rec['clean_truncated'] = bool(budget.used >= budget.limit)
            rec['objective_pred'] = (rec['risk_adjusted_volume_pred']
                                      - placement_w * total_container_volume * rec['violation_ratio_pred'])

        return {
            'label': label,
            'module_path': module_path,
            'optimize_budget': optimize_budget,
            'total_container_volume': total_container_volume,
            'placement_penalty_weight': placement_w,
            'n_total_items': len(item_list),
            'lookahead_k': init_states.get('lookahead_k'),
            'chosen_order': chosen_order,
            'n_candidates_evaluated': len(records),
            'n_candidates_native': n_native,
            'n_candidates_unique': len(uniq),
            'optimize_elapsed_sec': elapsed,
            # groundtruth 側で「既積み荷物」を実測から除外するために保存する
            # (影シミュレータが予測するのは新規配置分だけなので、揃えないと比較が壊れる)
            'prepacked_index_list': sorted(int(i) for ids in prepacked_ids.values() for i in ids),
            'candidates': uniq,
        }
    finally:
        try:
            env.close()
        except Exception:
            pass


def cmd_collect(args):
    if args.optimize_budget is not None:
        os.environ['MYSOLVER_OPTIMIZE_BUDGET'] = str(args.optimize_budget)

    out = {'scenes': {}}
    for config_path in _expand(args.config_path):
        with open(config_path) as f:
            config = json.load(f)
        for task_id in config:
            label = f'{os.path.basename(config_path)}::{task_id}'
            print(f'[collect] {label} ...', flush=True)
            t0 = time.time()
            try:
                res = collect_scene(config[task_id], args.module_path, label, args.optimize_budget,
                                     extra_restarts=args.extra_restarts,
                                     clean_validate_sec=args.clean_validate_sec)
            except Exception:
                print(f'  ERROR: {traceback.format_exc()}')
                continue
            res['config_path'] = config_path
            res['task_id'] = task_id
            out['scenes'][label] = res
            print(f'  -> 候補{res.get("n_candidates_unique", 0)}件(重複除去前 '
                  f'{res.get("n_candidates_evaluated", 0)}件) / {time.time() - t0:.1f}s', flush=True)

    with open(args.out, 'w') as f:
        json.dump(out, f, indent=1)
    print(f'\nwrote {args.out}')


# ----------------------------------------------------------------------------
# groundtruth: 各候補順序を実エピソードで走らせて真値を測る
# ----------------------------------------------------------------------------
def run_episode_with_order(task_config, module_path, order, with_stability=False,
                            prepacked_index_set=None):
    """order を固定して1エピソードを実走させ、実測指標を返す。

    optimize() は呼ばず、与えられた order をそのまま env.set_item_order に渡す
    (= build_order がその順序を選んだ場合に実際に起きること)。online policy は
    Phase17以降決定的なので、1回走らせれば十分。
    """
    mod_prefix = '.'.join(module_path.rstrip('/').split('/'))
    agent_mod = importlib.import_module(mod_prefix + '.agent')

    env = GroundHandlingEnv(config=task_config, verbose=False, render_mode=None)
    try:
        env.reset_settings()
        agent = agent_mod.Agent(module_path)
        agent.get_init_states(env.get_init_states())
        if env.optimize:
            if not env.set_item_order(list(order)):
                return {'error': 'set_item_order rejected the order'}
        env.reset_item_stream()
        obs, info = env.reset(seed=42)

        max_policy = 0.0
        terminated = truncated = False
        place_states = {}
        while not terminated and not truncated:
            t0 = time.perf_counter()
            action = agent.policy(obs)
            max_policy = max(max_policy, time.perf_counter() - t0)
            obs, reward, terminated, truncated, info = env.step(action)
            place_states = {k: v for k, v in info['status'].items()}

        containers = env.container_manager.containers
        strict_eval = Evaluator(client=env.client, config={'inclusion_margin': STRICT_MARGIN})
        loose_eval = Evaluator(client=env.client, config={'inclusion_margin': LOOSE_MARGIN})
        fill_strict, out_strict = strict_eval.calculate_fill_rate(containers)
        fill_loose, out_loose = loose_eval.calculate_fill_rate(containers)

        n_packed = sum(len(c.packed_items) for c in containers)
        # calculate_fill_rate は list[Item] を返す(indexの列ではない)
        excluded = set(int(it.index) for it in out_strict)
        # **重要**: 影シミュレータが予測するのは「このエピソードで新たに置いた荷物」だけであり、
        # 既積み(prepacked)荷物は予測の対象外(初期状態として与えられている)。一方
        # env.container_manager の packed_items は既積みも含む。両者をそのまま比べると
        # prepacked シーンで予測が一律に過小に見える見かけの誤差が出るため、必ず切り分ける。
        pre = set(int(i) for i in (prepacked_index_set or ()))
        placed_volume_actual = 0.0        # 新規配置のみ
        counted_volume_actual = 0.0       # 新規配置のうち fill に計上されたもの
        counted_volume_total = 0.0        # 既積み込みの計上体積(= fill_strict の分子)
        n_new = 0
        n_new_counted = 0
        for c in containers:
            for it in c.packed_items:
                idx = int(it.index)
                vol = float(it.length) * float(it.width) * float(it.height)
                counted = idx not in excluded
                if counted:
                    counted_volume_total += vol
                if idx in pre:
                    continue
                n_new += 1
                placed_volume_actual += vol
                if counted:
                    counted_volume_actual += vol
                    n_new_counted += 1

        result = {
            'fill_strict': fill_strict,
            'fill_loose': fill_loose,
            'n_packed_total': n_packed,
            'n_placed_actual': n_new,
            'placed_volume_actual': placed_volume_actual,
            'counted_volume_actual': counted_volume_actual,
            'counted_volume_total': counted_volume_total,
            'fill_counted_ratio_strict': ((n_packed - len(out_strict)) / n_packed) if n_packed else 1.0,
            'fill_counted_ratio_new': (n_new_counted / n_new) if n_new else 1.0,
            'fill_counted_ratio_loose': ((n_packed - len(out_loose)) / n_packed) if n_packed else 1.0,
            'max_policy_sec': max_policy,
            'place_states': place_states,
        }
        if with_stability:
            from tools.scorer import Scorer
            scorer = Scorer(client=env.client, config=task_config)
            result['placement_score'] = scorer.calculate_placement_score(containers)
            result['soft_item_score'] = scorer.calculate_soft_item_score(containers)
            result['cog_score'] = scorer.calculate_cog_score(containers)
            result['stability_score'] = scorer.calculate_stability_score(containers)  # 破壊的: 最後
        return result
    except Exception:
        return {'error': traceback.format_exc().splitlines()[-1]}
    finally:
        try:
            env.close()
        except Exception:
            pass


def cmd_groundtruth(args):
    with open(args.pred) as f:
        pred = json.load(f)

    out = {'scenes': {}}
    for label, scene in pred['scenes'].items():
        cands = scene.get('candidates', [])
        if not cands:
            print(f'[gt] {label}: 候補なし、skip')
            continue
        with open(scene['config_path']) as f:
            task_config = json.load(f)[scene['task_id']]

        rows = []
        prepacked = scene.get('prepacked_index_list', [])
        print(f'[gt] {label}: {len(cands)}候補 (既積み{len(prepacked)}個は実測から除外)', flush=True)
        for i, cand in enumerate(cands):
            t0 = time.time()
            actual = run_episode_with_order(task_config, args.module_path, cand['order'],
                                             with_stability=args.with_stability,
                                             prepacked_index_set=prepacked)
            row = dict(cand)
            row['actual'] = actual
            rows.append(row)
            print(f'  [{i + 1}/{len(cands)}] pred_obj={cand["objective_pred"]:.4f} '
                  f'pred_n={cand["n_placed_pred"]} -> '
                  f'fill_strict={actual.get("fill_strict", float("nan")):.2f} '
                  f'n={actual.get("n_placed_actual", -1)} ({time.time() - t0:.1f}s)', flush=True)
        scene_out = {k: v for k, v in scene.items() if k != 'candidates'}
        scene_out['rows'] = rows
        out['scenes'][label] = scene_out
        with open(args.out, 'w') as f:      # 途中経過を都度保存(長時間ジョブ対策)
            json.dump(out, f, indent=1)

    with open(args.out, 'w') as f:
        json.dump(out, f, indent=1)
    print(f'\nwrote {args.out}')


# ----------------------------------------------------------------------------
# analyze: 相関・バイアス・regret
# ----------------------------------------------------------------------------
def analyze_scene(scene):
    rows = [r for r in scene['rows'] if 'error' not in r['actual']]
    if len(rows) < 2:
        return None
    vol = scene['total_container_volume']
    scale = 100.0 / vol if vol else 0.0

    pred_obj = [r['objective_pred'] for r in rows]
    # 影シミュレータが予測するのは「新規配置分」だけなので、実測側も新規分に揃えて比べる。
    # 一方 fill_strict は既積みを含む真の目的値であり、順位(Spearman)は既積み分が
    # 全候補共通の定数オフセットなので影響を受けない。
    pred_fill_risk = [r['risk_adjusted_volume_pred'] * scale for r in rows]
    pred_fill_raw = [r['placed_volume_pred'] * scale for r in rows]
    act_fill = [r['actual']['fill_strict'] for r in rows]
    act_fill_loose = [r['actual']['fill_loose'] for r in rows]
    act_raw = [r['actual']['placed_volume_actual'] * scale for r in rows]
    act_counted_new = [r['actual']['counted_volume_actual'] * scale for r in rows]
    pred_n = [r['n_placed_pred'] for r in rows]
    act_n = [r['actual']['n_placed_actual'] for r in rows]

    # build_order は argmax(objective_pred) を選ぶ。その選択が失った実fill = regret。
    best_pred_i = max(range(len(rows)), key=lambda i: pred_obj[i])
    best_act_i = max(range(len(rows)), key=lambda i: act_fill[i])
    regret = act_fill[best_act_i] - act_fill[best_pred_i]

    # oracle が「予測値そのものではなく順位だけ」を使う点に合わせ、順位相関を主指標にする
    return {
        'n_candidates': len(rows),
        'spearman_obj_vs_fill': _spearman(pred_obj, act_fill),
        'spearman_riskvol_vs_fill': _spearman(pred_fill_risk, act_fill),
        'spearman_rawvol_vs_fill': _spearman(pred_fill_raw, act_fill),
        'spearman_predn_vs_actn': _spearman(pred_n, act_n),
        'spearman_obj_vs_fill_loose': _spearman(pred_obj, act_fill_loose),
        'pearson_riskvol_vs_fill': _pearson(pred_fill_risk, act_fill),
        'spearman_riskvol_vs_countednew': _spearman(pred_fill_risk, act_counted_new),
        # --- 系統バイアス(絶対値のずれ。いずれも新規配置分どうしで揃えて比較) ---
        'mean_pred_fill_risk': _mean(pred_fill_risk),
        'mean_pred_fill_raw': _mean(pred_fill_raw),
        'mean_actual_fill': _mean(act_fill),
        'mean_actual_counted_new': _mean(act_counted_new),
        'mean_actual_raw_fill': _mean(act_raw),
        'bias_risk_minus_actual': _mean(pred_fill_risk) - _mean(act_counted_new),
        'mean_pred_n': _mean(pred_n),
        'mean_act_n': _mean(act_n),
        # --- 誤差の分解 ---
        # (A) 配置予測誤差: 影シミュレータが「置ける」と思った体積 vs 実際に置けた体積
        'bias_placement_pred': _mean(pred_fill_raw) - _mean(act_raw),
        'spearman_placement_pred': _spearman(pred_fill_raw, act_raw),
        # (B) 計上予測誤差: 置けた体積のうち fill に計上される割合の予測 vs 実測
        'mean_pred_counted_frac': _mean([r['risk_adjusted_volume_pred'] / r['placed_volume_pred']
                                          if r['placed_volume_pred'] > 0 else float('nan') for r in rows]),
        'mean_actual_counted_frac': _mean([r['actual']['counted_volume_actual'] / r['actual']['placed_volume_actual']
                                            if r['actual']['placed_volume_actual'] > 0 else float('nan')
                                            for r in rows]),
        'mean_actual_counted_ratio_items': _mean([r['actual']['fill_counted_ratio_strict'] for r in rows]),
        # --- 意思決定への影響 ---
        'chosen_fill': act_fill[best_pred_i],
        'best_possible_fill': act_fill[best_act_i],
        'regret_fill': regret,
        'fill_spread': max(act_fill) - min(act_fill),
        'n_budget_exhausted': sum(1 for r in rows if r.get('budget_exhausted')),
    }


def cmd_analyze(args):
    scenes = {}
    for path in _expand(args.gt):
        with open(path) as f:
            data = json.load(f)
        for label, scene in data['scenes'].items():
            scenes[label] = scene

    results = {}
    for label, scene in sorted(scenes.items()):
        r = analyze_scene(scene)
        if r is not None:
            results[label] = r

    print('=' * 118)
    print('Phase20 ターゲット1: 影シミュレータの代理誤差')
    print('=' * 118)
    print(f'{"scene":34s} {"N":>3s} {"Spearman":>9s} {"Spear(vol)":>10s} '
          f'{"predF":>7s} {"actF_new":>8s} {"bias":>7s} {"act_fill":>8s} '
          f'{"chosen":>7s} {"best":>7s} {"regret":>7s}')
    print('-' * 118)
    for label, r in results.items():
        print(f'{label[:34]:34s} {r["n_candidates"]:3d} {r["spearman_obj_vs_fill"]:9.3f} '
              f'{r["spearman_riskvol_vs_fill"]:10.3f} '
              f'{r["mean_pred_fill_risk"]:7.2f} {r["mean_actual_counted_new"]:8.2f} '
              f'{r["bias_risk_minus_actual"]:+7.2f} {r["mean_actual_fill"]:8.2f} '
              f'{r["chosen_fill"]:7.2f} {r["best_possible_fill"]:7.2f} {r["regret_fill"]:7.2f}')
    print('-' * 118)
    if results:
        v = results.values()
        print(f'{"MEAN":34s} {_mean([r["n_candidates"] for r in v]):3.0f} '
              f'{_mean([r["spearman_obj_vs_fill"] for r in v]):9.3f} '
              f'{_mean([r["spearman_riskvol_vs_fill"] for r in v]):10.3f} '
              f'{_mean([r["mean_pred_fill_risk"] for r in v]):7.2f} '
              f'{_mean([r["mean_actual_counted_new"] for r in v]):8.2f} '
              f'{_mean([r["bias_risk_minus_actual"] for r in v]):+7.2f} '
              f'{_mean([r["mean_actual_fill"] for r in v]):8.2f} '
              f'{_mean([r["chosen_fill"] for r in v]):7.2f} '
              f'{_mean([r["best_possible_fill"] for r in v]):7.2f} '
              f'{_mean([r["regret_fill"] for r in v]):7.2f}')

    print('\n' + '=' * 118)
    print('誤差の分解: (A)配置予測 = どの荷物が置けるかの予測 / (B)計上予測 = 置けた体積のうちfillに載る割合')
    print('=' * 118)
    print(f'{"scene":38s} {"A:Spear":>8s} {"A:bias":>8s} | '
          f'{"B:pred_frac":>11s} {"B:act_frac":>10s} {"B:diff":>7s} | {"act_item_ratio":>14s}')
    print('-' * 118)
    for label, r in results.items():
        print(f'{label[:38]:38s} {r["spearman_placement_pred"]:8.3f} {r["bias_placement_pred"]:+8.2f} | '
              f'{r["mean_pred_counted_frac"]:11.3f} {r["mean_actual_counted_frac"]:10.3f} '
              f'{r["mean_pred_counted_frac"] - r["mean_actual_counted_frac"]:+7.3f} | '
              f'{r["mean_actual_counted_ratio_items"]:14.3f}')
    print('-' * 118)
    if results:
        print(f'{"MEAN":38s} {_mean([r["spearman_placement_pred"] for r in results.values()]):8.3f} '
              f'{_mean([r["bias_placement_pred"] for r in results.values()]):+8.2f} | '
              f'{_mean([r["mean_pred_counted_frac"] for r in results.values()]):11.3f} '
              f'{_mean([r["mean_actual_counted_frac"] for r in results.values()]):10.3f} '
              f'{_mean([r["mean_pred_counted_frac"] for r in results.values()]) - _mean([r["mean_actual_counted_frac"] for r in results.values()]):+7.3f} | '
              f'{_mean([r["mean_actual_counted_ratio_items"] for r in results.values()]):14.3f}')

    if args.out:
        with open(args.out, 'w') as f:
            json.dump(results, f, indent=1)
        print(f'\nwrote {args.out}')


def _expand(patterns):
    paths = []
    for pattern in patterns:
        matched = sorted(glob.glob(pattern))
        paths.extend(matched if matched else [pattern])
    return paths


def main():
    ap = argparse.ArgumentParser()
    sub = ap.add_subparsers(dest='cmd', required=True)

    c = sub.add_parser('collect')
    c.add_argument('--config-path', nargs='+', required=True)
    c.add_argument('--module-path', default='agents/mysolver/')
    c.add_argument('--optimize-budget', type=float, default=None)
    c.add_argument('--extra-restarts', type=int, default=0,
                    help='build_orderが実際に評価した候補に加えて生成する追加候補数(順位相関のN確保用)')
    c.add_argument('--clean-validate-sec', type=float, default=60.0,
                    help='全候補を打ち切りなしで評価し直すための名目予算[s]')
    c.add_argument('--out', required=True)
    c.set_defaults(func=cmd_collect)

    g = sub.add_parser('groundtruth')
    g.add_argument('--pred', required=True)
    g.add_argument('--module-path', default='agents/mysolver/')
    g.add_argument('--with-stability', action='store_true')
    g.add_argument('--out', required=True)
    g.set_defaults(func=cmd_groundtruth)

    a = sub.add_parser('analyze')
    a.add_argument('--gt', nargs='+', required=True)
    a.add_argument('--out', default=None)
    a.set_defaults(func=cmd_analyze)

    args = ap.parse_args()
    args.func(args)


if __name__ == '__main__':
    main()
