# Phase49 報告: ε制約cogの実装(作業1・作業2)— **作業2のゲートで停止**

指示どおり、作業1・作業2のみ実施し、**作業3以降には進んでいない**。

---

## 作業1: cog代理の実装

### (1-1)(1-2) 実装

`agents/mysolver/simulate.py` に、`tools/scorer.py::Scorer._get_floor_ceil_z` /
`calculate_cog_score` と**同一の正規化式**をそのまま移植した2関数を追加した
(独自の式は作っていない)。

```python
def _floor_ceil_z(container: dict) -> tuple[float, float]:
    """tools/scorer.py::Scorer._get_floor_ceil_z と同一の正規化式。"""
    center = container['center']
    height = container['height']
    floor_z = center[2] - height / 2.0
    ceil_z = center[2] + height / 2.0
    for pt, nv in zip(container.get('points', []), container.get('n_vecs', [])):
        if abs(nv[0]) < 1e-6 and abs(nv[1]) < 1e-6:
            if nv[2] < -0.5:
                floor_z = pt[2]
            elif nv[2] > 0.5:
                ceil_z = pt[2]
    return floor_z, ceil_z


def cog_proxy_score(containers: list[dict]) -> float:
    """tools/scorer.py::Scorer.calculate_cog_score と同一の質量加重・正規化式。"""
    total_mass = 0.0
    weighted_h = 0.0
    for container in containers:
        packed = container.get('packed_items', [])
        if not packed:
            continue
        floor_z, ceil_z = _floor_ceil_z(container)
        effective_height = max(ceil_z - floor_z, 1e-6)
        for item in packed:
            pos = item.get('pos')
            if pos is None:
                continue
            mass = item.get('mass', 0.0)
            normalized_h = (pos[2] - floor_z) / effective_height
            normalized_h = min(max(normalized_h, 0.0), 1.0)
            weighted_h += mass * normalized_h
            total_mass += mass
    if total_mass == 0:
        return 100.0
    avg_h = weighted_h / total_mass
    return min(max(100.0 * (1.0 - avg_h), 0.0), 100.0)
```

**scorer.pyの式との対応**: floor/ceil推定(points/n_vecs走査による精緻化つきfallback)・
質量加重・[0,1]クリップ・`100*(1-avg_h)`の最終変換まで**一字一句同じ計算**。違いは
入力データの出どころだけ——本物は real `Container`/`Item`オブジェクトの属性
(`.points`/`.n_vecs`/`.center`、pybullet実姿勢`item.get_pose()`)を読むのに対し、
代理は observation dict表現(`container['points']`/`container['n_vecs']`/
`container['center']`、`item['pos']`)を読む。CLAUDE_CODE_指示書.md §2の観測dict契約上
`points`/`n_vecs`は本物の`Container.create()`と同じ値がそのまま入っているため、
床/天井面の推定ロジックも含めて完全に同一の計算になる。

`simulate_order()`には新しい状態追跡を一切追加していない——`container['packed_items']`
は`_place()`が最初から`pos`(world座標)を書き込んでおり、既存データの集計だけで済んだ。

戻り値は新しいオプション引数 `compute_cog_proxy: bool = False` で制御し、**既定False
のときは従来どおり5要素タプルのまま**、Trueのときだけ末尾にcog代理値を追加した
6要素タプルを返す(既存の呼び出し側20箇所前後――`ordering.py`本体1箇所と
`tools/phase29-34*.py`の多数――はすべて5値の位置引数unpackをしており、
無条件で6要素にすると軒並み壊れるため)。

### (1-3) ビット単位不変性の確認 — **7/8一致、A03に本フェーズと無関係の既存差異を発見**

`bash scripts/bp_check.sh` で決定的8シーンを確認した。

```
B01: n=40 first10=[30, 13, 37, 26, 38, 22, 9, 3, 2, 17]   OK(過去の記録と一致)
B02: n=40 first10=[27, 7, 6, 29, 38, 19, 23, 34, 12, 15]   OK
B03: n=80 first10=[62, 36, 37, 7, 24, 30, 48, 9, 50, 25]   OK
B04: n=80 first10=[51, 43, 55, 53, 23, 68, 63, 66, 54, 61] OK
P04: n=34 first10=[16, 15, 18, 22, 24, 3, 0, 21, 5, 29]    OK
A01: n=40 first10=[13, 3, 35, 23, 17, 14, 30, 38, 2, 0]    OK
A02: n=80 first10=[11, 26, 53, 60, 58, 74, 63, 8, 72, 22]  OK
A03: n=40 first10=[38, 0, 6, 26, 7, 33, 21, 25, 1, 19]     ★過去の記録[13, 37, 28, 35, 22, 15, 25, 32, 29, 7]と不一致
```

**A03のみ、Phase41〜48で一貫して確認してきた過去の記録値と異なる。** ただし
`git stash`で本フェーズの変更(`agents/mysolver/simulate.py`)を完全に取り除いた
**コミット済みの元コード**でA03単体を再実行しても**同じ新しい値**が出ることを確認した
——**本フェーズの変更とは無関係**であることを実証済み。原因は特定していないが、
セッション間で環境(BLAS/浮動小数点丸め等、docs/migration_to_mac.mdが警告している
既知のリスク)が変わった可能性がある。numpy/scipy/pybulletのバージョンは
`env_snapshot.txt`と一致しており、パッケージの更新ではなさそうだが未確定。
**この件は本フェーズの評価基準(「私の変更が既定経路を壊していないか」)には合格している**
(変更前後で完全に同一の出力)が、Phase41〜48が前提にしていた「決定的8シーンの基準値」
自体が今回のセッションでは再現しないという、本フェーズと独立した重大な指摘として
別途記録しておく(次フェーズ以降で追跡が必要)。

---

## 作業2(最重要ゲート): 代理の妥当性検証

### 方法

`tools/phase49_cog_proxy_eval.py`(新規)で、`results/phase29_cand_g1.json`/
`g2.json`が記録した21シーン・130候補の`order`を`simulate_order(compute_cog_proxy=True)`
に再生し(新しい探索は一切なし、Phase35のreplica忠実度検証と同じ立て付け)、
`results/phase30_cand_eval.json`の本物のcog_score(Scorer実測)と突き合わせた。

### 結果

| 指標 | 値 |
|---|---:|
| 全候補プール(130件)を一括りにしたSpearman(参考、下記「注意」参照) | 0.839 |
| **シーン内順位のSpearman平均(21シーン、これが実際に効く指標)** | **0.544** |
| シーン内Spearmanの中央値 | 0.400 |
| Spearman<0.6のシーン数 | 11/21 |
| 既積みなし(16シーン)平均 | 0.523 |
| 既積みあり(5シーン)平均 | 0.611 |
| 代理値-実測値の平均差(系統バイアス) | −1.97pt(範囲 −14.60〜+8.62) |

**注意(重要)**: 130件を一括りにした「全体」のSpearman(0.839)は、シーンごとの
cogの基準レベルの違い(シーンAは総じて70台、シーンBは総じて50台、等)によって
**見かけ上高く**なる統計的アーティファクトであり、ε制約選択が実際に使う量
(**同じシーンの候補どうしを比較して順位付ける**)とは対応しない。指示の
「シーン内順位のSpearman相関」を文字どおり計算した結果が上表の0.544であり、
**この値が判定基準に照らすべき「全体」の数値**である(0.839は参考値として残すが、
判定には使わない)。

シーン別の内訳(21シーン):

| シーン | n | 既積み | Spearman |
|---|---:|---|---:|
| A01_1c_40_plain | 6 | なし | 0.928 |
| A02_1c_80_plain | 7 | なし | 0.929 |
| A03_1c_40_shelf | 6 | なし | 0.257 |
| A04_2c_80_noprio | 6 | なし | 0.257 |
| A05_2c_80_prio | 6 | なし | 0.371 |
| A06_1c_40_small | 6 | なし | **-0.143** |
| A07_1c_40_bulky | 10 | なし | 0.988 |
| A08_2c_140_extreme | 6 | なし | 0.714 |
| C01_1c_40_shelf | 6 | なし | 0.257 |
| C02_2c_55_shelfprio | 5 | なし | 0.400 |
| C03_2c_80_prio | 5 | なし | 0.300 |
| D01_A_1c_40_softheavy | 6 | なし | 0.943 |
| D02_A_1c_40_prioheavy_nocont | 6 | なし | 0.314 |
| D03_A_2c_60_prioheavy_cont | 6 | なし | 0.086 |
| D04_A_1c_40_flat | 6 | なし | 0.886 |
| D05_A_1c_40_tall | 6 | なし | 0.886 |
| P01_A_1c_pre6 | 7 | あり | 0.357 |
| P02_A_1c_pre10 | 6 | あり | 0.657 |
| P03_A_2c_pre8_prio | 6 | あり | 0.943 |
| P05_C_2c_pre8_shelfprio | 5 | あり | 0.100 |
| P06_A_1c_pre12_dense | 7 | あり | 1.000 |

### 散布の様子(系統的なズレ)

代理値は実測より平均−1.97pt低い(常時ではなく範囲は−14.6〜+8.6pt)。単純な加法補正
(オフセットを引く/足す)では順位の食い違いは直らない——A06(Spearman=−0.143)を見ると、
**本物で最良(cog_real=74.15)の候補が代理では最低順位**という、加法補正では
説明できない**順序の逆転**が起きている。個別に見ると、影シミュレータが「狙った着地点」
(plannerの出力そのまま)を使うのに対し、本物は実際にpybulletでその順序を再生した
結果の`packed_items`(=どのアイテムが最終的に「配置済み」として数えられたか)を使う
——**構築時の合法性判定と本物の物理判定で、置ける/置けないの判定が食い違う場合、
比較しているアイテム集合自体が違う**ことが、順位逆転の主因と考えられる(Phase6/Phase20
がfillで直面したsim-to-realギャップと同種の問題)。

### 判定

**シーン内Spearman平均 0.544 < 0.60 → この路線は再考。**

指示の判定基準に照らし、**作業3以降には進まない**。11/21シーン(過半数)で
Spearman<0.6、うち2シーン(A06, D03)はほぼ0か負相関。この代理をε制約選択に使うと、
「fillを落とさずcogを最大化したつもり」で選んだ候補が、**本物のcog評価では最良でない
候補である可能性が高い**——Phase34がρ=−0.321で確定させた「代理を山登りする手はすべて
失敗する」構図と同じ罠に、cog代理も落ちたと判断する。

---

## 作業7: 回帰テスト

`tools/test_cog_proxy.py`(新規)を追加し、git addした。

```
MYSOLVER_HARD_WALL_LIMIT=3000 PYTHONPATH=. python tools/test_cog_proxy.py
```

4ケース(手計算した既知入力での式の一致・空コンテナで100.0・[0,1]クリップ・
`simulate_order()`のタプル長が既定False=5要素/True=6要素で正しく切り替わること)を
確認済み(4/4 OK)。副作用なし(results/には書かない)。

---

## この時点で停止する

指示どおり、作業3(候補プール収集をρ-testから独立させる)には着手していない。
判定基準に照らして「この路線は再考」という結論であるため、次の指示を待つ。

`results/phase49_cog_proxy_eval.json`(130候補の代理値/実測値の生データ)も
参照用に保存した。

---

## 変更ファイル

- `agents/mysolver/simulate.py`(cog代理関数の追加、`simulate_order()`への
  `compute_cog_proxy`オプション引数追加。既定Falseで完全に無変更)
- `tools/phase49_cog_proxy_eval.py`(新規、作業2の検証スクリプト)
- `tools/test_cog_proxy.py`(新規、作業7の回帰テスト)
- `results/phase49_cog_proxy_eval.json`(作業2の実測データ)
- `results/phase49_report.md`(本ファイル)

`tools/scorer.py`・既存26シーンは無変更。risk_vol選択の置き換えも行っていない。
