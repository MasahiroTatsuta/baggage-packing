"""
optimize() 用の積付順序決定。

以前は「優先荷物を最優先」で並べていたが、それだと中型の優先荷物が
先に床の一部を断片的に占有してしまい、後から来る大型荷物の搬入経路
(コンテナ開口部の限られたX範囲を通ってY方向へ押し込む)を塞いでしまい、
早期に置き場所が尽きる問題があった。

現在の planner.py は「優先(ソフト)荷物の上に非優先(非ソフト)荷物を
乗せない」というハード制約を候補生成時に強制するため、置く順番に
関わらず下敷き(placement/soft_item スコアの減点)は発生しない。
そのため順序は純粋に「詰めやすさ」を優先してよく、大きく重いものを
土台(下段)に先に置き、ソフト・小物を後段(上段・隙間埋め)に回す。
"""


def order_items(item_list: list[dict]) -> list[int]:
    def volume_of(item: dict) -> float:
        return item.get('volume', item['length'] * item['width'] * item['height'])

    def sort_key(item: dict):
        return (
            1 if item.get('is_soft', False) else 0,             # ソフトは体積が大きくても後段(隙間埋め)に回す。
                                                                  # 先に置くと(潰れにくいぶん)床の要所を早期に
                                                                  # 占有してしまい、後続の大型荷物の搬入経路を
                                                                  # 塞いでしまうため。
            -volume_of(item),                                   # 体積が大きいものを土台として先に
            -item.get('mass', 0.0),                             # 同体積なら重いものを先に(cog/stability上有利)
            0 if item.get('is_prioritized', False) else 1,      # 同条件なら優先荷物をわずかに先に(専用コンテナが埋まる前に配置)
        )

    sorted_items = sorted(item_list, key=sort_key)
    return [item['index'] for item in sorted_items]
