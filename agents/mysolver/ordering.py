"""optimize() 用の積付順序決定。「重い/大きい/優先を先(下段)に、ソフトを後(上段)に」の決定的ソート。"""


def order_items(item_list: list[dict]) -> list[int]:
    def volume_of(item: dict) -> float:
        return item.get('volume', item['length'] * item['width'] * item['height'])

    def sort_key(item: dict):
        return (
            0 if item.get('is_prioritized', False) else 1,   # 優先荷物を先に
            1 if item.get('is_soft', False) else 0,           # ソフトは後(上段)に
            -volume_of(item),                                  # 体積が大きい順
            -item.get('mass', 0.0),                            # 質量が重い順
        )

    sorted_items = sorted(item_list, key=sort_key)
    return [item['index'] for item in sorted_items]
