"""WBC maximal-space プリミティブの単体テスト(純ロジック、pybullet 不要)。

実行: python3 tools/test_maximal_space.py
"""
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from agents.mysolver import wallbuild as wb

_fail = 0


def check(cond, msg):
    global _fail
    print(('  OK  ' if cond else '  FAIL') + '  ' + msg)
    if not cond:
        _fail += 1


def vol(s):
    return max(0.0, s[4] - s[1]) * max(0.0, s[5] - s[2]) * max(0.0, s[6] - s[3])


def overlaps(a, b):
    return (a[1] < b[4] - 1e-9 and b[1] < a[4] - 1e-9
            and a[2] < b[5] - 1e-9 and b[2] < a[5] - 1e-9
            and a[3] < b[6] - 1e-9 and b[3] < a[6] - 1e-9)


print('=== _split_space: 中央 box ===')
S = (0, 0.0, 0.0, 0.0, 10.0, 10.0, 10.0)
box = (0, 3.0, 3.0, 3.0, 7.0, 7.0, 7.0)
slabs = wb._split_space(S, box)
check(len(slabs) == 6, f'6 slab 生成 (got {len(slabs)})')
check(all(not overlaps(sl, box) for sl in slabs), 'どの slab も box と重ならない')
# 空間内の box 外の任意点が、少なくとも1 slab に含まれること(被覆)
import itertools
pts_out = [(x, y, z) for x in (1, 5, 9) for y in (1, 5, 9) for z in (1, 5, 9)
           if not (3 < x < 7 and 3 < y < 7 and 3 < z < 7)]
covered = all(any(sl[1] <= x <= sl[4] and sl[2] <= y <= sl[5] and sl[3] <= z <= sl[6]
                  for sl in slabs) for (x, y, z) in pts_out)
check(covered, 'box 外の格子点は全て slab に被覆される')

print('=== _split_space: box が空間外 ===')
check(wb._split_space(S, (0, 20, 20, 20, 25, 25, 25)) == [S], '重ならない box → [S] 不変')

print('=== _split_space: box が空間より大きい(全内包)===')
check(wb._split_space(S, (0, -1, -1, -1, 11, 11, 11)) == [], '空間を覆う box → []')

print('=== _prune: 内包の除去 ===')
big = (0, 0.0, 0.0, 0.0, 10.0, 10.0, 10.0)
small = (0, 1.0, 1.0, 1.0, 5.0, 5.0, 5.0)
p = wb._prune([big, small])
check(p == [big], f'small(内包) は除去 → [big] (got {p})')

print('=== _prune: 同一タプル重複 ===')
p = wb._prune([big, big])
check(p == [big], f'重複は1つに (got {p})')

print('=== _prune: 交差(相互非内包)は両方残す ===')
a = (0, 0.0, 0.0, 0.0, 6.0, 10.0, 10.0)
b = (0, 4.0, 0.0, 0.0, 10.0, 10.0, 10.0)
p = wb._prune([a, b])
check(len(p) == 2, f'交差は両方残る (got {len(p)})')

print('=== collapse シナリオ: 大空間を 3 個の非重複 box で順次分割 ===')
wb.WBC_MIN_SPACE = 0.01
Sset = [(0, 0.0, 0.0, 0.0, 2.0, 2.0, 2.0)]
boxes = [(0, 0.0, 0.0, 0.0, 0.4, 0.4, 0.4),
         (0, 0.5, 0.0, 0.0, 0.9, 0.4, 0.4),
         (0, 0.0, 0.5, 0.0, 0.4, 0.9, 0.4)]
for k, bx in enumerate(boxes):
    nxt = []
    for s in Sset:
        nxt.extend(wb._split_space(s, bx))
    Sset = wb._prune(nxt)
    big_left = max(vol(s) for s in Sset)
    print(f'  after box{k}: |S|={len(Sset)}  max_space_vol={big_left:.3f}')
    check(len(Sset) >= 1, f'box{k} 後も |S|>=1')
    check(big_left > 1.0, f'box{k} 後も体積 1.0 超の空間が残る (元 8.0 から 3 個 0.064 を引いた)')

print()
if _fail:
    print(f'!!! {_fail} 件 FAIL')
    sys.exit(1)
print('全テスト OK')
