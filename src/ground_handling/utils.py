import math
import numpy as np


ORNS = [
    [0, 0, 0],                 # 0: 寸法 [l, w, h] (そのまま)
    [math.pi/2, 0, 0],         # 1: 寸法 [l, h, w] (X軸で90度回転)
    [0, math.pi/2, 0],         # 2: 寸法 [h, w, l] (Y軸で90度回転)
    [0, 0, math.pi/2],         # 3: 寸法 [w, l, h] (Z軸で90度回転)
    [0, math.pi/2, math.pi/2], # 4: 寸法 [w, h, l]
    [math.pi/2, 0, math.pi/2]  # 5: 寸法 [h, l, w] 
]


def get_half_ext(original_lwh: list[float], orn_idx: int) -> list[float]:
    if orn_idx == 0:
        return [original_lwh[0]/2, original_lwh[1]/2, original_lwh[2]/2]
    elif orn_idx==1:
        return [original_lwh[0]/2, original_lwh[2]/2, original_lwh[1]/2]
    elif orn_idx==2:
        return [original_lwh[2]/2, original_lwh[1]/2, original_lwh[0]/2]
    elif orn_idx==3:
        return [original_lwh[1]/2, original_lwh[0]/2, original_lwh[2]/2]
    elif orn_idx==4:
        return [original_lwh[1]/2, original_lwh[2]/2, original_lwh[0]/2]
    elif orn_idx==5:
        return [original_lwh[2]/2, original_lwh[0]/2, original_lwh[1]/2]
    else:
        return [original_lwh[0]/2, original_lwh[1]/2, original_lwh[2]/2]


def center_xy(poly, cx, cy):
    return [[x - cx, y - cy] for x, y in poly]


def aff(origin:list[list[float]], rot: list[list[float]], intercept: tuple[float, float, float]):
    aff_transformed = np.dot(np.array(origin), np.array(rot).T) + intercept

    return list(map(tuple, aff_transformed))


def line_intersection_2d(p1, d1, p2, d2):
    """2直線 p1+t*d1, p2+s*d2 の交点"""
    cross = d1[0] * d2[1] - d1[1] * d2[0]
    if abs(cross) < 1e-12:
        raise ValueError("offset failed: parallel lines")
    dx = p2[0] - p1[0]
    dy = p2[1] - p1[1]
    t = (dx * d2[1] - dy * d2[0]) / cross
    return [p1[0] + t * d1[0], p1[1] + t * d1[1]]


def offset_convex_polygon_ccw(poly, offset):
    """
    CCW順の凸多角形を、各辺から offset だけ内側へオフセット
    """
    lines = []
    n = len(poly)

    for i in range(n):
        a = poly[i]
        b = poly[(i + 1) % n]
        ex = b[0] - a[0]
        ey = b[1] - a[1]
        L = math.hypot(ex, ey)
        if L < 1e-12:
            raise ValueError("invalid polygon")

        # CCW polygon の内向き法線
        nx = -ey / L
        ny =  ex / L

        shifted_point = [a[0] + nx * offset, a[1] + ny * offset]
        direction = [ex, ey]
        lines.append((shifted_point, direction))

    inner = []
    for i in range(n):
        p1, d1 = lines[i - 1]
        p2, d2 = lines[i]
        inner.append(line_intersection_2d(p1, d1, p2, d2))
    return inner


def triangulate_fan(indices, reverse=False):
    tris = []
    for i in range(1, len(indices) - 1):
        if reverse:
            tris.append([indices[0], indices[i + 1], indices[i]])
        else:
            tris.append([indices[0], indices[i], indices[i + 1]])
    return tris


def write_open_cut_corner_cup_obj(
    file_path: str,
    width: float = 0.6,          # 長方形の幅
    height: float = 0.6,         # 長方形の高さ
    cut_x: float = 0.12,         # 上辺側の切り落とし量
    cut_y: float = 0.12,         # 左辺側の切り落とし量
    depth: float = 0.3,          # 柱の奥行き（上面から底面まで）
    wall: float = 0.01,          # 壁厚
    bottom: float = 0.01         # 底厚
    ) -> tuple[list[list[float]], list[list[float]]]:
    """
    左上角を切り落とした五角形断面の、中空・上面開口・底あり形状をOBJ出力
    断面はXY平面、奥行き方向はZ
    """

    if wall <= 0 or bottom <= 0:
        raise ValueError("wall, bottom must be > 0")
    if depth <= bottom:
        raise ValueError("depth must be > bottom")
    if cut_x <= 0 or cut_y <= 0:
        raise ValueError("cut_x, cut_y must be > 0")
    if cut_x >= width or cut_y >= height:
        raise ValueError("cut is too large")

    # 外形（CCW）
    outer2d = [
        [cut_x, 0.0],
        [width, 0.0],
        [width, height],
        [0.0, height],
        [0.0, cut_y],
    ]

    inner2d = offset_convex_polygon_ccw(outer2d, wall)

    # 中心付近に寄せる（回転時に扱いやすくする）
    cx = width / 2.0
    cy = height / 2.0


    outer2d = center_xy(outer2d, cx, cy)
    inner2d = center_xy(inner2d, cx, cy)

    z0 = -depth / 2.0            # 外底
    z1 =  depth / 2.0            # 上端
    zi = z0 + bottom             # 内底

    vertices = []

    # 外底
    for x, y in outer2d:
        vertices.append([x, y, z0])

    # 外上端
    for x, y in outer2d:
        vertices.append([x, y, z1])

    # 内底
    for x, y in inner2d:
        vertices.append([x, y, zi])

    # 内上端
    for x, y in inner2d:
        vertices.append([x, y, z1])

    n = 5
    ob = list(range(0, n))         # outer bottom
    ot = list(range(n, 2 * n))     # outer top
    ib = list(range(2 * n, 3 * n)) # inner bottom
    it = list(range(3 * n, 4 * n)) # inner top

    faces = []

    # 1) 外底面（下面）
    faces += triangulate_fan(ob, reverse=True)

    # 2) 外側面
    for i in range(n):
        j = (i + 1) % n
        faces.append([ob[i], ob[j], ot[j]])
        faces.append([ob[i], ot[j], ot[i]])

    # 3) 内底面（コップの底の内側）
    faces += triangulate_fan(ib, reverse=False)

    # 4) 内側面
    for i in range(n):
        j = (i + 1) % n
        faces.append([ib[i], it[j], ib[j]])
        faces.append([ib[i], it[i], it[j]])

    # 5) 上端の rim（縁）
    # これで「上面は開いているが、壁厚のぶんの縁はある」状態になる
    for i in range(n):
        j = (i + 1) % n
        faces.append([ot[i], ot[j], it[j]])
        faces.append([ot[i], it[j], it[i]])


    with open(file_path, "w", encoding="utf-8") as f:
        for v in vertices:
            f.write(f"v {v[0]:.8f} {v[1]:.8f} {v[2]:.8f}\n")
        for tri in faces:
            a, b, c = [idx + 1 for idx in tri]  # OBJは1始まり
            f.write(f"f {a} {b} {c}\n")
    m = len(inner2d)
    points = []
    n_vecs = []
    for i in range(m):
        ex = inner2d[(i+1)%m][0]-inner2d[i][0]
        ey = inner2d[(i+1)%m][1]-inner2d[i][1]
        ne = math.hypot(ex, ey)
        n_vec = [ey/ne, -ex/ne, 0]
        points.append(inner2d[i]+[0])
        n_vecs.append(n_vec)
    points.extend([[0,0,z1],[0,0,zi]])
    n_vecs.extend([[0,0,1],[0,0,-1]])

    return points, n_vecs


def write_corner_lid_obj(
    file_path: str,
    width: float = 0.6,
    height: float = 0.3,
    cut_x: float = 0.12,
    cut_y: float = 0.12,
    depth: float = 0.25,
    lid_thickness: float = 0.005
    ):
    """
    開口の欠け角を塞いで、見かけ上の開口を長方形にするための
    薄い蓋パーツ（独立オブジェクト）
    """

    outer2d = [
        [cut_x, 0.0],
        [cut_x, height],
        [0.0, height],
        [0.0, cut_y],
    ]

    cx = width / 2.0
    cy = height / 2.0

    outer2d = center_xy(outer2d, cx, cy)

    z_top_open = depth / 2.0
    z0 = z_top_open
    z1 = z_top_open + lid_thickness

    vertices = [
        [outer2d[0][0], outer2d[0][1], z0],  # 0
        [outer2d[1][0], outer2d[1][1], z0],  # 1
        [outer2d[2][0], outer2d[2][1], z0],  # 2
        [outer2d[3][0], outer2d[3][1], z0],  # 3
        [outer2d[0][0], outer2d[0][1], z1],  # 4
        [outer2d[1][0], outer2d[1][1], z1],  # 5
        [outer2d[2][0], outer2d[2][1], z1],  # 6
        [outer2d[3][0], outer2d[3][1], z1],  # 7
    ]

    faces = [
        # 下面
        [0, 2, 1],
        [0, 3, 2],
        # 上面
        [4, 5, 6],
        [4, 6, 7],

        # # 側面
        [0, 1, 4], [4, 1, 5],
        [1, 2, 6], [1, 6, 5],
        [2, 3, 7], [7, 6, 2],
        [3, 0, 4], [4, 7, 3]
    ]

    with open(file_path, "w", encoding="utf-8") as f:
        for v in vertices:
            f.write(f"v {v[0]:.8f} {v[1]:.8f} {v[2]:.8f}\n")
        for tri in faces:
            a, b, c = [i + 1 for i in tri]
            f.write(f"f {a} {b} {c}\n")
