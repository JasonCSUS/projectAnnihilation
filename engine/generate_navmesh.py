import cv2 
import numpy as np
import struct
import os
import math
from shapely.geometry import Polygon, MultiPolygon
from shapely.ops import unary_union, triangulate

# ============================================================
# Configuration
# ============================================================
NAVMESH_OUTPUT_FILE = "../assets/navmesh_polygons.nav"

SKIP_APPROX = True
SKIP_CONVEX_HULL = True

SNAP_VALUE = 2
SHAPELY_SIMPLIFY_TOLERANCE = 2.0

# Amount (in pixels) to expand polygons before union, then shrink back.
mergeBuffer = 2.0

maps = [
    ("../assets/map1_collision.bmp", 0,    0),
    ("../assets/map2_collision.bmp", 2000, 0)
]

def snap_point(pt, snap=SNAP_VALUE):
    return (round(pt[0] / snap) * snap, round(pt[1] / snap) * snap)

def make_edge(p1, p2, snap=SNAP_VALUE):
    p1_rounded = snap_point(p1, snap)
    p2_rounded = snap_point(p2, snap)
    return (p1_rounded, p2_rounded) if p1_rounded < p2_rounded else (p2_rounded, p1_rounded)

def draw_hierarchy(mask, contours, hierarchy, idx, color):
    cv2.drawContours(mask, contours, idx, color, thickness=-1)
    next_idx = hierarchy[idx][0]
    child_idx = hierarchy[idx][2]
    if child_idx != -1:
        opposite_color = 0 if color == 255 else 255
        draw_hierarchy(mask, contours, hierarchy, child_idx, opposite_color)
    if next_idx != -1:
        draw_hierarchy(mask, contours, hierarchy, next_idx, color)

def shapely_simplify_polygon(pts, tolerance=SHAPELY_SIMPLIFY_TOLERANCE):
    if len(pts) < 3:
        return pts
    poly = Polygon(pts)
    if not poly.is_valid:
        poly = poly.buffer(0)
        if not poly.is_valid:
            return pts
    poly_simpl = poly.simplify(tolerance, preserve_topology=True)
    if poly_simpl.is_empty:
        return pts
    ext_coords = list(poly_simpl.exterior.coords)
    simplified_pts = [(int(round(x)), int(round(y))) for (x, y) in ext_coords]
    return simplified_pts

def is_convex_polygon(poly):
    n = len(poly)
    if n < 3:
        return False
    sign = None
    for i in range(n):
        dx1 = poly[(i+1) % n][0] - poly[i][0]
        dy1 = poly[(i+1) % n][1] - poly[i][1]
        dx2 = poly[(i+2) % n][0] - poly[(i+1) % n][0]
        dy2 = poly[(i+2) % n][1] - poly[(i+1) % n][1]
        cross = dx1 * dy2 - dy1 * dx2
        if cross != 0:
            current_sign = cross > 0
            if sign is None:
                sign = current_sign
            elif sign != current_sign:
                return False
    return True

def decompose_polygon(poly):
    shp_poly = Polygon(poly)
    if not shp_poly.is_valid:
        shp_poly = shp_poly.buffer(0)
    raw_tris = triangulate(shp_poly)
    final_tris = []
    for tri in raw_tris:
        clipped = tri.intersection(shp_poly)
        if clipped.is_empty or clipped.area <= 0:
            continue
        if clipped.geom_type == "Polygon":
            coords = list(clipped.exterior.coords)[:-1]
            final_tris.append([(int(round(x)), int(round(y))) for (x, y) in coords])
        elif clipped.geom_type == "MultiPolygon":
            for subpoly in clipped.geoms:
                if not subpoly.is_empty and subpoly.area > 0:
                    sub_coords = list(subpoly.exterior.coords)[:-1]
                    final_tris.append([(int(round(x)), int(round(y))) for (x, y) in sub_coords])
    return final_tris

def process_collision_bmp(map_path, offset_x, offset_y):
    if not os.path.exists(map_path):
        print("Image not found:", map_path)
        return []
    img = cv2.imread(map_path, cv2.IMREAD_GRAYSCALE)
    if img is None:
        print("Failed to load image:", map_path)
        return []
    ret, binary = cv2.threshold(img, 128, 255, cv2.THRESH_BINARY)
    if ret is False:
        print("Thresholding failed for", map_path)
        return []
    contours, hierarchy = cv2.findContours(binary, cv2.RETR_CCOMP, cv2.CHAIN_APPROX_SIMPLE)
    if hierarchy is None:
        print(f"No contours found in {map_path}")
        return []
    h, w = binary.shape
    mask = np.zeros((h, w), dtype=np.uint8)
    for i in range(len(contours)):
        if hierarchy[0][i][3] == -1:
            draw_hierarchy(mask, contours, hierarchy[0], i, 255)
    final_contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)
    print(f"After hole subtraction, found {len(final_contours)} outer contours in {map_path}")
    polygon_list = []
    for cnt in final_contours:
        if SKIP_APPROX:
            approx = cnt
        else:
            epsilon = 0.01 * cv2.arcLength(cnt, True)
            approx = cv2.approxPolyDP(cnt, epsilon, True)
        if len(approx) < 3:
            continue
        pts = [(pt[0][0] + offset_x, pt[0][1] + offset_y) for pt in approx]
        pts_simpl = shapely_simplify_polygon(pts, tolerance=SHAPELY_SIMPLIFY_TOLERANCE)
        if len(pts_simpl) < 3:
            continue
        polygon_list.append(pts_simpl)
    return polygon_list

# ============================================================
# Main script
# ============================================================
all_polygons = []
for map_path, offset_x, offset_y in maps:
    tile_polys = process_collision_bmp(map_path, offset_x, offset_y)
    all_polygons.extend(tile_polys)

# Step 1: Convert to Shapely polygons for the buffer-union trick
shapely_polys = []
for poly in all_polygons:
    sp = Polygon(poly)
    if not sp.is_valid:
        sp = sp.buffer(0)
    shapely_polys.append(sp)

# Step 2: Buffer out by mergeBuffer so near edges overlap, then union
buffered_out = [p.buffer(mergeBuffer) for p in shapely_polys if not p.is_empty]
merged_union = unary_union(buffered_out)

# Step 3: Buffer in by -mergeBuffer to return to original boundary
shrunk_union = merged_union.buffer(-mergeBuffer)

# Step 4: Break the union back into individual polygons
final_shapely_polys = []
if shrunk_union.is_empty:
    # No polygons left after merge
    pass
elif shrunk_union.geom_type == "Polygon":
    final_shapely_polys = [shrunk_union]
elif shrunk_union.geom_type == "MultiPolygon":
    for g in shrunk_union.geoms:
        if not g.is_empty:
            final_shapely_polys.append(g)

# Step 5: Decompose any polygon that is not convex
final_polygons = []
for sp in final_shapely_polys:
    coords = list(sp.exterior.coords)[:-1]
    base_poly = [(int(round(x)), int(round(y))) for x, y in coords]
    # Optional: run Shapely simplify again if you like
    # Decompose if non-convex
    if is_convex_polygon(base_poly):
        final_polygons.append(base_poly)
    else:
        decomposed = decompose_polygon(base_poly)
        final_polygons.extend(decomposed)

# Step 6: Build neighbor relationships
neighbors = {i: set() for i in range(len(final_polygons))}
edge_to_polys = {}
for idx, poly in enumerate(final_polygons):
    num_vertices = len(poly)
    for j in range(num_vertices):
        e = make_edge(poly[j], poly[(j + 1) % num_vertices], snap=SNAP_VALUE)
        edge_to_polys.setdefault(e, []).append(idx)

for edge, poly_indices in edge_to_polys.items():
    if len(poly_indices) > 1:
        for i in poly_indices:
            for j in poly_indices:
                if i != j:
                    neighbors[i].add(j)

# Step 7: Save navmesh
with open(NAVMESH_OUTPUT_FILE, "wb") as f:
    num_polys = len(final_polygons)
    f.write(struct.pack("i", num_polys))
    print("Saving", num_polys, "polygons to", NAVMESH_OUTPUT_FILE)
    for idx, poly in enumerate(final_polygons):
        f.write(struct.pack("i", len(poly)))
        for (x, y) in poly:
            f.write(struct.pack("ii", x, y))
        neigh = list(neighbors[idx])
        f.write(struct.pack("i", len(neigh)))
        for n_idx in neigh:
            f.write(struct.pack("i", n_idx))

print("Navmesh generation complete. File saved to", NAVMESH_OUTPUT_FILE)
