from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path
from typing import Any, Dict, Iterable, List, Tuple
import json
import struct

from PIL import Image, ImageDraw
from shapely.geometry import Point, Polygon, box, MultiPolygon
from shapely.geometry.base import BaseGeometry
from shapely.ops import unary_union, triangulate

from nav_editor_model import Project, EditorItem


@dataclass
class NavCell:
    id: int
    vertices: List[Tuple[float, float]]
    neighbors: List[int]
    source_region_labels: List[str]


@dataclass
class ExportResult:
    project_json: Path
    nav_json: Path
    nav_bin: Path
    preview_png: Path


def item_to_geometry(item: EditorItem, circle_resolution: int) -> BaseGeometry:
    if item.kind == 'rect':
        return box(item.x, item.y, item.x + item.w, item.y + item.h)
    if item.kind == 'circle':
        r = min(item.w, item.h) / 2.0
        cx = item.x + item.w / 2.0
        cy = item.y + item.h / 2.0
        return Point(cx, cy).buffer(r, resolution=max(8, circle_resolution // 4))
    if item.kind == 'point':
        return Point(item.x, item.y)
    raise ValueError(f'Unsupported item kind: {item.kind}')


def _quantize_polygon(poly: Polygon, tolerance: float) -> Polygon:
    return poly.simplify(tolerance, preserve_topology=True)


def _is_convex(poly: Polygon, eps: float = 1e-6) -> bool:
    coords = list(poly.exterior.coords)
    if len(coords) < 4:
        return False
    coords = coords[:-1]
    sign = 0
    n = len(coords)
    for i in range(n):
        x1, y1 = coords[i]
        x2, y2 = coords[(i + 1) % n]
        x3, y3 = coords[(i + 2) % n]
        cross = (x2 - x1) * (y3 - y2) - (y2 - y1) * (x3 - x2)
        if abs(cross) <= eps:
            continue
        current = 1 if cross > 0 else -1
        if sign == 0:
            sign = current
        elif sign != current:
            return False
    return True


def _greedy_merge_convex(polys: List[Polygon], walkable: BaseGeometry) -> List[Polygon]:
    merged = polys[:]
    changed = True
    area_eps = 1e-4
    while changed:
        changed = False
        for i in range(len(merged)):
            if changed:
                break
            for j in range(i + 1, len(merged)):
                a = merged[i]
                b = merged[j]
                shared = a.boundary.intersection(b.boundary)
                if shared.length <= 1e-3:
                    continue
                union = a.union(b)
                if union.geom_type != 'Polygon':
                    continue
                if not _is_convex(union):
                    continue
                if abs(union.area - (a.area + b.area)) > area_eps:
                    continue
                if not walkable.buffer(1e-6).covers(union):
                    continue
                merged[i] = union
                merged.pop(j)
                changed = True
                break
    return merged


def _ensure_ccw(vertices: List[Tuple[float, float]]) -> List[Tuple[float, float]]:
    area2 = 0.0
    n = len(vertices)
    for i in range(n):
        x1, y1 = vertices[i]
        x2, y2 = vertices[(i + 1) % n]
        area2 += (x1 * y2) - (x2 * y1)
    if area2 < 0:
        return list(reversed(vertices))
    return vertices


def _int_vertices(vertices: List[Tuple[float, float]]) -> List[Tuple[int, int]]:
    ints = [(int(round(x)), int(round(y))) for x, y in vertices]
    cleaned: List[Tuple[int, int]] = []
    for v in ints:
        if not cleaned or cleaned[-1] != v:
            cleaned.append(v)
    if len(cleaned) >= 2 and cleaned[0] == cleaned[-1]:
        cleaned.pop()
    return cleaned


def _shared_edge_length(a: Polygon, b: Polygon) -> float:
    shared = a.boundary.intersection(b.boundary)
    return float(shared.length)


def build_walkable_union(project: Project) -> Tuple[BaseGeometry, Dict[str, BaseGeometry]]:
    geoms: List[BaseGeometry] = []
    by_label: Dict[str, BaseGeometry] = {}

    for item in project.items:
        if item.category != 'nav':
            continue
        if item.role == 'blocked':
            continue
        if item.kind == 'point':
            continue

        g = item_to_geometry(item, project.circle_resolution)
        geoms.append(g)
        by_label[item.label] = g

    if not geoms:
        return Polygon(), {}

    union = unary_union(geoms)
    union = _quantize_polygon(union, project.simplify_tolerance)
    return union, by_label


def build_navmesh_cells(project: Project) -> Tuple[List[NavCell], List[Polygon], BaseGeometry]:
    walkable, by_label = build_walkable_union(project)
    if walkable.is_empty:
        return [], [], walkable

    pieces: Iterable[Polygon]
    if isinstance(walkable, Polygon):
        pieces = [walkable]
    elif isinstance(walkable, MultiPolygon):
        pieces = list(walkable.geoms)
    else:
        pieces = []

    triangles: List[Polygon] = []
    for piece in pieces:
        raw_tris = triangulate(piece)
        for tri in raw_tris:
            clipped = tri.intersection(piece)
            if clipped.is_empty:
                continue
            if clipped.geom_type == 'Polygon' and clipped.area > 1.0:
                triangles.append(clipped)

    convex_cells = _greedy_merge_convex(triangles, walkable)

    cells: List[NavCell] = []
    for idx, poly in enumerate(convex_cells):
        coords = [(float(x), float(y)) for x, y in list(poly.exterior.coords)[:-1]]
        coords = _ensure_ccw(coords)
        labels: List[str] = []
        for label, region_geom in by_label.items():
            if region_geom.intersects(poly.centroid):
                labels.append(label)
        cells.append(NavCell(id=idx, vertices=coords, neighbors=[], source_region_labels=labels))

    for i in range(len(convex_cells)):
        for j in range(i + 1, len(convex_cells)):
            if _shared_edge_length(convex_cells[i], convex_cells[j]) > 1e-3:
                cells[i].neighbors.append(j)
                cells[j].neighbors.append(i)

    for cell in cells:
        cell.neighbors = sorted(set(cell.neighbors))

    return cells, convex_cells, walkable


def build_region_graph(project: Project) -> List[Dict[str, Any]]:
    entries: List[Dict[str, Any]] = []

    nav_items = [
        item for item in project.items
        if item.category == 'nav' and item.role != 'blocked' and item.kind != 'point'
    ]
    geoms = {item.label: item_to_geometry(item, project.circle_resolution) for item in nav_items}
    labels = list(geoms.keys())

    for item in nav_items:
        geom = geoms[item.label]
        centroid = geom.representative_point()
        neighbors: List[str] = []
        for other_label in labels:
            if other_label == item.label:
                continue
            if geom.buffer(project.snap_distance * 0.55).intersects(geoms[other_label]):
                neighbors.append(other_label)
        entries.append({
            'label': item.label,
            'role': item.role,
            'kind': item.kind,
            'center': [float(centroid.x), float(centroid.y)],
            'neighbors': sorted(set(neighbors)),
        })

    return entries


def serialize_item(item: EditorItem) -> Dict[str, Any]:
    payload: Dict[str, Any] = {
        'id': item.id,
        'label': item.label,
        'category': item.category,
        'kind': item.kind,
        'x': round(float(item.x), 2),
        'y': round(float(item.y), 2),
    }

    if item.kind == 'point':
        payload['position'] = [round(float(item.x), 2), round(float(item.y), 2)]
    else:
        payload['w'] = round(float(item.w), 2)
        payload['h'] = round(float(item.h), 2)
        payload['center'] = [
            round(float(item.x + item.w / 2.0), 2),
            round(float(item.y + item.h / 2.0), 2),
        ]

    if item.category == 'nav':
        payload['role'] = item.role

    return payload


def export_preview_png(project: Project, walkable: BaseGeometry, out_path: Path) -> None:
    img = Image.new('RGBA', (project.canvas_width, project.canvas_height), (28, 30, 34, 255))

    draw = ImageDraw.Draw(img, 'RGBA')

    for item in project.items:
        if item.category == 'nav':
            fill = (70, 110, 165, 90) if item.role != 'blocked' else (150, 60, 60, 110)
            outline = (200, 220, 240, 255) if item.role != 'blocked' else (240, 150, 150, 255)
        elif item.category == 'trigger':
            fill = (190, 140, 40, 80)
            outline = (255, 220, 120, 255)
        elif item.category == 'object':
            fill = (90, 150, 90, 110)
            outline = (170, 240, 170, 255)
        else:
            fill = (0, 0, 0, 0)
            outline = (255, 255, 0, 255)

        if item.kind == 'rect':
            bbox = [item.x, item.y, item.x + item.w, item.y + item.h]
            draw.rectangle(bbox, fill=fill, outline=outline, width=2)
        elif item.kind == 'circle':
            bbox = [item.x, item.y, item.x + item.w, item.y + item.h]
            draw.ellipse(bbox, fill=fill, outline=outline, width=2)
        elif item.kind == 'point':
            px, py = item.x, item.y
            draw.line((px - 8, py, px + 8, py), fill=outline, width=2)
            draw.line((px, py - 8, px, py + 8), fill=outline, width=2)

    def draw_poly(poly: Polygon) -> None:
        pts = [(float(x), float(y)) for x, y in poly.exterior.coords]
        draw.line(pts + [pts[0]], fill=(120, 255, 160, 255), width=3)

    if not walkable.is_empty:
        if isinstance(walkable, Polygon):
            draw_poly(walkable)
        elif isinstance(walkable, MultiPolygon):
            for poly in walkable.geoms:
                draw_poly(poly)
    img = img.convert('RGB')
    img.save(out_path, format='BMP')


def export_navmesh_binary(cells: List[NavCell], out_path: Path) -> None:
    with open(out_path, 'wb') as f:
        f.write(struct.pack('<i', len(cells)))
        for cell in cells:
            verts = _int_vertices(cell.vertices)
            if len(verts) < 3:
                raise ValueError(f'Nav cell {cell.id} has fewer than 3 vertices after quantization.')
            f.write(struct.pack('<i', len(verts)))
            for x, y in verts:
                f.write(struct.pack('<ii', x, y))
            f.write(struct.pack('<i', len(cell.neighbors)))
            for n in cell.neighbors:
                f.write(struct.pack('<i', int(n)))


def export_all(project: Project, output_dir: str, project_path: str | None = None) -> ExportResult:
    out_dir = Path(output_dir)
    out_dir.mkdir(parents=True, exist_ok=True)

    walkable_cells, _, walkable_geom = build_navmesh_cells(project)
    region_graph = build_region_graph(project)

    triggers = [serialize_item(item) for item in project.items if item.category == 'trigger']
    objects = [serialize_item(item) for item in project.items if item.category == 'object']
    points = [serialize_item(item) for item in project.items if item.category == 'point']
    nav_items = [serialize_item(item) for item in project.items if item.category == 'nav']

    project_json = Path(project_path) if project_path else out_dir / 'map_project.json'
    nav_json = out_dir / 'navmesh.json'
    nav_bin = out_dir / 'navmesh_polygons.nav'
    preview_png = out_dir / 'map_preview.bmp'

    from nav_editor_model import save_project
    save_project(str(project_json), project)

    nav_payload = {
        'canvas': {
            'width': project.canvas_width,
            'height': project.canvas_height,
            'grid_size': project.grid_size,
        },
        'nav_items': nav_items,
        'regions': region_graph,
        'cells': [
            {
                'id': cell.id,
                'vertices': [[round(x, 2), round(y, 2)] for x, y in cell.vertices],
                'neighbors': cell.neighbors,
                'source_region_labels': cell.source_region_labels,
            }
            for cell in walkable_cells
        ],
        'triggers': triggers,
        'objects': objects,
        'points': points,
    }

    with open(nav_json, 'w', encoding='utf-8') as f:
        json.dump(nav_payload, f, indent=2)

    export_navmesh_binary(walkable_cells, nav_bin)
    export_preview_png(project, walkable_geom, preview_png)
    return ExportResult(project_json=project_json, nav_json=nav_json, nav_bin=nav_bin, preview_png=preview_png)