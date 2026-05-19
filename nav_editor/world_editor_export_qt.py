from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path
from typing import Any, Dict, Iterable, List, Tuple
import json
import re
import struct

from PIL import Image, ImageDraw
from shapely.geometry import Point, Polygon, box, MultiPolygon, GeometryCollection
from shapely.geometry.base import BaseGeometry
from shapely.ops import unary_union, triangulate

from world_editor_model_qt import Project, EditorItem


@dataclass
class NavCell:
    id: int
    vertices: List[Tuple[float, float]]
    neighbors: List[int]
    source_region_labels: List[str]
    source_region_ids: List[str]
    blocked: bool = False
    blocked_by_labels: List[str] | None = None
    blocked_by_ids: List[str] | None = None


@dataclass
class ExportResult:
    project_json: Path
    entities_json: Path
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


def _quantize_geometry(geom: BaseGeometry, tolerance: float) -> BaseGeometry:
    return geom.simplify(tolerance, preserve_topology=True)


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


def _greedy_merge_convex(polys: List[Polygon], allowed_region: BaseGeometry) -> List[Polygon]:
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
                if not allowed_region.buffer(1e-6).covers(union):
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


def _sanitize_runtime_token(label: str) -> str:
    token = re.sub(r'[^A-Za-z0-9_]+', '_', label.strip())
    token = re.sub(r'_+', '_', token).strip('_')
    return token or 'item'


def _infer_blocker_type(item: EditorItem) -> str:
    label = item.label.strip().lower()
    if 'spawner' in label:
        return 'spawner'
    if 'blockade' in label or 'gate' in label or 'door' in label or 'barrier' in label:
        return 'blockade'
    return 'unknown'


def build_runtime_item_ids(project: Project) -> Dict[int, str]:
    counts: Dict[str, int] = {}
    for item in project.items:
        counts[item.label] = counts.get(item.label, 0) + 1

    ids: Dict[int, str] = {}
    for item in project.items:
        base = _sanitize_runtime_token(item.label if item.label else f'item_{item.id}')
        if counts.get(item.label, 0) > 1:
            ids[item.id] = f'{base}__{item.id}'
        else:
            ids[item.id] = base
    return ids


def _collect_nav_geometries(
    project: Project,
) -> Tuple[List[Tuple[EditorItem, BaseGeometry]], List[Tuple[EditorItem, BaseGeometry]], Dict[int, BaseGeometry]]:
    normal_geoms: List[Tuple[EditorItem, BaseGeometry]] = []
    blocked_geoms: List[Tuple[EditorItem, BaseGeometry]] = []
    by_item_id: Dict[int, BaseGeometry] = {}

    for item in project.items:
        if item.category != 'nav':
            continue
        if item.kind == 'point':
            continue

        geom = item_to_geometry(item, project.circle_resolution)
        by_item_id[item.id] = geom

        if item.role == 'blocked':
            blocked_geoms.append((item, geom))
        else:
            normal_geoms.append((item, geom))

    return normal_geoms, blocked_geoms, by_item_id


def _iter_polygons(geom: BaseGeometry) -> Iterable[Polygon]:
    if geom.is_empty:
        return []
    if isinstance(geom, Polygon):
        return [geom]
    if isinstance(geom, MultiPolygon):
        return list(geom.geoms)
    if isinstance(geom, GeometryCollection):
        return [g for g in geom.geoms if isinstance(g, Polygon) and not g.is_empty]
    return []


def build_partitioned_nav_geometries(
    project: Project,
    runtime_ids: Dict[int, str],
) -> Tuple[BaseGeometry, BaseGeometry, Dict[int, BaseGeometry], Dict[int, BaseGeometry]]:
    normal_geoms, blocked_geoms, by_item_id = _collect_nav_geometries(project)

    if not normal_geoms and not blocked_geoms:
        return Polygon(), Polygon(), {}, {}

    total_nav_parts: List[BaseGeometry] = [geom for _, geom in normal_geoms] + [geom for _, geom in blocked_geoms]
    total_nav = unary_union(total_nav_parts) if total_nav_parts else Polygon()
    total_nav = _quantize_geometry(total_nav, project.simplify_tolerance)

    blocker_geoms_by_item_id: Dict[int, BaseGeometry] = {}
    if blocked_geoms:
        for item, geom in blocked_geoms:
            clipped = geom.intersection(total_nav)
            clipped = _quantize_geometry(clipped, project.simplify_tolerance)
            if not clipped.is_empty:
                blocker_geoms_by_item_id[item.id] = clipped

    blocked_union: BaseGeometry = Polygon()
    if blocker_geoms_by_item_id:
        blocked_union = unary_union(list(blocker_geoms_by_item_id.values()))
        blocked_union = _quantize_geometry(blocked_union, project.simplify_tolerance)

    open_nav = total_nav
    if not blocked_union.is_empty:
        open_nav = total_nav.difference(blocked_union)
        open_nav = _quantize_geometry(open_nav, project.simplify_tolerance)

    return open_nav, blocked_union, by_item_id, blocker_geoms_by_item_id


def _mesh_partition(partition: BaseGeometry, simplify_tolerance: float) -> List[Polygon]:
    triangles: List[Polygon] = []
    for piece in _iter_polygons(partition):
        raw_tris = triangulate(piece)
        for tri in raw_tris:
            clipped = tri.intersection(piece)
            if clipped.is_empty:
                continue
            for poly in _iter_polygons(clipped):
                if poly.area > 1.0:
                    triangles.append(_quantize_polygon(poly, simplify_tolerance))

    if not triangles:
        return []

    return _greedy_merge_convex(triangles, partition)


def build_navmesh_cells(project: Project,
                        runtime_ids: Dict[int, str]) -> Tuple[List[NavCell], List[Polygon], BaseGeometry, BaseGeometry]:
    open_nav, blocked_nav, by_item_id, blocker_geoms_by_item_id = build_partitioned_nav_geometries(project, runtime_ids)

    open_cells = _mesh_partition(open_nav, project.simplify_tolerance)
    blocked_cells = _mesh_partition(blocked_nav, project.simplify_tolerance)

    nav_items = [item for item in project.items if item.category == 'nav' and item.kind != 'point']

    all_polys: List[Polygon] = []
    all_cells: List[NavCell] = []

    def append_cells(polys: List[Polygon], blocked: bool) -> None:
        for poly in polys:
            idx = len(all_cells)
            coords = [(float(x), float(y)) for x, y in list(poly.exterior.coords)[:-1]]
            coords = _ensure_ccw(coords)

            labels: List[str] = []
            region_ids: List[str] = []
            centroid = poly.representative_point()
            for item in nav_items:
                region_geom = by_item_id.get(item.id)
                if region_geom is not None and region_geom.intersects(centroid):
                    labels.append(item.label)
                    region_ids.append(runtime_ids[item.id])

            blocked_by_labels: List[str] = []
            blocked_by_ids: List[str] = []
            if blocked:
                for item in nav_items:
                    blocker_geom = blocker_geoms_by_item_id.get(item.id)
                    if blocker_geom is not None and blocker_geom.intersects(centroid):
                        blocked_by_labels.append(item.label)
                        blocked_by_ids.append(runtime_ids[item.id])

            all_polys.append(poly)
            all_cells.append(
                NavCell(
                    id=idx,
                    vertices=coords,
                    neighbors=[],
                    source_region_labels=sorted(set(labels)),
                    source_region_ids=sorted(set(region_ids)),
                    blocked=blocked,
                    blocked_by_labels=sorted(set(blocked_by_labels)),
                    blocked_by_ids=sorted(set(blocked_by_ids)),
                )
            )

    append_cells(open_cells, False)
    append_cells(blocked_cells, True)

    for i in range(len(all_polys)):
        for j in range(i + 1, len(all_polys)):
            if _shared_edge_length(all_polys[i], all_polys[j]) > 1e-3:
                all_cells[i].neighbors.append(j)
                all_cells[j].neighbors.append(i)

    for cell in all_cells:
        cell.neighbors = sorted(set(cell.neighbors))

    return all_cells, all_polys, open_nav, blocked_nav


def build_region_graph(project: Project,
                       runtime_ids: Dict[int, str],
                       cells: List[NavCell]) -> List[Dict[str, Any]]:
    entries: List[Dict[str, Any]] = []

    nav_items = [
        item for item in project.items
        if item.category == 'nav' and item.kind != 'point' and item.role != 'blocked'
    ]
    geom_entries = [(item, item_to_geometry(item, project.circle_resolution)) for item in nav_items]

    polygon_ids_by_region_label: Dict[str, set[int]] = {}
    polygon_ids_by_region_id: Dict[str, set[int]] = {}
    for cell in cells:
        if cell.blocked:
            continue
        for label in cell.source_region_labels:
            polygon_ids_by_region_label.setdefault(label, set()).add(cell.id)
        for region_id in cell.source_region_ids:
            polygon_ids_by_region_id.setdefault(region_id, set()).add(cell.id)

    for item, geom in geom_entries:
        centroid = geom.representative_point()
        neighbors: List[str] = []
        neighbor_ids: List[str] = []
        for other_item, other_geom in geom_entries:
            if other_item.id == item.id:
                continue
            if geom.buffer(project.snap_distance * 0.55).intersects(other_geom):
                neighbors.append(other_item.label)
                neighbor_ids.append(runtime_ids[other_item.id])

        region_runtime_id = runtime_ids[item.id]
        polygon_ids = sorted(
            polygon_ids_by_region_label.get(item.label, set()) |
            polygon_ids_by_region_id.get(region_runtime_id, set())
        )

        entries.append({
            'label': item.label,
            'runtime_id': region_runtime_id,
            'source_item_id': item.id,
            'role': item.role,
            'kind': item.kind,
            'center': [float(centroid.x), float(centroid.y)],
            'neighbors': sorted(set(neighbors)),
            'neighbor_ids': sorted(set(neighbor_ids)),
            'polygon_ids': polygon_ids,
        })

    return entries


def build_runtime_blockers(project: Project,
                           cells: List[NavCell],
                           cell_polys: List[Polygon],
                           runtime_ids: Dict[int, str],
                           region_graph: List[Dict[str, Any]]) -> List[Dict[str, Any]]:
    blockers: List[Dict[str, Any]] = []

    region_by_runtime_id = {entry['runtime_id']: entry for entry in region_graph}

    for item in project.items:
        if item.category != 'nav':
            continue
        if item.role != 'blocked':
            continue
        if item.kind == 'point':
            continue

        geom = item_to_geometry(item, project.circle_resolution)
        cell_ids: List[int] = []
        owning_labels: List[str] = []
        owning_ids: List[str] = []

        for idx, cell_poly in enumerate(cell_polys):
            if not cells[idx].blocked:
                continue
            if geom.intersects(cell_poly):
                cell_ids.append(idx)
                owning_labels.extend(cells[idx].source_region_labels)
                owning_ids.extend(cells[idx].source_region_ids)

        owning_region = ''
        if owning_labels:
            counts: Dict[str, int] = {}
            for label in owning_labels:
                counts[label] = counts.get(label, 0) + 1
            owning_region = max(counts.items(), key=lambda kv: kv[1])[0]
        elif owning_ids:
            counts: Dict[str, int] = {}
            for region_id in owning_ids:
                counts[region_id] = counts.get(region_id, 0) + 1
            best_runtime_id = max(counts.items(), key=lambda kv: kv[1])[0]
            owning_region = region_by_runtime_id.get(best_runtime_id, {}).get('label', '')

        blockers.append({
            'label': item.label,
            'runtime_id': runtime_ids[item.id],
            'source_item_id': item.id,
            'toggle_id': runtime_ids[item.id],
            'kind': item.kind,
            'blocker_type': _infer_blocker_type(item),
            'owning_region': owning_region,
            'enabled_on_start': True,
            'x': round(float(item.x), 2),
            'y': round(float(item.y), 2),
            'w': round(float(item.w), 2),
            'h': round(float(item.h), 2),
            'cell_ids': sorted(set(cell_ids)),
        })

    return blockers


def _canonical_edge_key(a: Tuple[int, int], b: Tuple[int, int]) -> Tuple[int, int, int, int]:
    if a < b:
        return a[0], a[1], b[0], b[1]
    return b[0], b[1], a[0], a[1]


def build_wall_edges(cells: List[NavCell]) -> List[Dict[str, Any]]:
    edge_owners: Dict[Tuple[int, int, int, int], List[Dict[str, Any]]] = {}

    for cell in cells:
        verts = _int_vertices(cell.vertices)
        if len(verts) < 2:
            continue

        for i in range(len(verts)):
            a = verts[i]
            b = verts[(i + 1) % len(verts)]
            key = _canonical_edge_key(a, b)
            edge_owners.setdefault(key, []).append({
                'cell_id': cell.id,
                'a': a,
                'b': b,
                'blocked': cell.blocked,
                'blocked_by_ids': list(cell.blocked_by_ids or []),
                'blocked_by_labels': list(cell.blocked_by_labels or []),
            })

    wall_edges: List[Dict[str, Any]] = []

    for owners in edge_owners.values():
        if len(owners) == 1:
            owner = owners[0]
            if owner['blocked']:
                continue

            wall_edges.append({
                'ax': owner['a'][0],
                'ay': owner['a'][1],
                'bx': owner['b'][0],
                'by': owner['b'][1],
                'owner_poly': owner['cell_id'],
                'wall_type': 'boundary',
                'toggle_id': '',
            })
            continue

        if len(owners) != 2:
            continue

        open_owner = next((o for o in owners if not o['blocked']), None)
        blocked_owner = next((o for o in owners if o['blocked']), None)

        if open_owner is None or blocked_owner is None:
            continue

        toggle_ids = sorted(set(blocked_owner['blocked_by_ids']))
        if not toggle_ids:
            continue

        for toggle_id in toggle_ids:
            wall_edges.append({
                'ax': open_owner['a'][0],
                'ay': open_owner['a'][1],
                'bx': open_owner['b'][0],
                'by': open_owner['b'][1],
                'owner_poly': open_owner['cell_id'],
                'wall_type': 'blocker',
                'toggle_id': toggle_id,
            })

    return wall_edges


def serialize_item(item: EditorItem, runtime_id: str) -> Dict[str, Any]:
    payload: Dict[str, Any] = {
        'id': item.id,
        'runtime_id': runtime_id,
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


def export_preview_png(project: Project,
                       open_nav: BaseGeometry,
                       blocked_nav: BaseGeometry,
                       out_path: Path) -> None:
    img = Image.new('RGBA', (project.canvas_width, project.canvas_height), (28, 30, 34, 255))
    draw = ImageDraw.Draw(img, 'RGBA')

    for item in project.items:
        if item.category != 'nav':
            continue

        fill = (70, 110, 165, 90) if item.role != 'blocked' else (150, 60, 60, 110)
        outline = (200, 220, 240, 255) if item.role != 'blocked' else (240, 150, 150, 255)

        if item.kind == 'rect':
            bbox = [item.x, item.y, item.x + item.w, item.y + item.h]
            draw.rectangle(bbox, fill=fill, outline=outline, width=2)
        elif item.kind == 'circle':
            bbox = [item.x, item.y, item.x + item.w, item.y + item.h]
            draw.ellipse(bbox, fill=fill, outline=outline, width=2)

    def draw_poly(poly: Polygon, color: Tuple[int, int, int, int]) -> None:
        pts = [(float(x), float(y)) for x, y in poly.exterior.coords]
        draw.line(pts + [pts[0]], fill=color, width=3)

    for poly in _iter_polygons(open_nav):
        draw_poly(poly, (120, 255, 160, 255))
    for poly in _iter_polygons(blocked_nav):
        draw_poly(poly, (255, 120, 120, 255))

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

    runtime_ids = build_runtime_item_ids(project)

    nav_cells, cell_polys, open_nav_geom, blocked_nav_geom = build_navmesh_cells(project, runtime_ids)
    region_graph = build_region_graph(project, runtime_ids, nav_cells)
    runtime_blockers = build_runtime_blockers(project, nav_cells, cell_polys, runtime_ids, region_graph)
    wall_edges = build_wall_edges(nav_cells)

    triggers = [serialize_item(item, runtime_ids[item.id]) for item in project.items if item.category == 'trigger']
    objects = [serialize_item(item, runtime_ids[item.id]) for item in project.items if item.category == 'object']
    points = [serialize_item(item, runtime_ids[item.id]) for item in project.items if item.category == 'point']
    nav_items = [serialize_item(item, runtime_ids[item.id]) for item in project.items if item.category == 'nav']

    project_json = Path(project_path) if project_path else out_dir / 'map_project.json'
    entities_json = out_dir / 'entities.json'
    nav_json = out_dir / 'navmesh.json'
    nav_bin = out_dir / 'navmesh_polygons.nav'
    preview_png = out_dir / 'map_preview.bmp'

    from world_editor_model_qt import save_project
    save_project(str(project_json), project)

    with open(entities_json, 'w', encoding='utf-8') as f:
        json.dump(
            {'entities': [entity.to_dict() for entity in project.entities]},
            f,
            indent=2
        )

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
                'source_region_ids': cell.source_region_ids,
                'blocked': cell.blocked,
                'blocked_by_labels': list(cell.blocked_by_labels or []),
                'blocked_by_ids': list(cell.blocked_by_ids or []),
            }
            for cell in nav_cells
        ],
        'runtime_blockers': runtime_blockers,
        'wall_edges': wall_edges,
        'triggers': triggers,
        'objects': objects,
        'points': points,
    }

    with open(nav_json, 'w', encoding='utf-8') as f:
        json.dump(nav_payload, f, indent=2)

    export_navmesh_binary(nav_cells, nav_bin)
    export_preview_png(project, open_nav_geom, blocked_nav_geom, preview_png)

    return ExportResult(
        project_json=project_json,
        entities_json=entities_json,
        nav_json=nav_json,
        nav_bin=nav_bin,
        preview_png=preview_png,
    )
