from __future__ import annotations

import argparse
import hashlib
import heapq
import json
import math
import struct
from dataclasses import dataclass
from pathlib import Path
from typing import Dict, List, Optional, Set, Tuple


TRAINER_VERSION = 2


@dataclass
class Vec2:
    x: float
    y: float


@dataclass
class Polygon:
    id: int
    vertices: List[Vec2]
    neighbors: List[int]

    @property
    def centroid(self) -> Vec2:
        if not self.vertices:
            return Vec2(0.0, 0.0)
        sx = sum(v.x for v in self.vertices)
        sy = sum(v.y for v in self.vertices)
        return Vec2(sx / len(self.vertices), sy / len(self.vertices))


@dataclass
class CandidatePath:
    nodes: List[int]
    cost: float


def dist(a: Vec2, b: Vec2) -> float:
    dx = a.x - b.x
    dy = a.y - b.y
    return math.hypot(dx, dy)


def heading(a: Vec2, b: Vec2) -> float:
    return math.atan2(b.y - a.y, b.x - a.x)


def angle_delta_abs(a: float, b: float) -> float:
    d = a - b
    while d > math.pi:
        d -= 2.0 * math.pi
    while d < -math.pi:
        d += 2.0 * math.pi
    return abs(d)


def read_nav_bin(path: Path) -> List[Polygon]:
    data = path.read_bytes()
    off = 0

    def read_i32() -> int:
        nonlocal off
        value = struct.unpack_from('<i', data, off)[0]
        off += 4
        return value

    poly_count = read_i32()
    polys: List[Polygon] = []
    for pid in range(poly_count):
        vcount = read_i32()
        verts: List[Vec2] = []
        for _ in range(vcount):
            x = read_i32()
            y = read_i32()
            verts.append(Vec2(float(x), float(y)))
        ncount = read_i32()
        neighbors = [read_i32() for _ in range(ncount)]
        polys.append(Polygon(pid, verts, neighbors))
    return polys


def shared_edge_length(a: Polygon, b: Polygon) -> float:
    shared: List[Tuple[float, float]] = []
    for va in a.vertices:
        for vb in b.vertices:
            if int(round(va.x)) == int(round(vb.x)) and int(round(va.y)) == int(round(vb.y)):
                shared.append((va.x, va.y))
    if len(shared) < 2:
        return 0.0
    best = 0.0
    for i in range(len(shared)):
        for j in range(i + 1, len(shared)):
            ax, ay = shared[i]
            bx, by = shared[j]
            best = max(best, math.hypot(ax - bx, ay - by))
    return best


def load_blocked_cells_from_nav_json(path: Path) -> Set[int]:
    if not path.exists():
        return set()

    payload = json.loads(path.read_text(encoding='utf-8'))
    blocked: Set[int] = set()

    for blocker in payload.get("runtime_blockers", []):
        if not blocker.get("enabled_on_start", False):
            continue
        for cell_id in blocker.get("cell_ids", []):
            blocked.add(int(cell_id))

    return blocked


def build_edge_metrics(polys: List[Polygon],
                       blocked_cells: Set[int],
                       bucket_radius: int) -> Dict[Tuple[int, int], Dict[str, float]]:
    metrics: Dict[Tuple[int, int], Dict[str, float]] = {}
    min_portal_len = float(bucket_radius * 2 + 1)

    for poly in polys:
        if poly.id in blocked_cells:
            continue

        ca = poly.centroid
        for nb in poly.neighbors:
            if nb in blocked_cells:
                continue

            other = polys[nb]
            portal_len = shared_edge_length(poly, other)
            if bucket_radius > 0 and portal_len < min_portal_len:
                continue

            cb = other.centroid
            metrics[(poly.id, nb)] = {
                'distance': dist(ca, cb),
                'portal_len': portal_len,
            }
    return metrics


def edge_step_cost(prev: Optional[int], cur: int, nxt: int,
                   polys: List[Polygon],
                   edge_metrics: Dict[Tuple[int, int], Dict[str, float]],
                   turn_weight: float,
                   narrow_weight: float) -> float:
    metric = edge_metrics[(cur, nxt)]
    base = metric['distance']

    turn_pen = 0.0
    if prev is not None and prev != cur:
        h1 = heading(polys[prev].centroid, polys[cur].centroid)
        h2 = heading(polys[cur].centroid, polys[nxt].centroid)
        turn_pen = angle_delta_abs(h1, h2)

    portal_len = max(metric['portal_len'], 1.0)
    narrow_pen = 1.0 / portal_len

    return base + turn_weight * turn_pen + narrow_weight * narrow_pen


def a_star_edge_state(start: int,
                      goal: int,
                      polys: List[Polygon],
                      edge_metrics: Dict[Tuple[int, int], Dict[str, float]],
                      turn_weight: float,
                      narrow_weight: float) -> CandidatePath:
    if start == goal:
        return CandidatePath([start], 0.0)

    if start >= len(polys) or goal >= len(polys):
        return CandidatePath([], float('inf'))

    start_state = (-1, start)
    frontier: List[Tuple[float, float, Tuple[int, int]]] = []
    heapq.heappush(frontier, (0.0, 0.0, start_state))

    g_cost: Dict[Tuple[int, int], float] = {start_state: 0.0}
    came_from: Dict[Tuple[int, int], Optional[Tuple[int, int]]] = {start_state: None}

    while frontier:
        _, cur_g, state = heapq.heappop(frontier)
        prev_poly, cur_poly = state

        if cur_poly == goal:
            seq: List[int] = []
            s: Optional[Tuple[int, int]] = state
            while s is not None:
                seq.append(s[1])
                s = came_from[s]
            seq.reverse()
            dedup: List[int] = []
            for node in seq:
                if not dedup or dedup[-1] != node:
                    dedup.append(node)
            return CandidatePath(dedup, cur_g)

        if cur_g > g_cost.get(state, float('inf')):
            continue

        for nxt in polys[cur_poly].neighbors:
            if (cur_poly, nxt) not in edge_metrics:
                continue

            step = edge_step_cost(
                None if prev_poly < 0 else prev_poly,
                cur_poly,
                nxt,
                polys,
                edge_metrics,
                turn_weight,
                narrow_weight,
            )
            new_g = cur_g + step
            next_state = (cur_poly, nxt)
            if new_g >= g_cost.get(next_state, float('inf')):
                continue
            g_cost[next_state] = new_g
            came_from[next_state] = state
            h = dist(polys[nxt].centroid, polys[goal].centroid)
            heapq.heappush(frontier, (new_g + h, new_g, next_state))

    return CandidatePath([], float('inf'))


def penalized_shortest_path(start: int,
                            goal: int,
                            polys: List[Polygon],
                            edge_metrics: Dict[Tuple[int, int], Dict[str, float]],
                            turn_weight: float,
                            narrow_weight: float,
                            edge_penalties: Dict[Tuple[int, int], float],
                            penalty_scale: float) -> CandidatePath:
    start_state = (-1, start)
    frontier: List[Tuple[float, float, Tuple[int, int]]] = []
    heapq.heappush(frontier, (0.0, 0.0, start_state))

    g_cost: Dict[Tuple[int, int], float] = {start_state: 0.0}
    came_from: Dict[Tuple[int, int], Optional[Tuple[int, int]]] = {start_state: None}

    while frontier:
        _, cur_g, state = heapq.heappop(frontier)
        prev_poly, cur_poly = state

        if cur_poly == goal:
            seq: List[int] = []
            s: Optional[Tuple[int, int]] = state
            while s is not None:
                seq.append(s[1])
                s = came_from[s]
            seq.reverse()
            dedup: List[int] = []
            for node in seq:
                if not dedup or dedup[-1] != node:
                    dedup.append(node)
            return CandidatePath(dedup, cur_g)

        if cur_g > g_cost.get(state, float('inf')):
            continue

        for nxt in polys[cur_poly].neighbors:
            if (cur_poly, nxt) not in edge_metrics:
                continue

            step = edge_step_cost(
                None if prev_poly < 0 else prev_poly,
                cur_poly,
                nxt,
                polys,
                edge_metrics,
                turn_weight,
                narrow_weight,
            )
            step += edge_penalties.get((cur_poly, nxt), 0.0) * penalty_scale
            new_g = cur_g + step
            next_state = (cur_poly, nxt)
            if new_g >= g_cost.get(next_state, float('inf')):
                continue
            g_cost[next_state] = new_g
            came_from[next_state] = state
            h = dist(polys[nxt].centroid, polys[goal].centroid)
            heapq.heappush(frontier, (new_g + h, new_g, next_state))

    return CandidatePath([], float('inf'))


def path_total_turn(path: List[int], polys: List[Polygon]) -> float:
    if len(path) < 3:
        return 0.0
    total = 0.0
    for i in range(1, len(path) - 1):
        h1 = heading(polys[path[i - 1]].centroid, polys[path[i]].centroid)
        h2 = heading(polys[path[i]].centroid, polys[path[i + 1]].centroid)
        total += angle_delta_abs(h1, h2)
    return total


def path_portal_narrowness(path: List[int], edge_metrics: Dict[Tuple[int, int], Dict[str, float]]) -> float:
    if len(path) < 2:
        return 0.0
    total = 0.0
    for i in range(len(path) - 1):
        portal_len = max(edge_metrics[(path[i], path[i + 1])]['portal_len'], 1.0)
        total += 1.0 / portal_len
    return total


def route_score(path: List[int],
                polys: List[Polygon],
                edge_metrics: Dict[Tuple[int, int], Dict[str, float]],
                turn_weight: float,
                narrow_weight: float,
                portal_hop_weight: float) -> float:
    if len(path) < 2:
        return float('inf')
    length = 0.0
    for i in range(len(path) - 1):
        length += edge_metrics[(path[i], path[i + 1])]['distance']
    turn = path_total_turn(path, polys)
    narrow = path_portal_narrowness(path, edge_metrics)
    hops = max(0, len(path) - 2)
    return length + turn_weight * turn + narrow_weight * narrow + portal_hop_weight * hops


def best_of_diverse_candidates(start: int,
                               goal: int,
                               polys: List[Polygon],
                               edge_metrics: Dict[Tuple[int, int], Dict[str, float]],
                               k_paths: int,
                               turn_weight: float,
                               narrow_weight: float,
                               portal_hop_weight: float) -> CandidatePath:
    penalties: Dict[Tuple[int, int], float] = {}
    candidates: List[CandidatePath] = []

    base = a_star_edge_state(start, goal, polys, edge_metrics, turn_weight, narrow_weight)
    if not base.nodes:
        return base
    candidates.append(base)

    for _ in range(max(0, k_paths - 1)):
        for i in range(len(candidates[-1].nodes) - 1):
            edge = (candidates[-1].nodes[i], candidates[-1].nodes[i + 1])
            penalties[edge] = penalties.get(edge, 0.0) + 1.0
        cand = penalized_shortest_path(
            start, goal, polys, edge_metrics,
            turn_weight, narrow_weight,
            penalties, 30.0,
        )
        if not cand.nodes:
            continue
        if any(c.nodes == cand.nodes for c in candidates):
            continue
        candidates.append(cand)

    best = min(
        candidates,
        key=lambda c: route_score(c.nodes, polys, edge_metrics, turn_weight, narrow_weight, portal_hop_weight),
    )
    best.cost = route_score(best.nodes, polys, edge_metrics, turn_weight, narrow_weight, portal_hop_weight)
    return best


def train_goal_conditioned_priors(polys: List[Polygon],
                                  edge_metrics: Dict[Tuple[int, int], Dict[str, float]],
                                  blocked_cells: Set[int],
                                  k_paths: int,
                                  turn_weight: float,
                                  narrow_weight: float,
                                  portal_hop_weight: float,
                                  bonus_scale: float) -> Dict[int, Dict[Tuple[int, int], float]]:
    priors: Dict[int, Dict[Tuple[int, int], float]] = {}

    for goal in range(len(polys)):
        if goal in blocked_cells:
            continue

        edge_bonus: Dict[Tuple[int, int], float] = {}
        for start in range(len(polys)):
            if start == goal or start in blocked_cells:
                continue

            best = best_of_diverse_candidates(
                start, goal, polys, edge_metrics,
                k_paths, turn_weight, narrow_weight, portal_hop_weight,
            )
            if not best.nodes or len(best.nodes) < 2:
                continue

            quality = 1.0 / max(best.cost, 1.0)
            for i in range(len(best.nodes) - 1):
                edge = (best.nodes[i], best.nodes[i + 1])
                edge_bonus[edge] = edge_bonus.get(edge, 0.0) + quality

        if not edge_bonus:
            priors[goal] = {}
            continue

        max_val = max(edge_bonus.values())
        if max_val <= 0.0:
            priors[goal] = {}
            continue

        normalized: Dict[Tuple[int, int], float] = {}
        for edge, value in edge_bonus.items():
            normalized[edge] = (value / max_val) * bonus_scale
        priors[goal] = normalized

    return priors


def write_priors_json(out_path: Path,
                      per_bucket_priors: Dict[int, Dict[int, Dict[Tuple[int, int], float]]],
                      per_bucket_metrics: Dict[int, Dict[Tuple[int, int], Dict[str, float]]],
                      source_signature: str,
                      buckets: List[int]) -> None:
    payload = {
        'version': TRAINER_VERSION,
        'trainer_version': TRAINER_VERSION,
        'source_signature': source_signature,
        'buckets': buckets,
        'goal_priors': [],
        'edge_metrics': [],
    }

    for bucket in buckets:
        edge_metrics = per_bucket_metrics.get(bucket, {})
        for (a, b), metric in sorted(edge_metrics.items()):
            payload['edge_metrics'].append({
                'bucket': bucket,
                'from': a,
                'to': b,
                'distance': round(metric['distance'], 4),
                'portal_len': round(metric['portal_len'], 4),
            })

        priors = per_bucket_priors.get(bucket, {})
        for goal_poly, edge_bonus in sorted(priors.items()):
            transitions = [
                {'from': a, 'to': b, 'bonus': round(bonus, 6)}
                for (a, b), bonus in sorted(edge_bonus.items())
                if bonus > 1e-8
            ]
            payload['goal_priors'].append({
                'bucket': bucket,
                'goal_poly': goal_poly,
                'transitions': transitions,
            })

    out_path.write_text(json.dumps(payload, indent=2), encoding='utf-8')


def compute_source_signature(nav_bin: Path, nav_json: Path, buckets: List[int]) -> str:
    h = hashlib.sha1()
    h.update(nav_bin.read_bytes())
    if nav_json.exists():
        h.update(nav_json.read_bytes())
    h.update(json.dumps({'buckets': buckets, 'trainer_version': TRAINER_VERSION}).encode('utf-8'))
    return h.hexdigest()


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument('--nav-bin', default='assets/navmesh_polygons.nav')
    parser.add_argument('--nav-json', default='assets/navmesh.json')
    parser.add_argument('--out', default='assets/nav_priors.json')
    parser.add_argument('--buckets', default='20,30,40')
    parser.add_argument('--k-paths', type=int, default=4)
    parser.add_argument('--turn-weight', type=float, default=55.0)
    parser.add_argument('--narrow-weight', type=float, default=20.0)
    parser.add_argument('--portal-hop-weight', type=float, default=3.5)
    parser.add_argument('--bonus-scale', type=float, default=12.0)
    args = parser.parse_args()

    nav_bin = Path(args.nav_bin)
    nav_json = Path(args.nav_json)
    out_path = Path(args.out)
    buckets = [int(x.strip()) for x in args.buckets.split(',') if x.strip()]

    polys = read_nav_bin(nav_bin)
    blocked_cells = load_blocked_cells_from_nav_json(nav_json)

    per_bucket_metrics: Dict[int, Dict[Tuple[int, int], Dict[str, float]]] = {}
    per_bucket_priors: Dict[int, Dict[int, Dict[Tuple[int, int], float]]] = {}

    for bucket in buckets:
        edge_metrics = build_edge_metrics(polys, blocked_cells, bucket)
        per_bucket_metrics[bucket] = edge_metrics
        per_bucket_priors[bucket] = train_goal_conditioned_priors(
            polys,
            edge_metrics,
            blocked_cells,
            args.k_paths,
            args.turn_weight,
            args.narrow_weight,
            args.portal_hop_weight,
            args.bonus_scale,
        )

    signature = compute_source_signature(nav_bin, nav_json, buckets)
    write_priors_json(out_path, per_bucket_priors, per_bucket_metrics, signature, buckets)

    print(f"Wrote bucket-aware priors to {out_path}")


if __name__ == '__main__':
    main()
