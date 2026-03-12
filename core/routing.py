import heapq
import json
from pathlib import Path
from dataclasses import asdict
from typing import Dict, List, Optional, Set, Tuple

from config import GRID, snap_to_grid
from core.models import PCBDesign, RoutedSegment, RoutingResult, Via

LAYER_TOP = "L1_TOP"
LAYER_BOTTOM = "L4_BOTTOM"
LAYERS = [LAYER_TOP, LAYER_BOTTOM]


def _to_idx(v: float) -> int:
    return int(round(v / GRID))


def _to_coord(i: int) -> float:
    return snap_to_grid(i * GRID)


def _pin_points_for_net(design: PCBDesign, net_name: str) -> List[Tuple[float, float]]:
    net = design.nets[net_name]
    pts: List[Tuple[float, float]] = []
    seen = set()
    for ref, pin_name in net.connections:
        if ref not in design.components:
            continue
        p = design.components[ref].rotated_pin_position(pin_name)
        key = (round(p[0], 6), round(p[1], 6))
        if key in seen:
            continue
        seen.add(key)
        pts.append((snap_to_grid(p[0]), snap_to_grid(p[1])))
    return pts


def _mst_edges(points: List[Tuple[float, float]]) -> List[Tuple[int, int]]:
    if len(points) <= 1:
        return []

    n = len(points)
    used = [False] * n
    dist = [float("inf")] * n
    parent = [-1] * n
    dist[0] = 0.0
    edges: List[Tuple[int, int]] = []

    for _ in range(n):
        u = -1
        best = float("inf")
        for i in range(n):
            if not used[i] and dist[i] < best:
                best = dist[i]
                u = i
        if u < 0:
            break
        used[u] = True
        if parent[u] >= 0:
            edges.append((parent[u], u))

        ux, uy = points[u]
        for v in range(n):
            if used[v]:
                continue
            vx, vy = points[v]
            d = abs(ux - vx) + abs(uy - vy)
            if d < dist[v]:
                dist[v] = d
                parent[v] = u

    return edges


def _build_component_obstacles(design: PCBDesign, margin: float = 0.2) -> Set[Tuple[int, int]]:
    blocked: Set[Tuple[int, int]] = set()
    for comp in design.components.values():
        x1, y1, x2, y2 = comp.bounding_box()
        x1 -= margin
        y1 -= margin
        x2 += margin
        y2 += margin

        ix1, iy1 = _to_idx(x1), _to_idx(y1)
        ix2, iy2 = _to_idx(x2), _to_idx(y2)
        for ix in range(min(ix1, ix2), max(ix1, ix2) + 1):
            for iy in range(min(iy1, iy2), max(iy1, iy2) + 1):
                blocked.add((ix, iy))
    return blocked


def _a_star_route(
    board_w: float,
    board_h: float,
    start: Tuple[int, int],
    goal: Tuple[int, int],
    occ: List[Dict[Tuple[int, int], str]],
    net_name: str,
    always_allow: Set[Tuple[int, int]],
    via_penalty: float = 8.0,
) -> Optional[List[Tuple[int, int, int]]]:
    max_x = _to_idx(board_w)
    max_y = _to_idx(board_h)

    def walkable(x: int, y: int, layer: int) -> bool:
        if x < 0 or y < 0 or x > max_x or y > max_y:
            return False
        owner = occ[layer].get((x, y))
        if owner is None or owner == net_name:
            return True
        return (x, y) in always_allow

    def heuristic(x: int, y: int) -> float:
        return abs(x - goal[0]) + abs(y - goal[1])

    pq: List[Tuple[float, float, int, int, int]] = []
    dist: Dict[Tuple[int, int, int], float] = {}
    prev: Dict[Tuple[int, int, int], Tuple[int, int, int]] = {}

    for layer in (0, 1):
        s = (start[0], start[1], layer)
        if not walkable(s[0], s[1], layer):
            continue
        dist[s] = 0.0
        heapq.heappush(pq, (heuristic(s[0], s[1]), 0.0, s[0], s[1], layer))

    goal_node: Optional[Tuple[int, int, int]] = None

    while pq:
        _, g, x, y, layer = heapq.heappop(pq)
        node = (x, y, layer)
        if g != dist.get(node, float("inf")):
            continue

        if (x, y) == goal:
            goal_node = node
            break

        for dx, dy in ((1, 0), (-1, 0), (0, 1), (0, -1)):
            nx, ny = x + dx, y + dy
            if not walkable(nx, ny, layer):
                continue
            cand = g + 1.0
            nxt = (nx, ny, layer)
            if cand < dist.get(nxt, float("inf")):
                dist[nxt] = cand
                prev[nxt] = node
                heapq.heappush(pq, (cand + heuristic(nx, ny), cand, nx, ny, layer))

        other = 1 - layer
        if walkable(x, y, other):
            cand = g + via_penalty
            nxt = (x, y, other)
            if cand < dist.get(nxt, float("inf")):
                dist[nxt] = cand
                prev[nxt] = node
                heapq.heappush(pq, (cand + heuristic(x, y), cand, x, y, other))

    if goal_node is None:
        return None

    path = [goal_node]
    cur = goal_node
    while cur in prev:
        cur = prev[cur]
        path.append(cur)
    path.reverse()
    return path


def _path_to_primitives(net_name: str, path: List[Tuple[int, int, int]]) -> Tuple[List[RoutedSegment], List[Via]]:
    segments: List[RoutedSegment] = []
    vias: List[Via] = []

    if len(path) < 2:
        return segments, vias

    sx, sy, sl = path[0]
    px, py = sx, sy

    for x, y, layer in path[1:]:
        if layer != sl:
            vias.append(
                Via(
                    net=net_name,
                    x=_to_coord(px),
                    y=_to_coord(py),
                    from_layer=LAYERS[sl],
                    to_layer=LAYERS[layer],
                )
            )
            sl = layer
            sx, sy = x, y
            px, py = x, y
            continue

        if (x != px and y != py):
            px, py = x, y
            continue

        dx0, dy0 = px - sx, py - sy
        dx1, dy1 = x - px, y - py
        if (dx0, dy0) != (0, 0) and (dx0 == 0) != (dx1 == 0):
            segments.append(
                RoutedSegment(
                    net=net_name,
                    layer=LAYERS[sl],
                    x1=_to_coord(sx),
                    y1=_to_coord(sy),
                    x2=_to_coord(px),
                    y2=_to_coord(py),
                )
            )
            sx, sy = px, py

        px, py = x, y

    if (sx, sy) != (px, py):
        segments.append(
            RoutedSegment(
                net=net_name,
                layer=LAYERS[sl],
                x1=_to_coord(sx),
                y1=_to_coord(sy),
                x2=_to_coord(px),
                y2=_to_coord(py),
            )
        )

    return segments, vias


def route_design_v1(design: PCBDesign) -> RoutingResult:
    result = RoutingResult()

    comp_blocked = _build_component_obstacles(design)

    # Allow escape around pads so traces can leave components.
    for comp in design.components.values():
        for pin_name in comp.pins.keys():
            px, py = comp.rotated_pin_position(pin_name)
            ix, iy = _to_idx(px), _to_idx(py)
            for dx, dy in ((0, 0), (1, 0), (-1, 0), (0, 1), (0, -1)):
                comp_blocked.discard((ix + dx, iy + dy))
    occ: List[Dict[Tuple[int, int], str]] = [dict(), dict()]
    for xy in comp_blocked:
        occ[0][xy] = "__BLOCKED__"
        occ[1][xy] = "__BLOCKED__"

    for net_name in design.nets.keys():
        points = _pin_points_for_net(design, net_name)
        if len(points) < 2:
            result.routed_nets[net_name] = True
            continue

        points_idx = [(_to_idx(x), _to_idx(y)) for x, y in points]
        always_allow = set(points_idx)

        ok = True
        for u, v in _mst_edges(points):
            start = points_idx[u]
            goal = points_idx[v]
            path = _a_star_route(
                board_w=design.board.width,
                board_h=design.board.height,
                start=start,
                goal=goal,
                occ=occ,
                net_name=net_name,
                always_allow=always_allow,
            )
            if path is None:
                ok = False
                break

            segs, vias = _path_to_primitives(net_name, path)
            result.segments.extend(segs)
            result.vias.extend(vias)

            for x, y, layer in path:
                owner = occ[layer].get((x, y))
                if owner in (None, net_name):
                    occ[layer][(x, y)] = net_name

        result.routed_nets[net_name] = ok

    design.routing_result = result
    return result


def export_routing_result_json(design: PCBDesign, out_path: str) -> None:
    rr = design.routing_result
    if rr is None:
        raise ValueError("design.routing_result is empty; run route_design_v1 first")

    by_net: Dict[str, Dict[str, object]] = {}

    for net_name in design.nets.keys():
        by_net[net_name] = {
            "routed": bool(rr.routed_nets.get(net_name, False)),
            "segments": [],
            "vias": [],
            "via_count": 0,
        }

    for seg in rr.segments:
        by_net[seg.net]["segments"].append(asdict(seg))

    for via in rr.vias:
        by_net[via.net]["vias"].append(asdict(via))

    for net_name in by_net.keys():
        by_net[net_name]["via_count"] = len(by_net[net_name]["vias"])

    payload = {
        "layers": LAYERS,
        "summary": {
            "total_nets": len(design.nets),
            "routed_nets": sum(1 for v in rr.routed_nets.values() if v),
            "total_segments": len(rr.segments),
            "total_vias": len(rr.vias),
        },
        "nets": by_net,
    }

    out_file = Path(out_path)
    out_file.parent.mkdir(parents=True, exist_ok=True)
    with out_file.open("w", encoding="utf-8") as f:
        json.dump(payload, f, indent=2, ensure_ascii=False)
