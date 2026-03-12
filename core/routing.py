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

def _is_port_component(comp) -> bool:
    ct = str(getattr(comp, "comp_type", "")).upper()
    ref = str(getattr(comp, "ref", "")).upper()
    return ("PORT" in ct) or ref.startswith("P_") or ref.startswith("PORT_")

def _to_idx(v: float) -> int:
    return int(round(v / GRID))


def _to_coord(i: int) -> float:
    return snap_to_grid(i * GRID)


def _pin_terminal(comp, pin_name: str) -> Tuple[Tuple[float, float], Tuple[int, int], Tuple[int, int]]:
    px, py = comp.rotated_pin_position(pin_name)
    pin_xy = (px, py)
    pin_idx = (_to_idx(px), _to_idx(py))

    x1, y1, x2, y2 = comp.bounding_box()
    side_dist = {
        "left": abs(px - x1),
        "right": abs(px - x2),
        "bottom": abs(py - y1),
        "top": abs(py - y2),
    }
    side = min(side_dist, key=side_dist.get)

    # Escape is one grid step outward from the pin in the nearest outward normal
    # direction. This keeps stub alignment tight to the actual pin geometry.
    if side == "left":
        escape = (pin_idx[0] - 1, pin_idx[1])
    elif side == "right":
        escape = (pin_idx[0] + 1, pin_idx[1])
    elif side == "bottom":
        escape = (pin_idx[0], pin_idx[1] - 1)
    else:
        escape = (pin_idx[0], pin_idx[1] + 1)

    if escape == pin_idx:
        ex, ey = escape
        if side == "left":
            escape = (ex - 1, ey)
        elif side == "right":
            escape = (ex + 1, ey)
        elif side == "bottom":
            escape = (ex, ey - 1)
        else:
            escape = (ex, ey + 1)

    return pin_xy, pin_idx, escape


def _net_terminals_for_net(
    design: PCBDesign,
    net_name: str,
    include_ports: bool = False,
) -> List[Tuple[Tuple[int, int], Tuple[int, int]]]:
    net = design.nets[net_name]
    terminals: List[Tuple[Tuple[float, float], Tuple[int, int], Tuple[int, int]]] = []
    seen = set()
    for ref, pin_name in net.connections:
        if ref not in design.components:
            continue
        comp = design.components[ref]
        if (not include_ports) and _is_port_component(comp):
            continue
        pin_xy, pin_idx, esc_idx = _pin_terminal(comp, pin_name)
        key = (pin_idx, esc_idx)
        if key in seen:
            continue
        seen.add(key)
        terminals.append((pin_xy, pin_idx, esc_idx))
    return terminals


def _mst_edges(points: List[Tuple[int, int]]) -> List[Tuple[int, int]]:
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

def _mark_line_cells(occ_layer: Dict[Tuple[int, int], str], net_name: str, a: Tuple[int, int], b: Tuple[int, int]) -> None:
    x1, y1 = a
    x2, y2 = b
    if x1 == x2:
        step = 1 if y2 >= y1 else -1
        for y in range(y1, y2 + step, step):
            owner = occ_layer.get((x1, y))
            if owner in (None, net_name):
                occ_layer[(x1, y)] = net_name
    elif y1 == y2:
        step = 1 if x2 >= x1 else -1
        for x in range(x1, x2 + step, step):
            owner = occ_layer.get((x, y1))
            if owner in (None, net_name):
                occ_layer[(x, y1)] = net_name


def _add_pin_stub(
    result: RoutingResult,
    occ: List[Dict[Tuple[int, int], str]],
    net_name: str,
    pin_xy: Tuple[float, float],
    pin_idx: Tuple[int, int],
    esc_idx: Tuple[int, int],
) -> None:
    if pin_idx == esc_idx:
        return

    x1, y1 = pin_idx
    x2, y2 = esc_idx
    if x1 != x2 and y1 != y2:
        mid = (x2, y1)
        result.segments.append(
            RoutedSegment(net=net_name, layer=LAYER_TOP, x1=pin_xy[0], y1=pin_xy[1], x2=_to_coord(mid[0]), y2=_to_coord(mid[1]))
        )
        result.segments.append(
            RoutedSegment(net=net_name, layer=LAYER_TOP, x1=_to_coord(mid[0]), y1=_to_coord(mid[1]), x2=_to_coord(x2), y2=_to_coord(y2))
        )
        _mark_line_cells(occ[0], net_name, pin_idx, mid)
        _mark_line_cells(occ[0], net_name, mid, esc_idx)
        return

    result.segments.append(
        RoutedSegment(net=net_name, layer=LAYER_TOP, x1=pin_xy[0], y1=pin_xy[1], x2=_to_coord(x2), y2=_to_coord(y2))
    )
    _mark_line_cells(occ[0], net_name, pin_idx, esc_idx)

def _a_star_route(
    board_w: float,
    board_h: float,
    start: Tuple[int, int],
    goal: Tuple[int, int],
    occ: List[Dict[Tuple[int, int], str]],
    net_name: str,
    always_allow: Set[Tuple[int, int]],
    via_penalty: float = 0.15,
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


def route_design_v1(
    design: PCBDesign,
    include_ports: bool = False,
    via_penalty: float = 0.15,
) -> RoutingResult:
    result = RoutingResult()

    comp_blocked = _build_component_obstacles(design)

    occ: List[Dict[Tuple[int, int], str]] = [dict(), dict()]

    pin_escape_cells: Set[Tuple[int, int]] = set()
    for net_name in design.nets.keys():
        for _, pin_idx, esc_idx in _net_terminals_for_net(design, net_name, include_ports=include_ports):
            pin_escape_cells.add(pin_idx)
            pin_escape_cells.add(esc_idx)

    for cell in pin_escape_cells:
        comp_blocked.discard(cell)

    for xy in comp_blocked:
        occ[0][xy] = "__BLOCKED__"
        occ[1][xy] = "__BLOCKED__"

    for net_name in design.nets.keys():
        terminals = _net_terminals_for_net(design, net_name, include_ports=include_ports)
        if len(terminals) < 2:
            result.routed_nets[net_name] = True
            continue

        # Backward-compatible aliases: earlier revisions used points / points_idx names.
        escape_points = [esc for _, _, esc in terminals]
        points = escape_points
        points_idx = points
        always_allow = set(points_idx)

        for pin_xy, pin_idx, esc_idx in terminals:
            _add_pin_stub(result, occ, net_name, pin_xy, pin_idx, esc_idx)

        points_idx = [(_to_idx(x), _to_idx(y)) for x, y in points]
        always_allow = set(points_idx)

        ok = True
        for u, v in _mst_edges(points_idx):
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
                via_penalty=via_penalty,
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
