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


def _build_component_obstacles_by_layer(
    design: PCBDesign,
    margin_top: float = 0.1,
    margin_bottom: float = 0.0,
) -> Tuple[Set[Tuple[int, int]], Set[Tuple[int, int]]]:
    blocked_top: Set[Tuple[int, int]] = set()
    blocked_bottom: Set[Tuple[int, int]] = set()
    for comp in design.components.values():
        x1, y1, x2, y2 = comp.bounding_box()

        tx1, ty1, tx2, ty2 = x1 - margin_top, y1 - margin_top, x2 + margin_top, y2 + margin_top
        ix1, iy1 = _to_idx(tx1), _to_idx(ty1)
        ix2, iy2 = _to_idx(tx2), _to_idx(ty2)

        for ix in range(min(ix1, ix2), max(ix1, ix2) + 1):
            for iy in range(min(iy1, iy2), max(iy1, iy2) + 1):
                blocked_top.add((ix, iy))

                # Assume most components are placed on top side; allow more freedom on
                # bottom side to avoid pin-area starvation. Keep ports blocked on both
                # sides as boundary connectors.
            if _is_port_component(comp):
                bx1, by1, bx2, by2 = x1 - margin_bottom, y1 - margin_bottom, x2 + margin_bottom, y2 + margin_bottom
                bix1, biy1 = _to_idx(bx1), _to_idx(by1)
                bix2, biy2 = _to_idx(bx2), _to_idx(by2)
                for ix in range(min(bix1, bix2), max(bix1, bix2) + 1):
                    for iy in range(min(biy1, biy2), max(biy1, biy2) + 1):
                        blocked_bottom.add((ix, iy))

            return blocked_top, blocked_bottom

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

def _line_cells(a: Tuple[int, int], b: Tuple[int, int]) -> List[Tuple[int, int]]:
    x1, y1 = a
    x2, y2 = b
    if x1 == x2:
        step = 1 if y2 >= y1 else -1
        return [(x1, y) for y in range(y1, y2 + step, step)]
    if y1 == y2:
        step = 1 if x2 >= x1 else -1
        return [(x, y1) for x in range(x1, x2 + step, step)]
    return []


def _line_is_clear(occ_layer: Dict[Tuple[int, int], str], net_name: str, a: Tuple[int, int], b: Tuple[int, int]) -> bool:
    cells = _line_cells(a, b)
    if not cells:
        return False
    for c in cells:
        owner = occ_layer.get(c)
        if owner not in (None, net_name):
            return False
    return True


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

    def add_top_line(a: Tuple[int, int], b: Tuple[int, int], start_xy: Optional[Tuple[float, float]] = None) -> None:
        x1i, y1i = a
        x2i, y2i = b
        if start_xy is None:
            sx, sy = _to_coord(x1i), _to_coord(y1i)
        else:
            sx, sy = start_xy
        result.segments.append(
            RoutedSegment(net=net_name, layer=LAYER_TOP, x1=sx, y1=sy, x2=_to_coord(x2i), y2=_to_coord(y2i))
        )
        _mark_line_cells(occ[0], net_name, a, b)

    # Prefer top-layer direct/orthogonal stub if clear.
    if x1 != x2 and y1 != y2:
        mid1 = (x2, y1)
        mid2 = (x1, y2)
        if _line_is_clear(occ[0], net_name, pin_idx, mid1) and _line_is_clear(occ[0], net_name, mid1, esc_idx):
            add_top_line(pin_idx, mid1, start_xy=pin_xy)
            add_top_line(mid1, esc_idx)
            return
        if _line_is_clear(occ[0], net_name, pin_idx, mid2) and _line_is_clear(occ[0], net_name, mid2, esc_idx):
            add_top_line(pin_idx, mid2, start_xy=pin_xy)
            add_top_line(mid2, esc_idx)
            return
    else:
        if _line_is_clear(occ[0], net_name, pin_idx, esc_idx):
            add_top_line(pin_idx, esc_idx, start_xy=pin_xy)
            return

        # If top is blocked by other nets, drop to bottom with a via at pin_idx
        # so we avoid same-layer short/crossing stubs.
    if (x1 == x2 or y1 == y2) and _line_is_clear(occ[1], net_name, pin_idx, esc_idx):

        result.segments.append(
            RoutedSegment(net=net_name, layer=LAYER_TOP, x1=pin_xy[0], y1=pin_xy[1], x2=_to_coord(x1), y2=_to_coord(y1))
        )
        result.vias.append(
            Via(net=net_name, x=_to_coord(x1), y=_to_coord(y1), from_layer=LAYER_TOP, to_layer=LAYER_BOTTOM)
        )
        result.segments.append(
            RoutedSegment(net=net_name, layer=LAYER_BOTTOM, x1=_to_coord(x1), y1=_to_coord(y1), x2=_to_coord(x2),
                          y2=_to_coord(y2))
        )
        occ[0][pin_idx] = net_name
        _mark_line_cells(occ[1], net_name, pin_idx, esc_idx)
        return

  # Last resort: keep old behavior to avoid disconnecting pins.
    if x1 != x2 and y1 != y2:
        mid = (x2, y1)
        add_top_line(pin_idx, mid, start_xy=pin_xy)
        add_top_line(mid, esc_idx)
        return

    add_top_line(pin_idx, esc_idx, start_xy=pin_xy)


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

    # Start from top layer; if routing needs bottom it must place vias explicitly.
    layer = 0
    s = (start[0], start[1], layer)
    if walkable(s[0], s[1], layer):
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

def _collect_terminals_by_net(
    design: PCBDesign,
    include_ports: bool = False,
) -> Dict[str, List[Tuple[Tuple[float, float], Tuple[int, int], Tuple[int, int]]]]:
    terminals_by_net: Dict[str, List[Tuple[Tuple[float, float], Tuple[int, int], Tuple[int, int]]]] = {}
    used_escape: Set[Tuple[int, int]] = set()

    for net_name, net in design.nets.items():
        terminals: List[Tuple[Tuple[float, float], Tuple[int, int], Tuple[int, int]]] = []
        seen = set()
        for ref, pin_name in net.connections:
            if ref not in design.components:
                continue
            comp = design.components[ref]
            if (not include_ports) and _is_port_component(comp):
                continue

            pin_xy, pin_idx, esc_idx = _pin_terminal(comp, pin_name)
            step_x = esc_idx[0] - pin_idx[0]
            step_y = esc_idx[1] - pin_idx[1]
            if step_x == 0 and step_y == 0:
                step_x = 1

            # Avoid sharing the same escape grid by different nets/pins,
            # otherwise traces can appear to short/cross without vias.
            while esc_idx in used_escape and esc_idx != pin_idx:
                esc_idx = (esc_idx[0] + step_x, esc_idx[1] + step_y)

            key = (pin_idx, esc_idx)
            if key in seen:
                continue
            seen.add(key)
            terminals.append((pin_xy, pin_idx, esc_idx))
            used_escape.add(esc_idx)

        terminals_by_net[net_name] = terminals

    return terminals_by_net

def _net_routing_order(
    design: PCBDesign,
    terminals_by_net: Dict[str, List[Tuple[Tuple[float, float], Tuple[int, int], Tuple[int, int]]]],
) -> List[str]:
    def net_priority(net_name: str) -> Tuple[int, int, int, int]:
        terminals = terminals_by_net.get(net_name, [])
        points_idx = [esc for _, _, esc in terminals]
        if not points_idx:
            return (1, 0, 0)

        xs = [p[0] for p in points_idx]
        ys = [p[1] for p in points_idx]
        span = (max(xs) - min(xs)) + (max(ys) - min(ys))

        touches_port = 0
        touches_core_amp = 0
        for ref, _ in design.nets[net_name].connections:
            comp = design.components.get(ref)
            if comp is None:
                continue
            if _is_port_component(comp):
                touches_port = 1
            if ref in ("U9", "U11"):
                touches_core_amp = 1

        # Prioritize amplifier-core nets first to avoid pin-area starvation,
        # then long-span nets, and route port nets later.
        return (-touches_core_amp, -span, -len(points_idx), touches_port)

    return sorted(terminals_by_net.keys(), key=net_priority)

def route_design_v1(
    design: PCBDesign,
    include_ports: bool = False,
    via_penalty: float = 0.15,
) -> RoutingResult:
    result = RoutingResult()

    comp_blocked_top, comp_blocked_bottom = _build_component_obstacles_by_layer(design)

    occ: List[Dict[Tuple[int, int], str]] = [dict(), dict()]

    terminals_by_net = _collect_terminals_by_net(design, include_ports=include_ports)

    pin_escape_cells: Set[Tuple[int, int]] = set()
    for terminals in terminals_by_net.values():
        for _, pin_idx, esc_idx in terminals:
            pin_escape_cells.add(pin_idx)
            pin_escape_cells.add(esc_idx)

    for cell in pin_escape_cells:
        comp_blocked_top.discard(cell)
        comp_blocked_bottom.discard(cell)

    for xy in comp_blocked_top:
        occ[0][xy] = "__BLOCKED__"
    for xy in comp_blocked_bottom:
        occ[1][xy] = "__BLOCKED__"

    for net_name in _net_routing_order(design, terminals_by_net=terminals_by_net):
        terminals = terminals_by_net.get(net_name, [])

        if len(terminals) < 2:
            result.routed_nets[net_name] = True
            continue

        # Terminal escape points are already in grid indices.
        # Keep them as-is to avoid double conversion / scaling drift.
        points_idx = [esc for _, _, esc in terminals]
        always_allow = set(points_idx)

        occ_snapshot = [occ[0].copy(), occ[1].copy()]
        strategy_via_penalties = [via_penalty, max(0.01, via_penalty * 0.5)]
        strategy_edge_reverse = [True, False]

        ok = False
        committed_segments: List[RoutedSegment] = []
        committed_vias: List[Via] = []

        for edge_reverse in strategy_edge_reverse:
            if ok:
                break
            for vp in strategy_via_penalties:
                trial_occ = [occ_snapshot[0].copy(), occ_snapshot[1].copy()]
                net_result = RoutingResult()

                for pin_xy, pin_idx, esc_idx in terminals:
                    _add_pin_stub(net_result, trial_occ, net_name, pin_xy, pin_idx, esc_idx)

                edges = _mst_edges(points_idx)
                edges.sort(
                    key=lambda uv: abs(points_idx[uv[0]][0] - points_idx[uv[1]][0])
                                   + abs(points_idx[uv[0]][1] - points_idx[uv[1]][1]),
                    reverse=edge_reverse,
                )

                trial_ok = True
                for u, v in edges:
                    start = points_idx[u]
                    goal = points_idx[v]
                    path = _a_star_route(
                        board_w=design.board.width,
                        board_h=design.board.height,
                        start=start,
                        goal=goal,
                        occ=trial_occ,
                        net_name=net_name,
                        always_allow=always_allow,
                        via_penalty=vp,
                    )
                    if path is None:
                        trial_ok = False
                        break

                    # If A* reaches the goal on bottom layer, force a final via back
                    # to top at the same XY so terminal connectivity remains explicit.
                    if path[-1][2] != 0:
                        gx, gy, _ = path[-1]
                        path.append((gx, gy, 0))

                    segs, vias = _path_to_primitives(net_name, path)
                    net_result.segments.extend(segs)
                    net_result.vias.extend(vias)

                    for x, y, layer in path:
                        owner = trial_occ[layer].get((x, y))
                        if owner in (None, net_name):
                            trial_occ[layer][(x, y)] = net_name

                if trial_ok:
                    ok = True
                    occ[0] = trial_occ[0]
                    occ[1] = trial_occ[1]
                    committed_segments = net_result.segments
                    committed_vias = net_result.vias
                    break

        if ok:
            result.segments.extend(committed_segments)
            result.vias.extend(committed_vias)
        else:
            occ[0] = occ_snapshot[0]
            occ[1] = occ_snapshot[1]

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
