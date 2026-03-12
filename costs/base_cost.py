from core.geometry import overlap_area, out_of_bounds_penalty, out_of_region_penalty
from config import MIN_COMPONENT_SPACING

def is_port_component(comp) -> bool:
    ct = str(getattr(comp, "comp_type", "")).upper()
    ref = str(getattr(comp, "ref", "")).upper()
    return ("PORT" in ct) or ref.startswith("P_") or ref.startswith("PORT_")


def is_real_component(comp) -> bool:
    return not is_port_component(comp)


def manhattan_point(p1, p2) -> float:
    return abs(p1[0] - p2[0]) + abs(p1[1] - p2[1])


def mst_manhattan_length(points) -> float:
    if len(points) <= 1:
        return 0.0

    n = len(points)
    visited = [False] * n
    dist = [float("inf")] * n
    dist[0] = 0.0
    total = 0.0

    for _ in range(n):
        u = -1
        best = float("inf")
        for i in range(n):
            if not visited[i] and dist[i] < best:
                best = dist[i]
                u = i

        if u == -1:
            break

        visited[u] = True
        total += dist[u]

        for v in range(n):
            if not visited[v]:
                d = manhattan_point(points[u], points[v])
                if d < dist[v]:
                    dist[v] = d

    return total


def net_wire_cost(design) -> float:
    total = 0.0
    for net in design.nets.values():
        pts = []
        seen = set()

        for ref, pin in net.connections:
            if ref not in design.components:
                continue
            comp = design.components[ref]
            if not is_real_component(comp):
                continue
            if ref in seen:
                continue
            seen.add(ref)
            pts.append(comp.rotated_pin_position(pin))

        if len(pts) < 2:
            continue

        xs = [p[0] for p in pts]
        ys = [p[1] for p in pts]
        total += (max(xs) - min(xs)) + (max(ys) - min(ys))
    return total


def net_mst_wire_cost(design) -> float:
    total = 0.0
    for net in design.nets.values():
        pts = []
        seen = set()

        for ref, pin in net.connections:
            if ref not in design.components:
                continue
            comp = design.components[ref]
            if not is_real_component(comp):
                continue

            key = (ref, pin)
            if key in seen:
                continue
            seen.add(key)
            pts.append(comp.rotated_pin_position(pin))

        if len(pts) >= 2:
            total += mst_manhattan_length(pts)

    return total


def overlap_cost(design) -> float:
    comps = [c for c in design.components.values() if is_real_component(c)]
    total = 0.0
    for i in range(len(comps)):
        for j in range(i + 1, len(comps)):
            total += overlap_area(comps[i], comps[j])
    return total


def overlap_cost_for_component(design, ref: str) -> float:
    target = design.components[ref]
    if not is_real_component(target):
        return 0.0

    total = 0.0
    for other_ref, other in design.components.items():
        if other_ref == ref:
            continue
        if not is_real_component(other):
            continue
        total += overlap_area(target, other)
    return total

def _pair_clearance_shortfall(a, b, min_spacing: float) -> float:
    ax1, ay1, ax2, ay2 = a.bounding_box()
    bx1, by1, bx2, by2 = b.bounding_box()

    dx = max(0.0, max(bx1 - ax2, ax1 - bx2))
    dy = max(0.0, max(by1 - ay2, ay1 - by2))

    if dx > 0.0 and dy > 0.0:
        edge_dist = (dx * dx + dy * dy) ** 0.5
    else:
        edge_dist = max(dx, dy)

    return max(0.0, min_spacing - edge_dist)


def spacing_cost(design, min_spacing: float = MIN_COMPONENT_SPACING) -> float:
    comps = [c for c in design.components.values() if is_real_component(c)]
    total = 0.0
    for i in range(len(comps)):
        for j in range(i + 1, len(comps)):
            shortfall = _pair_clearance_shortfall(comps[i], comps[j], min_spacing)
            if shortfall > 0.0:
                total += shortfall * shortfall
    return total


def spacing_cost_for_component(design, ref: str, min_spacing: float = MIN_COMPONENT_SPACING) -> float:
    target = design.components[ref]
    if not is_real_component(target):
        return 0.0

    total = 0.0
    for other_ref, other in design.components.items():
        if other_ref == ref:
            continue
        if not is_real_component(other):
            continue
        shortfall = _pair_clearance_shortfall(target, other, min_spacing)
        if shortfall > 0.0:
            total += shortfall * shortfall
    return total

def boundary_cost(design) -> float:
    total = 0.0
    for comp in design.components.values():
        if not is_real_component(comp):
            continue
        total += out_of_bounds_penalty(comp, design.board)
    return total


def region_cost(design) -> float:
    if design.region is None:
        return 0.0

    total = 0.0
    for comp in design.components.values():
        if not is_real_component(comp):
            continue
        total += out_of_region_penalty(comp, design.region)
    return total


def local_wire_cost_for_component(design, ref: str) -> float:
    touched_nets = []
    for net in design.nets.values():
        for conn_ref, _ in net.connections:
            if conn_ref == ref:
                touched_nets.append(net)
                break

    local_wire = 0.0
    for net in touched_nets:
        pts = []
        seen = set()
        for conn_ref, pin in net.connections:
            if conn_ref not in design.components:
                continue
            c = design.components[conn_ref]
            if not is_real_component(c):
                continue
            key = (conn_ref, pin)
            if key in seen:
                continue
            seen.add(key)
            pts.append(c.rotated_pin_position(pin))

        if len(pts) >= 2:
            xs = [p[0] for p in pts]
            ys = [p[1] for p in pts]
            local_wire += (max(xs) - min(xs)) + (max(ys) - min(ys))

    return local_wire