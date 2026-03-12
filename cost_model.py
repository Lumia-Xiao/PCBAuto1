from models import PCBDesign
from geometry import (
    manhattan,
    euclidean,
    overlap_area,
    out_of_bounds_penalty,
    out_of_region_penalty,
)


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


class PlacementCost:
    def __init__(self, design: PCBDesign):
        self.design = design

        # Base weights
        self.w_wire = 0.24
        self.w_overlap = 7000.0
        self.w_boundary = 1800.0
        self.w_region = 3200.0
        self.w_supply_decap = 10.0

        self.w_channel_group = 1.5
        self.w_stage_match = 3.5
        self.w_chip_close = 20.0
        self.w_chip_rotation = 10.0

        self.w_rect_struct = 20.0
        self.w_flow = 24.0
        self.w_row_align = 10.0
        self.w_port_bind = 3.0

        # Tight compaction weights
        self.w_compact = 6.5
        self.w_order = 20.0
        self.w_opamp_gap = 24.0

        self.channel_groups = {
            "CH1": ["R65", "R68", "R33", "R35", "R11", "R38", "C46", "C48"],
            "CH2": ["R102", "R103", "R42", "R44", "R40", "R46", "C50", "C52"],
            "CH3": ["R48", "R50", "R52", "R54", "C54", "C56"],
            "CH4": ["R56", "R58", "R60", "R62", "C58", "C60"],
        }

        self.channel_rows = {
            "CH1": 72.0,
            "CH2": 55.0,
            "CH3": 36.0,
            "CH4": 18.0,
        }

        self.input_group = {
            "R65", "R68", "R33", "R35", "R11", "C46",
            "R102", "R103", "R42", "R44", "R40", "C50",
            "R48", "R50", "R52", "C54",
            "R56", "R58", "R60", "C58",
        }

        self.output_group = {
            "R38", "C48",
            "R46", "C52",
            "R54", "C56",
            "R62", "C60",
        }

    def net_wire_cost(self) -> float:
        total = 0.0
        for net in self.design.nets.values():
            pts = []
            seen = set()

            for ref, pin in net.connections:
                comp = self.design.components[ref]
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

    def net_mst_wire_cost(self) -> float:
        total = 0.0
        for net in self.design.nets.values():
            pts = []
            seen = set()

            for ref, pin in net.connections:
                comp = self.design.components[ref]
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

    def total_routing_length(self) -> float:
        return self.net_mst_wire_cost()

    def report_wirelength(self) -> None:
        hpwl_total = self.net_wire_cost()
        mst_total = self.net_mst_wire_cost()

        print("\n===== Wirelength Report =====")
        print(f"HPWL total wirelength estimate = {hpwl_total:.3f} mm")
        print(f"MST  total wirelength estimate = {mst_total:.3f} mm")

    def report_wirelength_by_net(self) -> None:
        print("\n===== Wirelength by Net =====")
        grand_total = 0.0

        for net_name, net in self.design.nets.items():
            pts = []
            seen = set()

            for ref, pin in net.connections:
                comp = self.design.components[ref]
                if not is_real_component(comp):
                    continue

                key = (ref, pin)
                if key in seen:
                    continue
                seen.add(key)
                pts.append(comp.rotated_pin_position(pin))

            if len(pts) < 2:
                length = 0.0
            else:
                length = mst_manhattan_length(pts)

            grand_total += length
            print(f"{net_name:>20s}: {length:8.3f} mm")

        print(f"{'TOTAL':>20s}: {grand_total:8.3f} mm")

    def overlap_cost(self) -> float:
        comps = [c for c in self.design.components.values() if is_real_component(c)]
        total = 0.0
        for i in range(len(comps)):
            for j in range(i + 1, len(comps)):
                total += overlap_area(comps[i], comps[j])
        return total

    def overlap_cost_for_component(self, ref: str) -> float:
        target = self.design.components[ref]
        if not is_real_component(target):
            return 0.0

        total = 0.0
        for other_ref, other in self.design.components.items():
            if other_ref == ref:
                continue
            if not is_real_component(other):
                continue
            total += overlap_area(target, other)
        return total

    def boundary_cost(self) -> float:
        total = 0.0
        for comp in self.design.components.values():
            if not is_real_component(comp):
                continue
            total += out_of_bounds_penalty(comp, self.design.board)
        return total

    def region_cost(self) -> float:
        if self.design.region is None:
            return 0.0
        total = 0.0
        for comp in self.design.components.values():
            if not is_real_component(comp):
                continue
            total += out_of_region_penalty(comp, self.design.region)
        return total

    def supply_decap_cost(self) -> float:
        total = 0.0
        pairs = [
            ("U9", "4", "C38"),
            ("U9", "4", "C42"),
            ("U11", "4", "C39"),
            ("U11", "4", "C43"),
        ]
        for u_ref, pwr_pin, c_ref in pairs:
            if u_ref not in self.design.components or c_ref not in self.design.components:
                continue
            p_u = self.design.components[u_ref].rotated_pin_position(pwr_pin)
            c = self.design.components[c_ref]
            p_c1 = c.rotated_pin_position("1")
            p_c2 = c.rotated_pin_position("2")
            total += min(manhattan(p_u, p_c1), manhattan(p_u, p_c2))
        return total

    def channel_group_cost(self) -> float:
        total = 0.0
        if "U9" not in self.design.components or "U11" not in self.design.components:
            return 0.0

        u9x, _ = self.design.components["U9"].center()
        u11x, _ = self.design.components["U11"].center()

        anchor_x = (u9x + u11x) / 2 - 8.0
        anchors = {
            "CH1": (anchor_x, self.channel_rows["CH1"]),
            "CH2": (anchor_x, self.channel_rows["CH2"]),
            "CH3": (anchor_x, self.channel_rows["CH3"]),
            "CH4": (anchor_x, self.channel_rows["CH4"]),
        }

        for ch, refs in self.channel_groups.items():
            ax, ay = anchors[ch]
            for ref in refs:
                if ref in self.design.components:
                    cx, cy = self.design.components[ref].center()
                    total += 0.18 * (abs(cx - ax) + abs(cy - ay))
        return total

    def stage_match_cost(self) -> float:
        total = 0.0
        pairs = [
            (("U9", "1"), ("U11", "3")),
            (("U9", "7"), ("U11", "5")),
            (("U9", "8"), ("U11", "10")),
            (("U9", "14"), ("U11", "12")),
        ]
        for a, b in pairs:
            if a[0] not in self.design.components or b[0] not in self.design.components:
                continue
            p1 = self.design.components[a[0]].rotated_pin_position(a[1])
            p2 = self.design.components[b[0]].rotated_pin_position(b[1])
            total += manhattan(p1, p2)
        return total

    def chip_close_cost(self) -> float:
        if "U9" not in self.design.components or "U11" not in self.design.components:
            return 0.0

        u9 = self.design.components["U9"]
        u11 = self.design.components["U11"]
        d = euclidean(u9.center(), u11.center())

        d_target = 18.0
        if d <= d_target:
            return 0.0
        return (d - d_target) ** 2

    def chip_rotation_cost(self) -> float:
        total = 0.0
        for ref in ("U9", "U11"):
            if ref not in self.design.components:
                continue
            rot = self.design.components[ref].rotation
            if rot not in (0, 180):
                total += 2.0
        return total

    def rectangular_structure_cost(self) -> float:
        total = 0.0
        if "U9" not in self.design.components or "U11" not in self.design.components:
            return 0.0

        u9 = self.design.components["U9"]
        u11 = self.design.components["U11"]

        total += abs(u9.y - 47.0) * 0.4
        total += abs(u11.y - 47.0) * 0.4

        if u9.x >= u11.x - 10.0:
            total += (u9.x - (u11.x - 10.0) + 1.0) * 10.0

        for cref, uref in [("C38", "U9"), ("C42", "U9"), ("C39", "U11"), ("C43", "U11")]:
            if cref not in self.design.components or uref not in self.design.components:
                continue
            c = self.design.components[cref]
            u = self.design.components[uref]
            if c.y < u.y:
                total += (u.y - c.y) * 3.0
            total += 0.2 * abs(c.x - u.x)

        return total

    def flow_direction_cost(self) -> float:
        total = 0.0
        if "U9" not in self.design.components or "U11" not in self.design.components:
            return 0.0

        u9 = self.design.components["U9"]
        u11 = self.design.components["U11"]

        for ref in self.input_group:
            if ref not in self.design.components:
                continue
            c = self.design.components[ref]
            if c.x > u9.x - 1.0:
                total += (c.x - (u9.x - 1.0)) * 4.2

        for ref in self.output_group:
            if ref not in self.design.components:
                continue
            c = self.design.components[ref]
            if c.x < u11.x + 1.0:
                total += ((u11.x + 1.0) - c.x) * 4.8

        if u11.x < u9.x + 12.0:
            total += ((u9.x + 12.0) - u11.x) * 8.0

        return total

    def row_alignment_cost(self) -> float:
        total = 0.0
        for ch, refs in self.channel_groups.items():
            y_target = self.channel_rows[ch]
            for ref in refs:
                if ref not in self.design.components:
                    continue
                c = self.design.components[ref]
                total += abs(c.y - y_target) * 0.35

        pin_row_pairs = [
            (("U9", "1"), "CH1"), (("U11", "1"), "CH1"),
            (("U9", "7"), "CH2"), (("U11", "7"), "CH2"),
            (("U9", "8"), "CH3"), (("U11", "8"), "CH3"),
            (("U9", "14"), "CH4"), (("U11", "14"), "CH4"),
        ]
        for (ref, pin), ch in pin_row_pairs:
            if ref not in self.design.components:
                continue
            _, py = self.design.components[ref].rotated_pin_position(pin)
            total += abs(py - self.channel_rows[ch]) * 0.55

        return total

    def port_binding_cost(self) -> float:
        total = 0.0

        port_to_input_passives = {
            "P_IS3": ["R33", "R65", "R68"],
            "P_IS4": ["R42", "R102", "R103"],
            "P_VP3": ["R50"],
            "P_VN3": ["R52"],
            "P_VP4": ["R58"],
            "P_VN4": ["R60"],
        }

        for pref, refs in port_to_input_passives.items():
            if pref not in self.design.components:
                continue
            p = self.design.components[pref].center()
            for ref in refs:
                if ref not in self.design.components:
                    continue
                c = self.design.components[ref].center()
                total += 0.08 * manhattan(p, c)

        out_bind = {
            "P_ADCB1": ["U11"],
            "P_ADCB2": ["U11"],
            "P_ADCB3": ["U11"],
            "P_ADCD0": ["U11"],
        }
        for pref, refs in out_bind.items():
            if pref not in self.design.components:
                continue
            p = self.design.components[pref].center()
            for ref in refs:
                if ref not in self.design.components:
                    continue
                c = self.design.components[ref].center()
                total += 0.04 * manhattan(p, c)

        return total

    def compactness_cost(self) -> float:
        comps = [c for c in self.design.components.values() if is_real_component(c)]
        if not comps:
            return 0.0

        xs = [c.x for c in comps]
        ys = [c.y for c in comps]
        cx = sum(xs) / len(xs)
        cy = sum(ys) / len(ys)

        total = 0.0
        for c in comps:
            dx = c.x - cx
            dy = c.y - cy
            total += 1.4 * dx * dx + 1.0 * dy * dy

        x1 = min(c.bounding_box()[0] for c in comps)
        y1 = min(c.bounding_box()[1] for c in comps)
        x2 = max(c.bounding_box()[2] for c in comps)
        y2 = max(c.bounding_box()[3] for c in comps)
        w = x2 - x1
        h = y2 - y1
        total += 8.0 * w + 6.0 * h

        return total

    def ordering_cost(self) -> float:
        if "U9" not in self.design.components or "U11" not in self.design.components:
            return 0.0

        total = 0.0
        u9 = self.design.components["U9"]
        u11 = self.design.components["U11"]

        if u11.x < u9.x + 10.0:
            total += ((u9.x + 10.0) - u11.x) ** 2

        for ref in self.input_group:
            if ref not in self.design.components:
                continue
            c = self.design.components[ref]
            if c.x > u9.x - 1.5:
                total += (c.x - (u9.x - 1.5)) ** 2

        for ref in self.output_group:
            if ref not in self.design.components:
                continue
            c = self.design.components[ref]
            if c.x < u11.x + 1.5:
                total += ((u11.x + 1.5) - c.x) ** 2

        return total

    def opamp_gap_cost(self) -> float:
        if "U9" not in self.design.components or "U11" not in self.design.components:
            return 0.0

        u9 = self.design.components["U9"]
        u11 = self.design.components["U11"]

        dx = u11.x - u9.x
        dy = u11.y - u9.y

        target_dx = 14.0
        target_dy = 0.0
        return 2.0 * (dx - target_dx) ** 2 + 0.8 * (dy - target_dy) ** 2

    def total_cost(self) -> float:
        return (
            self.w_wire * self.net_wire_cost()
            + self.w_overlap * self.overlap_cost()
            + self.w_boundary * self.boundary_cost()
            + self.w_region * self.region_cost()
            + self.w_supply_decap * self.supply_decap_cost()
            + self.w_channel_group * self.channel_group_cost()
            + self.w_stage_match * self.stage_match_cost()
            + self.w_chip_close * self.chip_close_cost()
            + self.w_chip_rotation * self.chip_rotation_cost()
            + self.w_rect_struct * self.rectangular_structure_cost()
            + self.w_flow * self.flow_direction_cost()
            + self.w_row_align * self.row_alignment_cost()
            + self.w_port_bind * self.port_binding_cost()
            + self.w_compact * self.compactness_cost()
            + self.w_order * self.ordering_cost()
            + self.w_opamp_gap * self.opamp_gap_cost()
        )

    def local_cost_for_component(self, ref: str) -> float:
        """
        Approximate local cost used for incremental annealing.
        Only includes terms that can be significantly affected by moving one component.
        """
        if ref not in self.design.components:
            return 0.0

        comp = self.design.components[ref]
        if not is_real_component(comp):
            return 0.0

        total = 0.0

        # 1) overlap / boundary / region: definitely local
        total += self.w_overlap * self.overlap_cost_for_component(ref)

        total += self.w_boundary * out_of_bounds_penalty(comp, self.design.board)

        if self.design.region is not None:
            total += self.w_region * out_of_region_penalty(comp, self.design.region)

        # 2) local wire cost: only nets touching this component
        touched_nets = []
        for net in self.design.nets.values():
            for conn_ref, _ in net.connections:
                if conn_ref == ref:
                    touched_nets.append(net)
                    break

        local_wire = 0.0
        for net in touched_nets:
            pts = []
            seen = set()
            for conn_ref, pin in net.connections:
                c = self.design.components[conn_ref]
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

        total += self.w_wire * local_wire

        return total