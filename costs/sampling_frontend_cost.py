from core.models import PCBDesign
from core.geometry import manhattan, euclidean
from costs.generic_cost import GenericPlacementCost
from costs.base_cost import is_real_component


class PlacementCost(GenericPlacementCost):
    def __init__(self, design: PCBDesign, rules):
        super().__init__(design)
        self.rules = rules

        # Sampling-frontend-specific weights
        self.w_supply_decap = 10.0
        self.w_channel_group = 1.5
        self.w_stage_match = 3.5
        self.w_chip_close = 20.0
        self.w_chip_rotation = 10.0

        self.w_rect_struct = 20.0
        self.w_flow = 24.0
        self.w_row_align = 10.0
        self.w_port_bind = 3.0

        self.w_compact = 6.5
        self.w_order = 20.0
        self.w_opamp_gap = 24.0

    def supply_decap_cost(self) -> float:
        total = 0.0

        for u_ref, pwr_pin, c_ref in self.rules.decap_rules:
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

        anchor_refs = self.rules.anchor_refs
        if len(anchor_refs) < 2:
            return 0.0
        if anchor_refs[0] not in self.design.components or anchor_refs[1] not in self.design.components:
            return 0.0

        a0x, _ = self.design.components[anchor_refs[0]].center()
        a1x, _ = self.design.components[anchor_refs[1]].center()

        anchor_x = (a0x + a1x) / 2.0 - 8.0
        anchors = {
            ch: (anchor_x, self.rules.channel_rows[ch])
            for ch in self.rules.channel_groups.keys()
            if ch in self.rules.channel_rows
        }

        for ch, refs in self.rules.channel_groups.items():
            if ch not in anchors:
                continue
            ax, ay = anchors[ch]
            for ref in refs:
                if ref in self.design.components:
                    cx, cy = self.design.components[ref].center()
                    total += 0.18 * (abs(cx - ax) + abs(cy - ay))

        return total

    def stage_match_cost(self) -> float:
        total = 0.0

        for a, b in self.rules.stage_match_pairs:
            if a[0] not in self.design.components or b[0] not in self.design.components:
                continue

            p1 = self.design.components[a[0]].rotated_pin_position(a[1])
            p2 = self.design.components[b[0]].rotated_pin_position(b[1])
            total += manhattan(p1, p2)

        return total

    def chip_close_cost(self) -> float:
        anchor_refs = self.rules.anchor_refs
        if len(anchor_refs) < 2:
            return 0.0
        if anchor_refs[0] not in self.design.components or anchor_refs[1] not in self.design.components:
            return 0.0

        c0 = self.design.components[anchor_refs[0]]
        c1 = self.design.components[anchor_refs[1]]
        d = euclidean(c0.center(), c1.center())

        d_target = 18.0
        if d <= d_target:
            return 0.0
        return (d - d_target) ** 2

    def chip_rotation_cost(self) -> float:
        total = 0.0
        for ref in self.rules.anchor_refs:
            if ref not in self.design.components:
                continue

            rot = self.design.components[ref].rotation
            if rot not in (0, 180):
                total += 2.0

        return total

    def rectangular_structure_cost(self) -> float:
        total = 0.0
        anchor_refs = self.rules.anchor_refs
        if len(anchor_refs) < 2:
            return 0.0
        if anchor_refs[0] not in self.design.components or anchor_refs[1] not in self.design.components:
            return 0.0

        c0 = self.design.components[anchor_refs[0]]
        c1 = self.design.components[anchor_refs[1]]

        total += abs(c0.y - self.rules.target_chip_row_y) * 0.4
        total += abs(c1.y - self.rules.target_chip_row_y) * 0.4

        if c0.x >= c1.x - self.rules.min_chip_dx:
            total += (c0.x - (c1.x - self.rules.min_chip_dx) + 1.0) * 10.0

        for uref, _, cref in self.rules.decap_rules:
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
        anchor_refs = self.rules.anchor_refs
        if len(anchor_refs) < 2:
            return 0.0
        if anchor_refs[0] not in self.design.components or anchor_refs[1] not in self.design.components:
            return 0.0

        c0 = self.design.components[anchor_refs[0]]
        c1 = self.design.components[anchor_refs[1]]

        for ref in self.rules.input_group:
            if ref not in self.design.components:
                continue

            c = self.design.components[ref]
            if c.x > c0.x - 1.0:
                total += (c.x - (c0.x - 1.0)) * 4.2

        for ref in self.rules.output_group:
            if ref not in self.design.components:
                continue

            c = self.design.components[ref]
            if c.x < c1.x + 1.0:
                total += ((c1.x + 1.0) - c.x) * 4.8

        if c1.x < c0.x + self.rules.preferred_flow_dx:
            total += ((c0.x + self.rules.preferred_flow_dx) - c1.x) * 8.0

        return total

    def row_alignment_cost(self) -> float:
        total = 0.0

        for ch, refs in self.rules.channel_groups.items():
            if ch not in self.rules.channel_rows:
                continue
            y_target = self.rules.channel_rows[ch]
            for ref in refs:
                if ref not in self.design.components:
                    continue
                c = self.design.components[ref]
                total += abs(c.y - y_target) * 0.35

        if len(self.rules.anchor_refs) >= 2:
            pin_row_pairs = [
                ((self.rules.anchor_refs[0], "1"), "CH1"), ((self.rules.anchor_refs[1], "1"), "CH1"),
                ((self.rules.anchor_refs[0], "7"), "CH2"), ((self.rules.anchor_refs[1], "7"), "CH2"),
                ((self.rules.anchor_refs[0], "8"), "CH3"), ((self.rules.anchor_refs[1], "8"), "CH3"),
                ((self.rules.anchor_refs[0], "14"), "CH4"), ((self.rules.anchor_refs[1], "14"), "CH4"),
            ]

            for (ref, pin), ch in pin_row_pairs:
                if ref not in self.design.components or ch not in self.rules.channel_rows:
                    continue
                _, py = self.design.components[ref].rotated_pin_position(pin)
                total += abs(py - self.rules.channel_rows[ch]) * 0.55

        return total

    def port_binding_cost(self) -> float:
        total = 0.0

        for pref, refs in self.rules.port_input_bind.items():
            if pref not in self.design.components:
                continue

            p = self.design.components[pref].center()
            for ref in refs:
                if ref not in self.design.components:
                    continue
                c = self.design.components[ref].center()
                total += 0.08 * manhattan(p, c)

        for pref, refs in self.rules.port_output_bind.items():
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
        anchor_refs = self.rules.anchor_refs
        if len(anchor_refs) < 2:
            return 0.0
        if anchor_refs[0] not in self.design.components or anchor_refs[1] not in self.design.components:
            return 0.0

        total = 0.0
        c0 = self.design.components[anchor_refs[0]]
        c1 = self.design.components[anchor_refs[1]]

        if c1.x < c0.x + self.rules.min_chip_dx:
            total += ((c0.x + self.rules.min_chip_dx) - c1.x) ** 2

        for ref in self.rules.input_group:
            if ref not in self.design.components:
                continue
            c = self.design.components[ref]
            if c.x > c0.x - 1.5:
                total += (c.x - (c0.x - 1.5)) ** 2

        for ref in self.rules.output_group:
            if ref not in self.design.components:
                continue
            c = self.design.components[ref]
            if c.x < c1.x + 1.5:
                total += ((c1.x + 1.5) - c.x) ** 2

        return total

    def opamp_gap_cost(self) -> float:
        anchor_refs = self.rules.anchor_refs
        if len(anchor_refs) < 2:
            return 0.0
        if anchor_refs[0] not in self.design.components or anchor_refs[1] not in self.design.components:
            return 0.0

        c0 = self.design.components[anchor_refs[0]]
        c1 = self.design.components[anchor_refs[1]]

        dx = c1.x - c0.x
        dy = c1.y - c0.y

        return (
            2.0 * (dx - self.rules.target_chip_dx) ** 2
            + 0.8 * (dy - self.rules.target_chip_dy) ** 2
        )

    def total_cost(self) -> float:
        return (
            super().total_cost()
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
        return super().local_cost_for_component(ref)