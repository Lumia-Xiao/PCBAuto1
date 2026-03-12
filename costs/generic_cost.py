from core.models import PCBDesign
from core.geometry import out_of_bounds_penalty, out_of_region_penalty
from costs.base_cost import (
    is_real_component,
    net_wire_cost,
    net_mst_wire_cost,
    overlap_cost,
    overlap_cost_for_component,
    boundary_cost,
    region_cost,
    local_wire_cost_for_component,
    spacing_cost,
    spacing_cost_for_component,
)


class GenericPlacementCost:
    def __init__(self, design: PCBDesign):
        self.design = design

        # Generic weights
        self.w_wire = 0.24
        self.w_overlap = 7000.0
        self.w_boundary = 1800.0
        self.w_region = 3200.0
        self.w_spacing = 2600.0

    def _is_real_component(self, comp) -> bool:
        return is_real_component(comp)

    def net_wire_cost(self) -> float:
        return net_wire_cost(self.design)

    def net_mst_wire_cost(self) -> float:
        return net_mst_wire_cost(self.design)

    def total_routing_length(self) -> float:
        return self.net_mst_wire_cost()

    def overlap_cost(self) -> float:
        return overlap_cost(self.design)

    def overlap_cost_for_component(self, ref: str) -> float:
        return overlap_cost_for_component(self.design, ref)

    def boundary_cost(self) -> float:
        return boundary_cost(self.design)

    def region_cost(self) -> float:
        return region_cost(self.design)

    def spacing_cost(self) -> float:
        return spacing_cost(self.design)

    def total_cost(self) -> float:
        return (
            self.w_wire * self.net_wire_cost()
            + self.w_overlap * self.overlap_cost()
            + self.w_boundary * self.boundary_cost()
            + self.w_region * self.region_cost()
            + self.w_spacing * self.spacing_cost()
        )

    def local_cost_for_component(self, ref: str) -> float:
        if ref not in self.design.components:
            return 0.0

        comp = self.design.components[ref]
        if not self._is_real_component(comp):
            return 0.0

        total = 0.0
        total += self.w_overlap * self.overlap_cost_for_component(ref)
        total += self.w_boundary * out_of_bounds_penalty(comp, self.design.board)

        if self.design.region is not None:
            total += self.w_region * out_of_region_penalty(comp, self.design.region)

        total += self.w_wire * local_wire_cost_for_component(self.design, ref)
        total += self.w_spacing * spacing_cost_for_component(self.design, ref)
        return total

    def report_wirelength(self) -> None:
        hpwl = self.net_wire_cost()
        mst = self.net_mst_wire_cost()

        print("===== Wirelength Report =====")
        print(f"HPWL total wirelength estimate = {hpwl:.3f} mm")
        print(f"MST  total wirelength estimate = {mst:.3f} mm")

    def report_wirelength_by_net(self) -> None:
        print("===== Wirelength By Net =====")

        hpwl_total = 0.0
        mst_total = 0.0

        for net_name, net in self.design.nets.items():
            pts_hpwl = []
            pts_mst = []
            seen_hpwl = set()
            seen_mst = set()

            for ref, pin in net.connections:
                if ref not in self.design.components:
                    continue

                comp = self.design.components[ref]
                if not self._is_real_component(comp):
                    continue

                if ref not in seen_hpwl:
                    seen_hpwl.add(ref)
                    pts_hpwl.append(comp.rotated_pin_position(pin))

                key = (ref, pin)
                if key not in seen_mst:
                    seen_mst.add(key)
                    pts_mst.append(comp.rotated_pin_position(pin))

            hpwl_len = 0.0
            mst_len = 0.0

            if len(pts_hpwl) >= 2:
                xs = [p[0] for p in pts_hpwl]
                ys = [p[1] for p in pts_hpwl]
                hpwl_len = (max(xs) - min(xs)) + (max(ys) - min(ys))

            if len(pts_mst) >= 2:
                from costs.base_cost import mst_manhattan_length
                mst_len = mst_manhattan_length(pts_mst)

            hpwl_total += hpwl_len
            mst_total += mst_len

            print(
                f"{net_name:>16s} : "
                f"HPWL = {hpwl_len:8.3f} mm | "
                f"MST = {mst_len:8.3f} mm"
            )

        print("----- Summary -----")
        print(f"HPWL total = {hpwl_total:.3f} mm")
        print(f"MST  total = {mst_total:.3f} mm")