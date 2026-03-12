from __future__ import annotations

from typing import Tuple

from config import snap_to_grid
from core.models import PCBDesign


def is_port_component(comp) -> bool:
    ct = str(getattr(comp, "comp_type", "")).upper()
    ref = str(getattr(comp, "ref", "")).upper()
    return ("PORT" in ct) or ref.startswith("P_") or ref.startswith("PORT_")


def get_layout_bbox(
    design: PCBDesign,
    margin: float = 0.0,
    exclude_ports: bool = True
) -> Tuple[float, float, float, float]:
    """
    只统计真实元件，不让 PORT 把 compaction 区域撑大。
    """
    xs1, ys1, xs2, ys2 = [], [], [], []

    for comp in design.components.values():
        if exclude_ports and is_port_component(comp):
            continue
        x1, y1, x2, y2 = comp.bounding_box()
        xs1.append(x1)
        ys1.append(y1)
        xs2.append(x2)
        ys2.append(y2)

    if not xs1:
        return 0.0, 0.0, 100.0, 80.0

    return (
        min(xs1) - margin,
        min(ys1) - margin,
        max(xs2) + margin,
        max(ys2) + margin,
    )


def compact_translate_to_region_corner(design: PCBDesign, padding: float = 1.0) -> None:
    if design.region is None:
        return

    x1, y1, _, _ = get_layout_bbox(design, margin=0.0, exclude_ports=True)

    dx = (design.region.x_min + padding) - x1
    dy = (design.region.y_min + padding) - y1

    for comp in design.components.values():
        if is_port_component(comp):
            continue
        comp.x = snap_to_grid(comp.x + dx)
        comp.y = snap_to_grid(comp.y + dy)


def compact_center_into_region(design: PCBDesign) -> None:
    if design.region is None:
        return

    x1, y1, x2, y2 = get_layout_bbox(design, margin=0.0, exclude_ports=True)
    bx = 0.5 * (x1 + x2)
    by = 0.5 * (y1 + y2)

    rx = 0.5 * (design.region.x_min + design.region.x_max)
    ry = 0.5 * (design.region.y_min + design.region.y_max)

    dx = rx - bx
    dy = ry - by

    for comp in design.components.values():
        if is_port_component(comp):
            continue
        comp.x = snap_to_grid(comp.x + dx)
        comp.y = snap_to_grid(comp.y + dy)