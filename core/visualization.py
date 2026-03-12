import matplotlib.pyplot as plt
from matplotlib.patches import Rectangle

from core.models import PCBDesign
from core.layout_utils import get_layout_bbox


def draw_design(design: PCBDesign, title: str = "PCB Placement") -> None:
    fig, ax = plt.subplots(figsize=(15, 10))

    ax.add_patch(Rectangle((0, 0), design.board.width, design.board.height, fill=False, linewidth=2.2))

    if design.region is not None:
        ax.add_patch(Rectangle(
            (design.region.x_min, design.region.y_min),
            design.region.width,
            design.region.height,
            fill=False,
            linestyle="--",
            linewidth=1.6,
            edgecolor="red",
            alpha=0.8,
        ))

    bx1, by1, bx2, by2 = get_layout_bbox(design, margin=0.0, exclude_ports=True)
    ax.add_patch(Rectangle(
        (bx1, by1),
        bx2 - bx1,
        by2 - by1,
        fill=False,
        linestyle="-",
        linewidth=1.8,
        edgecolor="blue",
        alpha=0.9,
    ))

    for comp in design.components.values():
        x1, y1, x2, y2 = comp.bounding_box()
        w = x2 - x1
        h = y2 - y1

        if "SOIC14" in comp.comp_type:
            color = "#d6e6ff"
        elif "RES" in comp.comp_type:
            color = "#dff2df"
        elif "CAP" in comp.comp_type:
            color = "#ffe8cc"
        else:
            color = "#f4d7ff"

        rect = Rectangle((x1, y1), w, h, facecolor=color, edgecolor="black", linewidth=1.2)
        ax.add_patch(rect)

        if "PORT" in comp.comp_type:
            ax.text(comp.x, comp.y, comp.ref.replace("P_", ""), ha="center", va="center", fontsize=8, weight="bold")
        else:
            ax.text(comp.x, comp.y, f"{comp.ref}\n{comp.rotation}°", ha="center", va="center", fontsize=7)

        for pin_name in comp.pins:
            px, py = comp.rotated_pin_position(pin_name)
            ax.plot(px, py, "ko", markersize=2.2)

    if design.routing_result is not None and design.routing_result.segments:
        layer_colors = {
            "L1_TOP": "#1f77b4",
            "L4_BOTTOM": "#d62728",
        }
        for seg in design.routing_result.segments:
            ax.plot(
                [seg.x1, seg.x2],
                [seg.y1, seg.y2],
                "-",
                linewidth=1.4,
                alpha=0.85,
                color=layer_colors.get(seg.layer, "#555555"),
            )

        for via in design.routing_result.vias:
            ax.plot(via.x, via.y, "o", markersize=3.2, color="#8a2be2", alpha=0.95)
    else:
        for net in design.nets.values():
            pts = [design.components[ref].rotated_pin_position(pin_name) for ref, pin_name in net.connections]
            if len(pts) >= 2:
                p0 = pts[0]
                for p in pts[1:]:
                    ax.plot([p0[0], p[0]], [p0[1], p[1]], "--", linewidth=0.8, alpha=0.35)

    for y, label in [(72, "CH1"), (55, "CH2"), (36, "CH3"), (18, "CH4")]:
        ax.plot([0, design.board.width], [y, y], ":", linewidth=0.8, color="gray", alpha=0.25)
        ax.text(2.0, y + 1.2, label, fontsize=8, color="gray")

    ax.text(8, 90, "INPUT SIDE", fontsize=10, weight="bold")
    ax.text(108, 90, "OUTPUT SIDE", fontsize=10, weight="bold")

    if design.region is not None:
        ax.text(
            design.region.x_min + 1.0,
            design.region.y_max + 1.5,
            f"Constraint region: W={design.region.width:.1f}, H={design.region.height:.1f}",
            fontsize=9,
            color="red",
            weight="bold"
        )

    ax.text(
        bx1 + 1.0,
        by2 + 1.5,
        f"Real bbox: W={bx2 - bx1:.1f}, H={by2 - by1:.1f}",
        fontsize=9,
        color="blue",
        weight="bold"
    )

    ax.set_title(title)
    ax.set_xlim(-3, design.board.width + 3)
    ax.set_ylim(-3, design.board.height + 3)
    ax.set_aspect("equal")
    ax.grid(True, linestyle=":")
    plt.show()