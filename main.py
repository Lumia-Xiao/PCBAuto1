from config import GRID
from designs.sampling_frontend import create_sampling_design
from costs.sampling_frontend_cost import PlacementCost
from core.compaction import progressive_compaction
from core.visualization import draw_design
from core.layout_utils import get_layout_bbox


def preset_key_components(design, u9_x: float, u9_y: float, dx: float) -> None:
    """
    Force U9 and U11 to start from the same Y coordinate,
    while sweeping different X spacing.

    U11.x = U9.x + dx
    U11.y = U9.y
    """
    u11_x = u9_x + dx
    u11_y = u9_y

    if "U9" in design.components:
        design.components["U9"].x = u9_x
        design.components["U9"].y = u9_y
        design.components["U9"].rotation = 0

    if "U11" in design.components:
        design.components["U11"].x = u11_x
        design.components["U11"].y = u11_y
        design.components["U11"].rotation = 0

    # Decoupling / nearby capacitors follow the op-amp anchors
    # Keep them slightly above each chip as a better initial skeleton.
    relative_presets = {
        "C38": (-2.0, 1.4),
        "C42": (0.3, 1.4),
        "C39": (-2.0, 1.4),
        "C43": (0.3, 1.4),
    }

    for ref, (dx_local, dy_local) in relative_presets.items():
        if ref not in design.components:
            continue

        if ref in ("C38", "C42"):
            design.components[ref].x = u9_x + dx_local
            design.components[ref].y = u9_y + dy_local
        else:
            design.components[ref].x = u11_x + dx_local
            design.components[ref].y = u11_y + dy_local


def print_layout_summary(design, title: str) -> None:
    cost_model = PlacementCost(design)
    print(f"\n{title}")
    print("Total cost =", round(cost_model.total_cost(), 3))

    if design.region is not None:
        print(
            f"Constraint region: "
            f"x=({design.region.x_min:.2f}, {design.region.x_max:.2f}), "
            f"y=({design.region.y_min:.2f}, {design.region.y_max:.2f}), "
            f"W={design.region.width:.2f}, H={design.region.height:.2f}"
        )

    bx1, by1, bx2, by2 = get_layout_bbox(design, margin=0.0, exclude_ports=True)
    print(
        f"Real bbox: "
        f"x=({bx1:.2f}, {bx2:.2f}), "
        f"y=({by1:.2f}, {by2:.2f}), "
        f"W={bx2 - bx1:.2f}, H={by2 - by1:.2f}, "
        f"Area={(bx2 - bx1) * (by2 - by1):.2f}"
    )


def get_layout_metrics(design):
    """
    Main optimization target:
    1) minimize real bbox area
    2) then minimize routing length
    3) then minimize total cost
    """
    bx1, by1, bx2, by2 = get_layout_bbox(design, margin=0.0, exclude_ports=True)
    width = bx2 - bx1
    height = by2 - by1
    area = width * height

    cost_model = PlacementCost(design)
    routing = cost_model.total_routing_length()
    total_cost = cost_model.total_cost()

    return {
        "bbox_x1": bx1,
        "bbox_y1": by1,
        "bbox_x2": bx2,
        "bbox_y2": by2,
        "width": width,
        "height": height,
        "area": area,
        "routing": routing,
        "cost": total_cost,
    }


def run_one_seed(u9_x, u9_y, dx, show_plot=False):
    """
    Create a fresh design for one seed, run the original flow,
    and return both the final design and metrics.
    """
    design = create_sampling_design()
    preset_key_components(design, u9_x=u9_x, u9_y=u9_y, dx=dx)

    if show_plot:
        draw_design(
            design,
            f"Initial Placement - Seed (U9_x={u9_x:.1f}, Y={u9_y:.1f}, dx={dx:.1f})"
        )

    progressive_compaction(
        design,
        x_loops=16,
        y_loops=12,
        shrink_step_x=3.5,
        shrink_step_y=2.0,
        inner_iterations=6000,
    )

    legalize_overlaps(design, max_passes=30, gap=0.5)

    # pack_passives_toward_opamps(design)

    metrics = get_layout_metrics(design)
    return design, metrics


def is_better(metrics_a, metrics_b):
    """
    Return True if metrics_a is better than metrics_b.

    Priority:
    1) smaller bbox area
    2) smaller routing length
    3) smaller total cost
    """
    if metrics_b is None:
        return True

    eps = 1e-6

    if metrics_a["area"] < metrics_b["area"] - eps:
        return True
    if abs(metrics_a["area"] - metrics_b["area"]) <= eps:
        if metrics_a["routing"] < metrics_b["routing"] - eps:
            return True
        if abs(metrics_a["routing"] - metrics_b["routing"]) <= eps:
            if metrics_a["cost"] < metrics_b["cost"] - eps:
                return True

    return False

def pack_passives_toward_opamps(design):
    """
    Simple post-pack:
    move small R/C parts closer to U9/U11 in a denser arrangement.
    This is a deterministic packing step, not full annealing.
    """
    if "U9" not in design.components or "U11" not in design.components:
        return

    u9 = design.components["U9"]
    u11 = design.components["U11"]

    left_parts = []
    right_parts = []

    for ref, comp in design.components.items():
        if ref in ("U9", "U11"):
            continue
        if getattr(comp, "fixed", False):
            continue

        prefix = "".join([c for c in ref if c.isalpha()]).upper()
        if prefix not in ("R", "C"):
            continue

        # Only pack small passives
        w = getattr(comp, "w", getattr(comp, "width", 0))
        h = getattr(comp, "h", getattr(comp, "height", 0))
        if w * h > 20.0:
            continue

        if comp.x < (u9.x + u11.x) / 2:
            left_parts.append((ref, comp))
        else:
            right_parts.append((ref, comp))

    # Sort by current y so the structure is relatively stable
    left_parts.sort(key=lambda t: (-t[1].y, t[1].x))
    right_parts.sort(key=lambda t: (-t[1].y, t[1].x))

    # Dense packing parameters
    x_pitch = 2.2
    y_pitch = 2.6

    # Pack around U9
    start_x_left = u9.x - 7.0
    start_y_left = u9.y + 9.0
    for i, (_, comp) in enumerate(left_parts):
        col = i // 6
        row = i % 6
        comp.x = start_x_left - col * x_pitch
        comp.y = start_y_left - row * y_pitch

    # Pack around U11
    start_x_right = u11.x + 1.5
    start_y_right = u11.y + 9.0
    for i, (_, comp) in enumerate(right_parts):
        col = i // 6
        row = i % 6
        comp.x = start_x_right + col * x_pitch
        comp.y = start_y_right - row * y_pitch

def main() -> None:
    print("GRID =", GRID, "mm")

    # -------------------------------------------------
    # Outer sweep on U9 / U11 initial skeleton
    # U9 and U11 must start on the same Y.
    # Only X spacing changes.
    # -------------------------------------------------
    x0 = 60.0
    y_candidates = [45.0, 47.0]
    dx_candidates = [8.0, 10.0, 12.0, 14.0]

    best_design = None
    best_metrics = None
    best_seed = None

    print("\n===== Seed Sweep Start =====")
    for y0 in y_candidates:
        for dx in dx_candidates:
            print(f"\n--- Trying seed: U9=({x0:.1f}, {y0:.1f}), U11.x offset={dx:.1f} ---")

            design, metrics = run_one_seed(
                u9_x=x0,
                u9_y=y0,
                dx=dx,
                show_plot=False,
            )

            print(
                f"Result bbox: W={metrics['width']:.2f}, "
                f"H={metrics['height']:.2f}, "
                f"Area={metrics['area']:.2f}"
            )
            print(f"Result routing length = {metrics['routing']:.3f} mm")
            print(f"Result total cost = {metrics['cost']:.3f}")

            if is_better(metrics, best_metrics):
                best_design = design
                best_metrics = metrics
                best_seed = (x0, y0, dx)
                print(">>> New best seed found.")

    print("\n===== Best Seed Sweep Result =====")
    print(
        f"Best seed: U9.x={best_seed[0]:.1f}, "
        f"U9.y=U11.y={best_seed[1]:.1f}, "
        f"dx={best_seed[2]:.1f}"
    )
    print(
        f"Best real bbox: "
        f"W={best_metrics['width']:.2f}, "
        f"H={best_metrics['height']:.2f}, "
        f"Area={best_metrics['area']:.2f}"
    )
    print(f"Best routing length = {best_metrics['routing']:.3f} mm")
    print(f"Best total cost = {best_metrics['cost']:.3f}")

    print("\nFinal component poses:")
    for ref, comp in best_design.components.items():
        print(
            f"{ref:>8s}: x={comp.x:7.2f}, y={comp.y:7.2f}, "
            f"rot={comp.rotation:3d}, fixed={comp.fixed}"
        )

    print_layout_summary(best_design, "Final Layout Summary")

    final_cost_model = PlacementCost(best_design)
    print(
        "Final estimated total routing length =",
        round(final_cost_model.total_routing_length(), 3),
        "mm"
    )
    final_cost_model.report_wirelength()
    final_cost_model.report_wirelength_by_net()

    draw_design(best_design, "Optimized Placement - Best Seed Sweep Layout")

def rect_of(comp):
    w = getattr(comp, "w", getattr(comp, "width", 0.0))
    h = getattr(comp, "h", getattr(comp, "height", 0.0))
    return (
        comp.x - w / 2.0,
        comp.y - h / 2.0,
        comp.x + w / 2.0,
        comp.y + h / 2.0,
    )


def overlap_amount(a, b):
    ax1, ay1, ax2, ay2 = rect_of(a)
    bx1, by1, bx2, by2 = rect_of(b)
    dx = min(ax2, bx2) - max(ax1, bx1)
    dy = min(ay2, by2) - max(ay1, by1)
    if dx > 0 and dy > 0:
        return dx, dy
    return 0.0, 0.0


def legalize_overlaps(design, max_passes=20, gap=0.4):
    comps = [
        c for c in design.components.values()
        if not getattr(c, "fixed", False)
    ]

    for _ in range(max_passes):
        moved = False
        for i in range(len(comps)):
            for j in range(i + 1, len(comps)):
                a = comps[i]
                b = comps[j]
                dx, dy = overlap_amount(a, b)
                if dx <= 0 or dy <= 0:
                    continue

                moved = True

                # push along the smaller penetration direction
                if dx < dy:
                    shift = dx / 2.0 + gap
                    if a.x <= b.x:
                        a.x -= shift
                        b.x += shift
                    else:
                        a.x += shift
                        b.x -= shift
                else:
                    shift = dy / 2.0 + gap
                    if a.y <= b.y:
                        a.y -= shift
                        b.y += shift
                    else:
                        a.y += shift
                        b.y -= shift

        if not moved:
            break

if __name__ == "__main__":
    main()