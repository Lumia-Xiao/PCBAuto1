from core.models import PlacementRegion
from core.annealer import SimulatedAnnealer
from core.layout_utils import get_layout_bbox, compact_center_into_region


MIN_W = 24.0
MIN_H = 18.0


def progressive_compaction(
    design,
    cost_model,
    constraints=None,
    x_loops=8,
    y_loops=6,
    shrink_step_x=3.0,
    shrink_step_y=1.8,
    inner_iterations=1200,
) -> None:
    sa = SimulatedAnnealer(
        design=design,
        cost_model=cost_model,
        constraints=constraints,
        seed=123,
    )

    x1, y1, x2, y2 = get_layout_bbox(design, margin=5.0, exclude_ports=True)

    if "U9" in design.components and "U11" in design.components:
        u9 = design.components["U9"]
        u11 = design.components["U11"]
        ux1 = min(u9.x, u11.x) - MIN_W
        ux2 = max(u9.x, u11.x) + MIN_W
        uy1 = min(u9.y, u11.y) - MIN_H
        uy2 = max(u9.y, u11.y) + MIN_H
        x1 = min(x1, ux1)
        x2 = max(x2, ux2)
        y1 = min(y1, uy1)
        y2 = max(y2, uy2)

    design.region = PlacementRegion(x1, y1, x2, y2)

    best_state = sa.snapshot()
    best_cost = cost_model.total_cost()

    print(
        f"Initial compact region: "
        f"x=({x1:.1f}, {x2:.1f}), y=({y1:.1f}, {y2:.1f}), "
        f"W={x2 - x1:.1f}, H={y2 - y1:.1f}"
    )

    sa.optimize(
        iterations=inner_iterations,
        t_start=70.0,
        t_end=0.05,
        verbose_interval=max(1, inner_iterations // 3),
        fine_mode=False,
        use_incremental=False,
    )

    current_cost = cost_model.total_cost()
    if current_cost < best_cost:
        best_cost = current_cost
        best_state = sa.snapshot()

    for k in range(x_loops):
        print(f"\n=== X-compaction round {k + 1} ===")
        print(f"Current region: W={design.region.width:.2f}, H={design.region.height:.2f}")

        old_region = design.region
        old_state = sa.snapshot()

        new_region = PlacementRegion(
            x_min=old_region.x_min + shrink_step_x / 2.0,
            y_min=old_region.y_min,
            x_max=old_region.x_max - shrink_step_x / 2.0,
            y_max=old_region.y_max,
        )

        if new_region.width < MIN_W:
            print("Width too small. Stop X-compaction.")
            break

        design.region = new_region
        compact_center_into_region(design)

        sa.optimize(
            iterations=max(2200, inner_iterations // 2),
            t_start=28.0,
            t_end=0.03,
            verbose_interval=max(1, inner_iterations // 4),
            fine_mode=True,
            use_incremental=False,
        )

        new_cost = cost_model.total_cost()
        bx1, by1, bx2, by2 = get_layout_bbox(design, margin=0.0, exclude_ports=True)
        real_w = bx2 - bx1

        if new_cost > best_cost * 1.18:
            print("X-shrink rejected. Region became too tight.")
            design.region = old_region
            sa.restore(old_state)
            break

        print(f"X-shrink accepted. Region W={design.region.width:.2f}, real bbox W={real_w:.2f}")
        if new_cost < best_cost:
            best_cost = new_cost
            best_state = sa.snapshot()

        rx1, ry1, rx2, ry2 = get_layout_bbox(design, margin=2.0, exclude_ports=True)
        new_w = max(MIN_W, rx2 - rx1)
        new_h = max(MIN_H, ry2 - ry1)
        cx = 0.5 * (rx1 + rx2)
        cy = 0.5 * (ry1 + ry2)
        design.region = PlacementRegion(
            cx - new_w / 2.0,
            cy - new_h / 2.0,
            cx + new_w / 2.0,
            cy + new_h / 2.0,
        )

    for k in range(y_loops):
        print(f"\n=== Y-compaction round {k + 1} ===")
        print(f"Current region: W={design.region.width:.2f}, H={design.region.height:.2f}")

        old_region = design.region
        old_state = sa.snapshot()

        new_region = PlacementRegion(
            x_min=old_region.x_min,
            y_min=old_region.y_min + shrink_step_y / 2.0,
            x_max=old_region.x_max,
            y_max=old_region.y_max - shrink_step_y / 2.0,
        )

        if new_region.height < MIN_H:
            print("Height too small. Stop Y-compaction.")
            break

        design.region = new_region
        compact_center_into_region(design)

        sa.optimize(
            iterations=max(1800, inner_iterations // 2),
            t_start=22.0,
            t_end=0.03,
            verbose_interval=max(1, inner_iterations // 4),
            fine_mode=True,
            use_incremental=False,
        )

        new_cost = cost_model.total_cost()
        bx1, by1, bx2, by2 = get_layout_bbox(design, margin=0.0, exclude_ports=True)
        real_h = by2 - by1

        if new_cost > best_cost * 1.18:
            print("Y-shrink rejected. Region became too tight.")
            design.region = old_region
            sa.restore(old_state)
            break

        print(f"Y-shrink accepted. Region H={design.region.height:.2f}, real bbox H={real_h:.2f}")
        if new_cost < best_cost:
            best_cost = new_cost
            best_state = sa.snapshot()

        rx1, ry1, rx2, ry2 = get_layout_bbox(design, margin=2.0, exclude_ports=True)
        new_w = max(MIN_W, rx2 - rx1)
        new_h = max(MIN_H, ry2 - ry1)
        cx = 0.5 * (rx1 + rx2)
        cy = 0.5 * (ry1 + ry2)
        design.region = PlacementRegion(
            cx - new_w / 2.0,
            cy - new_h / 2.0,
            cx + new_w / 2.0,
            cy + new_h / 2.0,
        )

    sa.restore(best_state)
    compact_center_into_region(design)

    final_sa = SimulatedAnnealer(
        design=design,
        cost_model=cost_model,
        constraints=constraints,
        seed=321,
    )
    final_sa.optimize(
        iterations=800,
        t_start=6.0,
        t_end=0.05,
        verbose_interval=400,
        fine_mode=True,
        use_incremental=False,
    )

    bx1, by1, bx2, by2 = get_layout_bbox(design, margin=0.0, exclude_ports=True)
    print(
        f"\nFinal real compact bbox: "
        f"x=({bx1:.2f}, {bx2:.2f}), y=({by1:.2f}, {by2:.2f}), "
        f"W={bx2 - bx1:.2f}, H={by2 - by1:.2f}"
    )