# core/annealer.py

import math
import random
from typing import Dict, Tuple, Optional, List

from config import GRID, snap_to_grid, snap_angle
from core.models import PCBDesign, Component
from costs.sampling_frontend_cost import PlacementCost
from rules.constraints import Constraints


class SimulatedAnnealer:
    def __init__(
        self,
        design: PCBDesign,
        cost_model: PlacementCost,
        constraints: Optional[Constraints] = None,
        seed: int = 42,
    ):
        self.design = design
        self.cost_model = cost_model
        self.constraints = constraints or Constraints()
        self._movable_components: Optional[List[Component]] = None
        random.seed(seed)

    def snapshot(self) -> Dict[str, Tuple[float, float, int]]:
        return {
            ref: (comp.x, comp.y, comp.rotation)
            for ref, comp in self.design.components.items()
        }

    def restore(self, state: Dict[str, Tuple[float, float, int]]) -> None:
        for ref, (x, y, rot) in state.items():
            comp = self.design.components[ref]
            comp.x = snap_to_grid(x)
            comp.y = snap_to_grid(y)
            comp.rotation = snap_angle(rot)

    def movable_components(self) -> List[Component]:
        if self._movable_components is None:
            self._movable_components = [
                c for c in self.design.components.values()
                if self.constraints.is_movable(c)
            ]
        return self._movable_components

    def pick_movable_component(self) -> Component:
        movable = self.movable_components()
        if not movable:
            raise RuntimeError("No movable real components available.")
        return random.choice(movable)

    def random_initialize(self) -> None:
        region = self.design.region

        for comp in self.design.components.values():
            if not self.constraints.is_movable(comp):
                continue

            if region is None:
                xmin = comp.width / 2
                xmax = self.design.board.width - comp.width / 2
                ymin = comp.height / 2
                ymax = self.design.board.height - comp.height / 2
            else:
                xmin = region.x_min + comp.width / 2
                xmax = region.x_max - comp.width / 2
                ymin = region.y_min + comp.height / 2
                ymax = region.y_max - comp.height / 2

            if xmax <= xmin:
                xmax = xmin + GRID
            if ymax <= ymin:
                ymax = ymin + GRID

            comp.x = snap_to_grid(random.uniform(xmin, xmax))
            comp.y = snap_to_grid(random.uniform(ymin, ymax))

            allowed_rotations = self.constraints.get_allowed_rotations(comp.ref)
            comp.rotation = snap_angle(random.choice(allowed_rotations))

    def propose_move_for_component(self, comp: Component, fine_mode: bool = False) -> None:
        move_type = random.choice(["shift", "rotate", "shift_rotate"])

        shift_scale = self.constraints.get_move_step(comp.ref, fine_mode=fine_mode)

        if move_type in ("shift", "shift_rotate"):
            max_step = max(1, int(round(shift_scale / GRID)))
            dx = random.randint(-max_step, max_step) * GRID
            dy = random.randint(-max_step, max_step) * GRID
            comp.move(dx, dy)
            comp.x = snap_to_grid(comp.x)
            comp.y = snap_to_grid(comp.y)

        if move_type in ("rotate", "shift_rotate"):
            allowed_rotations = self.constraints.get_allowed_rotations(comp.ref)
            comp.rotation = snap_angle(random.choice(allowed_rotations))

    def optimize(
        self,
        iterations: int = 10000,
        t_start: float = 100.0,
        t_end: float = 0.05,
        verbose_interval: int = 2000,
        use_incremental: bool = False,
        fine_mode: bool = False,
    ) -> None:
        current_cost = self.cost_model.total_cost()
        best_cost = current_cost
        best_state = self.snapshot()

        for step in range(iterations):
            t = t_start * ((t_end / t_start) ** (step / max(1, iterations - 1)))

            comp = self.pick_movable_component()
            old_state = (comp.x, comp.y, comp.rotation)
            old_cost = current_cost

            old_local_cost: Optional[float] = None
            new_local_cost: Optional[float] = None

            if use_incremental and hasattr(self.cost_model, "local_cost_for_component"):
                old_local_cost = self.cost_model.local_cost_for_component(comp.ref)

            self.propose_move_for_component(comp, fine_mode=fine_mode)

            if use_incremental and hasattr(self.cost_model, "local_cost_for_component"):
                new_local_cost = self.cost_model.local_cost_for_component(comp.ref)
                new_cost = old_cost - old_local_cost + new_local_cost
            else:
                new_cost = self.cost_model.total_cost()

            delta = new_cost - old_cost
            if delta <= 0:
                accept = True
            else:
                prob = math.exp(-delta / max(t, 1e-12))
                accept = (random.random() < prob)

            if accept:
                current_cost = new_cost
                if new_cost < best_cost:
                    best_cost = new_cost
                    best_state = self.snapshot()
            else:
                comp.x = snap_to_grid(old_state[0])
                comp.y = snap_to_grid(old_state[1])
                comp.rotation = snap_angle(old_state[2])
                current_cost = old_cost

            if verbose_interval > 0 and (step + 1) % verbose_interval == 0:
                print(
                    f"Step {step + 1:5d} | "
                    f"T={t:8.4f} | "
                    f"Current={current_cost:10.3f} | "
                    f"Best={best_cost:10.3f}"
                )

        self.restore(best_state)
        print(f"Optimization finished. Best cost = {best_cost:.3f}")
