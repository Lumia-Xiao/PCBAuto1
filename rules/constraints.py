# rules/constraints.py

from __future__ import annotations

from dataclasses import dataclass, field
from typing import Dict, Iterable, List, Optional, Set

from core.models import Component


@dataclass
class Constraints:
    """
    Generic constraint container for placement / annealing.

    V2 goal:
    - annealer should not know any circuit-specific names like U9/U11
    - annealer should not know semantic types like opamp / SOIC14
    - all such policies are queried through this object
    """

    # ref -> allowed rotations, e.g. {"U9": [0, 180], "U11": [0, 180]}
    rotation_rules: Dict[str, List[int]] = field(default_factory=dict)

    # ref -> movable or not
    movable_rules: Dict[str, bool] = field(default_factory=dict)

    # ref -> shift scale in mm-like board unit, e.g. {"U9": 1.0, "R1": 1.5}
    move_step_rules: Dict[str, float] = field(default_factory=dict)

    # ref -> tag set, e.g. {"U9": {"anchor", "opamp"}, "C38": {"decap"}}
    component_tags: Dict[str, Set[str]] = field(default_factory=dict)

    # Optional generic identification for port-like components
    port_prefixes: tuple[str, ...] = ("P_", "PORT_")
    port_keywords: tuple[str, ...] = ("PORT",)

    # Default policies
    default_rotations: List[int] = field(default_factory=lambda: [0, 90, 180, 270])
    default_move_step: float = 1.5
    default_fine_move_step: float = 1.0

    def _norm_ref(self, ref: str) -> str:
        return str(ref).upper()

    def _norm_tag(self, tag: str) -> str:
        return str(tag).lower()

    def get_allowed_rotations(self, ref: str) -> List[int]:
        ref_u = self._norm_ref(ref)
        angles = self.rotation_rules.get(ref_u, self.default_rotations)
        if not angles:
            return list(self.default_rotations)
        return list(angles)

    def is_movable(self, comp: Component) -> bool:
        """
        Component-level movable policy.
        Priority:
        1) comp.fixed attribute
        2) explicit movable_rules
        3) port detection
        4) default True
        """
        if getattr(comp, "fixed", False):
            return False

        ref_u = self._norm_ref(getattr(comp, "ref", ""))
        if ref_u in self.movable_rules:
            return bool(self.movable_rules[ref_u])

        if self.is_port_component(comp):
            return False

        return True

    def get_move_step(self, ref: str, fine_mode: bool = False) -> float:
        ref_u = self._norm_ref(ref)
        if ref_u in self.move_step_rules:
            return float(self.move_step_rules[ref_u])
        return float(self.default_fine_move_step if fine_mode else self.default_move_step)

    def has_tag(self, ref: str, tag: str) -> bool:
        ref_u = self._norm_ref(ref)
        tag_l = self._norm_tag(tag)
        return tag_l in {self._norm_tag(x) for x in self.component_tags.get(ref_u, set())}

    def get_tags(self, ref: str) -> Set[str]:
        ref_u = self._norm_ref(ref)
        return {self._norm_tag(x) for x in self.component_tags.get(ref_u, set())}

    def is_port_component(self, comp: Component) -> bool:
        ct = str(getattr(comp, "comp_type", "")).upper()
        ref = str(getattr(comp, "ref", "")).upper()

        if any(keyword in ct for keyword in self.port_keywords):
            return True

        if any(ref.startswith(prefix) for prefix in self.port_prefixes):
            return True

        return False

    @classmethod
    def from_rules(
        cls,
        rotation_rules: Optional[Dict[str, Iterable[int]]] = None,
        movable_rules: Optional[Dict[str, bool]] = None,
        move_step_rules: Optional[Dict[str, float]] = None,
        component_tags: Optional[Dict[str, Iterable[str]]] = None,
        default_rotations: Optional[Iterable[int]] = None,
        default_move_step: float = 1.5,
        default_fine_move_step: float = 1.0,
    ) -> "Constraints":
        return cls(
            rotation_rules={
                str(k).upper(): [int(v) for v in vals]
                for k, vals in (rotation_rules or {}).items()
            },
            movable_rules={
                str(k).upper(): bool(v)
                for k, v in (movable_rules or {}).items()
            },
            move_step_rules={
                str(k).upper(): float(v)
                for k, v in (move_step_rules or {}).items()
            },
            component_tags={
                str(k).upper(): {str(vv).lower() for vv in vals}
                for k, vals in (component_tags or {}).items()
            },
            default_rotations=[int(v) for v in (default_rotations or [0, 90, 180, 270])],
            default_move_step=float(default_move_step),
            default_fine_move_step=float(default_fine_move_step),
        )