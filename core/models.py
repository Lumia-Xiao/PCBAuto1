from __future__ import annotations

from dataclasses import dataclass, field
from typing import Dict, List, Tuple, Optional

from config import snap_to_grid, snap_angle


@dataclass
class Pin:
    name: str
    x_rel: float
    y_rel: float
    net: Optional[str] = None


@dataclass
class Component:
    ref: str
    comp_type: str
    width: float
    height: float
    pins: Dict[str, Pin]

    x: float = 0.0
    y: float = 0.0
    rotation: int = 0
    fixed: bool = False

    def center(self) -> Tuple[float, float]:
        return self.x, self.y

    def rotated_pin_position(self, pin_name: str) -> Tuple[float, float]:
        pin = self.pins[pin_name]
        xr, yr = pin.x_rel, pin.y_rel

        if self.rotation == 0:
            dx, dy = xr, yr
        elif self.rotation == 90:
            dx, dy = -yr, xr
        elif self.rotation == 180:
            dx, dy = -xr, -yr
        elif self.rotation == 270:
            dx, dy = yr, -xr
        else:
            raise ValueError(f"Unsupported rotation: {self.rotation}")

        return self.x + dx, self.y + dy

    def bounding_box(self) -> Tuple[float, float, float, float]:
        if self.rotation in (0, 180):
            w, h = self.width, self.height
        else:
            w, h = self.height, self.width

        xmin = self.x - w / 2
        xmax = self.x + w / 2
        ymin = self.y - h / 2
        ymax = self.y + h / 2
        return xmin, ymin, xmax, ymax

    def move(self, dx: float, dy: float) -> None:
        if not self.fixed:
            self.x = snap_to_grid(self.x + dx)
            self.y = snap_to_grid(self.y + dy)

    def set_pose(self, x: float, y: float, rotation: int) -> None:
        if not self.fixed:
            self.x = snap_to_grid(x)
            self.y = snap_to_grid(y)
            self.rotation = snap_angle(rotation)


@dataclass
class Net:
    name: str
    connections: List[Tuple[str, str]]


@dataclass
class Board:
    width: float
    height: float


@dataclass
class PlacementRegion:
    x_min: float
    y_min: float
    x_max: float
    y_max: float

    @property
    def width(self) -> float:
        return self.x_max - self.x_min

    @property
    def height(self) -> float:
        return self.y_max - self.y_min


@dataclass
class PCBDesign:
    board: Board
    components: Dict[str, Component] = field(default_factory=dict)
    nets: Dict[str, Net] = field(default_factory=dict)
    region: Optional[PlacementRegion] = None

    def assign_pin_nets(self) -> None:
        for net in self.nets.values():
            for ref, pin_name in net.connections:
                self.components[ref].pins[pin_name].net = net.name