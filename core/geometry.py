import math
from typing import Tuple

from core.models import Component, Board, PlacementRegion


def manhattan(p1: Tuple[float, float], p2: Tuple[float, float]) -> float:
    return abs(p1[0] - p2[0]) + abs(p1[1] - p2[1])


def euclidean(p1: Tuple[float, float], p2: Tuple[float, float]) -> float:
    return math.hypot(p1[0] - p2[0], p1[1] - p2[1])


def overlap_area(a: Component, b: Component) -> float:
    ax1, ay1, ax2, ay2 = a.bounding_box()
    bx1, by1, bx2, by2 = b.bounding_box()

    x_overlap = max(0.0, min(ax2, bx2) - max(ax1, bx1))
    y_overlap = max(0.0, min(ay2, by2) - max(ay1, by1))
    return x_overlap * y_overlap


def out_of_bounds_penalty(comp: Component, board: Board) -> float:
    x1, y1, x2, y2 = comp.bounding_box()
    penalty = 0.0
    if x1 < 0:
        penalty += abs(x1)
    if y1 < 0:
        penalty += abs(y1)
    if x2 > board.width:
        penalty += abs(x2 - board.width)
    if y2 > board.height:
        penalty += abs(y2 - board.height)
    return penalty


def out_of_region_penalty(comp: Component, region: PlacementRegion) -> float:
    x1, y1, x2, y2 = comp.bounding_box()
    penalty = 0.0
    if x1 < region.x_min:
        penalty += abs(x1 - region.x_min)
    if y1 < region.y_min:
        penalty += abs(y1 - region.y_min)
    if x2 > region.x_max:
        penalty += abs(x2 - region.x_max)
    if y2 > region.y_max:
        penalty += abs(y2 - region.y_max)
    return penalty