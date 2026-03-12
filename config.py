GRID = 0.5  # mm
MIN_COMPONENT_SPACING = 2.0  # mm

def snap_to_grid(v: float, grid: float = GRID) -> float:
    return round(v / grid) * grid


def snap_angle(angle: int) -> int:
    angle = angle % 360
    candidates = [0, 90, 180, 270]
    return min(candidates, key=lambda a: abs(a - angle))