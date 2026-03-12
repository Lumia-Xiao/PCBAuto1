from core.models import Component, Pin


def make_0805(ref: str, comp_type: str = "PASSIVE_0805") -> Component:
    return Component(
        ref=ref,
        comp_type=comp_type,
        width=2.0,
        height=1.25,
        pins={
            "1": Pin("1", -0.75, 0.0),
            "2": Pin("2", 0.75, 0.0),
        },
        fixed=False,
    )


def make_soic14(ref: str, comp_type: str = "SOIC14") -> Component:
    body_w = 8.6
    body_h = 6.2

    left_x = -body_w / 2 + 0.55
    right_x = body_w / 2 - 0.55

    y_positions = [2.85, 1.90, 0.95, 0.0, -0.95, -1.90, -2.85]

    pins = {}
    for i in range(7):
        pins[str(i + 1)] = Pin(str(i + 1), left_x, y_positions[i])

    right_y_positions = [-2.85, -1.90, -0.95, 0.0, 0.95, 1.90, 2.85]
    for i in range(7):
        pins[str(i + 8)] = Pin(str(i + 8), right_x, right_y_positions[i])

    return Component(
        ref=ref,
        comp_type=comp_type,
        width=body_w,
        height=body_h,
        pins=pins,
        fixed=False,
    )


def make_io_port(ref: str, side: str = "left") -> Component:
    if side == "left":
        pins = {"1": Pin("1", 0.9, 0.0)}
    else:
        pins = {"1": Pin("1", -0.9, 0.0)}

    return Component(
        ref=ref,
        comp_type=f"PORT_{side.upper()}",
        width=1.8,
        height=1.2,
        pins=pins,
        fixed=True,
    )