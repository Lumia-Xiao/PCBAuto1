from typing import Dict, List, Tuple

from config import snap_to_grid
from core.models import Board, PCBDesign, Net, Component
from core.footprints import make_0805, make_soic14, make_io_port


def create_sampling_design() -> PCBDesign:
    board = Board(width=140.0, height=95.0)
    components: Dict[str, Component] = {}

    components["U9"] = make_soic14("U9", "OPA4171_SOIC14")
    components["U11"] = make_soic14("U11", "OPA4171_SOIC14")

    resistor_refs = [
        "R65", "R68", "R33", "R35", "R11", "R38",
        "R102", "R103", "R42", "R44", "R40", "R46",
        "R48", "R50", "R52", "R54",
        "R56", "R58", "R60", "R62",
    ]
    for ref in resistor_refs:
        components[ref] = make_0805(ref, "RES_0805")

    capacitor_refs = [
        "C38", "C42", "C39", "C43",
        "C46", "C48", "C50", "C52",
        "C54", "C56", "C58", "C60",
    ]
    for ref in capacitor_refs:
        components[ref] = make_0805(ref, "CAP_0805")

    left_ports = ["P_IS3", "P_IS4", "P_VP3", "P_VN3", "P_VP4", "P_VN4"]
    right_ports = ["P_ADCB1", "P_ADCB2", "P_ADCB3", "P_ADCD0"]

    for ref in left_ports:
        components[ref] = make_io_port(ref, "left")
    for ref in right_ports:
        components[ref] = make_io_port(ref, "right")

    port_pos = {
        "P_IS3": (4.0, 72.0, 0),
        "P_IS4": (4.0, 55.0, 0),
        "P_VP3": (4.0, 39.0, 0),
        "P_VN3": (4.0, 33.0, 0),
        "P_VP4": (4.0, 21.0, 0),
        "P_VN4": (4.0, 15.0, 0),

        "P_ADCB1": (136.0, 72.0, 0),
        "P_ADCB2": (136.0, 55.0, 0),
        "P_ADCB3": (136.0, 36.0, 0),
        "P_ADCD0": (136.0, 18.0, 0),
    }

    for ref, (x, y, rot) in port_pos.items():
        components[ref].x = snap_to_grid(x)
        components[ref].y = snap_to_grid(y)
        components[ref].rotation = rot
        components[ref].fixed = True

    components["U9"].x = snap_to_grid(58.0)
    components["U9"].y = snap_to_grid(47.0)
    components["U9"].rotation = 0

    components["U11"].x = snap_to_grid(84.0)
    components["U11"].y = snap_to_grid(47.0)
    components["U11"].rotation = 0

    init_pos = {
        "C38": (57.0, 61.5, 0), "C42": (57.0, 58.5, 0),
        "C39": (83.0, 61.5, 0), "C43": (83.0, 58.5, 0),

        "R65": (18.0, 74.0, 90), "R68": (18.0, 68.5, 90),
        "R33": (30.0, 73.0, 0), "R35": (30.0, 69.0, 0),
        "R11": (38.0, 72.0, 90), "C46": (40.0, 70.5, 90),
        "R38": (68.0, 70.5, 0), "C48": (68.0, 67.0, 90),

        "R102": (18.0, 57.0, 90), "R103": (18.0, 51.5, 90),
        "R42": (30.0, 56.0, 0), "R44": (30.0, 52.0, 0),
        "R40": (38.0, 55.0, 90), "C50": (40.0, 53.5, 90),
        "R46": (68.0, 53.5, 0), "C52": (68.0, 50.0, 90),

        "R48": (38.0, 38.0, 90), "R50": (30.0, 39.0, 0),
        "R52": (30.0, 33.0, 0), "C54": (40.0, 36.0, 90),
        "R54": (68.0, 35.5, 0), "C56": (68.0, 32.0, 90),

        "R56": (38.0, 20.0, 90), "R58": (30.0, 21.0, 0),
        "R60": (30.0, 15.0, 0), "C58": (40.0, 18.0, 90),
        "R62": (68.0, 17.5, 0), "C60": (68.0, 14.0, 90),
    }

    for ref, (x, y, rot) in init_pos.items():
        components[ref].set_pose(x, y, rot)

    nets: Dict[str, Net] = {}

    def add_net(name: str, conns: List[Tuple[str, str]]) -> None:
        nets[name] = Net(name, conns)

    add_net("D+5V_U9", [("U9", "4"), ("C38", "1"), ("C42", "1")])
    add_net("AGND_U9", [("U9", "11"), ("C38", "2"), ("C42", "2")])
    add_net("3V3_U11", [("U11", "4"), ("C39", "1"), ("C43", "1")])
    add_net("AGND_U11", [("U11", "11"), ("C39", "2"), ("C43", "2")])

    add_net("IS3", [("P_IS3", "1"), ("R33", "1")])
    add_net("CH1_REF", [("R65", "2"), ("R68", "1"), ("R35", "1")])
    add_net("D+5V_DIV1", [("R65", "1")])
    add_net("AGND_DIV1", [("R68", "2"), ("R11", "1")])
    add_net("N_CH1_P", [("R33", "2"), ("C46", "1"), ("U9", "3"), ("R11", "2")])
    add_net("N_CH1_N", [("R35", "2"), ("C46", "2"), ("U9", "2"), ("R38", "2"), ("C48", "2")])
    add_net("N_CH1_OUT", [("U9", "1"), ("R38", "1"), ("C48", "1"), ("U11", "3")])
    add_net("ADCB1", [("U11", "1"), ("U11", "2"), ("P_ADCB1", "1")])

    add_net("IS4", [("P_IS4", "1"), ("R42", "1")])
    add_net("CH2_REF", [("R102", "2"), ("R103", "1"), ("R44", "1")])
    add_net("D-5V_DIV2", [("R102", "1")])
    add_net("AGND_DIV2", [("R103", "2"), ("R40", "1")])
    add_net("N_CH2_P", [("R42", "2"), ("C50", "1"), ("U9", "5"), ("R40", "2")])
    add_net("N_CH2_N", [("R44", "2"), ("C50", "2"), ("U9", "6"), ("R46", "2"), ("C52", "2")])
    add_net("N_CH2_OUT", [("U9", "7"), ("R46", "1"), ("C52", "1"), ("U11", "5")])
    add_net("ADCB2", [("U11", "7"), ("U11", "6"), ("P_ADCB2", "1")])

    add_net("VP3", [("P_VP3", "1"), ("R50", "1")])
    add_net("VN3", [("P_VN3", "1"), ("R52", "1")])
    add_net("AGND_CH3", [("R48", "1")])
    add_net("N_CH3_P", [("R50", "2"), ("C54", "1"), ("U9", "10"), ("R48", "2")])
    add_net("N_CH3_N", [("R52", "2"), ("C54", "2"), ("U9", "9"), ("R54", "2"), ("C56", "2")])
    add_net("N_CH3_OUT", [("U9", "8"), ("R54", "1"), ("C56", "1"), ("U11", "10")])
    add_net("ADCB3", [("U11", "8"), ("U11", "9"), ("P_ADCB3", "1")])

    add_net("VP4", [("P_VP4", "1"), ("R58", "1")])
    add_net("VN4", [("P_VN4", "1"), ("R60", "1")])
    add_net("AGND_CH4", [("R56", "1")])
    add_net("N_CH4_P", [("R58", "2"), ("C58", "1"), ("U9", "12"), ("R56", "2")])
    add_net("N_CH4_N", [("R60", "2"), ("C58", "2"), ("U9", "13"), ("R62", "2"), ("C60", "2")])
    add_net("N_CH4_OUT", [("U9", "14"), ("R62", "1"), ("C60", "1"), ("U11", "12")])
    add_net("ADCD0", [("U11", "14"), ("U11", "13"), ("P_ADCD0", "1")])

    design = PCBDesign(board=board, components=components, nets=nets)
    design.assign_pin_nets()
    return design