# rules/anchors.py

SAMPLING_ANCHOR_REFS = ["U9", "U11"]

SAMPLING_DECAP_RULES = [
    ("U9", "4", "C38"),
    ("U9", "4", "C42"),
    ("U11", "4", "C39"),
    ("U11", "4", "C43"),
]

SAMPLING_STAGE_MATCH_PAIRS = [
    (("U9", "1"), ("U11", "3")),
    (("U9", "7"), ("U11", "5")),
    (("U9", "8"), ("U11", "10")),
    (("U9", "14"), ("U11", "12")),
]

SAMPLING_PORT_INPUT_BIND = {
    "P_IS3": ["R33", "R65", "R68"],
    "P_IS4": ["R42", "R102", "R103"],
    "P_VP3": ["R50"],
    "P_VN3": ["R52"],
    "P_VP4": ["R58"],
    "P_VN4": ["R60"],
}

SAMPLING_PORT_OUTPUT_BIND = {
    "P_ADCB1": ["U11"],
    "P_ADCB2": ["U11"],
    "P_ADCB3": ["U11"],
    "P_ADCD0": ["U11"],
}