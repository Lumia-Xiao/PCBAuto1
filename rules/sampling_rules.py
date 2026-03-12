# rules/sampling_rules.py

from dataclasses import dataclass
from rules.grouping import (
    SAMPLING_CHANNEL_GROUPS,
    SAMPLING_INPUT_GROUP,
    SAMPLING_OUTPUT_GROUP,
)
from rules.anchors import (
    SAMPLING_ANCHOR_REFS,
    SAMPLING_DECAP_RULES,
    SAMPLING_STAGE_MATCH_PAIRS,
    SAMPLING_PORT_INPUT_BIND,
    SAMPLING_PORT_OUTPUT_BIND,
)


@dataclass
class SamplingFrontendRules:
    channel_groups: dict
    channel_rows: dict
    input_group: set
    output_group: set
    anchor_refs: list
    decap_rules: list
    stage_match_pairs: list
    port_input_bind: dict
    port_output_bind: dict
    target_chip_dx: float = 14.0
    target_chip_dy: float = 0.0
    target_chip_row_y: float = 47.0
    min_chip_dx: float = 10.0
    preferred_flow_dx: float = 12.0


def build_sampling_frontend_rules() -> SamplingFrontendRules:
    return SamplingFrontendRules(
        channel_groups=SAMPLING_CHANNEL_GROUPS,
        channel_rows={
            "CH1": 72.0,
            "CH2": 55.0,
            "CH3": 36.0,
            "CH4": 18.0,
        },
        input_group=SAMPLING_INPUT_GROUP,
        output_group=SAMPLING_OUTPUT_GROUP,
        anchor_refs=SAMPLING_ANCHOR_REFS,
        decap_rules=SAMPLING_DECAP_RULES,
        stage_match_pairs=SAMPLING_STAGE_MATCH_PAIRS,
        port_input_bind=SAMPLING_PORT_INPUT_BIND,
        port_output_bind=SAMPLING_PORT_OUTPUT_BIND,
    )