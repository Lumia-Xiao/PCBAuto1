# PCBAuto1

A Python prototype for **automatic PCB component placement and simplified routing evaluation**.
The repository uses a sampling frontend circuit as an example and optimizes placement through a combined flow of **constraints + cost model + compaction + simulated annealing**.

## Project Goals

- Provide a reproducible framework for auto-placement experiments.
- Make engineering constraints explicit (movability, allowed rotations, step sizes, grouping).
- Support seed-based initialization and result comparison.
- Export visualization and routing JSON outputs for downstream analysis/integration.

## Feature Overview

- **Design construction**: board, components, pins, and netlist definitions.
- **Rules and constraints**: component tags, rotation constraints, movability rules, step control.
- **Cost evaluation**: distance/HPWL-like terms, overlap penalties, boundary penalties, rule penalties.
- **Optimization flow**:
  1. Coarse compaction
  2. Simulated annealing refinement
  3. Grid-based routing estimation and export
- **Outputs**: CLI summary + layout plot + `runs/routing_result_v1.json`.

## Repository Layout

```text
PCBAuto1/
├── core/                   # Core models, geometry, optimization, routing
├── costs/                  # Cost models (generic + sampling_frontend specific)
├── rules/                  # Constraints and scenario rules
├── designs/                # Example design definitions (sampling_frontend)
├── runs/                   # Executable scripts and sample outputs
├── config.py               # Global grid and base constants
├── design_factory.py       # Design creation entry (legacy/compat)
├── seed_sweep.py           # Generic seed sweep utility
├── requirements.txt        # Runtime dependencies
├── pyproject.toml          # Dev tooling config (ruff/black)
└── README.md
```

For a deeper module breakdown, see `docs/ARCHITECTURE.md`.

## Quick Start

### 1) Environment Setup

Python 3.10+ is recommended.

```bash
python -m venv .venv
source .venv/bin/activate
pip install -r requirements.txt
```

### 2) Run the Example Flow

```bash
python -m runs.run_sampling_frontend
```

The default flow includes:

- Initial key-component seeding (U9/U11)
- Compaction + simulated annealing
- Routing statistics + JSON export
- Layout visualization (matplotlib)

### 3) Check Outputs

- Routing result JSON: `runs/routing_result_v1.json`
- Console output includes bbox, routing length, cost, via count, etc.

## Configuration

### Global Settings (`config.py`)

- `GRID`: placement/routing grid size (mm)
- `MIN_COMPONENT_SPACING`: minimum component spacing (mm)

### Run Parameters (`runs/run_sampling_frontend.py`)

You can adjust:

- Seed candidates (`x0`, `y_candidates`, `dx_candidates`)
- Compaction parameters (loop counts, shrink steps, inner iterations)
- Annealing parameters (iterations, temperature range, fine mode)

## Development Notes

- Define new circuits/netlists under `designs/`.
- Add scenario-specific constraints/rules under `rules/`.
- Add scenario-specific cost terms under `costs/` rather than hardcoding in core logic.
- Keep experiment entrypoints under `runs/` for reproducibility.

## FAQ

### Q1: No plot window appears during execution. Why?

In headless environments, `matplotlib` may not open an interactive window. You can:

- Run locally with a GUI-enabled environment, or
- Save figures to files (`plt.savefig(...)`) and inspect them offline.

### Q2: How do I adapt this to another board/design?

Suggested process:

1. Copy and modify `designs/sampling_frontend.py`
2. Add scenario rules in `rules/<your_design>_rules.py`
3. Add scenario cost model in `costs/<your_design>_cost.py`
4. Add a run script in `runs/`

## License

No license file is currently declared in this repository.
If you plan to open-source it, add a `LICENSE` file and update this section.