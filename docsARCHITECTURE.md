# Architecture

This document explains module responsibilities, data flow, and extension paths in PCBAuto1.

## 1. End-to-End Data Flow

1. **Build design**: create a `PCBDesign` in `designs/` (board, components, nets).
2. **Load rules**: define constraints and scenario rules in `rules/`.
3. **Evaluate cost**: compute placement objective using `costs/`.
4. **Optimize layout**: iterate with `core/compaction.py` + `core/annealer.py`.
5. **Estimate routing**: generate simplified routing data in `core/routing.py`.
6. **Visualize/export**: plot placement and export JSON from `runs/`.

---

## 2. Core Modules

### 2.1 `core/models.py`

Defines fundamental data structures:

- `Component`: location, size, rotation, fixed flag
- `Pin`: relative pin position and net mapping
- `Net`: net connectivity
- `PCBDesign`: aggregate design object (`board/components/nets/region/routing_result`)

This is the central data contract used by all algorithms.

### 2.2 `core/geometry.py`

Geometry and penalty utilities, including distance terms, overlap area, and boundary penalties.

### 2.3 `core/annealer.py`

Simulated annealing optimizer:

- Filters movable components under constraints
- Proposes randomized move/rotation perturbations
- Accepts/rejects candidates based on temperature schedule
- Supports fine mode for refined search

### 2.4 `core/compaction.py`

Performs coarse compaction by shrinking a target region and repeatedly re-optimizing placement.

### 2.5 `core/routing.py`

Simplified grid router:

- Attempts grid-based net connections
- Produces `RoutedSegment` and `Via`
- Stores routing output in `PCBDesign.routing_result`
- Exports JSON results

### 2.6 `core/visualization.py`

Uses matplotlib to visualize placement for quick inspection.

---

## 3. Rules and Cost Layers

### 3.1 `rules/constraints.py`

Unified constraint interface for:

- Component movability
- Allowed rotations
- Move step sizes
- Tag system (e.g., `anchor`, `decap`)

### 3.2 `rules/sampling_rules.py`

Builds scenario-specific rules for sampling frontend (grouping, anchors, proximity relations, etc.).

### 3.3 `costs/`

- `generic_cost.py`: generic placement cost terms
- `sampling_frontend_cost.py`: scenario-specific composition
- `base_cost.py`: foundational penalties and helper filters

---

## 4. Design Definition and Run Entrypoint

### 4.1 `designs/sampling_frontend.py`

Defines the sample design:

- Board dimensions
- Components and footprints
- Fixed IO port locations
- Initial placement
- Net connections

### 4.2 `runs/run_sampling_frontend.py`

Default experiment script:

- Build design + apply seed preset
- Run compaction + annealing
- Print cost/bbox/routing metrics
- Call `route_design_v1` and export `routing_result_v1.json`

---

## 5. Extension Guide

### Add a New Design Scenario

1. Add a design definition under `designs/`.
2. Add corresponding rule builder(s) under `rules/`.
3. Add a scenario cost model under `costs/`.
4. Add a runnable script under `runs/`.

### Add a New Cost Term

1. Implement it in `costs/base_cost.py` or `costs/generic_cost.py`.
2. Combine it (with weight) in the scenario-specific cost model.
3. Print/export the metric in run scripts for regression tracking.

---

## 6. Engineering Recommendations

- Keep reusable logic in `core/`; keep scenario logic in `designs/rules/costs`.
- Use fixed random seeds in experiment scripts for reproducibility.
- Prefer structured outputs (JSON) for easier CI/dashboard integration.