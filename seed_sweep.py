# seed_sweep.py
from __future__ import annotations

from copy import deepcopy
from dataclasses import dataclass
from typing import Any, Dict, List, Optional, Tuple


@dataclass
class SweepMetrics:
    bbox_width: float
    bbox_height: float
    bbox_area: float
    hpwl: float = 0.0
    overlap_penalty: float = 0.0
    boundary_penalty: float = 0.0


@dataclass
class SweepResult:
    modules: List[Any]
    metrics: SweepMetrics
    score: float
    seed_info: Dict[str, float]


def _get_ref(module: Any) -> str:
    for key in ("ref", "name", "reference", "designator"):
        if hasattr(module, key):
            return str(getattr(module, key))
    raise AttributeError("Module has no ref/name/reference/designator attribute.")


def _get_xy(module: Any) -> Tuple[float, float]:
    return float(module.x), float(module.y)


def _set_xy(module: Any, x: float, y: float) -> None:
    module.x = float(x)
    module.y = float(y)


def _get_wh(module: Any) -> Tuple[float, float]:
    for wk in ("w", "width"):
        for hk in ("h", "height"):
            if hasattr(module, wk) and hasattr(module, hk):
                return float(getattr(module, wk)), float(getattr(module, hk))
    raise AttributeError("Module has no width/height attributes.")


def _find_module(modules: List[Any], ref: str) -> Any:
    for m in modules:
        if _get_ref(m) == ref:
            return m
    raise ValueError(f"Cannot find module {ref}")


def _real_bbox(modules: List[Any]) -> Tuple[float, float, float, float]:
    xmin = float("inf")
    ymin = float("inf")
    xmax = float("-inf")
    ymax = float("-inf")

    for m in modules:
        x, y = _get_xy(m)
        w, h = _get_wh(m)
        xmin = min(xmin, x - w / 2.0)
        xmax = max(xmax, x + w / 2.0)
        ymin = min(ymin, y - h / 2.0)
        ymax = max(ymax, y + h / 2.0)

    return xmin, ymin, xmax, ymax


def _bbox_metrics(modules: List[Any]) -> SweepMetrics:
    xmin, ymin, xmax, ymax = _real_bbox(modules)
    bw = xmax - xmin
    bh = ymax - ymin
    return SweepMetrics(
        bbox_width=bw,
        bbox_height=bh,
        bbox_area=bw * bh,
        hpwl=0.0,
        overlap_penalty=0.0,
        boundary_penalty=0.0,
    )


def _safe_call(func, *args, **kwargs):
    if func is None:
        return None
    try:
        return func(*args, **kwargs)
    except TypeError:
        return func(*args)


def _build_initial_seed(
    modules: List[Any],
    anchor_refs: Tuple[str, str],
    x0: float,
    y0: float,
    dx: float,
) -> List[Any]:
    trial_modules = deepcopy(modules)

    left_ref, right_ref = anchor_refs
    left_mod = _find_module(trial_modules, left_ref)
    right_mod = _find_module(trial_modules, right_ref)

    _set_xy(left_mod, x0, y0)
    _set_xy(right_mod, x0 + dx, y0)

    return trial_modules


def _optional_prepack_passives_near_anchors(
    modules: List[Any],
    left_ref: str,
    right_ref: str,
) -> None:
    """
    Optional light-touch initialization:
    Put small passives roughly around the two anchor ICs
    without changing large parts too aggressively.
    This function is intentionally conservative.
    """
    left_mod = _find_module(modules, left_ref)
    right_mod = _find_module(modules, right_ref)
    left_x, left_y = _get_xy(left_mod)
    right_x, right_y = _get_xy(right_mod)

    left_band_x = left_x - 5.5
    right_band_x = right_x - 1.0

    left_rows = [left_y + 7.0, left_y + 5.0, left_y + 3.0, left_y + 1.0, left_y - 1.0, left_y - 3.0]
    right_rows = [right_y + 7.0, right_y + 5.0, right_y + 3.0, right_y + 1.0, right_y - 1.0, right_y - 3.0]

    li = 0
    ri = 0

    for m in modules:
        ref = _get_ref(m)
        if ref in (left_ref, right_ref):
            continue

        w, h = _get_wh(m)
        area = w * h

        # Only pre-place small passives, leave larger parts alone
        if area > 20.0:
            continue

        prefix = "".join([c for c in ref if c.isalpha()]).upper()
        if prefix not in ("R", "C"):
            continue

        x, y = _get_xy(m)

        if x <= (left_x + right_x) * 0.5:
            row = left_rows[li % len(left_rows)]
            col = li // len(left_rows)
            _set_xy(m, left_band_x - 1.6 * col, row)
            li += 1
        else:
            row = right_rows[ri % len(right_rows)]
            col = ri // len(right_rows)
            _set_xy(m, right_band_x + 1.6 * col, row)
            ri += 1


def _evaluate_layout(
    modules: List[Any],
    nets: Any,
    board: Any,
    evaluate_fn=None,
    hpwl_fn=None,
) -> SweepMetrics:
    if evaluate_fn is not None:
        data = _safe_call(evaluate_fn, modules, nets, board)
        if isinstance(data, dict):
            return SweepMetrics(
                bbox_width=float(data.get("bbox_width", data.get("width", 0.0))),
                bbox_height=float(data.get("bbox_height", data.get("height", 0.0))),
                bbox_area=float(
                    data.get(
                        "bbox_area",
                        float(data.get("bbox_width", data.get("width", 0.0)))
                        * float(data.get("bbox_height", data.get("height", 0.0))),
                    )
                ),
                hpwl=float(data.get("hpwl", 0.0)),
                overlap_penalty=float(data.get("overlap_penalty", 0.0)),
                boundary_penalty=float(data.get("boundary_penalty", 0.0)),
            )

    metrics = _bbox_metrics(modules)
    if hpwl_fn is not None:
        try:
            metrics.hpwl = float(_safe_call(hpwl_fn, modules, nets))
        except Exception:
            metrics.hpwl = 0.0
    return metrics


def _score(metrics: SweepMetrics) -> float:
    # BBox-dominant scoring
    return (
        1000.0 * metrics.bbox_area
        + 10000.0 * metrics.overlap_penalty
        + 100.0 * metrics.boundary_penalty
        + 0.2 * metrics.hpwl
    )


def sweep_u9_u11_seeds(
    modules: List[Any],
    nets: Any,
    board: Any,
    run_flow_fn,
    evaluate_fn=None,
    hpwl_fn=None,
    left_ref: str = "U9",
    right_ref: str = "U11",
    x0: float = 55.0,
    y_candidates: Optional[List[float]] = None,
    dx_candidates: Optional[List[float]] = None,
    use_passive_prepack: bool = True,
    verbose: bool = True,
) -> SweepResult:
    """
    Outer-loop seed sweep:
    1) Force U9 and U11 onto the same Y
    2) Sweep different X distances
    3) Run the original placement flow for each seed
    4) Select the one with the smallest effective score
    """

    if y_candidates is None:
        y_candidates = [39.0, 40.0, 41.0, 42.0]

    if dx_candidates is None:
        dx_candidates = [10.0, 12.0, 14.0, 16.0, 18.0, 20.0]

    best_result: Optional[SweepResult] = None

    for y0 in y_candidates:
        for dx in dx_candidates:
            trial_modules = _build_initial_seed(
                modules=modules,
                anchor_refs=(left_ref, right_ref),
                x0=x0,
                y0=y0,
                dx=dx,
            )

            if use_passive_prepack:
                _optional_prepack_passives_near_anchors(
                    trial_modules,
                    left_ref=left_ref,
                    right_ref=right_ref,
                )

            # Run your original flow:
            # simulated annealing / legalization / compaction / etc.
            final_modules = _safe_call(run_flow_fn, trial_modules, nets, board)

            # Some codebases return (modules, extra_info)
            if isinstance(final_modules, tuple):
                final_modules = final_modules[0]

            metrics = _evaluate_layout(
                modules=final_modules,
                nets=nets,
                board=board,
                evaluate_fn=evaluate_fn,
                hpwl_fn=hpwl_fn,
            )
            score = _score(metrics)

            if verbose:
                print(
                    f"[SeedSweep] y={y0:.1f}, dx={dx:.1f} | "
                    f"bbox=({metrics.bbox_width:.2f}, {metrics.bbox_height:.2f}) "
                    f"area={metrics.bbox_area:.2f}, hpwl={metrics.hpwl:.2f}, score={score:.2f}"
                )

            result = SweepResult(
                modules=deepcopy(final_modules),
                metrics=metrics,
                score=score,
                seed_info={"x0": x0, "y0": y0, "dx": dx},
            )

            if best_result is None or result.score < best_result.score:
                best_result = result

    if best_result is None:
        raise RuntimeError("Seed sweep failed: no valid result was produced.")

    if verbose:
        print(
            f"[SeedSweep] BEST: x0={best_result.seed_info['x0']:.1f}, "
            f"y0={best_result.seed_info['y0']:.1f}, dx={best_result.seed_info['dx']:.1f} | "
            f"bbox=({best_result.metrics.bbox_width:.2f}, {best_result.metrics.bbox_height:.2f}) "
            f"area={best_result.metrics.bbox_area:.2f}, hpwl={best_result.metrics.hpwl:.2f}, "
            f"score={best_result.score:.2f}"
        )

    return best_result