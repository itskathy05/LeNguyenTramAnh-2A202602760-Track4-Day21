"""Reproducible LiDAR-camera calibration QA experiments for the lab."""
from __future__ import annotations

import argparse
import csv
import os
import platform
import tempfile
import time
from pathlib import Path

import cv2
os.environ.setdefault("MPLCONFIGDIR", str(Path(tempfile.gettempdir()) / "day6_calibration_qa_mpl"))
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np

from starter.datasets import dataset_type, list_frames, load_frame
from starter.projection import (
    draw_box2d,
    overlay_points,
    perturb_extrinsic,
    project_velo_to_image,
)

ROOT = Path(__file__).resolve().parents[1]
OUT = ROOT / "results"
FIG = OUT / "figures"
SEED = 42
DATASETS = {
    "synthetic": ("data/synthetic", ["000000"]),
    "kitti": ("data/kitti_mini", ["000011", "000012", "000019", "000049", "000061"]),
    "nuscenes": ("data/nuscenes_mini_subset", ["scene-0103_010", "scene-0103_020", "scene-1094_010", "scene-1094_020"]),
}
ROTATION_LEVELS = [-3.0, -2.0, -1.0, -0.5, 0.0, 0.5, 1.0, 2.0, 3.0]
TRANSLATION_LEVELS_CM = [-10.0, -5.0, -2.0, 0.0, 2.0, 5.0, 10.0]
RADIUS_PX = 3.0


def _frame(data_name: str, frame_id: str):
    data_root = ROOT / DATASETS[data_name][0]
    kwargs = {"use_ego_motion": True} if dataset_type(data_root) == "nuscenes" else {}
    return load_frame(data_root, frame_id, **kwargs)


def _project(points: np.ndarray, calib, image_shape):
    uv, depth, mask = project_velo_to_image(points, calib, image_shape)
    return uv, depth, mask


def _inside_boxes(uv: np.ndarray, labels) -> tuple[np.ndarray, list[np.ndarray]]:
    in_any = np.zeros(len(uv), dtype=bool)
    per_box = []
    for obj in labels:
        x1, y1, x2, y2 = obj.bbox
        m = (uv[:, 0] >= x1) & (uv[:, 0] <= x2) & (uv[:, 1] >= y1) & (uv[:, 1] <= y2)
        per_box.append(m)
        in_any |= m
    return in_any, per_box


def _edge_map(image: np.ndarray) -> np.ndarray:
    gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
    edges = cv2.Canny(gray, 80, 160)
    # Distance to the nearest image edge, clipped so isolated edges do not dominate.
    distance = cv2.distanceTransform((edges == 0).astype(np.uint8), cv2.DIST_L2, 3)
    return distance


def _metrics(points: np.ndarray, calib, image: np.ndarray, labels, clean_uv=None, clean_mask=None, edge_distance=None):
    uv, depth, mask = _project(points, calib, image.shape)
    finite_count = int(np.isfinite(points[:, :3]).all(axis=1).sum())
    fov_ratio = float(mask.sum() / max(finite_count, 1))
    base_indices = np.flatnonzero(clean_mask) if clean_mask is not None else np.flatnonzero(mask)
    base_uv = clean_uv if clean_uv is not None else uv
    base_inside, _ = _inside_boxes(base_uv, labels)
    tracked_indices = base_indices[base_inside]
    retention = float("nan")
    edge_score = float("nan")
    object_count = 0
    if len(tracked_indices):
        object_count = len(tracked_indices)
        # Keep projected coordinates aligned to source point indices for correspondence.
        moved_uv = np.full((len(points), 2), np.nan, dtype=np.float64)
        moved_uv[mask] = uv
        selected_uv = moved_uv[tracked_indices]
        valid_selected = np.isfinite(selected_uv).all(axis=1)
        moved_inside = np.zeros(len(tracked_indices), dtype=bool)
        for obj in labels:
            x1, y1, x2, y2 = obj.bbox
            moved_inside |= ((selected_uv[:, 0] >= x1) & (selected_uv[:, 0] <= x2)
                             & (selected_uv[:, 1] >= y1) & (selected_uv[:, 1] <= y2))
        retention = float(np.count_nonzero(moved_inside) / len(tracked_indices))
        if edge_distance is None:
            edge_distance = _edge_map(image)
        h, w = image.shape[:2]
        xs = np.clip(np.rint(np.nan_to_num(selected_uv[:, 0])).astype(int), 0, w - 1)
        ys = np.clip(np.rint(np.nan_to_num(selected_uv[:, 1])).astype(int), 0, h - 1)
        sampled_distance = edge_distance[ys, xs]
        if valid_selected.any():
            edge_score = float(np.mean(sampled_distance[valid_selected] <= RADIUS_PX))
    return {
        "inside_fov_ratio": fov_ratio,
        "object_point_retention": retention,
        "edge_alignment_score": edge_score,
        "object_points_baseline": object_count,
        "points_total": int(len(points)),
    }


def _baseline(data_name: str, frame_id: str):
    fr = _frame(data_name, frame_id)
    uv, depth, mask = _project(fr["points"], fr["calib"], fr["image"].shape)
    return fr, uv, depth, mask


def _calib_for(fr, perturbation: str, axis: str, value: float):
    rot = {"roll": 0.0, "pitch": 0.0, "yaw": 0.0}
    trans = [0.0, 0.0, 0.0]
    if perturbation == "rotation":
        rot[axis] = value
    else:
        trans["xyz".index(axis)] = value / 100.0
    return perturb_extrinsic(fr["calib"], rot["roll"], rot["pitch"], rot["yaw"], tuple(trans))


def _write_csv(path: Path, rows: list[dict]):
    path.parent.mkdir(parents=True, exist_ok=True)
    if not rows:
        return
    with path.open("w", newline="", encoding="utf-8") as f:
        writer = csv.DictWriter(f, fieldnames=list(rows[0]))
        writer.writeheader()
        writer.writerows(rows)


def run_demo():
    FIG.mkdir(parents=True, exist_ok=True)
    examples = [("synthetic", "000000"), ("kitti", "000019"), ("kitti", "000011"),
                ("kitti", "000004"), ("nuscenes", "scene-0103_010")]
    rows = []
    for name, fid in examples:
        fr, uv, depth, mask = _baseline(name, fid)
        vis = overlay_points(fr["image"], uv, depth, radius=2)
        for obj in fr["labels"]:
            vis = draw_box2d(vis, obj.bbox, label=obj.type)
        out = FIG / f"demo_{name}_{fid}.png"
        cv2.imwrite(str(out), vis)
        rows.append({"dataset": name, "frame_id": fid, "points": len(fr["points"]),
                     "inside_fov": int(mask.sum()), "inside_fov_ratio": f"{mask.mean():.6f}", "image": out.relative_to(ROOT).as_posix()})
        print(f"demo {name}/{fid}: {mask.sum()}/{len(mask)} points in image -> {out.relative_to(ROOT)}")
    _write_csv(OUT / "demo_summary.csv", rows)


def run_sweep():
    rows = []
    for name in ("kitti", "nuscenes"):
        for fid in DATASETS[name][1]:
            fr, clean_uv, _, clean_mask = _baseline(name, fid)
            edge_distance = _edge_map(fr["image"])
            axes_levels = [("rotation", axis, ROTATION_LEVELS, "deg") for axis in ("roll", "pitch", "yaw")]
            axes_levels += [("translation", axis, TRANSLATION_LEVELS_CM, "cm") for axis in "xyz"]
            for perturbation, axis, levels, unit in axes_levels:
                for value in levels:
                    calib = _calib_for(fr, perturbation, axis, value)
                    m = _metrics(fr["points"], calib, fr["image"], fr["labels"], clean_uv, clean_mask, edge_distance)
                    rows.append({"dataset": name, "frame_id": fid, "perturbation": perturbation,
                                 "axis": axis, "value": value, "unit": unit, "seed": SEED, **m})
    thresholds = {}
    for name in ("kitti", "nuscenes"):
        clean_scores = [float(r["edge_alignment_score"]) for r in rows
                       if r["dataset"] == name and r["frame_id"] in DATASETS[name][1]
                       and r["perturbation"] == "rotation" and r["axis"] == "yaw" and float(r["value"]) == 0
                       and np.isfinite(float(r["edge_alignment_score"]))]
        # Conservative dataset-specific threshold: lowest score measured on the clean reference frames.
        thresholds[name] = min(clean_scores) if clean_scores else float("nan")
    for row in rows:
        threshold = thresholds[row["dataset"]]
        score = float(row["edge_alignment_score"])
        retention = float(row["object_point_retention"])
        row["drift_threshold"] = threshold
        row["drift_detected"] = bool(np.isfinite(score) and np.isfinite(threshold) and score < threshold)
        row["retention_threshold"] = 0.95
        row["retention_drift_detected"] = bool(np.isfinite(retention) and retention < 0.95)
    _write_csv(OUT / "calibration_sweep.csv", rows)
    _plot_sweep(rows)
    _plot_dataset_summary(rows)
    _make_failure(rows)
    print(f"sweep: {len(rows)} configurations -> results/calibration_sweep.csv")


def _plot_sweep(rows):
    FIG.mkdir(parents=True, exist_ok=True)
    fig, axes = plt.subplots(2, 3, figsize=(14, 8), constrained_layout=True)
    for ax, (kind, axis, unit) in zip(axes.flat, [("rotation", a, "deg") for a in ("roll", "pitch", "yaw")] + [("translation", a, "cm") for a in "xyz"]):
        for dataset in ("kitti", "nuscenes"):
            subset = [r for r in rows if r["dataset"] == dataset and r["perturbation"] == kind and r["axis"] == axis]
            values = sorted(set(float(r["value"]) for r in subset))
            means = [np.nanmean([float(r["object_point_retention"]) for r in subset if float(r["value"]) == v]) for v in values]
            ax.plot(values, means, marker="o", label=dataset)
        ax.set(title=f"{axis} ({unit})", xlabel=f"{axis} perturbation ({unit})", ylabel="Object point retention", ylim=(-0.03, 1.03))
        ax.grid(alpha=.25)
        ax.legend()
    fig.suptitle("LiDAR-camera calibration drift sensitivity")
    fig.savefig(FIG / "calibration_sensitivity.png", dpi=160)
    plt.close(fig)


def _plot_dataset_summary(rows):
    fig, axes = plt.subplots(1, 2, figsize=(11, 4), constrained_layout=True)
    for ax, metric, title in zip(axes, ("object_point_retention", "edge_alignment_score"), ("Object point retention", "Edge alignment score")):
        for dataset in ("kitti", "nuscenes"):
            subset = [r for r in rows if r["dataset"] == dataset and r["perturbation"] == "rotation" and r["axis"] == "yaw"]
            xs = sorted(set(float(r["value"]) for r in subset))
            ys = [np.nanmean([float(r[metric]) for r in subset if float(r["value"]) == x]) for x in xs]
            ax.plot(xs, ys, marker="o", label=dataset)
        ax.set(xlabel="Yaw drift (degrees)", ylabel="Score", title=title, ylim=(-.03, 1.03))
        if metric == "edge_alignment_score":
            for dataset, style in (("kitti", "--"), ("nuscenes", ":")):
                threshold = next(float(r["drift_threshold"]) for r in rows if r["dataset"] == dataset)
                ax.axhline(threshold, linestyle=style, alpha=.6, label=f"{dataset} threshold")
        ax.grid(alpha=.25)
        ax.legend()
    fig.savefig(FIG / "detector_comparison.png", dpi=160)
    plt.close(fig)


def run_stress():
    rows = []
    levels = [("random_dropout", value) for value in (1.0, .7, .5, .3)]
    levels += [("gaussian_noise", value) for value in (0.0, .02, .05, .10)]
    for name in ("kitti", "nuscenes"):
        for fid in DATASETS[name][1]:
            fr, clean_uv, _, clean_mask = _baseline(name, fid)
            edge_distance = _edge_map(fr["image"])
            original = fr["points"]
            base_inside, _ = _inside_boxes(clean_uv, fr["labels"])
            base_ids = np.flatnonzero(clean_mask)[base_inside]
            base_count = len(base_ids)
            for perturbation, value in levels:
                points = original.copy()
                if perturbation == "random_dropout":
                    rng = np.random.default_rng(SEED)
                    keep = rng.random(len(original)) < value
                    points[~keep, :3] = np.nan  # preserve indices so retention measures the same original points
                else:
                    rng = np.random.default_rng(SEED)
                    points[:, :3] += rng.normal(0, value, size=(len(points), 3)).astype(points.dtype)
                m = _metrics(points, fr["calib"], fr["image"], fr["labels"], clean_uv, clean_mask, edge_distance)
                rows.append({"dataset": name, "frame_id": fid, "perturbation": perturbation,
                             "axis": "all", "value": value, "unit": "ratio" if perturbation == "random_dropout" else "m",
                             "seed": SEED, "baseline_object_points": base_count, **m})
    _write_csv(OUT / "degradation_stress.csv", rows)
    _plot_stress(rows)
    print(f"stress: {len(rows)} rows -> results/degradation_stress.csv")


def _plot_stress(rows):
    fig, axes = plt.subplots(1, 2, figsize=(11, 4), constrained_layout=True)
    for ax, perturbation, xlabel in zip(axes, ("random_dropout", "gaussian_noise"), ("Keep ratio", "Noise sigma (m)")):
        for dataset in ("kitti", "nuscenes"):
            subset = [r for r in rows if r["dataset"] == dataset and r["perturbation"] == perturbation]
            xs = sorted(set(float(r["value"]) for r in subset))
            ys = [np.nanmean([float(r["object_point_retention"]) for r in subset if float(r["value"]) == x]) for x in xs]
            ax.plot(xs, ys, marker="o", label=dataset)
        ax.set(xlabel=xlabel, ylabel="Object point count / clean baseline", title=perturbation)
        ax.grid(alpha=.25)
        ax.legend()
    fig.savefig(FIG / "degradation_stress.png", dpi=160)
    plt.close(fig)


def run_latency():
    rows = []
    for name in ("kitti", "nuscenes"):
        for fid in DATASETS[name][1]:
            fr = _frame(name, fid)
            project_velo_to_image(fr["points"], fr["calib"], fr["image"].shape)  # warm-up
            samples = []
            for rep in range(30):
                t0 = time.perf_counter()
                project_velo_to_image(fr["points"], fr["calib"], fr["image"].shape)
                ms = (time.perf_counter() - t0) * 1000
                samples.append(ms)
            p50, p95 = np.percentile(samples, [50, 95])
            for rep, ms in enumerate(samples, start=1):
                rows.append({"dataset": name, "frame_id": fid, "repeat": rep, "latency_ms": f"{ms:.6f}",
                             "p50_ms": f"{p50:.6f}", "p95_ms": f"{p95:.6f}", "seed": SEED,
                             "python": platform.python_version(), "platform": platform.platform(),
                             "cpu_model": platform.processor() or "not reported by OS",
                             "logical_cpu_count": os.cpu_count()})
            print(f"latency {name}/{fid}: p50={p50:.3f} ms p95={p95:.3f} ms")
    _write_csv(OUT / "latency.csv", rows)
    _plot_latency(rows)
    print(f"latency: {len(rows)} timed runs -> results/latency.csv")


def _plot_latency(rows):
    fig, ax = plt.subplots(figsize=(7, 4), constrained_layout=True)
    for dataset in ("kitti", "nuscenes"):
        ys = [float(r["latency_ms"]) for r in rows if r["dataset"] == dataset]
        ax.boxplot(ys, positions=[1 if dataset == "kitti" else 2], tick_labels=[dataset], showfliers=False)
    ax.set(ylabel="Projection latency (ms)", title="30 repeats per frame; first call excluded")
    ax.grid(axis="y", alpha=.25)
    fig.savefig(FIG / "latency_distribution.png", dpi=160)
    plt.close(fig)


def _make_failure(rows):
    # Select the strongest object-retention failure missed by the dataset-calibrated edge threshold.
    candidates = [r for r in rows if r["dataset"] == "kitti" and r["frame_id"] == "000012"
                  and float(r["value"]) != 0 and not r["drift_detected"]]
    if not candidates:
        candidates = [r for r in rows if r["dataset"] == "kitti" and r["frame_id"] == "000012" and float(r["value"]) != 0]
    if not candidates:
        return
    fr, clean_uv, clean_depth, clean_mask = _baseline("kitti", "000012")
    chosen = min(candidates, key=lambda r: (float(r["object_point_retention"]), -abs(float(r["value"]))))
    value = float(chosen["value"])
    perturbation, axis = chosen["perturbation"], chosen["axis"]
    calib = _calib_for(fr, perturbation, axis, value)
    uv, depth, _ = _project(fr["points"], calib, fr["image"].shape)
    drift_vis = overlay_points(fr["image"], uv, depth, radius=2)
    for obj in fr["labels"]:
        drift_vis = draw_box2d(drift_vis, obj.bbox, color=(0, 255, 0), label=obj.type)
    clean_vis = overlay_points(fr["image"], clean_uv, clean_depth, radius=2)
    for obj in fr["labels"]:
        clean_vis = draw_box2d(clean_vis, obj.bbox, color=(0, 255, 0), label=obj.type)
    h = min(clean_vis.shape[0], drift_vis.shape[0])
    panel = np.hstack([clean_vis[:h], drift_vis[:h]])
    cv2.putText(panel, "CLEAN", (20, 32), cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 255, 255), 2)
    unit = chosen["unit"]
    cv2.putText(panel, f"DRIFT {axis}={value:g} {unit}", (clean_vis.shape[1] + 20, 32), cv2.FONT_HERSHEY_SIMPLEX, .8, (255, 255, 255), 2)
    cv2.putText(panel, f"retention={float(chosen['object_point_retention']):.3f} edge={float(chosen['edge_alignment_score']):.3f} threshold={float(chosen['drift_threshold']):.3f}",
                (20, h - 15), cv2.FONT_HERSHEY_SIMPLEX, .55, (255, 255, 255), 2)
    FIG.mkdir(parents=True, exist_ok=True)
    cv2.imwrite(str(FIG / "fail_01_pitch_drift_edge_false_negative.png"), panel)
    print(f"failure selected: KITTI 000012 {axis}={value:g} {unit}, retention={chosen['object_point_retention']}, edge={chosen['edge_alignment_score']}, threshold={chosen['drift_threshold']}")


def run_all():
    run_demo()
    run_sweep()
    run_stress()
    run_latency()


def self_check():
    from starter.projection import cam_to_image, velo_to_cam
    fr = _frame("synthetic", "000000")
    cam = velo_to_cam(np.array([[10.0, 0.0, 0.0]]), fr["calib"])
    uv, depth, mask = cam_to_image(cam, fr["calib"].P2, fr["image"].shape)
    assert abs(cam[0, 2] - 9.73) < .15, cam
    assert mask.tolist() == [True] and np.allclose(uv[0], [614, 175], atol=4), (uv, depth, mask)
    sample = np.array([[1, 1, 5], [np.nan, 0, 2], [0, 0, -1], [1e5, 1e5, 2]], dtype=float)
    _, _, m = cam_to_image(sample, fr["calib"].P2, fr["image"].shape)
    assert m.tolist() == [True, False, False, False], m
    empty_uv, empty_depth, empty_mask = cam_to_image(np.empty((0, 3)), fr["calib"].P2, fr["image"].shape)
    assert empty_uv.shape == (0, 2) and empty_depth.shape == (0,) and empty_mask.shape == (0,)
    print(f"self-check PASS: cam={cam[0].round(3).tolist()} uv={uv[0].round(2).tolist()}")


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    sub = parser.add_subparsers(dest="command", required=True)
    for name in ("demo", "sweep", "stress", "latency", "all", "self-check"):
        sub.add_parser(name)
    args = parser.parse_args()
    {"demo": run_demo, "sweep": run_sweep, "stress": run_stress, "latency": run_latency,
     "all": run_all, "self-check": self_check}[args.command]()


if __name__ == "__main__":
    main()
