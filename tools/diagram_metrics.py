#!/usr/bin/env python3

import csv

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib.lines import Line2D

from plot_yaw_diagram import style_axes


LATACC, MOMENT = 0, 1

METRIC_KEYS = [
    "max_latacc_at_zero_moment",
    "min_latacc_at_zero_moment",
    "max_latacc_overall",
    "min_latacc_overall",
    "max_moment_at_zero_latacc",
    "min_moment_at_zero_latacc",
    "max_moment_overall",
    "min_moment_overall",
]


def read_metrics_csv(path):
    """Wczytaj metrics.csv (produkt C++) -> {metric: {latAcc, yawMoment}}."""
    metrics = {}
    with open(path, newline="") as f:
        for row in csv.DictReader(f):
            metrics[row["metric"]] = {
                "latAcc": float(row["latAcc"]),
                "yawMoment": float(row["yawMoment"]),
            }
    return metrics


def read_hull_csv(path):
    """Wczytaj hull.csv (wierzcholki otoczki) -> [(latAcc, yawMoment), ...]."""
    hull = []
    with open(path, newline="") as f:
        for row in csv.DictReader(f):
            hull.append((float(row["latAcc"]), float(row["yawMoment"])))
    return hull


SUMMARY_COLUMNS = [
    ("max_latacc_at_zero_moment", "latAcc"),
    ("min_latacc_at_zero_moment", "latAcc"),
    ("max_latacc_overall", "latAcc"),
    ("max_latacc_overall", "yawMoment"),
    ("min_latacc_overall", "latAcc"),
    ("min_latacc_overall", "yawMoment"),
    ("max_moment_at_zero_latacc", "yawMoment"),
    ("min_moment_at_zero_latacc", "yawMoment"),
    ("max_moment_overall", "yawMoment"),
    ("max_moment_overall", "latAcc"),
    ("min_moment_overall", "yawMoment"),
    ("min_moment_overall", "latAcc"),
]

SUMMARY_HEADERS = [
    "setup",
    "max_latacc_at_zero_moment",
    "min_latacc_at_zero_moment",
    "max_latacc_overall",
    "max_latacc_overall_moment",
    "min_latacc_overall",
    "min_latacc_overall_moment",
    "max_moment_at_zero_latacc",
    "min_moment_at_zero_latacc",
    "max_moment_overall",
    "max_moment_overall_latacc",
    "min_moment_overall",
    "min_moment_overall_latacc",
]


def write_metrics_summary_csv(path, entries):
    with open(path, "w", newline="") as f:
        writer = csv.writer(f)
        writer.writerow(SUMMARY_HEADERS)
        for entry in entries:
            metrics = entry["metrics"]
            row = [entry["label"]]
            for metric_name, coordinate in SUMMARY_COLUMNS:
                point = metrics.get(metric_name)
                row.append("" if point is None else f"{point[coordinate]:.4f}")
            writer.writerow(row)


POINT_STYLES = {
    "max_latacc_at_zero_moment": ("tab:green", "D"),
    "min_latacc_at_zero_moment": ("tab:green", "D"),
    "max_latacc_overall": ("tab:blue", "o"),
    "min_latacc_overall": ("tab:blue", "o"),
    "max_moment_at_zero_latacc": ("tab:orange", "s"),
    "min_moment_at_zero_latacc": ("tab:orange", "s"),
    "max_moment_overall": ("tab:red", "^"),
    "min_moment_overall": ("tab:red", "v"),
}

LEGEND_ENTRIES = [
    ("tab:green", "D", "trimmed lat acc (Mz=0)"),
    ("tab:blue", "o", "peak lat acc"),
    ("tab:orange", "s", "yaw moment @ ay=0"),
    ("tab:red", "^", "peak yaw moment"),
]


def format_annotation(metric_name, point):
    if "latacc_at_zero_moment" in metric_name:
        return f"{point['latAcc']:.1f} m/s²"
    if "moment_at_zero_latacc" in metric_name:
        return f"{point['yawMoment']:.0f} N·m"
    if "latacc" in metric_name:
        return f"{point['latAcc']:.1f} m/s²\n@ {point['yawMoment']:.0f} N·m"
    return f"{point['yawMoment']:.0f} N·m\n@ {point['latAcc']:.1f} m/s²"


def render_metrics_figure(points, metrics, hull, title_prefix=""):
    fig, ax = plt.subplots(figsize=(10, 8))
    ax.scatter([point["latAcc"] for point in points], [point["yawMoment"] for point in points],
               s=3, color="0.8", alpha=0.4, zorder=1)
    if hull:
        ax.plot([vertex[LATACC] for vertex in hull] + [hull[0][LATACC]],
                [vertex[MOMENT] for vertex in hull] + [hull[0][MOMENT]],
                color="0.5", linewidth=1.0, zorder=2)
    for metric_name, (color, marker) in POINT_STYLES.items():
        point = metrics.get(metric_name)
        if point is None:
            continue
        ax.scatter([point["latAcc"]], [point["yawMoment"]], color=color, marker=marker, s=70,
                   edgecolor="black", linewidth=0.5, zorder=5)
        ax.annotate(format_annotation(metric_name, point), (point["latAcc"], point["yawMoment"]),
                    textcoords="offset points", xytext=(6, 6), fontsize=8, zorder=6)
    style_axes(ax, f"{title_prefix}Yaw moment diagram — key operating points")
    legend_handles = [Line2D([0], [0], marker=marker, color="w", markerfacecolor=color,
                             markeredgecolor="black", label=label, markersize=8)
                      for color, marker, label in LEGEND_ENTRIES]
    ax.legend(handles=legend_handles, loc="best")
    fig.tight_layout()
    return fig


def render_envelope_overlay(entries):
    fig, ax = plt.subplots(figsize=(11, 8))
    colormap = plt.get_cmap("viridis")
    setup_count = len(entries)
    for index, entry in enumerate(entries):
        hull = entry["hull"]
        if not hull:
            continue
        color = colormap(index / max(1, setup_count - 1))
        ax.plot([vertex[LATACC] for vertex in hull] + [hull[0][LATACC]],
                [vertex[MOMENT] for vertex in hull] + [hull[0][MOMENT]],
                color=color, linewidth=1.5, alpha=0.9, label=entry["label"])
        for metric_name in ("max_latacc_at_zero_moment", "min_latacc_at_zero_moment"):
            point = entry["metrics"].get(metric_name)
            if point is not None:
                ax.scatter([point["latAcc"]], [0.0], color=color, marker="D", s=25, zorder=5)
    style_axes(ax, "All setups — operating envelopes (convex hull)")
    ax.legend(loc="best", fontsize=8)
    fig.tight_layout()
    return fig


BAR_SPECS = [
    ("max_latacc_at_zero_moment", "latAcc", "max trimmed aᵧ (Mz=0) [m/s²]"),
    ("min_latacc_at_zero_moment", "latAcc", "min trimmed aᵧ (Mz=0) [m/s²]"),
    ("max_latacc_overall", "latAcc", "peak aᵧ [m/s²]"),
    ("max_moment_at_zero_latacc", "yawMoment", "max yaw moment @ aᵧ=0 [N·m]"),
]


def render_metrics_bars(entries):
    fig, axes = plt.subplots(2, 2, figsize=(13, 9))
    setup_labels = [entry["label"] for entry in entries]
    bar_positions = range(len(entries))
    for ax, (metric_name, coordinate, title) in zip(axes.ravel(), BAR_SPECS):
        bar_values = [(entry["metrics"].get(metric_name) or {}).get(coordinate, float("nan"))
                      for entry in entries]
        ax.bar(list(bar_positions), bar_values, color="tab:blue")
        ax.set_xticks(list(bar_positions))
        ax.set_xticklabels(setup_labels, rotation=45, ha="right", fontsize=8)
        ax.set_title(title, fontsize=10)
        ax.grid(True, axis="y", alpha=0.3)
    fig.suptitle("All setups — key MMM metrics", fontsize=14)
    fig.tight_layout()
    return fig


def render_metrics_summary_figures(entries):
    return {
        "all_envelopes": render_envelope_overlay(entries),
        "all_metrics_bars": render_metrics_bars(entries),
    }
