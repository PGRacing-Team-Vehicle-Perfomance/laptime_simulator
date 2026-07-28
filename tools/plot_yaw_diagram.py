#!/usr/bin/env python3

import sys
import csv
import os
import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib.cm import ScalarMappable
from matplotlib.colors import Normalize
from collections import defaultdict


STEERING_CMAP = "coolwarm"
SLIP_CMAP = "PRGn"

LATACC_LABEL = "Lateral acceleration [m/s²]"
YAWMOMENT_LABEL = "Yaw moment [N·m]"


def read_csv(path):
    data = []
    with open(path, newline="") as csvfile:
        r = csv.DictReader(csvfile)
        for row in r:
            data.append(
                {
                    "steering": float(row["steering"]),
                    "slip": float(row["slip"]),
                    "latAcc": float(row["latAcc"]),
                    "yawMoment": float(row["yawMoment"]),
                }
            )
    return data


def group_by(data, key):
    grouped = defaultdict(list)
    for point in data:
        grouped[point[key]].append(point)
    return grouped


def symmetric_norm(values):
    limit = max(abs(v) for v in values) if values else 1.0
    return Normalize(vmin=-limit, vmax=limit)


def plot_isolines(ax, grouped, sort_key, cmap, norm):
    mappable = ScalarMappable(norm=norm, cmap=cmap)
    for angle in sorted(grouped.keys()):
        points = sorted(grouped[angle], key=lambda p: p[sort_key])
        x = [p["latAcc"] for p in points]
        y = [p["yawMoment"] for p in points]
        ax.plot(x, y, color=mappable.to_rgba(angle), alpha=0.85, linewidth=1.0)
    return mappable


def add_colorbar(fig, ax, mappable, label):
    cbar = fig.colorbar(mappable, ax=ax, pad=0.02)
    cbar.set_label(label)
    return cbar


def style_axes(ax, title, subtitle):
    ax.axhline(0.0, color="0.6", linewidth=0.8, zorder=0)
    ax.axvline(0.0, color="0.6", linewidth=0.8, zorder=0)
    ax.set_xlabel(LATACC_LABEL)
    ax.set_ylabel(YAWMOMENT_LABEL)
    ax.set_title(f"{title}\n{subtitle}", fontsize=11)
    ax.grid(True, alpha=0.3)


def render_steering(by_steering, title_prefix=""):
    norm = symmetric_norm(list(by_steering.keys()))
    fig, ax = plt.subplots(figsize=(10, 8))
    mappable = plot_isolines(ax, by_steering, "slip", STEERING_CMAP, norm)
    add_colorbar(fig, ax, mappable, "Steering angle [°]")
    style_axes(
        ax,
        f"{title_prefix}Yaw moment diagram — constant-steering isolines",
        "each line = fixed steering angle, chassis slip swept",
    )
    fig.tight_layout()
    return fig


def render_slip(by_slip, title_prefix=""):
    norm = symmetric_norm(list(by_slip.keys()))
    fig, ax = plt.subplots(figsize=(10, 8))
    mappable = plot_isolines(ax, by_slip, "steering", SLIP_CMAP, norm)
    add_colorbar(fig, ax, mappable, "Chassis slip angle [°]")
    style_axes(
        ax,
        f"{title_prefix}Yaw moment diagram — constant-slip isolines",
        "each line = fixed chassis slip angle, steering swept",
    )
    fig.tight_layout()
    return fig


def render_combined(by_steering, by_slip, title_prefix=""):
    steering_norm = symmetric_norm(list(by_steering.keys()))
    slip_norm = symmetric_norm(list(by_slip.keys()))
    fig, ax = plt.subplots(figsize=(11, 8))
    steering_mappable = plot_isolines(ax, by_steering, "slip", STEERING_CMAP, steering_norm)
    slip_mappable = plot_isolines(ax, by_slip, "steering", SLIP_CMAP, slip_norm)
    add_colorbar(fig, ax, steering_mappable, "Steering angle [°]")
    add_colorbar(fig, ax, slip_mappable, "Chassis slip angle [°]")
    style_axes(
        ax,
        f"{title_prefix}Yaw moment diagram (MMM)",
        "constant-steering (coolwarm) and constant-slip (PRGn) isolines",
    )
    fig.tight_layout()
    return fig


def render_isoline_figures(data, title_prefix=""):
    by_steering = group_by(data, "steering")
    by_slip = group_by(data, "slip")
    return {
        "steering": render_steering(by_steering, title_prefix),
        "slip": render_slip(by_slip, title_prefix),
        "combined": render_combined(by_steering, by_slip, title_prefix),
    }


def save_figures(figures, out_dir, prefix=""):
    os.makedirs(out_dir, exist_ok=True)
    paths = {}
    for name, fig in figures.items():
        out_png = os.path.join(out_dir, f"{prefix}{name}.png")
        fig.savefig(out_png, dpi=120)
        plt.close(fig)
        paths[name] = out_png
        print(f"Saved plot to {out_png}")
    return paths


def main():
    if len(sys.argv) < 2:
        print("Usage: python3 tools/plot_yaw_diagram.py path/to/yaw_diagram.csv")
        sys.exit(1)
    path = sys.argv[1]
    data = read_csv(path)

    base = os.path.basename(path)
    name = base[:-4] if base.lower().endswith(".csv") else base
    out_dir = os.path.join(os.path.dirname(os.path.abspath(path)), name)

    figures = render_isoline_figures(data)
    save_figures(figures, out_dir)


if __name__ == "__main__":
    main()
