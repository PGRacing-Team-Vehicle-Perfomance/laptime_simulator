#!/usr/bin/env python3

import sys
import csv
import os
import math
import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
from collections import defaultdict


STEERING_COLOR = "tab:blue"
SLIP_COLOR = "tab:red"

LATACC_LABEL = "Lateral acceleration [m/s²]"
YAWMOMENT_LABEL = "Yaw moment [N·m]"


def read_csv(path):
    data = []
    with open(path, newline="") as csvfile:
        r = csv.DictReader(csvfile)
        for row in r:
            point = {
                "steering": float(row["steering"]),
                "slip": float(row["slip"]),
                "latAcc": float(row["latAcc"]),
                "yawMoment": float(row["yawMoment"]),
                "baseSteering": float(row.get("baseSteering", 1.0)) > 0.5,
                "baseSlip": float(row.get("baseSlip", 1.0)) > 0.5,
            }
            if math.isfinite(point["latAcc"]) and math.isfinite(point["yawMoment"]):
                data.append(point)
    return data


FAMILY_FLAG = {"steering": "baseSteering", "slip": "baseSlip"}


def group_by(data, key):
    flag = FAMILY_FLAG.get(key)
    grouped = defaultdict(list)
    for point in data:
        if flag is None or point.get(flag, True):
            grouped[point[key]].append(point)
    return grouped


def plot_steering_isolines(ax, by_steering, color):
    for steering_angle in sorted(by_steering.keys()):
        points = sorted(by_steering[steering_angle], key=lambda p: p["slip"])
        ax.plot([p["latAcc"] for p in points], [p["yawMoment"] for p in points],
                color=color, alpha=0.6, linewidth=0.8)


def plot_slip_isolines(ax, by_slip, color):
    for slip_angle in sorted(by_slip.keys()):
        points = sorted(by_slip[slip_angle], key=lambda p: p["steering"])
        ax.plot([p["latAcc"] for p in points], [p["yawMoment"] for p in points],
                color=color, alpha=0.6, linewidth=0.8)


def style_axes(ax, title):
    ax.axhline(0.0, color="0.7", linewidth=0.8, zorder=0)
    ax.axvline(0.0, color="0.7", linewidth=0.8, zorder=0)
    ax.set_xlabel(LATACC_LABEL)
    ax.set_ylabel(YAWMOMENT_LABEL)
    ax.set_title(title)
    ax.grid(True, alpha=0.3)


def render_steering(by_steering, title_prefix=""):
    fig, ax = plt.subplots(figsize=(10, 8))
    plot_steering_isolines(ax, by_steering, STEERING_COLOR)
    ax.plot([], [], color=STEERING_COLOR, label="constant steering")
    style_axes(ax, f"{title_prefix}Yaw moment diagram — steering isolines")
    ax.legend(loc="best")
    fig.tight_layout()
    return fig


def render_slip(by_slip, title_prefix=""):
    fig, ax = plt.subplots(figsize=(10, 8))
    plot_slip_isolines(ax, by_slip, SLIP_COLOR)
    ax.plot([], [], color=SLIP_COLOR, label="constant chassis slip")
    style_axes(ax, f"{title_prefix}Yaw moment diagram — chassis slip isolines")
    ax.legend(loc="best")
    fig.tight_layout()
    return fig


def render_combined(by_steering, by_slip, title_prefix=""):
    fig, ax = plt.subplots(figsize=(10, 8))
    plot_steering_isolines(ax, by_steering, STEERING_COLOR)
    plot_slip_isolines(ax, by_slip, SLIP_COLOR)
    ax.plot([], [], color=STEERING_COLOR, label="constant steering")
    ax.plot([], [], color=SLIP_COLOR, label="constant chassis slip")
    style_axes(ax, f"{title_prefix}Yaw moment diagram")
    ax.legend(loc="best")
    fig.tight_layout()
    return fig


def render_combined_zoom(by_steering, by_slip, data, title_prefix=""):
    fig = render_combined(by_steering, by_slip, title_prefix)
    ax = fig.axes[0]
    max_lat = max(p["latAcc"] for p in data)
    band = [p for p in data if p["latAcc"] >= 0.8 * max_lat]
    latitudes = [p["latAcc"] for p in band]
    moments = [p["yawMoment"] for p in band]
    x_pad = 0.02 * max_lat
    y_pad = 0.08 * ((max(moments) - min(moments)) or 1.0)
    ax.set_xlim(min(latitudes) - x_pad, max_lat + x_pad)
    ax.set_ylim(min(moments) - y_pad, max(moments) + y_pad)
    ax.set_title(f"{title_prefix}Yaw moment diagram — zoom on peak lateral acceleration")
    return fig


def render_isoline_figures(data, title_prefix=""):
    by_steering = group_by(data, "steering")
    by_slip = group_by(data, "slip")
    return {
        "steering": render_steering(by_steering, title_prefix),
        "slip": render_slip(by_slip, title_prefix),
        "combined": render_combined(by_steering, by_slip, title_prefix),
        "combined_zoom": render_combined_zoom(by_steering, by_slip, data, title_prefix),
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
