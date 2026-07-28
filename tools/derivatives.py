#!/usr/bin/env python3

import sys
import os
import numpy as np
import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib.colors import Normalize

from plot_yaw_diagram import (
    read_csv,
    group_by,
    save_figures,
    style_axes,
)


DERIVATIVE_CMAP = "RdBu_r"

STABILITY_KEY = "dMz_dslip"
CONTROL_KEY = "dMz_dsteering"

STABILITY_LABEL = "Stability  ∂Mz/∂slip [N·m/°]"
CONTROL_LABEL = "Control  ∂Mz/∂steering [N·m/°]"


def line_derivative(points, along_key):
    ordered = sorted(points, key=lambda p: p[along_key])
    axis = np.array([p[along_key] for p in ordered])
    moment = np.array([p["yawMoment"] for p in ordered])
    if len(axis) < 2:
        return {id(p): 0.0 for p in ordered}
    slope = np.gradient(moment, axis)
    return {id(p): float(s) for p, s in zip(ordered, slope)}


def compute_derivatives(data):
    stability = {}
    for steering_points in group_by(data, "steering").values():
        stability.update(line_derivative(steering_points, "slip"))

    control = {}
    for slip_points in group_by(data, "slip").values():
        control.update(line_derivative(slip_points, "steering"))

    for point in data:
        point[STABILITY_KEY] = stability[id(point)]
        point[CONTROL_KEY] = control[id(point)]
    return data


def robust_limit(values):
    if not values:
        return 1.0
    return float(np.percentile(np.abs(values), 98)) or max(abs(v) for v in values)


def symmetric_value_norm(values):
    limit = robust_limit(values)
    return Normalize(vmin=-limit, vmax=limit)


def render_derivative_field(data, value_key, cbar_label, title, subtitle):
    values = [p[value_key] for p in data]
    limit = robust_limit(values)
    norm = Normalize(vmin=-limit, vmax=limit)
    levels = np.linspace(-limit, limit, 21)
    fig, ax = plt.subplots(figsize=(11, 8))
    field = ax.tricontourf(
        [p["latAcc"] for p in data],
        [p["yawMoment"] for p in data],
        values,
        levels=levels,
        cmap=DERIVATIVE_CMAP,
        norm=norm,
        extend="both",
    )
    cbar = fig.colorbar(field, ax=ax, pad=0.02)
    cbar.set_label(cbar_label)
    style_axes(ax, f"{title}\n{subtitle}")
    fig.tight_layout()
    return fig


def grid_matrix(data, value_key):
    steering_axis = sorted({p["steering"] for p in data})
    slip_axis = sorted({p["slip"] for p in data})
    steering_index = {v: i for i, v in enumerate(steering_axis)}
    slip_index = {v: i for i, v in enumerate(slip_axis)}
    matrix = np.full((len(slip_axis), len(steering_axis)), np.nan)
    for point in data:
        matrix[slip_index[point["slip"]], steering_index[point["steering"]]] = point[value_key]
    return np.array(steering_axis), np.array(slip_axis), matrix


def render_derivative_grid(data, value_key, cbar_label, title, subtitle):
    steering_axis, slip_axis, matrix = grid_matrix(data, value_key)
    norm = symmetric_value_norm([p[value_key] for p in data])
    fig, ax = plt.subplots(figsize=(10, 8))
    mesh = ax.pcolormesh(steering_axis, slip_axis, matrix, cmap=DERIVATIVE_CMAP, norm=norm, shading="gouraud")
    cbar = fig.colorbar(mesh, ax=ax, pad=0.02)
    cbar.set_label(cbar_label)
    ax.set_xlabel("Steering angle [°]")
    ax.set_ylabel("Chassis slip angle [°]")
    ax.set_title(f"{title}\n{subtitle}", fontsize=11)
    fig.tight_layout()
    return fig


def render_derivative_figures(data, title_prefix=""):
    compute_derivatives(data)
    return {
        "control_heatmap": render_derivative_field(
            data,
            CONTROL_KEY,
            CONTROL_LABEL,
            f"{title_prefix}Control field ∂Mz/∂steering",
            "derivative of constant-slip isolines, mapped onto the MMM diagram",
        ),
        "stability_heatmap": render_derivative_field(
            data,
            STABILITY_KEY,
            STABILITY_LABEL,
            f"{title_prefix}Stability field ∂Mz/∂slip",
            "derivative of constant-steering isolines, mapped onto the MMM diagram",
        ),
        "control_grid": render_derivative_grid(
            data,
            CONTROL_KEY,
            CONTROL_LABEL,
            f"{title_prefix}Control ∂Mz/∂steering — steering×slip grid",
            "test view on the raw simulation grid",
        ),
        "stability_grid": render_derivative_grid(
            data,
            STABILITY_KEY,
            STABILITY_LABEL,
            f"{title_prefix}Stability ∂Mz/∂slip — steering×slip grid",
            "test view on the raw simulation grid",
        ),
    }


def main():
    if len(sys.argv) < 2:
        print("Usage: python3 tools/derivatives.py path/to/yaw_diagram.csv")
        sys.exit(1)
    path = sys.argv[1]
    data = read_csv(path)

    base = os.path.basename(path)
    name = base[:-4] if base.lower().endswith(".csv") else base
    out_dir = os.path.join(os.path.dirname(os.path.abspath(path)), name)

    figures = render_derivative_figures(data)
    save_figures(figures, out_dir)


if __name__ == "__main__":
    main()
