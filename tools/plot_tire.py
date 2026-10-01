#!/usr/bin/env python3

import csv
import os
import sys
from collections import defaultdict

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt


def read_tire(path):
    sweeps = defaultdict(lambda: defaultdict(list))
    with open(path, newline="") as f:
        for row in csv.DictReader(f):
            sweeps[row["sweep"]][float(row["load"])].append(
                (float(row["input"]), float(row["Fx"]), float(row["Fy"]), float(row["Mz"]))
            )
    return sweeps


def plot_channel(ax, by_load, column, xlabel, ylabel, title):
    for load in sorted(by_load):
        points = sorted(by_load[load])
        ax.plot([p[0] for p in points], [p[column] for p in points], label=f"{load:.0f} N")
    ax.axhline(0, color="black", linewidth=0.6)
    ax.axvline(0, color="black", linewidth=0.6)
    ax.set_xlabel(xlabel)
    ax.set_ylabel(ylabel)
    ax.set_title(title)
    ax.grid(alpha=0.3)
    ax.legend(title="vertical load", fontsize=8)


def render(path):
    sweeps = read_tire(path)
    out_dir = os.path.dirname(os.path.abspath(path))
    name = os.path.splitext(os.path.basename(path))[0]

    fig, axes = plt.subplots(2, 2, figsize=(15, 11))
    plot_channel(axes[0][0], sweeps["slipRatio"], 1, "slip ratio [-]", "Fx [N]",
                 "Longitudinal force vs slip ratio (slip angle 0)")
    plot_channel(axes[0][1], sweeps["slipAngle"], 2, "slip angle [deg]", "Fy [N]",
                 "Lateral force vs slip angle (slip ratio 0)")
    plot_channel(axes[1][0], sweeps["slipAngle"], 3, "slip angle [deg]", "Mz [N·m]",
                 "Self-aligning moment vs slip angle")
    plot_channel(axes[1][1], sweeps["slipRatio"], 2, "slip ratio [-]", "Fy [N]",
                 "Lateral force vs slip ratio (slip angle 0)")
    fig.suptitle(f"Tire model — {name}", fontsize=14)
    fig.tight_layout()
    out_path = os.path.join(out_dir, f"{name}.png")
    fig.savefig(out_path, dpi=120)
    print(f"Saved {out_path}")


if __name__ == "__main__":
    if len(sys.argv) < 2:
        print("Usage: python3 tools/plot_tire.py build/tire_model.csv")
        sys.exit(1)
    render(sys.argv[1])
