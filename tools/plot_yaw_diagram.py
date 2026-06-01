#!/usr/bin/env python3

import sys
import csv
import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
from collections import defaultdict


STEERING_COLOR = "tab:blue"
SLIP_COLOR = "tab:red"


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


def plot_steering_isolines(ax, by_steering, color):
    for steering_angle in sorted(by_steering.keys()):
        points = sorted(by_steering[steering_angle], key=lambda p: p["slip"])
        x = [p["latAcc"] for p in points]
        y = [p["yawMoment"] for p in points]
        ax.plot(x, y, color=color, alpha=0.6, linewidth=0.8)


def plot_slip_isolines(ax, by_slip, color):
    for slip_angle in sorted(by_slip.keys()):
        points = sorted(by_slip[slip_angle], key=lambda p: p["steering"])
        x = [p["latAcc"] for p in points]
        y = [p["yawMoment"] for p in points]
        ax.plot(x, y, color=color, alpha=0.6, linewidth=0.8)


def finalize(ax, data, title):
    x_all = [p["latAcc"] for p in data]
    y_all = [p["yawMoment"] for p in data]
    ax.scatter(x_all, y_all, s=6, color="royalblue", alpha=0.6, zorder=5)
    ax.set_xlabel("Lateral acceleration")
    ax.set_ylabel("Yaw moment")
    ax.set_title(title)
    ax.grid(True, alpha=0.3)
    ax.legend(loc="best")


def save(fig, base_path, suffix):
    out_png = f"{base_path}_{suffix}.png"
    fig.tight_layout()
    fig.savefig(out_png)
    print(f"Saved plot to {out_png}")


def main():
    if len(sys.argv) < 2:
        print("Usage: python3 tools/plot_yaw_diagram.py path/to/yaw_diagram.csv")
        sys.exit(1)
    path = sys.argv[1]
    data = read_csv(path)

    by_steering = defaultdict(list)
    by_slip = defaultdict(list)
    for point in data:
        by_steering[point["steering"]].append(point)
        by_slip[point["slip"]].append(point)

    base_path = path[:-4] if path.lower().endswith(".csv") else path

    fig1, ax1 = plt.subplots(figsize=(10, 8))
    plot_steering_isolines(ax1, by_steering, STEERING_COLOR)
    plot_slip_isolines(ax1, by_slip, SLIP_COLOR)
    ax1.plot([], [], color=STEERING_COLOR, label="constant steering")
    ax1.plot([], [], color=SLIP_COLOR, label="constant chassis slip")
    finalize(ax1, data, "Yaw moment diagram")
    save(fig1, base_path, "combined")

    fig2, ax2 = plt.subplots(figsize=(10, 8))
    plot_steering_isolines(ax2, by_steering, STEERING_COLOR)
    ax2.plot([], [], color=STEERING_COLOR, label="constant steering")
    finalize(ax2, data, "Yaw moment diagram — steering isolines")
    save(fig2, base_path, "steering")

    fig3, ax3 = plt.subplots(figsize=(10, 8))
    plot_slip_isolines(ax3, by_slip, SLIP_COLOR)
    ax3.plot([], [], color=SLIP_COLOR, label="constant chassis slip")
    finalize(ax3, data, "Yaw moment diagram — chassis slip isolines")
    save(fig3, base_path, "slip")


if __name__ == "__main__":
    main()
