#!/usr/bin/env python3
"""Extract the steady-state trim locus (yaw moment = 0) from a yaw moment diagram
and plot the control/response relationships along it.

At yaw moment = 0 the car is in a trimmed, balanced cornering state. Along that
locus this tool extracts and plots:

  * chassis slip angle beta   vs steering angle delta
  * steering angle delta      vs lateral acceleration
  * chassis slip angle beta   vs lateral acceleration

Lateral acceleration (the simulator's `latAcc` column) is on the horizontal axis
of the acceleration plots.

Works on two kinds of input:
  * a full yaw moment diagram  -> yaw=0 crossings are interpolated along each
    isoline family;
  * an already-trimmed dataset (the simulator's yawZeroMode output, where every
    row already sits on yaw=0) -> every row is used directly.

Usage:
  python3 tools/plot_yaw_zero_trim.py build/yaw_diagram.csv
"""

import csv
import math
import os
import sys
from collections import defaultdict

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt

STEERING_COLOR = "tab:blue"
LATERAL_COLOR = "tab:purple"


def read_csv(path):
    """Load rows keeping steering, slip, lateral acceleration and yaw moment."""
    data = []
    with open(path, newline="") as csvfile:
        reader = csv.DictReader(csvfile)
        for row in reader:
            lat_acc = float(row["latAcc"])
            yaw = float(row["yawMoment"])
            if not (math.isfinite(lat_acc) and math.isfinite(yaw)):
                continue
            data.append({
                "steering": float(row["steering"]),
                "slip": float(row["slip"]),
                "lateral": lat_acc,
                "yawMoment": yaw,
                "baseSteering": float(row.get("baseSteering", 1.0)) > 0.5,
                "baseSlip": float(row.get("baseSlip", 1.0)) > 0.5,
            })
    return data


def group_isolines(data, key, flag):
    """Group into isolines keyed by `key`, keeping only rows on that family."""
    grouped = defaultdict(list)
    for point in data:
        if point.get(flag, True):
            grouped[point[key]].append(point)
    return grouped


def zero_crossings(points, sort_key, fields):
    """Linearly interpolate `fields` where yawMoment crosses zero along `points`."""
    ordered = sorted(points, key=lambda p: p[sort_key])
    crossings = []
    for a, b in zip(ordered, ordered[1:]):
        ya, yb = a["yawMoment"], b["yawMoment"]
        if ya == 0.0:
            crossings.append({f: a[f] for f in fields})
            continue
        if (ya < 0.0) != (yb < 0.0):  # sign change => bracketed root
            t = -ya / (yb - ya)
            crossings.append({f: a[f] + t * (b[f] - a[f]) for f in fields})
    # capture an exact zero on the final point (loop above skips it)
    if ordered and ordered[-1]["yawMoment"] == 0.0:
        crossings.append({f: ordered[-1][f] for f in fields})
    return crossings


# Full yaw moment diagrams span hundreds-to-thousands of N·m; a dataset already solved
# on the yaw=0 trim locus (the simulator's yawZeroMode) keeps every residual under its
# yawZeroResidualTolerance (default 50), so a cap comfortably above that cleanly tells
# the two apart.
TRIMMED_YAW_THRESHOLD = 100.0


def is_trimmed(data):
    """True if every row already sits on yaw moment = 0 (yawZeroMode output)."""
    return bool(data) and max(abs(p["yawMoment"]) for p in data) < TRIMMED_YAW_THRESHOLD


def trim_locus(data):
    """Return trim points ordered for the vs-steering and vs-slip plots.

    steer_cross: ordered by steering (exact delta on the x-axis)
    slip_cross:  ordered by slip     (exact beta on the x-axis)

    For a full diagram these are yaw=0 crossings interpolated along each isoline
    family; for an already-trimmed dataset every row is used directly.
    """
    fields = ("steering", "slip", "lateral")

    if is_trimmed(data):
        pts = [{f: p[f] for f in fields} for p in data]
        return (sorted(pts, key=lambda p: p["steering"]),
                sorted(pts, key=lambda p: p["slip"]))

    by_steering = group_isolines(data, "steering", "baseSteering")
    steer_cross = []
    for steering in sorted(by_steering):
        steer_cross.extend(zero_crossings(by_steering[steering], "slip", fields))
    steer_cross.sort(key=lambda p: p["steering"])

    by_slip = group_isolines(data, "slip", "baseSlip")
    slip_cross = []
    for slip in sorted(by_slip):
        slip_cross.extend(zero_crossings(by_slip[slip], "steering", fields))
    slip_cross.sort(key=lambda p: p["slip"])

    return steer_cross, slip_cross


def _scatter(ax, points, xkey, ykey, color, label):
    # The yaw=0 trim locus can be multivalued (stable/unstable branches at the
    # same input), so plot equilibrium points as a scatter rather than a
    # connected line, which would draw misleading vertical jumps between branches.
    xs = [p[xkey] for p in points]
    ys = [p[ykey] for p in points]
    ax.scatter(xs, ys, color=color, s=16, label=label)


def render_slip_vs_steering(steer_cross):
    fig, ax = plt.subplots(figsize=(9, 7))
    _scatter(ax, steer_cross, "steering", "slip", STEERING_COLOR, "trim (yaw=0)")
    ax.axhline(0.0, color="0.7", linewidth=0.8, zorder=0)
    ax.axvline(0.0, color="0.7", linewidth=0.8, zorder=0)
    ax.set_xlabel("Steering angle δ [deg]")
    ax.set_ylabel("Chassis slip angle β [deg]")
    ax.set_title("Chassis slip vs steering at yaw moment = 0")
    ax.grid(True, alpha=0.3)
    ax.legend(loc="best")
    fig.tight_layout()
    return fig


def render_vs_lateral(cross, ykey, ylabel, title):
    fig, ax = plt.subplots(figsize=(9, 7))
    _scatter(ax, cross, "lateral", ykey, LATERAL_COLOR, "trim (yaw=0)")
    ax.axhline(0.0, color="0.7", linewidth=0.8, zorder=0)
    ax.axvline(0.0, color="0.7", linewidth=0.8, zorder=0)
    ax.set_xlabel("Lateral acceleration [m/s²]")
    ax.set_ylabel(ylabel)
    ax.set_title(title)
    ax.grid(True, alpha=0.3)
    ax.legend(loc="best")
    fig.tight_layout()
    return fig


def write_locus_csv(steer_cross, slip_cross, out_path):
    with open(out_path, "w", newline="") as f:
        w = csv.writer(f)
        w.writerow(["family", "steering", "slip", "lateral"])
        for p in steer_cross:
            w.writerow(["steering", p["steering"], p["slip"], p["lateral"]])
        for p in slip_cross:
            w.writerow(["slip", p["steering"], p["slip"], p["lateral"]])
    print(f"Wrote trim locus to {out_path}")


def main():
    if len(sys.argv) < 2:
        print("Usage: python3 tools/plot_yaw_zero_trim.py path/to/yaw_diagram.csv")
        sys.exit(1)
    path = sys.argv[1]
    data = read_csv(path)

    print(f"Input {'already trimmed (yaw=0)' if is_trimmed(data) else 'full diagram'}; "
          f"{len(data)} rows")
    steer_cross, slip_cross = trim_locus(data)
    if not steer_cross and not slip_cross:
        print("No yaw moment = 0 crossings found in the data.", file=sys.stderr)
        sys.exit(1)

    base = os.path.basename(path)
    name = base[:-4] if base.lower().endswith(".csv") else base
    out_dir = os.path.join(os.path.dirname(os.path.abspath(path)), f"{name}_yaw0")
    os.makedirs(out_dir, exist_ok=True)

    figures = {
        "slip_vs_steering": render_slip_vs_steering(steer_cross),
        "steering_vs_lateral": render_vs_lateral(
            steer_cross, "steering", "Steering angle δ [deg]",
            "Steering vs lateral acceleration at yaw moment = 0"),
        "slip_vs_lateral": render_vs_lateral(
            slip_cross, "slip", "Chassis slip angle β [deg]",
            "Chassis slip vs lateral acceleration at yaw moment = 0"),
    }
    for fig_name, fig in figures.items():
        out_png = os.path.join(out_dir, f"{fig_name}.png")
        fig.savefig(out_png, dpi=120)
        plt.close(fig)
        print(f"Saved plot to {out_png}")

    write_locus_csv(steer_cross, slip_cross, os.path.join(out_dir, "trim_locus.csv"))


if __name__ == "__main__":
    main()
