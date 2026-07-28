#!/usr/bin/env python3

import argparse
import csv
import os
import shutil
import subprocess
import sys

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
import matplotlib.image as mpimg

from plot_yaw_diagram import read_csv, render_isoline_figures, save_figures
from derivatives import render_derivative_figures


TOE_WHEELS = {
    "front": (("toeAngle.FL", 1.0), ("toeAngle.FR", -1.0)),
    "rear": (("toeAngle.RL", 1.0), ("toeAngle.RR", -1.0)),
}

DIRECTION_SIGNS = {
    "plus": (1.0,),
    "minus": (-1.0,),
    "both": (-1.0, 1.0),
}

MONTAGE_TYPES = ["steering", "slip", "combined", "control_heatmap", "stability_heatmap"]


def resolve_axles(axle):
    return ["front", "rear"] if axle == "both" else [axle]


def parse_knobs(args):
    knobs = []
    if args.spec:
        with open(args.spec, newline="") as f:
            for row in csv.DictReader(f):
                knobs.append(
                    {
                        "param": row["param"].strip(),
                        "delta": float(row["delta"]),
                        "direction": row["direction"].strip(),
                        "axle": row["axle"].strip(),
                    }
                )
    if args.delta is not None:
        knobs.append(
            {
                "param": args.param,
                "delta": args.delta,
                "direction": args.direction,
                "axle": args.axle,
            }
        )
    return knobs


def read_config_lines(path):
    with open(path, newline="") as f:
        return f.readlines()


def apply_overrides(lines, overrides):
    result = []
    for line in lines:
        stripped = line.rstrip("\n")
        parts = stripped.split(",")
        if len(parts) >= 3:
            key = f"{parts[0]}.{parts[1]}"
            if key in overrides:
                parts[2] = f"{overrides[key]:g}"
                result.append(",".join(parts) + "\n")
                continue
        result.append(line)
    return result


def base_toe_values(config_path):
    values = {}
    for row in csv.reader(open(config_path, newline="")):
        if len(row) >= 3 and row[0] == "Vehicle" and row[1].startswith("toeAngle."):
            try:
                values[row[1]] = float(row[2])
            except ValueError:
                pass
    return values


def build_setups(knobs, base_toe):
    setups = [{"name": "setup_00_base", "label": "base", "axle": None, "delta": 0.0, "overrides": {}}]
    index = 1
    seen = set()
    for knob in knobs:
        if knob["param"] != "toe":
            raise ValueError(f"unsupported param '{knob['param']}' (only 'toe' for now)")
        for axle in resolve_axles(knob["axle"]):
            for sign in DIRECTION_SIGNS[knob["direction"]]:
                delta = sign * knob["delta"]
                dedup = (axle, round(delta, 6))
                if dedup in seen:
                    continue
                seen.add(dedup)
                overrides = {}
                for wheel_param, wheel_sign in TOE_WHEELS[axle]:
                    overrides[f"Vehicle.{wheel_param}"] = base_toe.get(wheel_param, 0.0) + wheel_sign * delta
                sign_tag = "+" if delta >= 0 else "-"
                setups.append(
                    {
                        "name": f"setup_{index:02d}_toe_{axle}_{sign_tag}{abs(delta):g}",
                        "label": f"toe {axle} {sign_tag}{abs(delta):g}°",
                        "axle": axle,
                        "delta": delta,
                        "overrides": overrides,
                    }
                )
                index += 1
    return setups


def next_run_dir(results_dir):
    os.makedirs(results_dir, exist_ok=True)
    existing = [d for d in os.listdir(results_dir) if d.startswith("run_")]
    numbers = [int(d[4:]) for d in existing if d[4:].isdigit()]
    run_id = (max(numbers) + 1) if numbers else 1
    return os.path.join(results_dir, f"run_{run_id:03d}"), run_id


def run_simulation(binary, config_path, repo_root):
    subprocess.run([binary, config_path], cwd=repo_root, check=True, stdout=subprocess.DEVNULL)
    return os.path.join(repo_root, "build", "yaw_diagram.csv")


def render_setup(setup_dir, csv_path, title_prefix):
    data = read_csv(csv_path)
    isolines = render_isoline_figures(data, title_prefix)
    save_figures(isolines, setup_dir)
    derivatives = render_derivative_figures(data, title_prefix)
    save_figures(derivatives, setup_dir)


def montage_layout(setups):
    axles = sorted({s["axle"] for s in setups if s["axle"]})
    deltas = sorted({s["delta"] for s in setups if s["axle"]} | {0.0})
    base = next(s for s in setups if s["axle"] is None)
    grid = []
    for axle in axles:
        row = []
        for delta in deltas:
            if delta == 0.0:
                row.append(base)
            else:
                row.append(next((s for s in setups if s["axle"] == axle and s["delta"] == delta), None))
        grid.append((axle, row))
    return deltas, grid


def build_montage(setups, run_dir, summary_dir, plot_type):
    deltas, grid = montage_layout(setups)
    if not grid:
        return
    ncols = len(deltas)
    nrows = len(grid)
    fig, axes = plt.subplots(nrows, ncols, figsize=(5.5 * ncols, 4.4 * nrows), squeeze=False)
    for r, (axle, row) in enumerate(grid):
        for c, setup in enumerate(row):
            ax = axes[r][c]
            ax.axis("off")
            if setup is None:
                continue
            image_path = os.path.join(run_dir, setup["name"], f"{plot_type}.png")
            if os.path.exists(image_path):
                ax.imshow(mpimg.imread(image_path))
            ax.set_title(f"{axle} — {setup['label']}", fontsize=10)
    fig.suptitle(f"All setups — {plot_type}", fontsize=14)
    fig.tight_layout()
    out_png = os.path.join(summary_dir, f"all_{plot_type}.png")
    fig.savefig(out_png, dpi=110)
    plt.close(fig)
    print(f"Saved montage to {out_png}")


def parse_args():
    parser = argparse.ArgumentParser(description="Generate a matrix of setup visualizations from a base config.")
    parser.add_argument("--config", required=True, help="base vehicle config CSV")
    parser.add_argument("--param", default="toe", help="parameter knob (only 'toe' supported for now)")
    parser.add_argument("--delta", type=float, help="change amount in the parameter unit (deg for toe)")
    parser.add_argument("--direction", default="both", choices=["plus", "minus", "both"])
    parser.add_argument("--axle", default="both", choices=["front", "rear", "both"])
    parser.add_argument("--spec", help="CSV spec file with columns param,delta,direction,axle")
    parser.add_argument("--binary", default="build/laptime_simulator", help="simulator binary path")
    parser.add_argument("--results-dir", default="results", help="root output directory")
    return parser.parse_args()


def main():
    args = parse_args()
    repo_root = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
    config_path = os.path.abspath(args.config)
    binary = os.path.join(repo_root, args.binary) if not os.path.isabs(args.binary) else args.binary
    if not os.path.exists(binary):
        sys.exit(f"Simulator binary not found: {binary} (run 'make' first)")

    knobs = parse_knobs(args)
    if not knobs:
        sys.exit("No changes requested: pass --delta or --spec")

    base_lines = read_config_lines(config_path)
    setups = build_setups(knobs, base_toe_values(config_path))

    results_dir = os.path.join(repo_root, args.results_dir)
    run_dir, run_id = next_run_dir(results_dir)
    summary_dir = os.path.join(run_dir, "_summary")
    os.makedirs(summary_dir, exist_ok=True)
    print(f"Run {run_id:03d} → {run_dir} ({len(setups)} setups)")

    for setup in setups:
        setup_dir = os.path.join(run_dir, setup["name"])
        os.makedirs(setup_dir, exist_ok=True)
        setup_config = os.path.join(setup_dir, "config.csv")
        with open(setup_config, "w", newline="") as f:
            f.writelines(apply_overrides(base_lines, setup["overrides"]))
        produced_csv = run_simulation(binary, setup_config, repo_root)
        setup_csv = os.path.join(setup_dir, "yaw_diagram.csv")
        shutil.copyfile(produced_csv, setup_csv)
        render_setup(setup_dir, setup_csv, f"{setup['label']} — ")
        print(f"  {setup['name']} done")

    for plot_type in MONTAGE_TYPES:
        build_montage(setups, run_dir, summary_dir, plot_type)

    print(f"Done: {run_dir}")


if __name__ == "__main__":
    main()
