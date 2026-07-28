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
from derivatives import render_derivative_figures, render_diff_figures, write_enriched_csv


TOE_WHEELS = {
    "front": (("toeAngle.FL", 1.0), ("toeAngle.FR", -1.0)),
    "rear": (("toeAngle.RL", 1.0), ("toeAngle.RR", -1.0)),
}

DIRECTION_SIGNS = {
    "plus": (1.0,),
    "minus": (-1.0,),
    "both": (-1.0, 1.0),
}

MONTAGE_TYPES = [
    "steering",
    "slip",
    "combined",
    "control_heatmap",
    "stability_heatmap",
    "control_diff",
    "stability_diff",
    "control_diff_grid",
    "stability_diff_grid",
]


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


def format_delta(value):
    return f"{'+' if value >= 0 else '-'}{abs(value):g}"


def axle_toe_overrides(base_toe, axle, delta):
    overrides = {}
    for wheel_param, wheel_sign in TOE_WHEELS[axle]:
        overrides[f"Vehicle.{wheel_param}"] = base_toe.get(wheel_param, 0.0) + wheel_sign * delta
    return overrides


def axle_levels(knobs, axle):
    levels = {0.0}
    for knob in knobs:
        if knob["param"] != "toe":
            raise ValueError(f"unsupported param '{knob['param']}' (only 'toe' for now)")
        if knob["axle"] in (axle, "both"):
            for sign in DIRECTION_SIGNS[knob["direction"]]:
                levels.add(round(sign * knob["delta"], 6))
    return sorted(levels)


def build_setups(knobs, base_toe):
    front_levels = axle_levels(knobs, "front")
    rear_levels = axle_levels(knobs, "rear")
    setups = []
    index = 0
    for rear_delta in rear_levels:
        for front_delta in front_levels:
            overrides = {}
            overrides.update(axle_toe_overrides(base_toe, "front", front_delta))
            overrides.update(axle_toe_overrides(base_toe, "rear", rear_delta))
            tags = []
            if front_delta != 0.0:
                tags.append(f"f{format_delta(front_delta)}")
            if rear_delta != 0.0:
                tags.append(f"r{format_delta(rear_delta)}")
            slug = "base" if not tags else "_".join(tags)
            label = "base" if not tags else f"F {format_delta(front_delta)} / R {format_delta(rear_delta)}"
            setups.append(
                {
                    "name": f"setup_{index:02d}_{slug}",
                    "label": label,
                    "col": front_delta,
                    "row": rear_delta,
                    "is_base": not tags,
                    "overrides": overrides,
                }
            )
            index += 1
    return setups


def config_value(config_path, key):
    module, param = key.split(".", 1)
    for row in csv.reader(open(config_path, newline="")):
        if len(row) >= 3 and row[0] == module and row[1] == param:
            return float(row[2])
    raise ValueError(f"parameter '{key}' not found in config")


def sweep_values(base_value, args):
    if args.values:
        return [float(v) for v in args.values.split(",")]
    if args.percent:
        return [base_value * (1.0 + float(p) / 100.0) for p in args.percent.split(",")]
    if args.delta is not None:
        return [base_value + sign * args.delta for sign in DIRECTION_SIGNS[args.direction]]
    return []


def apply_offset(base_value, offset, mode):
    return base_value * (1.0 + offset / 100.0) if mode == "percent" else base_value + offset


def format_offset(offset, mode):
    return f"{offset:+g}%" if mode == "percent" else format_delta(offset)


def pair_levels(args):
    if args.percent:
        offsets = {float(p) for p in args.percent.split(",")}
        return sorted({0.0} | offsets), "percent"
    if args.delta is not None:
        offsets = {sign * args.delta for sign in DIRECTION_SIGNS[args.direction]}
        return sorted({0.0} | offsets), "delta"
    return [0.0], "delta"


def build_pair_setups(front_key, rear_key, base_front, base_rear, levels, mode, axle):
    front_levels = levels if axle in ("front", "both") else [0.0]
    rear_levels = levels if axle in ("rear", "both") else [0.0]
    setups = []
    index = 0
    for rear_offset in rear_levels:
        for front_offset in front_levels:
            overrides = {}
            if front_offset != 0.0:
                overrides[front_key] = apply_offset(base_front, front_offset, mode)
            if rear_offset != 0.0:
                overrides[rear_key] = apply_offset(base_rear, rear_offset, mode)
            is_base = front_offset == 0.0 and rear_offset == 0.0
            tags = []
            if front_offset != 0.0:
                tags.append(f"f{format_offset(front_offset, mode)}")
            if rear_offset != 0.0:
                tags.append(f"r{format_offset(rear_offset, mode)}")
            slug = "base" if is_base else "_".join(tags).replace("%", "pct")
            label = "base" if is_base else f"F {format_offset(front_offset, mode)} / R {format_offset(rear_offset, mode)}"
            setups.append(
                {
                    "name": f"setup_{index:02d}_{slug}",
                    "label": label,
                    "col": front_offset,
                    "row": rear_offset,
                    "is_base": is_base,
                    "overrides": overrides,
                }
            )
            index += 1
    return setups


def build_generic_setups(param_key, base_value, test_values):
    values = sorted({base_value} | set(test_values))
    setups = []
    for index, value in enumerate(values):
        is_base = abs(value - base_value) < 1e-12
        slug = "base" if is_base else f"{param_key.replace('.', '_')}_{value:g}"
        label = "base" if is_base else f"{param_key} = {value:g}"
        setups.append(
            {
                "name": f"setup_{index:02d}_{slug}",
                "label": label,
                "col": value,
                "row": 0.0,
                "is_base": is_base,
                "overrides": {} if is_base else {param_key: value},
            }
        )
    return setups


def build_all_setups(args, config_path):
    if args.param == "toe":
        setups = build_setups(parse_knobs(args), base_toe_values(config_path))
        return setups, "front toe", "rear toe"
    if "." not in args.param:
        front_key = f"Vehicle.front{args.param}"
        rear_key = f"Vehicle.rear{args.param}"
        levels, mode = pair_levels(args)
        setups = build_pair_setups(
            front_key,
            rear_key,
            config_value(config_path, front_key),
            config_value(config_path, rear_key),
            levels,
            mode,
            args.axle,
        )
        return setups, f"front {args.param}", f"rear {args.param}"
    base_value = config_value(config_path, args.param)
    setups = build_generic_setups(args.param, base_value, sweep_values(base_value, args))
    return setups, args.param, None


def next_run_dir(results_dir):
    os.makedirs(results_dir, exist_ok=True)
    existing = [d for d in os.listdir(results_dir) if d.startswith("run_")]
    numbers = [int(d[4:]) for d in existing if d[4:].isdigit()]
    run_id = (max(numbers) + 1) if numbers else 1
    return os.path.join(results_dir, f"run_{run_id:03d}"), run_id


def run_simulation(binary, config_path, repo_root):
    subprocess.run([binary, config_path], cwd=repo_root, check=True, stdout=subprocess.DEVNULL)
    return os.path.join(repo_root, "build", "yaw_diagram.csv")


def render_setup(setup_dir, csv_path, title_prefix, base_data=None):
    data = read_csv(csv_path)
    isolines = render_isoline_figures(data, title_prefix)
    save_figures(isolines, setup_dir)
    derivatives = render_derivative_figures(data, title_prefix)
    save_figures(derivatives, setup_dir)
    write_enriched_csv(csv_path, data)
    if base_data is not None:
        diffs = render_diff_figures(data, base_data, title_prefix)
        save_figures(diffs, setup_dir)
    return data


def montage_axes_levels(setups):
    col_levels = sorted({s["col"] for s in setups})
    row_levels = sorted({s["row"] for s in setups})
    return col_levels, row_levels


def build_montage(setups, run_dir, summary_dir, plot_type, col_label, row_label):
    col_levels, row_levels = montage_axes_levels(setups)
    lookup = {(s["col"], s["row"]): s for s in setups}
    rows = list(reversed(row_levels))
    ncols = len(col_levels)
    nrows = len(rows)
    fig, axes = plt.subplots(nrows, ncols, figsize=(5.5 * ncols, 4.6 * nrows), squeeze=False)
    for r, row_value in enumerate(rows):
        for c, col_value in enumerate(col_levels):
            ax = axes[r][c]
            ax.axis("off")
            setup = lookup.get((col_value, row_value))
            if setup is None:
                continue
            image_path = os.path.join(run_dir, setup["name"], f"{plot_type}.png")
            if os.path.exists(image_path):
                ax.imshow(mpimg.imread(image_path))
            ax.set_title(setup["label"], fontsize=10)
    axes_note = f"  (columns: {col_label}, rows: {row_label})" if row_label else f"  (swept: {col_label})"
    fig.suptitle(f"All setups — {plot_type}{axes_note}", fontsize=14)
    fig.tight_layout()
    out_png = os.path.join(summary_dir, f"all_{plot_type}.png")
    fig.savefig(out_png, dpi=110)
    plt.close(fig)
    print(f"Saved montage to {out_png}")


def parse_args():
    parser = argparse.ArgumentParser(description="Generate a matrix of setup visualizations from a base config.")
    parser.add_argument("--config", required=True, help="base vehicle config CSV")
    parser.add_argument("--param", default="toe", help="'toe' for the front/rear toe matrix, or a 'Module.param' config key")
    parser.add_argument("--delta", type=float, help="change the parameter by this amount (relative to baseline)")
    parser.add_argument("--values", help="comma-separated absolute values of the parameter to test vs baseline")
    parser.add_argument("--percent", help="comma-separated percentage changes vs baseline, e.g. '-10,-20'")
    parser.add_argument("--direction", default="both", choices=["plus", "minus", "both"])
    parser.add_argument("--axle", default="both", choices=["front", "rear", "both"])
    parser.add_argument("--spec", help="CSV spec file with columns param,delta,direction,axle (toe only)")
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

    base_lines = read_config_lines(config_path)
    config_name = os.path.splitext(os.path.basename(config_path))[0]
    setups, col_label, row_label = build_all_setups(args, config_path)
    single = len(setups) == 1
    if single:
        setups[0]["name"] = ""
        setups[0]["label"] = config_name

    results_dir = os.path.join(repo_root, args.results_dir)
    run_dir, run_id = next_run_dir(results_dir)
    os.makedirs(run_dir, exist_ok=True)
    print(f"Run {run_id:03d} → {run_dir} ({len(setups)} setup{'s' if not single else ''})")

    def process_setup(setup, base_data):
        setup_dir = run_dir if single else os.path.join(run_dir, setup["name"])
        os.makedirs(setup_dir, exist_ok=True)
        setup_config = os.path.join(setup_dir, "config.csv")
        with open(setup_config, "w", newline="") as f:
            f.writelines(apply_overrides(base_lines, setup["overrides"]))
        produced_csv = run_simulation(binary, setup_config, repo_root)
        setup_csv = os.path.join(setup_dir, "yaw_diagram.csv")
        shutil.copyfile(produced_csv, setup_csv)
        data = render_setup(setup_dir, setup_csv, f"{setup['label']} — ", base_data)
        print(f"  {setup['name'] or config_name} done")
        return data

    base_setup = next(s for s in setups if s["is_base"])
    base_data = process_setup(base_setup, None)
    base_csv = os.path.join(run_dir if single else os.path.join(run_dir, base_setup["name"]), "yaw_diagram.csv")
    for setup in setups:
        if setup is base_setup:
            continue
        process_setup(setup, base_data)
        shutil.copyfile(base_csv, os.path.join(run_dir, setup["name"], "baseline.csv"))

    if not single:
        summary_dir = os.path.join(run_dir, "_summary")
        os.makedirs(summary_dir, exist_ok=True)
        for plot_type in MONTAGE_TYPES:
            build_montage(setups, run_dir, summary_dir, plot_type, col_label, row_label)

    print(f"Done: {run_dir}")


if __name__ == "__main__":
    main()
