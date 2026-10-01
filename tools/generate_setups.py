#!/usr/bin/env python3

import argparse
import csv
import os
import shutil
import subprocess
import sys
import time

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
import matplotlib.image as mpimg
from matplotlib.colors import Normalize

from plot_yaw_diagram import read_csv, render_isoline_figures, save_figures
from diagram_metrics import (
    read_metrics_csv,
    read_hull_csv,
    render_metrics_figure,
    write_metrics_summary_csv,
    render_metrics_summary_figures,
)
from derivatives import (
    render_derivative_figures,
    render_diff_figures,
    write_enriched_csv,
    compute_derivatives,
    build_diff_data,
    draw_field,
    draw_grid,
    robust_limit,
    CONTROL_KEY,
    STABILITY_KEY,
    CONTROL_DIFF_KEY,
    STABILITY_DIFF_KEY,
)


ISOLINE_MONTAGE_TYPES = ["steering", "slip", "combined", "combined_zoom"]

# plot_type -> (kind, value_key, needs_baseline_diff, colorbar_label)
HEATMAP_MONTAGE_SPEC = {
    "control_heatmap": ("field", CONTROL_KEY, False, "Control  ∂Mz/∂steering [N·m/°]"),
    "stability_heatmap": ("field", STABILITY_KEY, False, "Stability  ∂Mz/∂slip [N·m/°]"),
    "control_grid": ("grid", CONTROL_KEY, False, "Control  ∂Mz/∂steering [N·m/°]"),
    "stability_grid": ("grid", STABILITY_KEY, False, "Stability  ∂Mz/∂slip [N·m/°]"),
    "control_diff": ("field", CONTROL_DIFF_KEY, True, "Δ Control  ∂Mz/∂steering [N·m/°]"),
    "stability_diff": ("field", STABILITY_DIFF_KEY, True, "Δ Stability  ∂Mz/∂slip [N·m/°]"),
    "control_diff_grid": ("grid", CONTROL_DIFF_KEY, True, "Δ Control  ∂Mz/∂steering [N·m/°]"),
    "stability_diff_grid": ("grid", STABILITY_DIFF_KEY, True, "Δ Stability  ∂Mz/∂slip [N·m/°]"),
}


TOE_KEYS = {
    "front": ["Vehicle.toeAngle.FL", "Vehicle.toeAngle.FR"],
    "rear": ["Vehicle.toeAngle.RL", "Vehicle.toeAngle.RR"],
}

SUSPENDED_MASS_KEYS = {
    "front": ["Vehicle.suspendedMassAtWheels.FL", "Vehicle.suspendedMassAtWheels.FR"],
    "rear": ["Vehicle.suspendedMassAtWheels.RL", "Vehicle.suspendedMassAtWheels.RR"],
}

DIRECTION_SIGNS = {
    "plus": (1.0,),
    "minus": (-1.0,),
    "both": (-1.0, 1.0),
}

MONTAGE_TYPES = ISOLINE_MONTAGE_TYPES + list(HEATMAP_MONTAGE_SPEC)


def print_setup_progress(done, total):
    width = 30
    filled = int(width * done / total) if total else width
    bar = "#" * filled + "-" * (width - filled)
    pct = 100 * done / total if total else 100
    sys.stderr.write(f"\r[setups]       [{bar}] {pct:3.0f}% ({done}/{total})")
    if done >= total:
        sys.stderr.write("\n")
    sys.stderr.flush()


def read_config_lines(path):
    with open(path, newline="") as f:
        return f.readlines()


def apply_overrides(lines, overrides):
    result = []
    matched = set()
    for line in lines:
        stripped = line.rstrip("\n")
        parts = stripped.split(",")
        if len(parts) >= 3:
            key = f"{parts[0]}.{parts[1]}"
            if key in overrides:
                parts[2] = f"{overrides[key]:g}"
                result.append(",".join(parts) + "\n")
                matched.add(key)
                continue
        result.append(line)
    missing = set(overrides) - matched
    if missing:
        raise KeyError(f"override keys not found in config: {sorted(missing)}")
    return result


def format_delta(value):
    return f"{'+' if value >= 0 else '-'}{abs(value):g}"


def config_value(config_path, key):
    module, param = key.split(".", 1)
    for row in csv.reader(open(config_path, newline="")):
        if len(row) >= 3 and row[0] == module and row[1] == param:
            return float(row[2])
    raise ValueError(f"parameter '{key}' not found in config")


def list_axle_pairs(config_path):
    fronts, rears = set(), set()
    for row in csv.reader(open(config_path, newline="")):
        if len(row) < 2 or row[0] != "Vehicle":
            continue
        param = row[1]
        if "." in param:
            continue
        if param.startswith("front"):
            fronts.add(param[len("front"):])
        elif param.startswith("rear"):
            rears.add(param[len("rear"):])
    return sorted(fronts & rears)


def axle_key_groups(param):
    if param == "toe":
        return TOE_KEYS["front"], TOE_KEYS["rear"]
    return [f"Vehicle.front{param}"], [f"Vehicle.rear{param}"]


def sweep_mode_levels(args):
    if args.values:
        return "abs", [float(v) for v in args.values.split(",")]
    if args.percent:
        return "percent", [float(p) for p in args.percent.split(",")]
    if args.delta is not None:
        return "delta", [sign * args.delta for sign in DIRECTION_SIGNS[args.direction]]
    return "delta", []


def level_value(base, mode, level):
    if mode == "abs":
        return level
    if mode == "percent":
        return base * (1.0 + level / 100.0)
    return base + level


def base_coord(base, mode):
    return base if mode == "abs" else 0.0


def format_level(coord, mode):
    if mode == "percent":
        return f"{coord:+g}%"
    if mode == "delta":
        return format_delta(coord)
    return f"{coord:g}"


def level_label(position, mode):
    return "base" if position["is_base"] else format_level(position["coord"], mode)


def axle_axis(keys, base, mode, levels, active):
    positions = [{"coord": base_coord(base, mode), "overrides": {}, "is_base": True}]
    if not active:
        return positions
    seen = {positions[0]["coord"]}
    for level in levels:
        value = level_value(base, mode, level)
        if abs(value - base) < 1e-12 or level in seen:
            continue
        seen.add(level)
        positions.append({"coord": level, "overrides": {k: value for k in keys}, "is_base": False})
    return positions


def build_axle_matrix_setups(front_keys, rear_keys, base_front, base_rear, mode, levels, axle):
    front_axis = axle_axis(front_keys, base_front, mode, levels, axle in ("front", "both"))
    rear_axis = axle_axis(rear_keys, base_rear, mode, levels, axle in ("rear", "both"))
    setups = []
    index = 0
    for rear_pos in rear_axis:
        for front_pos in front_axis:
            overrides = dict(front_pos["overrides"])
            overrides.update(rear_pos["overrides"])
            is_base = front_pos["is_base"] and rear_pos["is_base"]
            tags = []
            if not front_pos["is_base"]:
                tags.append(f"f{format_level(front_pos['coord'], mode)}")
            if not rear_pos["is_base"]:
                tags.append(f"r{format_level(rear_pos['coord'], mode)}")
            slug = "base" if is_base else "_".join(tags).replace("%", "pct")
            label = "base" if is_base else \
                f"F {level_label(front_pos, mode)} / R {level_label(rear_pos, mode)}"
            setups.append(
                {
                    "name": f"setup_{index:02d}_{slug}",
                    "label": label,
                    "col": front_pos["coord"],
                    "row": rear_pos["coord"],
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


def sweep_values(base_value, args):
    if args.values:
        return [float(v) for v in args.values.split(",")]
    if args.percent:
        return [base_value * (1.0 + float(p) / 100.0) for p in args.percent.split(",")]
    if args.delta is not None:
        return [base_value + sign * args.delta for sign in DIRECTION_SIGNS[args.direction]]
    return []


def build_config_setups(config_paths):
    setups = []
    for index, path in enumerate(config_paths):
        name = os.path.splitext(os.path.basename(path))[0]
        is_base = index == 0
        setups.append(
            {
                "name": f"setup_{index:02d}_{'base_' + name if is_base else name}",
                "label": (f"baseline: {name}" if is_base else name),
                "col": float(index),
                "row": 0.0,
                "is_base": is_base,
                "overrides": {},
                "config_file": os.path.abspath(path.strip()),
            }
        )
    return setups, "config", None


def build_all_setups(args, config_path):
    mode, levels = sweep_mode_levels(args)
    if args.param == "toe" or "." not in args.param:
        front_keys, rear_keys = axle_key_groups(args.param)
        setups = build_axle_matrix_setups(
            front_keys,
            rear_keys,
            config_value(config_path, front_keys[0]),
            config_value(config_path, rear_keys[0]),
            mode,
            levels,
            args.axle,
        )
        return setups, f"front {args.param}", f"rear {args.param}"
    base_value = config_value(config_path, args.param)
    setups = build_generic_setups(args.param, base_value, sweep_values(base_value, args))
    return setups, args.param, None


def read_oat_spec(path):
    knobs = []
    for row in csv.DictReader(open(path, newline="")):
        param = (row.get("param") or "").strip()
        keys = (row.get("keys") or "").strip()
        if not param and not keys:
            continue
        knobs.append(
            {
                "param": param,
                "keys": keys,
                "keys_down": (row.get("keys_down") or "").strip(),
                "label": (row.get("label") or "").strip(),
                "values": (row.get("values") or "").strip(),
                "percent": (row.get("percent") or "").strip(),
                "delta": (row.get("delta") or "").strip(),
                "direction": (row.get("direction") or "both").strip() or "both",
                "axle": (row.get("axle") or "both").strip() or "both",
            }
        )
    return knobs


def knob_mode_levels(knob):
    if knob["values"]:
        return "abs", [float(v) for v in knob["values"].split(",")]
    if knob["percent"]:
        return "percent", [float(p) for p in knob["percent"].split(",")]
    if knob["delta"]:
        delta = float(knob["delta"])
        return "delta", [sign * delta for sign in DIRECTION_SIGNS[knob["direction"]]]
    return "delta", []


def exact_knob_variations(param, mode, levels, config_path, label=None):
    base = config_value(config_path, param)
    name = label or param
    variations = []
    seen = set()
    for level in levels:
        value = level_value(base, mode, level)
        if abs(value - base) < 1e-12 or value in seen:
            continue
        seen.add(value)
        variations.append(
            {
                "slug": f"{slugify(name)}_{level_slug(level, mode)}",
                "label": f"{name} = {value:g}",
                "overrides": {param: value},
            }
        )
    return variations


def pair_knob_variations(param, axle, mode, levels, config_path, label=None):
    front_keys, rear_keys = axle_key_groups(param)
    name = label or param
    variations = []
    for side, keys in (("front", front_keys), ("rear", rear_keys)):
        if axle not in (side, "both"):
            continue
        base = config_value(config_path, keys[0])
        seen = set()
        for level in levels:
            value = level_value(base, mode, level)
            if abs(value - base) < 1e-12 or value in seen:
                continue
            seen.add(value)
            variations.append(
                {
                    "slug": f"{slugify(name)}_{side[0]}_{level_slug(level, mode)}",
                    "label": f"{name} {side} = {value:g}",
                    "overrides": {k: value for k in keys},
                }
            )
    return variations


def slugify(text):
    out = "".join(c if c.isalnum() else "_" for c in text)
    while "__" in out:
        out = out.replace("__", "_")
    return out.strip("_")


def level_slug(level, mode):
    magnitude = f"{abs(level):g}".replace(".", "_")
    return f"{'m' if level < 0 else 'p'}{magnitude}{'pct' if mode == 'percent' else ''}"


def group_knob_variations(knob, mode, levels, config_path):
    up_keys = [key.strip() for key in knob["keys"].split(";") if key.strip()]
    down_keys = [key.strip() for key in knob["keys_down"].split(";") if key.strip()]
    bases = {key: config_value(config_path, key) for key in up_keys + down_keys}
    label = knob["label"] or up_keys[0]
    reference = bases[up_keys[0]]
    variations = []
    seen = set()
    for level in levels:
        if abs(level_value(reference, mode, level) - reference) < 1e-12 or level in seen:
            continue
        seen.add(level)
        overrides = {key: level_value(bases[key], mode, level) for key in up_keys}
        for key in down_keys:
            if mode == "abs":
                # mirror the up-side delta around this key's own base (there is no
                # meaningful "opposite" of a raw absolute target otherwise)
                overrides[key] = bases[key] - (level - reference)
            else:
                overrides[key] = level_value(bases[key], mode, -level)
        tag = format_level(level, mode)
        variations.append(
            {
                "slug": f"{slugify(label)}_{level_slug(level, mode)}",
                "label": f"{label} {tag}",
                "overrides": overrides,
            }
        )
    return variations


def balance_target(base_percent, mode, level):
    return level if mode == "abs" else base_percent + level


def balance_knob_variations(knob, mode, levels, config_path):
    front_keys = SUSPENDED_MASS_KEYS["front"]
    rear_keys = SUSPENDED_MASS_KEYS["rear"]
    front_mass = {key: config_value(config_path, key) for key in front_keys}
    rear_mass = {key: config_value(config_path, key) for key in rear_keys}
    front_total = sum(front_mass.values())
    rear_total = sum(rear_mass.values())
    total = front_total + rear_total
    base_front_percent = 100.0 * front_total / total
    name = knob["label"] or "mass balance"
    variations = []
    seen = set()
    for level in levels:
        target = balance_target(base_front_percent, mode, level)
        if abs(target - base_front_percent) < 1e-9 or round(target, 6) in seen:
            continue
        seen.add(round(target, 6))
        new_front_total = total * target / 100.0
        new_rear_total = total - new_front_total
        overrides = {key: new_front_total * mass / front_total for key, mass in front_mass.items()}
        overrides.update({key: new_rear_total * mass / rear_total for key, mass in rear_mass.items()})
        variations.append(
            {
                "slug": f"{slugify(name)}_{level_slug(level, mode)}",
                "label": f"{name} = {target:.1f}% front",
                "overrides": overrides,
            }
        )
    return variations


def knob_variations(knob, config_path):
    mode, levels = knob_mode_levels(knob)
    if knob["keys"]:
        return group_knob_variations(knob, mode, levels, config_path)
    param = knob["param"]
    label = knob["label"] or None
    if param == "massBalance":
        return balance_knob_variations(knob, mode, levels, config_path)
    if param == "toe" or "." not in param:
        return pair_knob_variations(param, knob["axle"], mode, levels, config_path, label)
    return exact_knob_variations(param, mode, levels, config_path, label)


def build_oat_setups(spec_path, config_path):
    variations = []
    for knob in read_oat_spec(spec_path):
        variations.extend(knob_variations(knob, config_path))
    ncols = max(1, int(round((len(variations) + 1) ** 0.5)))
    setups = [
        {
            "name": "setup_00_base",
            "label": "base",
            "col": 0.0,
            "row": 0.0,
            "is_base": True,
            "overrides": {},
        }
    ]
    for offset, variation in enumerate(variations):
        index = offset + 1
        setups.append(
            {
                "name": f"setup_{index:02d}_{variation['slug']}",
                "label": variation["label"],
                "col": float(index % ncols),
                "row": float(-(index // ncols)),
                "is_base": False,
                "overrides": variation["overrides"],
            }
        )
    return setups, "one-at-a-time", None


def next_run_dir(results_dir):
    os.makedirs(results_dir, exist_ok=True)
    existing = [d for d in os.listdir(results_dir) if d.startswith("run_")]
    numbers = [int(d[4:]) for d in existing if d[4:].isdigit()]
    run_id = (max(numbers) + 1) if numbers else 1
    return os.path.join(results_dir, f"run_{run_id:03d}"), run_id


def run_simulation(binary, config_path, repo_root):
    build_dir = os.path.join(repo_root, "build")
    outputs = [os.path.join(build_dir, name)
               for name in ("yaw_diagram.csv", "metrics.csv", "hull.csv")]
    # clear stale outputs first so a failed write can't silently reuse a prior
    # setup's metrics/hull, and verify all three afterwards (an old binary that
    # predates diagramMetrics fails fast with a clear message)
    for path in outputs:
        if os.path.exists(path):
            os.remove(path)
    subprocess.run([binary, config_path], cwd=repo_root, check=True, stdout=subprocess.DEVNULL)
    for path in outputs:
        if not os.path.exists(path):
            raise RuntimeError(f"simulator did not produce {path}")
    return outputs[0]


def render_setup(setup_dir, csv_path, title_prefix, base_data=None):
    data = read_csv(csv_path)
    isolines = render_isoline_figures(data, title_prefix)
    save_figures(isolines, setup_dir)
    derivatives = render_derivative_figures(data, title_prefix)
    save_figures(derivatives, setup_dir)
    write_enriched_csv(csv_path, data)
    metrics = read_metrics_csv(os.path.join(setup_dir, "metrics.csv"))
    hull = read_hull_csv(os.path.join(setup_dir, "hull.csv"))
    save_figures({"metrics": render_metrics_figure(data, metrics, hull, title_prefix)}, setup_dir)
    if base_data is not None:
        diffs = render_diff_figures(data, base_data, title_prefix)
        save_figures(diffs, setup_dir)
    return data, metrics, hull


def montage_axes_levels(setups):
    col_levels = sorted({s["col"] for s in setups})
    row_levels = sorted({s["row"] for s in setups})
    return col_levels, row_levels


def load_setup_data(run_dir, setup):
    data = read_csv(os.path.join(run_dir, setup["name"], "yaw_diagram.csv"))
    compute_derivatives(data)
    return data


def montage_cell_data(setups, run_dir, needs_diff):
    base_data = load_setup_data(run_dir, next(s for s in setups if s["is_base"])) if needs_diff else None
    cells = {}
    for setup in setups:
        if needs_diff and setup["is_base"]:
            continue
        data = load_setup_data(run_dir, setup)
        cells[(setup["col"], setup["row"])] = build_diff_data(data, base_data) if needs_diff else data
    return cells


def build_montage(setups, run_dir, summary_dir, plot_type, col_label, row_label):
    col_levels, row_levels = montage_axes_levels(setups)
    lookup = {(s["col"], s["row"]): s for s in setups}
    rows = list(reversed(row_levels))
    ncols = len(col_levels)
    nrows = len(rows)
    fig, axes = plt.subplots(nrows, ncols, figsize=(5.5 * ncols, 4.6 * nrows), squeeze=False,
                             constrained_layout=True)

    spec = HEATMAP_MONTAGE_SPEC.get(plot_type)
    cells = {}
    norm = None
    if spec:
        _, value_key, needs_diff, _ = spec
        cells = montage_cell_data(setups, run_dir, needs_diff)
        all_values = [p[value_key] for data in cells.values() for p in data]
        limit = robust_limit(all_values) if all_values else 1.0
        norm = Normalize(vmin=-limit, vmax=limit)

    mappable = None
    for r, row_value in enumerate(rows):
        for c, col_value in enumerate(col_levels):
            ax = axes[r][c]
            ax.axis("off")
            setup = lookup.get((col_value, row_value))
            if setup is None:
                continue
            if spec:
                data = cells.get((col_value, row_value))
                if data is not None:
                    kind, value_key = spec[0], spec[1]
                    mappable = draw_field(ax, data, value_key, norm) if kind == "field" \
                        else draw_grid(ax, data, value_key, norm)
            else:
                image_path = os.path.join(run_dir, setup["name"], f"{plot_type}.png")
                if os.path.exists(image_path):
                    ax.imshow(mpimg.imread(image_path))
            ax.set_title(setup["label"], fontsize=10)

    if spec and mappable is not None:
        cbar = fig.colorbar(mappable, ax=axes.ravel().tolist(), fraction=0.02, pad=0.02)
        cbar.set_label(spec[3])

    axes_note = f"  (columns: {col_label}, rows: {row_label})" if row_label else f"  (swept: {col_label})"
    fig.suptitle(f"All setups — {plot_type}{axes_note}", fontsize=14)
    out_png = os.path.join(summary_dir, f"all_{plot_type}.png")
    fig.savefig(out_png, dpi=110)
    plt.close(fig)
    print(f"Saved montage to {out_png}")


SETUP_EXAMPLES = """
how to control the sweep (with 'make setups SETUP_ARGS=\"...\"'):

  what to sweep (choose the --param form):
    --param toe            front-by-rear symmetric toe matrix
    --param Karb           a front/rear PAIR: expands to Vehicle.frontKarb and
                           Vehicle.rearKarb, swept as a front-by-rear matrix
                           (works for any front<X>/rear<X> pair: Karb, Kspring, ...)
                           use --axle to sweep only one side
    --param Vehicle.suspendedMassHeight
                           one exact config key, swept on its own (--axle ignored)
    --configs a,b,c        use whole config files as points (first is baseline)
    --spec sweep.csv       one-at-a-time (OAT) study: each row is one param swept
                           alone from baseline, no cross-product between rows

  how to build the range:
    --delta 0.2            baseline-0.2, baseline, baseline+0.2
    --delta 0.2 --direction plus    one-sided: baseline and baseline+0.2
    --delta 0.2 --direction minus   one-sided: baseline-0.2 and baseline
    --values 1,1.1,1.2     explicit absolute values (baseline added too)
    --percent -20,-10,10,20   percentage changes vs baseline
    --axle front           for toe or a front/rear pair: change only the front
                           side (or rear / both, default both)

  --spec CSV columns: param, one of values/percent/delta, and optional
    direction (for delta) and axle (for toe or a front/rear pair). Example rows:
      param,values,percent,delta,direction,axle
      toe,\"0.8,-0.8\",,,,front
      Karb,,\"-20,20\",,,rear
      Vehicle.suspendedMassHeight,\"0.25,0.33\",,,,
    Instead of param, a row may set 'keys' (';'-separated exact config keys) to
    change several keys together in one setup, plus 'keys_down' for keys moved by
    the opposite offset (e.g. a front/rear balance) and 'label' for the display
    name. Example rows:
      label,keys,keys_down,percent
      track width,Vehicle.frontTrackWidth;Vehicle.rearTrackWidth,,\"-10,10\"
      mass balance,Vehicle.suspendedMassAtWheels.FL;Vehicle.suspendedMassAtWheels.FR,Vehicle.suspendedMassAtWheels.RL;Vehicle.suspendedMassAtWheels.RR,\"-10,10\"

  param 'massBalance' is a helper that shifts suspended mass front/rear at a
    constant total: values = absolute front share [%], percent/delta = offset in
    percentage-points from the current balance. Example row:
      param,label,percent
      massBalance,mass balance,\"-2,-1,1,2\"

examples:
  make setups
  make setups SETUP_ARGS="--config config_pacejka_v2.csv --param toe --values 0.8,0,-0.8 --axle front"
  make setups SETUP_ARGS="--config config_pacejka_v2.csv --param Karb --percent -20,-10,10,20"
  make setups SETUP_ARGS="--config config_pacejka_v2.csv --param Karb --delta 0.2 --axle rear"
  make setups SETUP_ARGS="--config config_pacejka_v2.csv --param Vehicle.suspendedMassHeight --values 0.25,0.29,0.33"
  make setups SETUP_ARGS="--config config_pacejka_v2.csv --spec sweep.csv"
  make setups SETUP_ARGS="--configs config_pacejka_v2.csv,config_pacejka_v1.csv,config_simple.csv"
"""


def parse_args():
    parser = argparse.ArgumentParser(
        description="Generate a matrix of setup visualizations from a base config.",
        epilog=SETUP_EXAMPLES,
        formatter_class=argparse.RawDescriptionHelpFormatter,
    )
    parser.add_argument("--config", help="base vehicle config CSV (for parameter sweeps)")
    parser.add_argument("--configs", help="comma-separated explicit config CSVs as sweep points; first is the baseline")
    parser.add_argument("--param", default="toe", help="'toe' for the front/rear toe matrix, or a 'Module.param' config key")
    parser.add_argument("--delta", type=float, help="offset the parameter by +/- this amount around baseline (see --direction)")
    parser.add_argument("--values", help="comma-separated absolute values of the parameter to test vs baseline")
    parser.add_argument("--percent", help="comma-separated percentage changes vs baseline, e.g. '-10,-20'")
    parser.add_argument("--direction", default="both", choices=["plus", "minus", "both"],
                        help="for --delta: sweep one side only or both (default both)")
    parser.add_argument("--axle", default="both", choices=["front", "rear", "both"],
                        help="for --param toe or a front/rear pair (e.g. --param Karb): sweep only "
                             "the front or rear side (default both); ignored for an exact key")
    parser.add_argument("--list-axle-params", action="store_true",
                        help="list the front/rear pair params usable as --param X for --config, then exit")
    parser.add_argument("--spec", help="CSV one-at-a-time spec (columns: param, one of values/percent/delta, "
                                       "optional direction/axle); each row is swept alone from baseline")
    parser.add_argument("--binary", default="build/laptime_simulator", help="simulator binary path")
    parser.add_argument("--results-dir", default="results", help="root output directory")
    return parser.parse_args()


def main():
    args = parse_args()
    if args.list_axle_params:
        config_path = os.path.abspath(args.config or "config_pacejka_v2.csv")
        print(f"front/rear pair params in {os.path.basename(config_path)} "
              f"(use as: --param X [--axle front|rear|both]):")
        for pair in list_axle_pairs(config_path):
            print(f"  {pair}")
        return
    repo_root = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
    binary = os.path.join(repo_root, args.binary) if not os.path.isabs(args.binary) else args.binary
    if not os.path.exists(binary):
        sys.exit(f"Simulator binary not found: {binary} (run 'make' first)")
    if not args.config and not args.configs:
        sys.exit("Provide --config (parameter sweep) or --configs (explicit config points)")
    if args.spec and not args.config:
        sys.exit("--spec (one-at-a-time sweep) needs --config for baseline values")

    base_lines = None
    config_name = ""
    if args.configs:
        setups, col_label, row_label = build_config_setups(args.configs.split(","))
    else:
        config_path = os.path.abspath(args.config)
        base_lines = read_config_lines(config_path)
        config_name = os.path.splitext(os.path.basename(config_path))[0]
        if args.spec:
            setups, col_label, row_label = build_oat_setups(os.path.abspath(args.spec), config_path)
        else:
            setups, col_label, row_label = build_all_setups(args, config_path)
    single = len(setups) == 1
    if single:
        setups[0]["name"] = ""
        setups[0]["label"] = config_name

    results_dir = os.path.join(repo_root, args.results_dir)
    run_dir, run_id = next_run_dir(results_dir)
    os.makedirs(run_dir, exist_ok=True)
    run_start = time.perf_counter()
    print(f"Run {run_id:03d} → {run_dir} ({len(setups)} setup{'s' if not single else ''})")

    setup_metrics = {}

    def process_setup(setup, base_data):
        setup_dir = run_dir if single else os.path.join(run_dir, setup["name"])
        os.makedirs(setup_dir, exist_ok=True)
        setup_config = os.path.join(setup_dir, "config.csv")
        if setup.get("config_file"):
            shutil.copyfile(setup["config_file"], setup_config)
        else:
            with open(setup_config, "w", newline="") as f:
                f.writelines(apply_overrides(base_lines, setup["overrides"]))
        sim_start = time.perf_counter()
        produced_csv = run_simulation(binary, setup_config, repo_root)
        sim_seconds = time.perf_counter() - sim_start
        setup_csv = os.path.join(setup_dir, "yaw_diagram.csv")
        shutil.copyfile(produced_csv, setup_csv)
        build_dir = os.path.dirname(produced_csv)
        for name in ("metrics.csv", "hull.csv"):
            shutil.copyfile(os.path.join(build_dir, name), os.path.join(setup_dir, name))
        render_start = time.perf_counter()
        data, metrics, hull = render_setup(setup_dir, setup_csv, f"{setup['label']} — ", base_data)
        setup_metrics[setup["name"]] = {"label": setup["label"], "metrics": metrics, "hull": hull}
        render_seconds = time.perf_counter() - render_start
        print(f"  {setup['name'] or config_name} done  (sim {sim_seconds:.1f}s, render {render_seconds:.1f}s)")
        return data

    total = len(setups)
    done = 0
    base_setup = next(s for s in setups if s["is_base"])
    base_data = process_setup(base_setup, None)
    done += 1
    print_setup_progress(done, total)
    base_csv = os.path.join(run_dir if single else os.path.join(run_dir, base_setup["name"]), "yaw_diagram.csv")
    for setup in setups:
        if setup is base_setup:
            continue
        process_setup(setup, base_data)
        done += 1
        print_setup_progress(done, total)
        shutil.copyfile(base_csv, os.path.join(run_dir, setup["name"], "baseline.csv"))

    if not single:
        summary_dir = os.path.join(run_dir, "_summary")
        os.makedirs(summary_dir, exist_ok=True)
        montage_start = time.perf_counter()
        for plot_type in MONTAGE_TYPES:
            build_montage(setups, run_dir, summary_dir, plot_type, col_label, row_label)
        entries = [setup_metrics[s["name"]] for s in setups if s["name"] in setup_metrics]
        write_metrics_summary_csv(os.path.join(summary_dir, "metrics_summary.csv"), entries)
        save_figures(render_metrics_summary_figures(entries), summary_dir)
        print(f"  montages done  ({time.perf_counter() - montage_start:.1f}s)")

    print(f"Done: {run_dir} (total {time.perf_counter() - run_start:.1f}s)")


if __name__ == "__main__":
    main()
