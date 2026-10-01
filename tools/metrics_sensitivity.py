#!/usr/bin/env python3
"""Charakterystyki czulosci metryk vs baseline + regresje lin/kwadratowe.

Wejscie: metrics_summary.csv z folderu _summary danego runu (kolumna
'setup' + kolumny metryk; wiersz 'base' = konfiguracja bazowa).

Dla kazdej metryki rysuje osobny panel: X = zmiana parametru setupu,
Y = procentowa zmiana metryki vs wiersz 'base'. Dodatkowo dla kazdej
pary (parametr, metryka) dopasowuje regresje liniowa i kwadratowa.

Wyjscie (obok pliku wejsciowego):
  metrics_regression.csv  - a/b(/c), R2 i wzory obu regresji + delta_r2
  metrics_sensitivity.png - panele z danymi i nalozonym fitem kwadratowym

Baseline'y parametrow wykrywane automatycznie ze srodka symetrycznego
sweepu (nie trzeba ich podawac).

Usage: python3 tools/metrics_sensitivity.py path/to/_summary/metrics_summary.csv
"""
import sys
import os
import csv
from collections import defaultdict

import numpy as np
import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt


def classify(label):
    """Zwroc (param, kind, value) albo None dla 'base'/nieznanego.

    kind: 'pct'     -> label typu 'track width +5%' (value = % ze znakiem)
          'massbal' -> 'mass balance = 47% front'   (value = % front)
          'abs'     -> 'cog height = 0.30387'        (value = wartosc bezwzgl.)
    """
    label = label.strip()
    if label == "base":
        return None
    if "=" in label:
        name, val = (s.strip() for s in label.split("=", 1))
        if name == "mass balance":
            return name, "massbal", float(val.split("%")[0].strip())
        return name, "abs", float(val)
    if "%" in label:
        toks = label.replace("%", "").strip().split()
        return " ".join(toks[:-1]), "pct", float(toks[-1])
    return None


def read(path):
    with open(path, newline="") as fh:
        raw = [r for r in csv.DictReader(fh) if r["setup"].strip()]
    metrics = [k for k in raw[0].keys() if k != "setup"]
    # deduplikacja identycznych wierszy (CSV bywa doklejany wielokrotnie)
    seen, rows = set(), []
    for r in raw:
        key = tuple(r[c] for c in raw[0])
        if key not in seen:
            seen.add(key)
            rows.append(r)
    base = next(r for r in rows if r["setup"] == "base")
    return rows, base, metrics


def baselines(rows):
    """Srodek sweepu dla parametrow 'abs' i 'massbal' (sweep symetryczny)."""
    vals = defaultdict(list)
    for r in rows:
        c = classify(r["setup"])
        if c and c[1] in ("abs", "massbal"):
            vals[c[0]].append(c[2])
    return {p: (min(v) + max(v)) / 2 for p, v in vals.items()}


def group(rows, base_of):
    """{param: [(x, row), ...]} gdzie x = zmiana parametru na osi X."""
    g = defaultdict(list)
    for r in rows:
        c = classify(r["setup"])
        if c is None:
            continue
        name, kind, v = c
        if kind == "pct":
            x = v
        elif kind == "massbal":
            x = v - base_of[name]           # punkty procentowe
        else:
            b = base_of[name]
            x = (v - b) / b * 100.0         # % zmiany vs baseline
        g[name].append((x, r))
    for p in g:
        g[p].sort(key=lambda t: t[0])
    return g


def _num(s):
    """float albo None dla pustej/brakujacej komorki."""
    s = (s or "").strip()
    return float(s) if s else None


def series(groups, base, param, metric):
    """(xs, ys) z doklejonym realnym punktem base (0,0).

    Puste tablice, gdy metryka bazowa jest pusta lub zerowa (zmiana procentowa
    nieokreslona); wiersze setupow z pusta metryka sa pomijane."""
    b = _num(base.get(metric))
    if b is None or b == 0.0:
        return np.array([]), np.array([])
    xs, ys = [], []
    for x, r in groups[param]:
        v = _num(r.get(metric))
        if v is None:
            continue
        xs.append(x)
        ys.append((v - b) / b * 100.0)
    xs.append(0.0)
    ys.append(0.0)
    order = np.argsort(xs)
    return np.array(xs)[order], np.array(ys)[order]


def r2(y, y_hat):
    ss_res = float(np.sum((y - y_hat) ** 2))
    ss_tot = float(np.sum((y - np.mean(y)) ** 2))
    if ss_tot == 0.0:
        return 1.0 if ss_res < 1e-12 else 0.0
    return 1.0 - ss_res / ss_tot


def g4(v):
    return f"{v:.4g}"


def write_regression(path, groups, base, metrics):
    with open(path, "w", newline="") as fh:
        w = csv.writer(fh)
        w.writerow([
            "parameter", "metric", "n_points",
            "lin_a", "lin_b", "lin_r2", "lin_formula",
            "quad_a", "quad_b", "quad_c", "quad_r2", "quad_formula",
            "delta_r2",
        ])
        for param in sorted(groups):
            for metric in metrics:
                xs, ys = series(groups, base, param, metric)
                distinct = len(set(np.round(xs, 9))) if len(xs) else 0
                if distinct < 2:
                    # za malo roznych punktow na jakikolwiek sensowny fit
                    w.writerow([param, metric, len(xs), "", "", "", "", "", "", "", "", "", ""])
                    continue
                la, lb = np.polyfit(xs, ys, 1)
                lr2 = r2(ys, la * xs + lb)
                if distinct < 3:
                    # fit liniowy jest dokladny, kwadratowy bylby niedookreslony
                    w.writerow([
                        param, metric, len(xs),
                        f"{la:.6g}", f"{lb:.6g}", f"{lr2:.6f}",
                        f"y = {g4(la)}*x + {g4(lb)}",
                        "", "", "", "", "", "",
                    ])
                    continue
                qa, qb, qc = np.polyfit(xs, ys, 2)
                qr2 = r2(ys, qa * xs ** 2 + qb * xs + qc)
                w.writerow([
                    param, metric, len(xs),
                    f"{la:.6g}", f"{lb:.6g}", f"{lr2:.6f}",
                    f"y = {g4(la)}*x + {g4(lb)}",
                    f"{qa:.6g}", f"{qb:.6g}", f"{qc:.6g}", f"{qr2:.6f}",
                    f"y = {g4(qa)}*x^2 + {g4(qb)}*x + {g4(qc)}",
                    f"{qr2 - lr2:.6f}",
                ])


def plot(path, groups, base, metrics):
    n = len(metrics)
    cols = 3
    rowsn = int(np.ceil(n / cols))
    fig, axes = plt.subplots(rowsn, cols, figsize=(cols * 5, rowsn * 3.4))
    axes = np.array(axes).reshape(-1)
    for ax, metric in zip(axes, metrics):
        for param in sorted(groups):
            xs, ys = series(groups, base, param, metric)
            if len(xs) == 0:
                continue
            line, = ax.plot(xs, ys, marker="o", ms=4, lw=1.2, label=param)
            if len(set(np.round(xs, 9))) >= 3:
                qa, qb, qc = np.polyfit(xs, ys, 2)
                xf = np.linspace(xs.min(), xs.max(), 50)
                ax.plot(xf, qa * xf ** 2 + qb * xf + qc,
                        color=line.get_color(), lw=0.8, ls="--", alpha=0.6)
        ax.axhline(0, color="0.7", lw=0.8, zorder=0)
        ax.axvline(0, color="0.7", lw=0.8, zorder=0)
        ax.set_title(metric, fontsize=8.5)
        ax.set_xlabel("parameter change [% / pp]", fontsize=8)
        ax.set_ylabel("metric change vs base [%]", fontsize=8)
        ax.tick_params(labelsize=7)
        ax.grid(True, alpha=0.25)
    for ax in axes[n:]:
        ax.set_visible(False)
    handles, labels = [], []
    for ax in axes[:n]:
        for handle, label in zip(*ax.get_legend_handles_labels()):
            if label not in labels:
                handles.append(handle)
                labels.append(label)
    if labels:
        fig.legend(handles, labels, loc="lower center", ncol=len(labels),
                   fontsize=8, frameon=False, bbox_to_anchor=(0.5, -0.01))
    fig.suptitle("Metric sensitivity vs baseline "
                 "(points = data, --- = quadratic fit)", fontsize=12)
    fig.tight_layout(rect=(0, 0.03, 1, 0.97))
    fig.savefig(path, dpi=140, bbox_inches="tight")


def main():
    if len(sys.argv) < 2:
        print("Usage: python3 tools/metrics_sensitivity.py "
              "path/to/_summary/metrics_summary.csv")
        sys.exit(1)
    in_path = sys.argv[1]
    out_dir = os.path.dirname(os.path.abspath(in_path))
    fit_path = os.path.join(out_dir, "metrics_regression.csv")
    png_path = os.path.join(out_dir, "metrics_sensitivity.png")

    rows, base, metrics = read(in_path)
    base_of = baselines(rows)
    groups = group(rows, base_of)

    write_regression(fit_path, groups, base, metrics)
    plot(png_path, groups, base, metrics)

    print(f"baseline'y (auto): "
          + ", ".join(f"{p}={b:g}" for p, b in sorted(base_of.items())))
    print(f"zapisano regresje: {fit_path}")
    print(f"zapisano wykres:   {png_path}")


if __name__ == "__main__":
    main()
