#!/usr/bin/env python3
"""Wykresy slupkowe nachylenia 'a' regresji liniowej per parametr.

Wejscie: metrics_regression.csv (produkt tools/metrics_sensitivity.py).

Domyslnie rysuje dwa panele:
  - max_latacc_overall            (wszystkie parametry)
  - max_moment_at_zero_latacc     (bez 'mass balance' - jego X jest w
                                   pkt proc., nie w %, wiec 'a' nie jest
                                   porownywalne z reszta)

Panele mozna nadpisac z linii polecen, kazdy jako 'metric' lub
'metric:wykluczony parametr[,kolejny]'.

Usage:
  python3 tools/plot_regression_slopes.py path/to/_summary/metrics_regression.csv
  python3 tools/plot_regression_slopes.py .../metrics_regression.csv \
      max_latacc_overall "max_moment_at_zero_latacc:mass balance"
"""
import sys
import os
import csv

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt


DEFAULT_PANELS = [
    ("max_latacc_at_zero_moment", set(),
     "Max lateral acceleration at zero yaw moment"),
    ("max_moment_at_zero_latacc", {"mass balance"},
     "Max yaw moment at zero lateral acceleration"),
]


def parse_panel(spec):
    """'metric[:excl1,excl2][|Full title]' -> (metric, excludes, title)."""
    title = None
    if "|" in spec:
        spec, title = (s.strip() for s in spec.split("|", 1))
    if ":" in spec:
        metric, excl = spec.split(":", 1)
        excludes = {p.strip() for p in excl.split(",") if p.strip()}
    else:
        metric, excludes = spec, set()
    metric = metric.strip()
    return metric, excludes, title or metric


def read_slopes(path):
    slopes = {}
    with open(path, newline="") as fh:
        for r in csv.DictReader(fh):
            a = r["lin_a"].strip()
            if not a:  # degenerate sweep (too few distinct points) -> no slope
                continue
            slopes[(r["metric"], r["parameter"])] = float(a)
    return slopes


def plot(slopes, panels, out_path):
    fig, axes = plt.subplots(1, len(panels), figsize=(6.5 * len(panels), 5))
    if len(panels) == 1:
        axes = [axes]
    for ax, (metric, exclude, title) in zip(axes, panels):
        cand = [p for (m, p) in slopes if m == metric and p not in exclude]
        # od lewej do prawej malejaco wg |wplywu|
        params = sorted(cand, key=lambda p: abs(slopes[(metric, p)]),
                        reverse=True)
        vals = [slopes[(metric, p)] for p in params]
        colors = ["#c0392b" if v < 0 else "#2471a3" for v in vals]
        bars = ax.bar(params, vals, color=colors)
        ax.axhline(0, color="0.4", lw=0.8)
        ax.set_title(title, fontsize=11)
        ax.set_ylabel("% per %", fontsize=9)
        ax.grid(axis="y", alpha=0.25)
        ax.margins(y=0.15)
        for b, v in zip(bars, vals):
            ax.annotate(f"{v:.3g}", (b.get_x() + b.get_width() / 2, v),
                        ha="center", va="bottom" if v >= 0 else "top",
                        fontsize=8, xytext=(0, 3 if v >= 0 else -3),
                        textcoords="offset points")
        ax.set_xticks(range(len(params)))
        ax.set_xticklabels(params, rotation=35, ha="right", fontsize=9)
    if len(panels) > 1:  # przy jednym panelu tytul zbiorczy jest zbedny
        fig.suptitle("Linear regression slope (a) by parameter", fontsize=13)
        fig.tight_layout(rect=(0, 0, 1, 0.96))
    else:
        fig.tight_layout()
    fig.savefig(out_path, dpi=140, bbox_inches="tight")


def main():
    if len(sys.argv) < 2:
        print("Usage: python3 tools/plot_regression_slopes.py "
              "path/to/_summary/metrics_regression.csv [panel ...]")
        sys.exit(1)
    in_path = sys.argv[1]
    panels = ([parse_panel(s) for s in sys.argv[2:]]
              if len(sys.argv) > 2 else DEFAULT_PANELS)

    out_dir = os.path.dirname(os.path.abspath(in_path))
    out_path = os.path.join(out_dir, "regression_slopes.png")

    slopes = read_slopes(in_path)
    plot(slopes, panels, out_path)

    print(f"zapisano: {out_path}")
    for metric, exclude, title in panels:
        print(f"\n{title}  [{metric}]")
        for p in sorted((p for (m, p) in slopes
                         if m == metric and p not in exclude),
                        key=lambda p: abs(slopes[(metric, p)]), reverse=True):
            print(f"  {p:16s} a = {slopes[(metric, p)]:+.5g}")


if __name__ == "__main__":
    main()
