#!/usr/bin/env python3
"""
Simple cross-platform CSV plotter for yaw diagram.
Usage: python3 tools/plot_yaw_diagram.py build/yaw_diagram.csv
"""
import sys
import csv
import matplotlib
import matplotlib.pyplot as plt
import os

def read_csv(path):
    x = []
    y = []
    with open(path, newline='') as csvfile:
        r = csv.DictReader(csvfile)
        for row in r:
            # steering,slip,latAcc,yawMoment
            x.append(float(row['latAcc']))
            y.append(float(row['yawMoment']))
    return x, y


def main():
    if len(sys.argv) < 2:
        print("Usage: python3 tools/plot_yaw_diagram.py path/to/yaw_diagram.csv")
        sys.exit(1)
    path = sys.argv[1]
    x, y = read_csv(path)
    plt.figure(figsize=(8,6))
    plt.scatter(x, y, s=6)
    plt.xlabel('lateral acceleration')
    plt.ylabel('Yaw moment')
    plt.title('Yaw moment vs steering angle')
    plt.grid(True)
    plt.tight_layout()
    # If running without an interactive display (headless / Agg backend), save to PNG.
    backend = matplotlib.get_backend().lower()
    headless = False
    if sys.platform.startswith('linux') and not os.environ.get('DISPLAY'):
        headless = True

    out_png = None
    if 'agg' in backend or headless:
        out_png = path[:-4] + '.png' if path.lower().endswith('.csv') else path + '.png'
        plt.savefig(out_png)
        print(f"Saved plot to {out_png}")
    else:
        try:
            plt.show()
        except Exception:
            out_png = path[:-4] + '.png' if path.lower().endswith('.csv') else path + '.png'
            plt.savefig(out_png)
            print(f"Saved plot to {out_png} (show failed)")

if __name__ == '__main__':
    main()
