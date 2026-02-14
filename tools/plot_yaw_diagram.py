#!/usr/bin/env python3

import sys
import csv
import matplotlib
import matplotlib.pyplot as plt
from collections import defaultdict

def read_csv(path):
    data = []
    with open(path, newline='') as csvfile:
        r = csv.DictReader(csvfile)
        for row in r:
            data.append({
                'steering': float(row['steering']),
                'slip': float(row['slip']),
                'latAcc': float(row['latAcc']),
                'yawMoment': float(row['yawMoment'])
            })
    return data


def main():
    if len(sys.argv) < 2:
        print("Usage: python3 tools/plot_yaw_diagram.py path/to/yaw_diagram.csv")
        sys.exit(1)
    path = sys.argv[1]
    data = read_csv(path)
    
    # Group by steering angle
    by_steering = defaultdict(list)
    # Group by slip angle
    by_slip = defaultdict(list)
    
    for point in data:
        by_steering[point['steering']].append(point)
        by_slip[point['slip']].append(point)
    
    plt.figure(figsize=(10, 8))
    
    # Draw lines for constant steering angles (varying slip)
    for steering_angle in sorted(by_steering.keys()):
        points = sorted(by_steering[steering_angle], key=lambda p: p['slip'])
        x = [p['latAcc'] for p in points]
        y = [p['yawMoment'] for p in points]
        plt.plot(x, y, alpha=0.5, linewidth=0.8)
    
    # Draw lines for constant slip angles (varying steering)
    for slip_angle in sorted(by_slip.keys()):
        points = sorted(by_slip[slip_angle], key=lambda p: p['steering'])
        x = [p['latAcc'] for p in points]
        y = [p['yawMoment'] for p in points]
        plt.plot(x, y, alpha=0.5, linewidth=0.8)
    
    # Add scatter points on top
    x_all = [p['latAcc'] for p in data]
    y_all = [p['yawMoment'] for p in data]
    plt.scatter(x_all, y_all, s=6, zorder=5)
    
    plt.xlabel('Lateral acceleration')
    plt.ylabel('Yaw moment')
    plt.title('Yaw moment diagram')
    plt.grid(True, alpha=0.3)
    plt.tight_layout()
    
    out_png = path[:-4] + '.png' if path.lower().endswith('.csv') else path + '.png'
    plt.savefig(out_png)
    print(f"Saved plot to {out_png}")


if __name__ == '__main__':
    main()
