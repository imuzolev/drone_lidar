"""
Smooth a PLY point cloud file to reduce noise and jaggedness.

Usage:
    python smooth_ply.py point_cloud.ply -o point_cloud_smooth.ply
    python smooth_ply.py point_cloud.ply -o out.ply --k 15 --alpha 0.5
"""

import argparse
import numpy as np
import os

try:
    from scipy.spatial import KDTree
except ImportError:
    KDTree = None


def load_ply_points_colors(filepath):
    """Load XYZ and RGB from ASCII PLY. Returns (points, colors) or (None, None) on error."""
    points = []
    colors = []
    in_header = True
    n_vertices = 0
    props = []

    with open(filepath, 'r') as f:
        for line in f:
            line = line.strip()
            if in_header:
                if line.startswith('element vertex'):
                    n_vertices = int(line.split()[-1])
                elif line.startswith('property'):
                    parts = line.split()
                    if len(parts) >= 2:
                        props.append(parts[-1])
                elif line == 'end_header':
                    in_header = False
                continue

            parts = line.split()
            if len(parts) < 3:
                continue

            try:
                x, y, z = float(parts[0]), float(parts[1]), float(parts[2])
                points.append([x, y, z])
                if len(parts) >= 6:
                    r, g, b = int(parts[3]), int(parts[4]), int(parts[5])
                    colors.append([r/255.0, g/255.0, b/255.0])
            except (ValueError, IndexError):
                continue

    if not points:
        return None, None
    pts = np.array(points, dtype=np.float64)
    cols = np.array(colors, dtype=np.float64) if len(colors) == len(points) else None
    return pts, cols


def save_ply(filepath, points, colors=None):
    """Save points (and optional colors) to ASCII PLY."""
    os.makedirs(os.path.dirname(os.path.abspath(filepath)) or '.', exist_ok=True)
    with open(filepath, 'w') as f:
        f.write("ply\nformat ascii 1.0\n")
        f.write(f"element vertex {len(points)}\n")
        f.write("property float x\nproperty float y\nproperty float z\n")
        if colors is not None:
            f.write("property uchar red\nproperty uchar green\nproperty uchar blue\n")
        f.write("end_header\n")
        for i in range(len(points)):
            x, y, z = points[i, 0], points[i, 1], points[i, 2]
            if colors is not None:
                r, g, b = (np.clip(colors[i] * 255, 0, 255)).astype(np.uint8)
                f.write(f"{x:.4f} {y:.4f} {z:.4f} {r} {g} {b}\n")
            else:
                f.write(f"{x:.4f} {y:.4f} {z:.4f}\n")


def smooth_points(points, k=12, alpha=0.4):
    """Smooth by moving each point toward centroid of k-nearest neighbors."""
    if KDTree is None or len(points) < k + 1:
        return points.copy()
    tree = KDTree(points)
    smoothed = np.zeros_like(points)
    batch_size = 10000
    for start in range(0, len(points), batch_size):
        end = min(start + batch_size, len(points))
        _, idx = tree.query(points[start:end], k=k + 1, workers=-1)
        for i in range(end - start):
            neighbors = points[idx[i]]
            centroid = neighbors.mean(axis=0)
            smoothed[start + i] = (1 - alpha) * points[start + i] + alpha * centroid
    return smoothed


def main():
    parser = argparse.ArgumentParser(description="Smooth PLY point cloud to reduce noise")
    parser.add_argument("input", help="Input PLY file")
    parser.add_argument("-o", "--output", default=None, help="Output PLY (default: input_smooth.ply)")
    parser.add_argument("-k", type=int, default=12, help="K neighbors for smoothing (default: 12)")
    parser.add_argument("--alpha", type=float, default=0.4,
                        help="Blend strength 0..1 toward centroid (default: 0.4)")
    parser.add_argument("--iters", type=int, default=1,
                        help="Number of smoothing passes (default: 1)")
    args = parser.parse_args()

    if KDTree is None:
        print("ERROR: scipy is required. Install: pip install scipy")
        return 1

    if not os.path.isfile(args.input):
        print(f"ERROR: File not found: {args.input}")
        return 1

    out = args.output or args.input.replace('.ply', '_smooth.ply')
    if out == args.input:
        out = args.input.replace('.ply', '_smooth.ply')

    print(f"Loading: {args.input}")
    points, colors = load_ply_points_colors(args.input)
    if points is None:
        print("ERROR: Failed to load PLY or no vertices found")
        return 1

    print(f"Points: {len(points):,}")
    print(f"Smoothing: k={args.k}, alpha={args.alpha}, iters={args.iters}")

    for i in range(args.iters):
        points = smooth_points(points, k=args.k, alpha=args.alpha)
        print(f"  Pass {i+1}/{args.iters} done")

    print(f"Saving: {out}")
    save_ply(out, points, colors)
    print("Done.")
    return 0


if __name__ == "__main__":
    exit(main())
