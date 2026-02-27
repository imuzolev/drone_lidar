"""
Point cloud PLY export with noise filtering.
"""

import os
import numpy as np

try:
    from scipy.spatial import KDTree
except ImportError:
    KDTree = None


def _voxel_downsample(self, points, voxel_size):
    """Voxel grid downsampling (snaps to grid centers to remove noise)."""
    if len(points) == 0:
        return points
    quantized = np.round(points / voxel_size) * voxel_size
    return np.unique(quantized, axis=0)


def _statistical_outlier_removal(self, points, nb_neighbors=20, std_ratio=2.0):
    """Remove points whose mean distance to k neighbours exceeds threshold."""
    if len(points) < nb_neighbors + 1 or KDTree is None:
        if KDTree is None:
            self.log("[FILTER] scipy not available, skipping SOR")
        return points

    self.log(f"[FILTER] Statistical Outlier Removal (k={nb_neighbors}, "
             f"std={std_ratio:.1f}) on {len(points):,} pts...")

    tree = KDTree(points)
    dists, _ = tree.query(points, k=nb_neighbors + 1)
    mean_dists = dists[:, 1:].mean(axis=1)

    global_mean = mean_dists.mean()
    global_std = mean_dists.std()
    threshold = global_mean + std_ratio * global_std

    inlier_mask = mean_dists <= threshold
    filtered = points[inlier_mask]
    removed = len(points) - len(filtered)
    self.log(f"[FILTER] SOR: removed {removed:,} outliers "
             f"({100 * removed / max(1, len(points)):.1f}%), "
             f"kept {len(filtered):,}")
    return filtered


def _radius_outlier_removal(self, points, radius=0.5, min_neighbors=5):
    """Remove points that have fewer than min_neighbors within radius."""
    if len(points) < min_neighbors or KDTree is None:
        if KDTree is None:
            self.log("[FILTER] scipy not available, skipping ROR")
        return points

    self.log(f"[FILTER] Radius Outlier Removal (r={radius:.2f}m, "
             f"min_nb={min_neighbors}) on {len(points):,} pts...")

    tree = KDTree(points)
    counts = tree.query_ball_point(points, r=radius, return_length=True)

    inlier_mask = counts >= (min_neighbors + 1)
    filtered = points[inlier_mask]
    removed = len(points) - len(filtered)
    self.log(f"[FILTER] ROR: removed {removed:,} sparse points "
             f"({100 * removed / max(1, len(points)):.1f}%), "
             f"kept {len(filtered):,}")
    return filtered


def _height_to_rgb(self, z_values):
    """Map Z values to blue-green-yellow-red colormap."""
    if len(z_values) == 0:
        return np.zeros((0, 3), dtype=np.uint8)
    z_min, z_max = z_values.min(), z_values.max()
    if z_max - z_min < 0.01:
        t = np.full_like(z_values, 0.5)
    else:
        t = (z_values - z_min) / (z_max - z_min)
        t = 1.0 - t
    r = np.clip(np.where(t < 0.5, 0.0, (t - 0.5) * 2.0), 0, 1)
    g = np.clip(np.where(t < 0.5, t * 2.0, 2.0 - t * 2.0), 0, 1)
    b = np.clip(np.where(t < 0.5, 1.0 - t * 2.0, 0.0), 0, 1)
    rgb = np.stack([r, g, b], axis=1)
    return (rgb * 255).astype(np.uint8)


def save_point_cloud_ply(self, filepath=None):
    """Save accumulated lidar point cloud to PLY with full noise filtering."""
    filepath = filepath or self.ply_output_path or "point_cloud.ply"

    if not self.all_lidar_points:
        self.log("[PLY] No lidar points collected.")
        return None

    self.log("")
    self.log("=" * 60)
    self.log("  POINT CLOUD POST-PROCESSING")
    self.log("=" * 60)

    self.log("[PLY] Merging point cloud from all scans...")
    all_pts = np.concatenate(self.all_lidar_points, axis=0)
    total_raw = len(all_pts)
    self.log(f"[PLY] Raw points: {total_raw:,} "
             f"(from {len(self.all_lidar_points)} scans)")

    if self.ply_voxel_size > 0:
        all_pts = self._voxel_downsample(all_pts, self.ply_voxel_size)
        self.log(f"[PLY] After voxel downsample ({self.ply_voxel_size}m): "
                 f"{len(all_pts):,} pts "
                 f"({100 * len(all_pts) / max(1, total_raw):.1f}%)")

    if len(all_pts) == 0:
        self.log("[PLY] No points after voxel filtering.")
        return None

    if self.sor_neighbors > 0:
        all_pts = self._statistical_outlier_removal(
            all_pts,
            nb_neighbors=self.sor_neighbors,
            std_ratio=self.sor_std_ratio
        )

    if len(all_pts) == 0:
        self.log("[PLY] No points after SOR.")
        return None

    if self.ror_min_neighbors > 0:
        all_pts = self._radius_outlier_removal(
            all_pts,
            radius=self.ror_radius,
            min_neighbors=self.ror_min_neighbors
        )

    if len(all_pts) == 0:
        self.log("[PLY] No points after ROR.")
        return None

    final_count = len(all_pts)
    reduction = 100 * (1.0 - final_count / max(1, total_raw))
    self.log(f"[PLY] Final: {final_count:,} pts "
             f"(removed {reduction:.1f}% of raw data)")

    colors = self._height_to_rgb(all_pts[:, 2])

    out_dir = os.path.dirname(os.path.abspath(filepath))
    os.makedirs(out_dir, exist_ok=True)

    self.log(f"[PLY] Writing {len(all_pts):,} points to: {filepath}")
    with open(filepath, 'w') as f:
        f.write("ply\n")
        f.write("format ascii 1.0\n")
        f.write("comment Generated by AutonomousExplorer full-coverage scan\n")
        f.write(f"comment Voxel size: {self.ply_voxel_size}m\n")
        f.write(f"comment SOR: k={self.sor_neighbors} std={self.sor_std_ratio}\n")
        f.write(f"comment ROR: r={self.ror_radius} min_nb={self.ror_min_neighbors}\n")
        f.write(f"comment Total scans: {len(self.all_lidar_points)}\n")
        f.write(f"comment Raw points: {total_raw}\n")
        f.write(f"comment Filtered points: {final_count}\n")
        f.write(f"element vertex {len(all_pts)}\n")
        f.write("property float x\n")
        f.write("property float y\n")
        f.write("property float z\n")
        f.write("property uchar red\n")
        f.write("property uchar green\n")
        f.write("property uchar blue\n")
        f.write("end_header\n")
        for i in range(len(all_pts)):
            f.write(f"{all_pts[i, 0]:.4f} {all_pts[i, 1]:.4f} "
                    f"{all_pts[i, 2]:.4f} "
                    f"{colors[i, 0]} {colors[i, 1]} {colors[i, 2]}\n")

    file_size_mb = os.path.getsize(filepath) / (1024 * 1024)
    self.log(f"[PLY] Saved! {file_size_mb:.2f} MB")
    self.log("=" * 60)
    return filepath
