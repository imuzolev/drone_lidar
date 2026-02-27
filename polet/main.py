"""
CLI entry point for AutonomousExplorer.
"""

import argparse

from polet.explorer import AutonomousExplorer


def main():
    parser = argparse.ArgumentParser(
        description="Autonomous Full-Coverage Drone Room Explorer"
    )
    parser.add_argument("--mock", action="store_true",
                        help="Use mock AirSim environment")
    parser.add_argument("--allow-mock-fallback", action="store_true",
                        help="Allow mock fallback if AirSim unavailable")
    parser.add_argument("--room-size", type=float, default=200.0,
                        help="Max map size in meters (default: 200)")
    parser.add_argument("--cell-size", type=float, default=2.0,
                        help="Grid cell size in meters (default: 2)")
    parser.add_argument("--altitude", type=float, default=2.0,
                        help="Flight altitude in meters (default: 2)")
    parser.add_argument("--speed", type=float, default=2.0,
                        help="Flight speed m/s (default: 2)")
    parser.add_argument("--safe-distance", type=float, default=1.0,
                        help="Min obstacle distance in meters (default: 1.0)")
    parser.add_argument("--auto-room-size", action="store_true",
                        help="Auto-detect building bounds from initial lidar scan")
    parser.add_argument("--auto-room-padding", type=float, default=2.0,
                        help="Extra map padding when auto-detecting (default: 2)")
    parser.add_argument("--scan-rotations", type=int, default=32,
                        help="Rotation steps for 360 scan (default: 32)")
    parser.add_argument("--visit-radius", type=float, default=3.0,
                        help="Radius to mark cells visited (default: 3.0)")
    parser.add_argument("--coverage-spacing", type=float, default=1.5,
                        help="Coverage spacing in meters (default: 1.5)")
    parser.add_argument("--ply-output", type=str, default="point_cloud.ply",
                        help="Output PLY file (default: point_cloud.ply)")
    parser.add_argument("--ply-voxel", type=float, default=0.025,
                        help="Voxel downsample size (default: 0.025)")
    parser.add_argument("--no-ply", action="store_true",
                        help="Disable PLY export")

    parser.add_argument("--fast", action="store_true",
                        help="Faster mode: fewer scans, larger voxel (for quick tests)")

    parser.add_argument("--lidar-min-range", type=float, default=0.3,
                        help="Min lidar range in meters (default: 0.3)")
    parser.add_argument("--lidar-max-range", type=float, default=40.0,
                        help="Max lidar range in meters (default: 40)")
    parser.add_argument("--ply-height-min", type=float, default=None,
                        help="Min Z for PLY points (default: auto)")
    parser.add_argument("--ply-height-max", type=float, default=None,
                        help="Max Z for PLY points (default: auto)")
    parser.add_argument("--sor-neighbors", type=int, default=40,
                        help="Statistical Outlier Removal: k-neighbours (default: 40)")
    parser.add_argument("--sor-std", type=float, default=1.5,
                        help="Statistical Outlier Removal: std ratio (default: 1.5)")
    parser.add_argument("--ror-radius", type=float, default=0.5,
                        help="Radius Outlier Removal: search radius m (default: 0.5)")
    parser.add_argument("--ror-min-neighbors", type=int, default=8,
                        help="Radius Outlier Removal: min neighbours (default: 8)")
    parser.add_argument("--no-sor", action="store_true",
                        help="Disable Statistical Outlier Removal")
    parser.add_argument("--no-ror", action="store_true",
                        help="Disable Radius Outlier Removal")

    parser.add_argument("--no-viz", action="store_true",
                        help="Disable in-sim debug visualization overlay")
    parser.add_argument("--viz-interval", type=int, default=3,
                        help="Update grid overlay every N waypoints (default: 3)")
    parser.add_argument("--log-file", type=str, default=None,
                        help="Path to diagnostic log file")
    parser.add_argument("--log-level", type=str, default="INFO",
                        choices=["DEBUG", "INFO", "WARNING", "ERROR"],
                        help="Diagnostic log verbosity (default: INFO)")
    args = parser.parse_args()

    scan_pause = 0.4
    scan_vel_threshold = 0.05
    use_adaptive = False
    if args.fast:
        args.scan_rotations = 16
        args.coverage_spacing = 2.5
        args.ply_voxel = 0.05
        args.sor_neighbors = 20
        args.sor_std = 2.0
        args.ror_min_neighbors = 5
        args.lidar_min_range = 0.5
        scan_pause = 0.15
        scan_vel_threshold = 0.1
        use_adaptive = True
        print("[FAST] Mode active: fewer scans, larger voxel (quick tests).")

    explorer = AutonomousExplorer(
        use_mock=args.mock,
        room_size=args.room_size,
        cell_size=args.cell_size,
        altitude=args.altitude,
        speed=args.speed,
        safe_distance=args.safe_distance,
        require_sim=(not args.mock and not args.allow_mock_fallback),
        auto_room_size=args.auto_room_size,
        auto_room_padding=args.auto_room_padding,
        scan_rotations=args.scan_rotations,
        visit_radius=args.visit_radius,
        coverage_spacing=args.coverage_spacing,
        lidar_min_range=args.lidar_min_range,
        lidar_max_range=args.lidar_max_range,
        ply_height_min=args.ply_height_min,
        ply_height_max=args.ply_height_max,
        sor_neighbors=0 if args.no_sor else args.sor_neighbors,
        sor_std_ratio=args.sor_std,
        ror_radius=args.ror_radius,
        ror_min_neighbors=0 if args.no_ror else args.ror_min_neighbors,
        scan_pause_after_stable=scan_pause,
        scan_stable_vel_threshold=scan_vel_threshold,
        use_adaptive_scan=use_adaptive,
        visualize_in_sim=not args.no_viz,
        viz_update_interval=args.viz_interval,
        log_file=args.log_file,
        log_level=args.log_level,
    )

    if not args.no_ply:
        explorer.ply_output_path = args.ply_output
        explorer.ply_voxel_size = args.ply_voxel

    explorer.run()


if __name__ == "__main__":
    main()
