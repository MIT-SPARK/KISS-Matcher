import argparse

import kiss_matcher
import numpy as np
import open3d as o3d  # Only for the load of pcd and visualization


# KITTI type bin file
def read_bin(bin_path):
    scan = np.fromfile(bin_path, dtype=np.float32)
    scan = scan.reshape((-1, 4))

    return scan[:, :3]


def read_pcd(pcd_path):
    pcd = o3d.io.read_point_cloud(pcd_path)
    return np.asarray(pcd.points)


def rotate_point_cloud_yaw(point_cloud, yaw_angle_deg):
    yaw_angle_rad = np.radians(yaw_angle_deg)
    rotation_matrix = np.array([
        [np.cos(yaw_angle_rad), -np.sin(yaw_angle_rad), 0],
        [np.sin(yaw_angle_rad),
         np.cos(yaw_angle_rad), 0],
        [0, 0, 1],
    ])
    return (rotation_matrix @ point_cloud.T).T


if __name__ == "__main__":
    # Just tips for checking whether pybinding goes sucessfully or not.
    # attributes = dir(kiss_matcher)
    # for attr in attributes:
    #     print(attr)

    parser = argparse.ArgumentParser(
        description="Point cloud registration using KISS-Matcher.")
    parser.add_argument(
        "--src_path",
        type=str,
        required=True,
        help="Source point cloud file (.bin or .pcd)",
    )
    parser.add_argument(
        "--tgt_path",
        type=str,
        required=True,
        help="Target point cloud file (.bin or .pcd)",
    )
    parser.add_argument(
        "--resolution",
        type=float,
        required=True,
        help="Resolution for processing (e.g., voxel size)",
    )
    parser.add_argument(
        "--yaw_aug_angle",
        type=float,
        required=False,
        help="Yaw augmentation angle in degrees",
    )

    args = parser.parse_args()

    # Load source point cloud
    if args.src_path.endswith(".bin"):
        src = read_bin(args.src_path)
    elif args.src_path.endswith(".pcd"):
        src = read_pcd(args.src_path)
    else:
        raise ValueError("Unsupported file format for src. Use .bin or .pcd")

    # Apply yaw augmentation if provided
    if args.yaw_aug_angle is not None:
        src = rotate_point_cloud_yaw(src, args.yaw_aug_angle)

    # Load target point cloud
    if args.tgt_path.endswith(".bin"):
        tgt = read_bin(args.tgt_path)
    elif args.tgt_path.endswith(".pcd"):
        tgt = read_pcd(args.tgt_path)
    else:
        raise ValueError("Unsupported file format for tgt. Use .bin or .pcd")

    print(f"Loaded source point cloud: {src.shape}")
    print(f"Loaded target point cloud: {tgt.shape}")
    print(f"Resolution: {args.resolution}")
    print(f"Yaw Augmentation Angle: {args.yaw_aug_angle}")

    params = kiss_matcher.KISSMatcherConfig(args.resolution)
    matcher = kiss_matcher.KISSMatcher(params)
    result = matcher.estimate(src, tgt)
    matcher.print()

    # ------------------------------------------------------------
    # Visualization
    # ------------------------------------------------------------
    # Apply transformation to src
    rotation_matrix = np.array(result.rotation)
    translation_vector = np.array(result.translation)
    transformed_src = (rotation_matrix @ src.T).T + translation_vector

    # Create Open3D point clouds
    src_o3d = o3d.geometry.PointCloud()
    src_o3d.points = o3d.utility.Vector3dVector(src)
    src_o3d.colors = o3d.utility.Vector3dVector(
        np.array([[0.78, 0.78, 0.78] for _ in range(src.shape[0])]))  # Gray

    tgt_o3d = o3d.geometry.PointCloud()
    tgt_o3d.points = o3d.utility.Vector3dVector(tgt)
    tgt_o3d.colors = o3d.utility.Vector3dVector(
        np.array([[0.35, 0.65, 0.90] for _ in range(tgt.shape[0])]))  # Cyan

    transformed_src_o3d = o3d.geometry.PointCloud()
    transformed_src_o3d.points = o3d.utility.Vector3dVector(transformed_src)
    transformed_src_o3d.colors = o3d.utility.Vector3dVector(
        np.array([[1.0, 0.63, 0.00]
                  for _ in range(transformed_src.shape[0])]))  # Orange

    # visualization
    vis = o3d.visualization.Visualizer()
    vis.create_window(window_name="KISS-Matcher Viz",
                      width=2400,
                      height=1600,
                      left=300,
                      top=300)

    vis.add_geometry(src_o3d)
    vis.add_geometry(tgt_o3d)
    vis.add_geometry(transformed_src_o3d)

    vis.run()
    vis.destroy_window()

    print("Visualization complete.")
