<div align="center">
    <h1>KISS-Matcher</h1>
    <a href="https://github.com/MIT-SPARK/KISS-Matcher"><img src="https://img.shields.io/badge/-C++-blue?logo=cplusplus" /></a>
    <a href="https://github.com/MIT-SPARK/KISS-Matcher"><img src="https://img.shields.io/badge/Python-3670A0?logo=python&logoColor=ffdd54" /></a>
    <a href="https://github.com/MIT-SPARK/KISS-Matcher"><img src="https://img.shields.io/badge/ROS2-Humble-blue" /></a>
    <a href="https://github.com/MIT-SPARK/KISS-Matcher"><img src="https://img.shields.io/badge/Linux-FCC624?logo=linux&logoColor=black" /></a>
    <a href="https://arxiv.org/abs/2409.15615"><img src="https://img.shields.io/badge/arXiv-b33737?logo=arXiv" /></a>
    <br />
    <br />
  <br />
  <br />
  <p align="center"><img src="https://github.com/user-attachments/assets/763bafef-c11a-4412-a9f7-f138fc12ff9f" alt="KISS Matcher" width="80%"/></p>
  <p><strong><em>Keep it simple, make it scalable.</em></strong></p>
</div>

______________________________________________________________________

# KISS-Matcher ROS2 Visualization

This repository provides a ROS2-based visualization tool for KISS-Matcher registration results.
It allows users to animate the transformation of a source point cloud to match a target point cloud.

## :package: Prerequisites

To run example codes, in addition to installation of KISS-Matcher (run `make cppinstall` first).

If you only need to download example datasets, please use [download_datasets.py](https://github.com/kimdaebeom/KISS-Matcher/blob/main/python/utils/download_datasets.py).

## :gear: How To Build & RUN

First, you should install KISS-Matcher as follows:

```
cd ${MAIN_DIR_OF_KISS_MATCHER_REPOSITORY}
make cppinstall
```

Then, build the ROS2 package

```bash
cd ${ROS2_WORKSPACE}
colcon build --packages-select kiss_matcher_ros
source install/setup.bash
```

Launch the visualization using the following command:

```bash
ros2 launch kiss_matcher_ros visualizer_launch.py
```

______________________________________________________________________

## 🛠 Configuration

You can customize the visualization parameters in **`config/params.yaml`** before launching the node.

### **Example Configuration**

```yaml
registration_visualizer:
  ros__parameters:
    base_dir: "src/KISS-Matcher/cpp/examples/build/data/"

    # Specify the source and target PCD files
    src_pcd_path: "Vel64/kitti_000540.pcd"
    tgt_pcd_path: "Vel64/kitti_001319.pcd"

    # Registration settings
    resolution: 0.2  # Voxel grid resolution
    moving_rate: 200.0  # Animation steps
    frame_rate: 30.0  # FPS for animation
    scale_factor: 1.0  # Scale factor for the point cloud
```

### **Parameter Descriptions**

| Parameter      | Description |
|---------------|-------------|
| `src_pcd_path`  | Path to the source PCD file |
| `tgt_pcd_path`  | Path to the target PCD file |
| `resolution`   | Voxel grid downsampling resolution |
| `moving_rate`  | Number of animation steps of transition |
| `frame_rate`   | Frames per second for animation in RViz |
| `scale_factor`        | Scaling factor applied to the point clouds |

______________________________________________________________________

## 📌 Notes

- Ensure your **PCD files** exist in the correct directory specified in `params.yaml`.
- If visualization does not start, verify that **PCL and ROS2 dependencies** are correctly installed.
- Modify the **frame rate (`frame_rate`) and animation steps (`moving_rate`)** for smoother visualization.

Now, you can visualize **KISS-Matcher registration results** in ROS2! 🎉
