# Modifications Made

## LiDAR Input Format

The input format for LiDAR is modified to accept a set of PCD files. Each PCD file should be named in a 8 character numerical format (i.e., padded with zeros as necessary) over a set of indices ranging from `pc_frame_start` and `pc_frame_end` (both inclusive).

The original FAST-Calib package checks the fields of the ROS message to determine the LiDAR type. Since the LiDAR input type has been modified, the configuration parameters `has_ring` and `is_livox` have been added. 

- **has_ring**: the PCD files have a ring field (of type uint16)
- **is_livox**: the LiDAR data came from the `livox_ros_driver::CustomMsg` type. 
  - The `line` field of these messages can be used as the ring field in this case, hence `has_ring` should also be true if `is_livox` is true.

### Example configuration for qr_params.yaml

```
pc_frame_start: 10
pc_frame_end: 20
pc_dir: <absolute path to directory containing PCD files>
is_livox: false
has_ring: true
```

In this example, the directory pointed to by `pc_dir` should contain a set of `.pcd` files ranging from `00000010.pcd` to `00000020.pcd` and the PCD files have a ring field.

## distance_filter_tool

- The Python script in `scripts/distance_filter_tool_mod.py` adds a `pcd` mode (set using the flag `--mode pcd`) for determining the filter parameters for the `pcl::PassThrough` filters in `lidar_detect.hpp`. 
- The `start_frame` and `end_frame` variables in the script should be modified to the values from `pc_frame_start` and `pc_frame_end` in `qr_params.yaml`.

## Manual QR/LiDAR Detections

This entails additional functionality to allow for adding manual annotation of Aruco marker corners and LiDAR centers.

### Manual QR annotation

- The Python script in `scripts/detect_and_label_markers.py` can be used to manually select the corners of the Aruco markers in an input image. 
- The script will print a formatted version of the corner locations, which can be directly added to the `qr_detect.hpp` file. 
- See the relevant comments (starting with "Example of manually labeled corners") for additional examples/information.

### Manual LiDAR annotation

- The Python script in `scripts/label_lidar_centers.py` can be used to manually annotate the circles in a LiDAR point cloud and compute the circle centers based on this. 
- The script will print out the 3D LiDAR center locations, which can be added to the `qr_params.yaml` file.

#### Example configuration for qr_params.yaml for manual LiDAR annotation

```
manual_lidar_detect: true
center1: [ 2.33266431, -0.12582024, -0.46863997]
center2: [ 2.2896755 , -0.64073509, -0.45719983]
center3: [ 2.32743198, -0.6252265 , -0.04157866]
center4: [ 2.36942853, -0.11958425, -0.05530122]
```

The above configuration enables manually annotated LiDAR center inputs, and defines the 3D locations of the 4 LiDAR centers in the input point cloud. If `manual_lidar_detect` is false, then the values of the `center*` params will be ignored.

# FAST-Calib

## FAST-Calib: LiDAR-Camera Extrinsic Calibration in One Second

FAST-Calib is an efficient target-based extrinsic calibration tool for LiDAR-camera systems (eg., [FAST-LIVO2](https://github.com/hku-mars/FAST-LIVO2)). 

**Key highlights include:** 

1. Support solid-state and mechanical LiDAR.
2. No need for any initial extrinsic parameters.
3. Achieve highly accurate calibration results **in just one seconds**.

In short, it makes extrinsic calibration as simple as intrinsic calibration.

**Related paper:** 

[FAST-Calib: LiDAR-Camera Extrinsic Calibration in One Second](https://www.arxiv.org/pdf/2507.17210)

📬 For further assistance or inquiries, please feel free to contact Chunran Zheng at zhengcr@connect.hku.hk.

<p align="center">
  <img src="./pics/calib.jpg" width="100%">
  <font color=#a0a0a0 size=2>Left: Example of Mid360 LiDAR calibration. Right: Point cloud colored with the calibrated extrinsics.</font>
</p>

<p align="center">
  <img src="./pics/all_lidar_type.jpg" width="100%">
  <font color=#a0a0a0 size=2>Circular hole extraction supports multiple LiDAR models.</font>
</p>

## 1. Prerequisites
PCL>=1.8, OpenCV>=4.0.

## 2. Run our examples
1. Prepare the static acquisition data in the `calib_data` folder (see [Single-scene Calibration Sample Data](https://connecthkuhk-my.sharepoint.com/:f:/g/personal/zhengcr_connect_hku_hk/Eq_k_4Mf_11Eggg4a5lbRzgBHwd0EivtCJd2ExtcNlu1FA?e=vjm4gH) from Mid360, Avia and Ouster, and [Multi-scene Calibration Sample Data](https://pan.baidu.com/s/1Mkw7EWfiFT68LEzdkQnxeg?pwd=nyuh) from Avia):
- rosbag containing point cloud messages
- corresponding image

2. Run the single-scene calibration process:
```bash
roslaunch fast_calib calib.launch
```

3. After completing Step 2 for at least three different scenes, you can perform multi-scene joint calibration:
```bash
roslaunch fast_calib multi_calib.launch
```

## 3. Run on your own sensor suite
1. Customize the calibration target in the image below, with the CAD model available [here](https://pan.baidu.com/s/14Q2zmEfY6Z2O5Cq4wgVljQ?pwd=2hhn).
2. Collect data from three scenes, with placement illustrated below, and record them into the corresponding rosbags.
3. Provide the instrinsic matrix in `qr_params.yaml`.
4. Set distance filter in `qr_params.yaml` for board point cloud (extra points are acceptable).
5. Calibrate now!

💡 **Note:** You can run `scripts/distance_filter_tool.py` to quickly obtain suitable filter parameters.
<p align="center">
  <img src="./pics/calibration_target.jpg" width="100%">
  <font color=#a0a0a0 size=2>Left: Actual calibration target | Right: Technical drawing with annotated dimensions.</font>
</p>
<p align="center">
  <img src="./pics/multi-scene.jpg" width="100%">
  <font color=#a0a0a0 size=2>Placement of the calibration target for multi-scene data collection: (a) facing forward, (b) oriented to the right, (c) oriented to the left.</font>
</p>

## 4. Appendix
The calibration target design is based on the [velo2cam_calibration](https://github.com/beltransen/velo2cam_calibration).

For further details on the algorithm workflow, see [this document](https://github.com/xuankuzcr/FAST-Calib/blob/main/workflow.md).
## 5. Acknowledgments

Special thanks to [Jiaming Xu](https://github.com/Xujiaming1) for his support, [Haotian Li](https://github.com/luo-xue) for the equipment, and the [velo2cam_calibration](https://github.com/beltransen/velo2cam_calibration) algorithm.