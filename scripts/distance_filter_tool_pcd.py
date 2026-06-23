#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Modification of original distance_filter_tool.py to remove ROS dependency entirely
Requires PointCloud2 msgs from ros bags to be converted to PCD files
"""

import os
import sys
import numpy as np
import open3d as o3d
import open3d.core as o3c
import argparse

# ===================== 通用：保存 PCD =====================

def save_pcd_with_intensity(points, intensities, output_path):
    """
    保存点云为带 intensity 字段的 PCD 文件 (ASCII 格式)
    points: list/ndarray of [x, y, z]
    intensities: list/ndarray of intensity
    """
    N = len(points)
    header = f"""# .PCD v0.7 - Point Cloud Data file format
VERSION 0.7
FIELDS x y z intensity
SIZE 4 4 4 4
TYPE F F F F
COUNT 1 1 1 1
WIDTH {N}
HEIGHT 1
POINTS {N}
DATA ascii
"""
    with open(output_path, 'w') as f:
        f.write(header)
        for (x, y, z), inten in zip(points, intensities):
            f.write(f"{x} {y} {z} {inten}\n")
    print(f"[PCD] 保存带 intensity 字段的点云到: {output_path}")

# ===================== Open3D 交互选点 & 保存范围 =====================

def select_and_save_points(pcd_folder, target_pcd_name):
    """
    在给定目录中读取指定 PCD 文件，用 Open3D 交互式选点并保存范围。
    """
    pcd_path = os.path.join(pcd_folder, target_pcd_name)
    if not os.path.isfile(pcd_path):
        print(f"[ERROR] 指定的 PCD 文件不存在: {pcd_path}", file=sys.stderr)
        return

    # 读取点云
    pcd = o3d.io.read_point_cloud(pcd_path)
    if not pcd.has_points():
        print(f"[ERROR] {target_pcd_name} 中没有点云数据，已跳过", file=sys.stderr)
        return

    print(f"\n正在处理: {target_pcd_name}")
    print("请在可视化窗口中按住 Shift 用鼠标左键选择点(至少4个)，然后按 Q 键关闭窗口")

    # 创建可视化窗口并添加点云
    vis = o3d.visualization.VisualizerWithEditing()
    vis.create_window(window_name=f"选择点 - {target_pcd_name}")
    vis.add_geometry(pcd)

    # 等待用户交互（Shift+左键选点, Q 退出）
    vis.run()
    vis.destroy_window()

    # 获取用户选择的点的索引
    selected_indices = vis.get_picked_points()

    if not selected_indices:
        print(f"[ERROR] 未选择任何点，{target_pcd_name} 没有保存文件", file=sys.stderr)
        return

    if len(selected_indices) < 4:
        print(f"[ERROR] 只选中了 {len(selected_indices)} 个点，少于 4 个，跳过该文件", file=sys.stderr)
        return

    # 只取前 4 个点
    selected_indices = selected_indices[:4]

    all_points = np.asarray(pcd.points)
    selected_points = all_points[selected_indices, :]  # 形状 (4, 3)

    # 计算四个点在各轴上的最小值和最大值
    mins = selected_points.min(axis=0)  # [x_min_raw, y_min_raw, z_min_raw]
    maxs = selected_points.max(axis=0)  # [x_max_raw, y_max_raw, z_max_raw]

    # 按你的定义扩展 0.2m
    x_min = mins[0] - 0.2
    x_max = maxs[0] + 0.2
    y_min = mins[1] - 0.2
    y_max = maxs[1] + 0.2
    z_min = mins[2] - 0.2
    z_max = maxs[2] + 0.2

    # 生成保存文件名 (与 PCD 文件同名，改为 txt)
    base_name = os.path.splitext(target_pcd_name)[0]
    save_file = os.path.join(pcd_folder, f"{base_name}.txt")

    with open(save_file, 'w') as f:
        f.write("# 4 selected points (x y z)\n")
        for p in selected_points:
            f.write(f"{p[0]:.6f} {p[1]:.6f} {p[2]:.6f}\n")

        f.write("# range values in order:\n")
        f.write(f"x_min: {x_min:.1f}\n")
        f.write(f"x_max: {x_max:.1f}\n")
        f.write(f"y_min: {y_min:.1f}\n")
        f.write(f"y_max: {y_max:.1f}\n")
        f.write(f"z_min: {z_min:.1f}\n")
        f.write(f"z_max: {z_max:.1f}\n")

    print(f"[Save] 已保存选点与范围到: {save_file}")
    print("点云处理完成。")

def read_pcd(pcd_dir, ii, has_ring=False):
    pcd_fp = os.path.join(pcd_dir, "{:08d}.pcd".format(ii))
    read_pc = o3d.t.io.read_point_cloud(pcd_fp)

    if has_ring:
        positions = np.asarray(read_pc.point['positions'].cpu().numpy())
        rings = np.asarray(read_pc.point['ring'].cpu().numpy())
        read_pc_np = np.concatenate([positions, rings], axis=-1)
    else:
        positions = np.asarray(read_pc.point['positions'].cpu().numpy())
        read_pc_np = positions

    return read_pc_np

def get_merged_pcd(in_pcd_dir, start, end, out_dir, has_ring=False):
    full_pc = None
    for i in range(start, end + 1):
        pc = read_pcd(in_pcd_dir, i, has_ring=has_ring)

        if full_pc is None:
            full_pc = pc
        else:
            full_pc = np.concatenate([full_pc, pc], axis=0)
    
    xyz = full_pc[:, :-1] if has_ring else full_pc
    pcd = o3d.t.geometry.PointCloud()
    pcd.point["positions"] = o3c.Tensor(xyz, dtype=o3c.Dtype.Float32)

    pcd_path = os.path.join(out_dir, 'merged_pcd.pcd')
    o3d.t.io.write_point_cloud(pcd_path, pcd)

    return pcd_path

def viz_pcd(pcd_path):
    read_pc = o3d.t.io.read_point_cloud(pcd_path)
    read_pc_np = np.asarray(read_pc.point['positions'].cpu().numpy())

    pc2 = o3d.geometry.PointCloud()
    pc2.points = o3d.utility.Vector3dVector(read_pc_np)
    o3d.visualization.draw_geometries([pc2])

# ===================== main =====================

if __name__ == "__main__":
    # parse args
    parser = argparse.ArgumentParser()
    parser.add_argument('--out_dir', type=str, required=True, help='path to output dir')
    parser.add_argument('--data_dir', type=str, required=True, 
                            help='path to directory containing the directory containing pcd files')
    parser.add_argument('--has_ring', action='store_true', help='whether or not point cloud has ring field')
    args = parser.parse_args()

    output_dir = args.out_dir
    data_dir = args.data_dir
    has_ring = args.has_ring

    # Mode of reading from PCD files
    in_pcd_dir = os.path.join(data_dir, 'velodyne_1_pcd')
    start_frame = 5
    end_frame = 15

    pcd_path = get_merged_pcd(in_pcd_dir, start_frame, end_frame, output_dir, has_ring=has_ring)

    # viz_pcd(pcd_path)

    # Interative point selection
    select_and_save_points(
        pcd_folder=output_dir,
        target_pcd_name=os.path.basename(pcd_path)
    )