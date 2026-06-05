import open3d as o3d
import open3d.core as o3c
import numpy as np
import matplotlib.pyplot as plt
from circle_fit import taubinSVD
import matplotlib.patches as patches
import copy
import json
import os
import argparse

# global for circle point selection
selected_points = []
all_pts = []
circle_params = []
circle_points = []

def crop_to_board(pcd, corners):
    # build local board frame
    u = corners[1] - corners[0]
    v = corners[3] - corners[0]

    u /= np.linalg.norm(u)
    v /= np.linalg.norm(v)

    n = np.cross(u, v)
    n /= np.linalg.norm(n)

    # re-orthogonalize
    v = np.cross(n, u)

    R = np.column_stack((u, v, n))

    # compute size
    local_pts = (corners - corners[0]) @ R

    min_bound = local_pts.min(axis=0)
    max_bound = local_pts.max(axis=0)

    extent = max_bound - min_bound
    extent[2] += 0.02

    # create bounding box + apply
    center_local = (min_bound + max_bound) / 2
    center_world = corners[0] + center_local @ R.T

    obb = o3d.geometry.OrientedBoundingBox(center_world, R, extent)

    cropped = pcd.crop(obb)

    return cropped

def project_to_2d(plane_model, plane_cloud):
    # Plane normal
    [a, b, c, d] = plane_model
    normal = np.array([a, b, c])

    # Create orthonormal basis (u, v)
    u = np.cross(normal, [0, 0, 1])
    if np.linalg.norm(u) < 1e-6:
        u = np.cross(normal, [0, 1, 0])
    u /= np.linalg.norm(u)

    v = np.cross(normal, u)

    # Origin = centroid
    points = np.asarray(plane_cloud.points)
    origin = points.mean(axis=0)

    # Project to 2D
    points_2d = np.stack([
        (points - origin) @ u,
        (points - origin) @ v
    ], axis=1)

    return points_2d, origin, u, v

def backproject_to_3d(origin, u, v, circle_params):
    centers3d = None
    for circle in circle_params:
        x, y, _ = circle
        p = origin + x * u + y * v
        
        if centers3d is None:
            centers3d = p
        else:
            centers3d = np.vstack((centers3d, p))

    return centers3d

def on_pick(event):
        global selected_points
        global all_pts

        # Get the indices of the points that were picked
        indices = event.ind 
        for i in indices:
            print(f"Selected point index {i}: (x={all_pts[i, 0]}, y={all_pts[i, 1]})")
        selected_points.extend([all_pts[i] for i in indices])

def select_circle_pts():
    global all_pts
    global selected_points

    x = all_pts[:, 0]
    y = all_pts[:, 1]

    fig, ax = plt.subplots()
    # Enable picking with a tolerance of 2 points
    scatter = ax.scatter(x, y, picker=2)

    # Connect the event handler to the figure canvas
    fig.canvas.mpl_connect('pick_event', on_pick)

    plt.show()

    selected_points = np.array(selected_points)

def refine_circles(circle_params):
    # circle_params should be in order of: tl, tr, br, bl (cw ordering)
    tl, tr, br, bl = circle_params

    ty = (tl[1] + tr[1]) / 2
    rx = (br[0] + tr[0]) / 2
    by = (bl[1] + br[1]) / 2
    lx = (tl[0] + bl[0]) / 2

    new_tl = (lx, ty, tl[2])
    new_tr = (rx, ty, tr[2])
    new_br = (rx, by, br[2])
    new_bl = (lx, by, bl[2])

    new_circle_params = [
        new_tl,
        new_tr,
        new_br,
        new_bl
    ]

    return new_circle_params

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

def get_merged_pcd(in_pcd_dir, start, end, has_ring=False):
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

    return pcd.to_legacy()

def select_corners(pcd):
    still = o3d.visualization.VisualizerWithEditing()
    still.create_window(window_name=f"Select corners")
    still.add_geometry(pcd)

    still.run()
    still.destroy_window()

    selected_indices = still.get_picked_points()

    all_points = np.asarray(pcd.points)
    selected_pts = all_points[selected_indices, :]

    print(f"picked points: {selected_pts}")

    return selected_pts

def select_and_save_circles(save_dir):
    global all_pts
    global selected_points
    global circle_points
    global circle_params

    points_paths = []

    all_pts_save_path = os.path.join(save_dir, 'all_2d_pts.npy')
    np.save(all_pts_save_path, all_pts)

    for i in range(4):
        print(f"select pts for circle {i + 1}")
        select_circle_pts()
        # fit circle and continue
        xc, yc, r, _ = taubinSVD(selected_points)
        circle_params.append((xc, yc, r))
        pts = copy.copy(selected_points)
        # save circle points as npy and ref path to npy
        save_path = os.path.join(save_dir, f"{i}.npy")
        points_paths.append(save_path)
        np.save(save_path, pts)
        circle_points.append(pts)
        # reset for next circle
        selected_points = []
    
    return all_pts_save_path, points_paths, circle_params

def plot_fitted_circles(save_path=None):
    global all_pts
    global circle_params
    global circle_points

    idx = 0
    fig, ax = plt.subplots()
    ax.scatter(all_pts[:, 0], all_pts[:, 1])
    for params, pts in zip(circle_params, circle_points):
        xc, yc, r = params
        circle = patches.Circle((xc, yc), r, color='r', fill=False)
        ax.add_patch(circle)
        ax.scatter(pts[:, 0], pts[:, 1])
        ax.scatter(xc, yc, label=f'center{idx+1}')
        idx += 1
    plt.legend()
    if save_path:
        plt.savefig(f'{save_path}.png')
    plt.show()

def visualize_3d_centers_w_pc(pcd_dir, pcd_start, pcd_end, centers3d, has_ring=False):
    orig_pcd = get_merged_pcd(pcd_dir, pcd_start, pcd_end, has_ring=has_ring)
    orig_pcd.paint_uniform_color([0.0, 0.0, 1.0])

    centers_pcd = o3d.geometry.PointCloud()
    centers_pcd.points = o3d.utility.Vector3dVector(centers3d)
    centers_pcd.paint_uniform_color([1.0, 0.0, 0.0])

    o3d.visualization.draw_geometries([orig_pcd, centers_pcd])

if __name__ == '__main__':
    # inputs if running from scatch: directory where pcd files are stored and range of pcd files to use
    # inputs if running from saved: path to data.json
    parser = argparse.ArgumentParser()
    parser.add_argument('--use_saved', action='store_true', help='whether or not to use saved data')
    # args for running from scratch
    parser.add_argument('--pcd_dir', type=str, required=False, help='path to directory where pcd files are')
    parser.add_argument('--pcd_start', type=int, required=False, help='start # of pcd range')
    parser.add_argument('--pcd_end', type=int, required=False, help='end # of pcd range')
    parser.add_argument('--has_ring', action='store_true', help='whether or not ring field is present')
    parser.add_argument('--save_dir', type=str, required=False, help='path to save data to')
    # args for running from saved
    parser.add_argument('--data_path', type=str, required=False, help='path to dir where data is saved')
    # misc args
    parser.add_argument('--no_viz', action='store_true', help='whether to disable visualizing outputs (2d and 3d centers)')
    args = parser.parse_args()

    # parse input args
    use_saved = args.use_saved
    no_viz = args.no_viz

    if not use_saved:
        # expecting pcd_dir, pcd_start, and pcd_end args
        pcd_dir = args.pcd_dir
        pcd_start = args.pcd_start
        pcd_end = args.pcd_end
        has_ring = args.has_ring
        save_dir = args.save_dir

        if not pcd_dir or not pcd_start or not pcd_end or not save_dir:
            raise ValueError("Must supply --pcd_dir, --pcd_start, --pcd_end, and --save_dir args")
        
        if os.path.exists(save_dir):
            overwrite = input(f"{save_dir} exists. Overwrite? y/n ")
            if overwrite == 'y':
                os.makedirs(save_dir, exist_ok=True)
            else:
                raise ValueError(f"{save_dir} already exists")
        else:
            os.makedirs(save_dir, exist_ok=False)
    else:
        data_path = args.data_path

        if not data_path:
            raise ValueError("Must supply --data_path arg")

    # run main logic
    if not use_saved:
        # load pcd
        pcd = get_merged_pcd(pcd_dir, pcd_start, pcd_end, has_ring=has_ring)
        
        # select corners of the calibratino board in original cloud
        selected_pts = select_corners(pcd)

        # crop cloud to just be points on the board
        cropped_cloud = crop_to_board(pcd, selected_pts)

        # o3d.visualization.draw_geometries([cropped_cloud])

        # plane segmentation with RANSAC
        coeffs, inliers = cropped_cloud.segment_plane(
            distance_threshold=0.05,
            ransac_n=3,
            num_iterations=1000
        )

        plane_cloud = cropped_cloud.select_by_index(inliers)

        # o3d.visualization.draw_geometries([plane_cloud])

        # project points to 2d and get origin/plane axes
        all_pts, origin, u, v = project_to_2d(coeffs, plane_cloud)

        # rectify plane

        # select points on each circle and save data
        # NOTE: circle pt selection should be in cw order
        all_pts_save_path, points_paths, circle_params = select_and_save_circles(save_dir)

        # backproject centers to 3d
        # 4x3 np array
        centers3d = backproject_to_3d(origin, u, v, circle_params)
        centers3d_save_path = os.path.join(save_dir, 'centers3d.npy')
        np.save(centers3d_save_path, centers3d)

        # write data.json
        params_json = [{"xc": x.item(), "yc": y.item(), "r": r.item()} for x, y, r in circle_params]

        data_json = {
            "pcd_dir": pcd_dir,
            "pcd_start": pcd_start,
            "pcd_end": pcd_end,
            "has_ring": has_ring,
            "points_2d_path": all_pts_save_path,
            "centers3d_path": centers3d_save_path,
            "circle_params": params_json,
            "circle_points_paths": points_paths,
            "plane_origin": origin.tolist(),
            "u": u.tolist(),
            "v": v.tolist()
        }

        print("Saving data to json")
        with open(os.path.join(save_dir, 'data.json'), 'w', encoding='utf-8') as f:
            json.dump(data_json, f, indent=4, ensure_ascii=False)
    else:
        with open(os.path.join(data_path, 'data.json'), 'r') as f:
            data = json.load(f)

        pcd_dir = data['pcd_dir']
        pcd_start = data['pcd_start']
        pcd_end = data['pcd_end']
        has_ring = data['has_ring']
        all_pts = np.load(data['points_2d_path'])
        centers3d = np.load(data['centers3d_path'])
        circle_params = [(circle['xc'], circle['yc'], circle['r']) for circle in data['circle_params']]
        circle_paths = data['circle_points_paths']
        origin = np.array(data['plane_origin'])
        u = np.array(data['u'])
        v = np.array(data['v'])

        # load circle points from paths
        circle_points = [np.load(path) for path in circle_paths]

    if not args.no_viz:
        # final plot of fitted circles
        data_dir = data_path if use_saved else save_dir
        save_path = os.path.join(data_dir, 'circle_fitting_poc_refined')
        plot_fitted_circles(save_path=save_path)

        # visualize 3d centers along with original point cloud
        visualize_3d_centers_w_pc(pcd_dir, pcd_start, pcd_end, centers3d, has_ring=has_ring)

    # print out 3d centers
    print(f"3D lidar centers: {centers3d}")