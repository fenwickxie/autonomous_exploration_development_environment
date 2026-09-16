#!/usr/bin/env python3
"""Python port of local_planner/paths/path_generator.m.

This keeps the same logic and file outputs as the original MATLAB script:
- generate the candidate path set (startPaths.ply, paths.ply, pathList.ply)
- generate voxel-to-path correspondence data (correspondences.txt)

Requires: numpy, scipy, matplotlib
"""

import os

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D  # noqa: F401
import numpy as np
from scipy.interpolate import CubicSpline
from scipy.spatial import cKDTree


def matlab_spline(x, y, xq):
    """Mirror MATLAB spline() behavior for the current use case."""
    x = np.asarray(x, dtype=float)
    y = np.asarray(y, dtype=float)
    xq = np.asarray(xq, dtype=float)

    if len(x) < 4:
        return np.interp(xq, x, y)
    return CubicSpline(x, y, bc_type="not-a-knot")(xq)


def write_ply(filename, data, props):
    """Write PLY with the same format and ordering as the MATLAB script."""
    num_points = data.shape[1]
    with open(filename, "w") as f:
        f.write("ply\n")
        f.write("format ascii 1.0\n")
        f.write(f"element vertex {num_points}\n")
        for name, fmt in props:
            ptype = "int" if fmt == "%d" else "float"
            f.write(f"property {ptype} {name}\n")
        f.write("end_header\n")
        line_fmt = " ".join(fmt for _, fmt in props) + "\n"
        for col in range(num_points):
            row = tuple(data[:, col])
            f.write(line_fmt % row)


def main():
    # %% generate path
    dis = 2.0
    # 增加路径距离后，Ackermann 底盘使用 11 度转弯幅度，路径总弧长为 3*dis=6 m。
    angle = 11.0
    delta_angle = angle / 3.0
    scale = 0.65

    path_start_all = []
    path_all = []
    path_list = []
    path_id = 0
    group_id = 0

    fig = plt.figure()
    ax = fig.add_subplot(projection="3d")
    ax.set_xlabel("X (m)")
    ax.set_ylabel("Y (m)")

    print("\nGenerating paths")

    for shift1 in np.arange(-angle, angle + 1e-12, delta_angle):
        waypts_start = np.array([
            [0.0, 0.0, 0.0],
            [dis, shift1, 0.0],
        ])

        path_start_r = np.arange(0.0, dis + 1e-9, 0.01)
        path_start_shift = matlab_spline(waypts_start[:, 0], waypts_start[:, 1], path_start_r)

        path_start_x = path_start_r * np.cos(path_start_shift * np.pi / 180.0)
        path_start_y = path_start_r * np.sin(path_start_shift * np.pi / 180.0)
        path_start_z = np.zeros_like(path_start_x)

        path_start = np.vstack([
            path_start_x,
            path_start_y,
            path_start_z,
            np.ones(path_start_x.shape, dtype=float) * group_id,
        ])
        path_start_all.append(path_start)

        for shift2 in np.arange(-angle * scale + shift1, angle * scale + shift1 + 1e-12, delta_angle * scale):
            for shift3 in np.arange(-angle * scale**2 + shift2, angle * scale**2 + shift2 + 1e-12, delta_angle * scale**2):
                waypts = np.column_stack([
                    np.concatenate([
                        path_start_r,
                        np.array([2.0 * dis, 3.0 * dis - 0.001, 3.0 * dis]),
                    ]),
                    np.concatenate([
                        path_start_shift,
                        np.array([shift2, shift3, shift3]),
                    ]),
                    np.concatenate([
                        path_start_z,
                        np.array([0.0, 0.0, 0.0]),
                    ]),
                ])

                path_r = np.arange(0.0, waypts[-1, 0] + 1e-9, 0.01)
                path_shift = matlab_spline(waypts[:, 0], waypts[:, 1], path_r)

                path_x = path_r * np.cos(path_shift * np.pi / 180.0)
                path_y = path_r * np.sin(path_shift * np.pi / 180.0)
                path_z = np.zeros_like(path_x)

                path = np.vstack([
                    path_x,
                    path_y,
                    path_z,
                    np.ones(path_x.shape, dtype=float) * path_id,
                    np.ones(path_x.shape, dtype=float) * group_id,
                ])
                path_all.append(path)
                path_list.append([path_x[-1], path_y[-1], path_z[-1], path_id, group_id])

                path_id += 1
                ax.plot(path_x, path_y, path_z)

        group_id += 1
        print(group_id)

    print(path_id)

    path_start_all = np.hstack(path_start_all)
    path_all = np.hstack(path_all)
    path_list = np.array(path_list, dtype=float).T

    write_ply(
        "startPaths.ply",
        path_start_all,
        [("x", "%f"), ("y", "%f"), ("z", "%f"), ("group_id", "%d")],
    )

    write_ply(
        "paths.ply",
        path_all,
        [("x", "%f"), ("y", "%f"), ("z", "%f"), ("path_id", "%d"), ("group_id", "%d")],
    )

    write_ply(
        "pathList.ply",
        path_list,
        [("end_x", "%f"), ("end_y", "%f"), ("end_z", "%f"), ("path_id", "%d"), ("group_id", "%d")],
    )

    plt.pause(1.0)

    # %% find correspondence
    voxel_size = 0.02
    search_radius = 0.55
    offset_x = 6.2
    offset_y = 4.5
    voxel_num_x = 311
    voxel_num_y = 451

    print("\nPreparing voxels")

    voxel_point_num = voxel_num_x * voxel_num_y
    voxel_points = np.zeros((voxel_point_num, 2), dtype=float)
    ind_point = 0
    for ind_x in range(voxel_num_x):
        x = offset_x - voxel_size * ind_x
        scale_y = x / offset_x + search_radius / offset_y * (offset_x - x) / offset_x
        for ind_y in range(voxel_num_y):
            y = scale_y * (offset_y - voxel_size * ind_y)

            voxel_points[ind_point, 0] = x
            voxel_points[ind_point, 1] = y
            ind_point += 1

    ax.plot(voxel_points[:, 0], voxel_points[:, 1], np.zeros(voxel_point_num), "k.")
    plt.pause(1.0)

    print("\nCollision checking")
    tree = cKDTree(path_all[0:2, :].T)
    path_ids = path_all[3, :].astype(np.int64)

    print("\nSaving correspondences")
    chunk_size = 2000
    with open("correspondences.txt", "w") as f:
        for chunk_start in range(0, voxel_point_num, chunk_size):
            chunk_end = min(chunk_start + chunk_size, voxel_point_num)
            chunk_matches = tree.query_ball_point(
                voxel_points[chunk_start:chunk_end],
                search_radius,
                return_sorted=False,
                workers=-1,
            )

            for local_idx, matches in enumerate(chunk_matches):
                i = chunk_start + local_idx
                f.write(f"{i} ")

                if len(matches) > 0:
                    for path_ind in np.unique(path_ids[matches]):
                        f.write(f"{path_ind} ")
                f.write("-1\n")

                if (i + 1) % 1000 == 0:
                    print(i + 1)

    print("\nProcessing complete")


if __name__ == "__main__":
    main()
