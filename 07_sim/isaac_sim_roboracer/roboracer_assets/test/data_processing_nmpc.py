import numpy as np
import argparse
import os
import sys
import pandas as pd
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

def load_npz_file(npz_path):
    if not os.path.isfile(npz_path):
        print(f"Error: file not found: {npz_path}", file=sys.stderr)
        return
    # Load the archive (this gives you a dict‐like object)
    data = np.load(npz_path, allow_pickle=True)
    keys = data.files

    if not keys:
        print("No arrays found in the .npz file.")
        return

    # print the keys / data
    print(f"Found {len(keys)} array(s) in '{npz_path}':")
    for key in keys:
        arr = data[key]
        print(f"  • {key}  (shape={arr.shape}, dtype={arr.dtype})")

    return data

if __name__ == "__main__":
    
    # ===========================================================================================
    # parse the file
    # ===========================================================================================
    filename = 'data_record.npz'
    data = load_npz_file(filename)

    # # print the data
    # for key in data.files:
    #     arr = data[key]
    #     print("=" * 80)
    #     print(f"{key}  (shape={arr.shape}, dtype={arr.dtype}):")
    #     print(arr)
    #     print("=" * 80)

    # ===========================================================================================
    # convert the data to a pandas data structure
    # ===========================================================================================
    data_dict = {
        key: list(data[key])
        for key in data.files
    }

    df = pd.DataFrame(data_dict)

    # disply options
    pd.set_option("display.max_columns",    None)   # show every column
    pd.set_option("display.max_rows",       None)   # show every row (you may not need this)
    pd.set_option("display.width",          None)   # auto-detect terminal width
    pd.set_option("display.max_colwidth",   None)   # don’t truncate cell contents

    # print(df.head())

    # ===========================================================================================
    # plot the data
    # ===========================================================================================
    robot_ids = data["robot_id"]                 # shape (N,)
    times     = data["timestamps"]               # shape (N,)
    root_pose = data["root_pose"]                # shape (N, 7)
    root_vel  = data["root_velocity"]            # shape (N, 6)
    root_acc  = data["root_acceleration_base_link"]  # shape (N, 3)

    # select the robots for which to plot the data
    unique_ids = np.unique(robot_ids)
    print("Available robot IDs:", unique_ids)
    selected_robots = [int(x) for x in unique_ids]

    # choose the x,y,z components from each data category
    categories = {
        "Position":           (root_pose[:, :3],    ["X", "Y", "Z"]),
        "Orientation (quat)": (root_pose[:, 3:],    ["w", "x", "y", "z"]),
        "Linear Velocity":    (root_vel[:, :3],     ["Vx", "Vy", "Vz"]),
        "Angular Velocity":   (root_vel[:, 3:],     ["ωx", "ωy", "ωz"]),
        "Linear Acceleration":(root_acc,            ["Ax", "Ay", "Az"]),
    }

    # # plot
    # line_styles = ["-", "--", ":", "-."]  # will cycle if you have >4 dims
    # cmap = plt.get_cmap("tab10")           # one distinct color per robot

    # for cat_name, (arr, labels) in categories.items():
    #     plt.figure(figsize=(10, 6))
    #     for i, robot in enumerate(selected_robots):
    #         mask = (robot_ids == robot)
    #         color = cmap(i)
    #         for j, comp in enumerate(labels):
    #             plt.plot(
    #                 times[mask],
    #                 arr[mask, j],
    #                 label=f"Robot {robot} • {comp}",
    #                 color=color,
    #                 linestyle=line_styles[j % len(line_styles)],
    #             )
    #     plt.title(f"{cat_name} vs Time")
    #     plt.xlabel("Time")
    #     plt.ylabel(cat_name)
    #     plt.legend(loc="best", fontsize="small", ncol=2)
    #     plt.grid(True)
    #     plt.tight_layout()
    #     plt.show()

    # ===========================================================================================
    # 3D plot
    # ===========================================================================================
    fig = plt.figure(figsize=(10, 8))
    ax  = fig.add_subplot(111, projection="3d")
    cmap = plt.get_cmap("tab10")
    
    for idx, robot in enumerate(selected_robots):
        mask = (robot_ids == robot)

        # positions
        x = root_pose[mask, 0]
        y = root_pose[mask, 1]
        z = root_pose[mask, 2]

        # linear velocity components
        vx = root_vel[mask, 0]
        vy = root_vel[mask, 1]
        vz = root_vel[mask, 2]

        color = cmap(idx)

        # trim the points at which vel is plotted
        n_pts = len(x)
        step = max(n_pts // 20, 1)
        xs, ys, zs   = x[::step], y[::step], z[::step]
        vxs, vys, vzs = vx[::step], vy[::step], vz[::step]

        ax.plot(x, y, z, color=color, label=f"Robot {robot}")
        ax.quiver(
            xs, ys, zs,       # arrow base
            vxs, vys, vzs,    # arrow direction
            length=0.08,    # scale factor for arrow length
            normalize=True,
            color=color,
            linewidth=1,
            arrow_length_ratio=0.1,
        )

    limits = max(np.max(np.abs(x)), np.max(np.abs(y)), np.max(np.abs(z)))

    ax.set_xlabel("X")
    ax.set_ylabel("Y")
    ax.set_zlabel("Z")
    ax.set_title("3D Trajectories with Velocity Arrows")
    ax.legend(loc="best")
    ax.set_xlim([-limits, limits])
    ax.set_ylim([-limits, limits])
    ax.set_zlim([-limits, limits])
    plt.tight_layout()
    plt.show()

    # ===========================================================================================
    # data validation
    # ===========================================================================================
    def next_position(pos, vel, acc, dt):
        next_pos = pos + vel * dt + 0.5 * acc * dt**2
        return next_pos
    
    for robot in selected_robots:
        mask = (robot_ids == robot)
        t_r   = times[mask]             # timestamps for this robot
        p_r   = root_pose[mask, :3]     # positions
        v_r   = root_vel[mask, :3]      # linear velocities
        a_r   = root_acc[mask]          # linear accelerations

        dt_r = np.diff(t_r) 
        pred = np.array([
            next_position(p_r[k], v_r[k], a_r[k], dt_r[k])
            for k in range(len(dt_r))
        ])   
        actual = p_r[1:]
        t_plot = t_r[1:] 

        fig, axes = plt.subplots(3, 1, figsize=(10, 8), sharex=True)
        coord_labels = ["X", "Y", "Z"]
        for i, label in enumerate(coord_labels):
            axes[i].plot(t_plot, actual[:, i],     label="actual")
            axes[i].plot(t_plot, pred[:, i], "--", label="predicted")
            axes[i].set_ylabel(label)
            axes[i].grid(True)
            axes[i].legend(loc="best", fontsize="small")

        axes[-1].set_xlabel("Time")
        fig.suptitle(f"Robot {robot}: Actual vs Predicted Position")
        fig.tight_layout(rect=[0, 0.03, 1, 0.95])
        plt.show()