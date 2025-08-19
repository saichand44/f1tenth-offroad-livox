import numpy as np
import argparse
import os
import sys
import pandas as pd
from scipy.spatial.transform import Rotation
import matplotlib.pyplot as plt



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
        print(f"  --> {key}  (shape={arr.shape}, dtype={arr.dtype})")

    return data

def compute_delx(states, timestamps):

    if len(states) != len(timestamps):
        raise ValueError("Length of states and timestamps must match.")
    
    del_x = np.zeros((states.shape[0], 14))  # last row for evey robot is 0's

    # get the number of robots
    robot_ids = states[:, 0]
    unique_ids = np.unique(robot_ids)
    selected_robots = [int(x) for x in unique_ids]

    idx_offset = 0
    for id in selected_robots:
        mask = (robot_ids == id)

        curr_robot = robot_ids[mask]             

        for i in range(curr_robot.shape[0]-1):
            # robot id
            del_x[idx_offset + i, 0] = robot_ids[mask][i]

            dt = timestamps[mask][i+1] - timestamps[mask][i]

            # root velocity
            del_x[idx_offset + i, 1:7] = (states[mask, 1:7][i + 1] - states[mask, 1:7][i]) / dt

            # quaternion
            del_x[idx_offset + i, 7:11] = Rotation.from_rotvec(states[mask, 4:7][i] * dt).as_quat(canonical=False, scalar_first=True)

            # acceleration
            del_x[idx_offset + i, 11:14] = (states[mask, 11:14][i + 1] - states[mask, 11:14][i]) / dt
        
        idx_offset += curr_robot.shape[0]

    return del_x
        
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
    # gather the data
    # ===========================================================================================
    data_dict = {
        key: list(data[key])
        for key in data.files
    }

    # ===========================================================================================
    # get the input data
    # ===========================================================================================

    robot_ids = data["robot_id"].reshape(-1, 1)      

    # position and orientation
    root_pose = data["root_pose"]
    root_vel = data["root_velocity"]
    root_acc  = data["root_acceleration_base_link"]

    # control inputs
    target_velocity = data["target_velocity"][:, :1]
    target_steering = data["target_steering"][:, :1]

    # track params
    q = root_pose[:, 3:]
    vehicle_pose = Rotation.from_quat(q, scalar_first=True)

    g_R_p = np.squeeze(data["g_R_p"], axis=1)
    p_R_g = Rotation.from_matrix(np.transpose(g_R_p, (0, 2, 1)))  # from plane to ground

    terrain_pose = vehicle_pose.as_matrix() @ p_R_g.as_matrix()
    terrain_quat = Rotation.from_matrix(terrain_pose).as_quat(canonical=False, scalar_first=True) # [w, x, y, z]
    
    # steering angle vel
    steering_joint_vel = np.abs(data["joint_velocity_front_left_wheel_steer"]).reshape(-1, 1) 

    # organize the inputs
    input_vec = np.hstack([robot_ids, root_vel, terrain_quat, root_acc, target_velocity, target_steering, steering_joint_vel])

    # ===========================================================================================
    # get the output data
    # ===========================================================================================
    times = data["timestamps"]
    output_vec = compute_delx(states=input_vec, timestamps=times)

    # ===========================================================================================
    # save the data in npz format
    # ===========================================================================================
    save_path = "processed_data.npz"
    np.savez_compressed(save_path, input=input_vec, output=output_vec)

    robot_id_to_plot = robot_ids[0, 0]
    mask = (robot_ids[:, 0] == robot_id_to_plot)

    # Actual values
    actual_pos = root_pose[mask, :3]
    actual_vel = root_vel[mask, :]

    # Integrated values
    reconstructed_pos = [actual_pos[0]]
    reconstructed_vel = [actual_vel[0]]

    for i in range(output_vec[mask].shape[0] - 1):
        dt = times[mask][i+1] - times[mask][i]
        # Integrate velocity
        new_pos = reconstructed_pos[-1] + output_vec[mask][i, 1:4] * dt
        reconstructed_pos.append(new_pos)
        # Integrate acceleration for velocity (only first 3 components)
        new_vel = reconstructed_vel[-1].copy()
        new_vel[:3] += output_vec[mask][i, 11:14] * dt
        reconstructed_vel.append(new_vel)

    reconstructed_pos = np.array(reconstructed_pos)
    reconstructed_vel = np.array(reconstructed_vel)

    # Plot position
    plt.figure(figsize=(12, 5))
    for j, label in enumerate(['x', 'y', 'z']):
        plt.subplot(1, 3, j+1)
        plt.plot(actual_pos[:, j], label='Actual')
        plt.plot(reconstructed_pos[:, j], label='Integrated')
        plt.title(f'Position {label}')
        plt.legend()
    plt.tight_layout()
    plt.show()

    # Plot velocity
    plt.figure(figsize=(12, 5))
    for j, label in enumerate(['vx', 'vy', 'vz']):
        plt.subplot(1, 3, j+1)
        plt.plot(actual_vel[:, j], label='Actual')
        plt.plot(reconstructed_vel[:, j], label='Integrated')
        plt.title(f'Velocity {label}')
        plt.legend()
    plt.tight_layout()
    plt.show()