import numpy as np
import matplotlib.pyplot as plt

# Load the saved data
data = np.load("data_record2.npz", allow_pickle=True)

# Extract arrays
target_velocity = np.array(data["target_velocity"])  # shape: (timesteps, num_envs, num_joints)
target_steering = np.array(data["target_steering"])  # shape: (timesteps, num_envs, num_joints)
timestamps = np.array(data["timestamps"])
robot_ids = np.array(data["robot_id"]).flatten() if "robot_id" in data else np.arange(target_velocity.shape[1])

# If robot_id is not present, fallback to env index
if "robot_id" in data:
    robot_ids = np.array(data["robot_id"]).flatten()
    num_robots = len(np.unique(robot_ids))
else:
    num_robots = target_velocity.shape[1]
    robot_ids = np.arange(num_robots)

# Plot velocity for each robot (first joint)
plt.figure(figsize=(12, 5))
for rid in np.unique(robot_ids):
    mask = (robot_ids == rid)
    # If data is stored as (timesteps, num_envs, num_joints)
    if target_velocity.shape[1] == len(robot_ids):
        plt.plot(timestamps, target_velocity[:, rid, 0], label=f'Robot {rid}')
    else:
        # If data is stored as (timesteps, num_joints)
        plt.plot(timestamps[mask], target_velocity[mask, 0], label=f'Robot {rid}')
plt.xlabel('Time [s]')
plt.ylabel('Velocity [m/s]')
plt.title('Velocity Input Over Time')
plt.legend()
plt.grid()

# Plot steering angle for each robot (first joint)
plt.figure(figsize=(12, 5))
for rid in np.unique(robot_ids):
    mask = (robot_ids == rid)
    if target_steering.shape[1] == len(robot_ids):
        plt.plot(timestamps, target_steering[:, rid, 0], label=f'Robot {rid}')
    else:
        plt.plot(timestamps[mask], target_steering[mask, 0], label=f'Robot {rid}')
plt.xlabel('Time [s]')
plt.ylabel('Steering Angle [rad]')
plt.title('Steering Angle Input Over Time')
plt.legend()
plt.grid()

plt.show()