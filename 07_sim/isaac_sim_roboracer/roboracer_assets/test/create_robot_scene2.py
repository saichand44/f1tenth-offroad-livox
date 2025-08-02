import argparse
import os

# Set environment variables to reduce GPU memory usage
os.environ["CUDA_LAUNCH_BLOCKING"] = "1"
os.environ["PYTORCH_CUDA_ALLOC_CONF"] = "max_split_size_mb:128"

from isaaclab.app import AppLauncher

# create argparser
parser = argparse.ArgumentParser(description="Creating a non-planar track scene with vehicle")
parser.add_argument("--num_envs", type=int, default=1, help="Number of environments to spawn.")
# parser.add_argument("--headless", action="store_true", help="Run in headless mode to save GPU memory.")

# append AppLauncher cli args
AppLauncher.add_app_launcher_args(parser)

# parse the arguments
args_cli = parser.parse_args()

# launch omniverse app
app_launcher = AppLauncher(args_cli)
simulation_app = app_launcher.app

"""Rest everything follows."""

import torch
import numpy as np
from scipy.spatial.transform import Rotation
import matplotlib.pyplot as plt

import isaaclab.sim as sim_utils
from isaaclab.assets import ArticulationCfg, AssetBaseCfg
from isaaclab.scene import InteractiveScene, InteractiveSceneCfg
from isaaclab.sim import SimulationContext
from isaaclab.utils import configclass

from roboracer_assets.mushr import MUSHR_CFG

# Nominal, min. max velocities and steering for each robot
MIN_VEL, MAX_VEL = 0.0, 5.0 
# MIN_VEL, MAX_VEL = 0.0, 0.0 
MIN_STEER, MAX_STEER = -0.8, 0.8 # radians 
# MIN_STEER, MAX_STEER = 0.0, 0.0 # radians 
GROUND_PLANE_ANGLE = -20.0 # degrees
# GROUND_PLANE_ANGLE = 0.0 # degrees
SAVE_DIR = os.path.dirname(os.path.abspath(__file__))
MAX_COUNT = 500
WHEEL_DIAMETER = 0.1 # in meters

def get_gravity_vec(angle_in_deg, g_original):
    """
    Compute the new gravity vector that corresponds to the rotation of ground plane
    by "angle" in deg in positive Y axis
    """
    angle_in_rad = np.deg2rad(angle_in_deg)
    # print(f"angle_in_rad: {angle_in_rad}")
    Ry = np.array([[np.cos(angle_in_rad), 0, np.sin(angle_in_rad)],
                   [0, 1, 0],
                   [-np.sin(angle_in_rad), 0, np.cos(angle_in_rad)]])
    # print(f"g_original:{g_original}")
    # print(f"Ry.T:{Ry.T}")
    g_transform = Ry.T @ g_original
    g_transform = g_transform.flatten()

    return (float(g_transform[0]), float(g_transform[1]), float(g_transform[2])), Ry.T

G_ORIGINAL = np.array([0.0, 0.0, -9.81]).reshape(3, 1)
G_TRANSFORM, G_R_P = get_gravity_vec(GROUND_PLANE_ANGLE, G_ORIGINAL)
# print(f"G_TRANSFORM:\n {G_TRANSFORM}")


@configclass
class NonPlanarSceneCfg(InteractiveSceneCfg):
    """
    Configuration for a scene with non-planar track and the vehicle
    """

    # Ground-plane
    ground = AssetBaseCfg(
        prim_path="/World/defaultGroundPlane", 
        spawn=sim_utils.GroundPlaneCfg(
            physics_material=sim_utils.RigidBodyMaterialCfg(
                static_friction = 1.0,
                dynamic_friction = 1.0,
                friction_combine_mode="average",
                restitution_combine_mode="average",
            )
        ))

    # Lights - reduced intensity to save GPU memory
    dome_light = AssetBaseCfg(
        prim_path="/World/Light", 
        spawn=sim_utils.DomeLightCfg(intensity=1000.0, color=(0.75, 0.75, 0.75))
    )

    # Articulation
    robot: ArticulationCfg = MUSHR_CFG.replace(prim_path="{ENV_REGEX_NS}/Vehicle")

# TODO: Need appropriate target vel to rpm conversion
def generate_target_velocity(min_vel, max_vel, num_envs, num_joints):
    """
    Generate a random target velocity for each robot within the given range.
    """
    #TODO: Check the units of joint velocity (rad/s or ded/s) and wheel radius and
    # coonvert the target velocity to appropriate dc motor rpms

    # Compute the normal distribution params
    mean = (max_vel + min_vel) / 2
    std = (max_vel - min_vel) / 6
    
    # Generate random velocities between min_vel and max_vel for each robot
    velocities = torch.normal(mean=mean, std=std, size=(num_envs, 1))
    velocities = torch.clamp(velocities, min=min_vel, max=max_vel)
    velocities = velocities.expand(-1, num_joints)
    return velocities

def generate_target_steering(min_steer, max_steer, num_envs, num_joints):
    """
    Generate a random target steering for each robot within the given range.
    """
    # Compute the normal distribution params
    mean = (max_steer + min_steer) / 2
    std = (max_steer - min_steer) / 6

    # Generate random steering values between min_steer and max_steer for each robot
    steering = torch.normal(mean=mean, std=std, size=(num_envs, 1))
    steering = torch.clamp(steering, min=min_steer, max=max_steer)
    steering = steering.expand(-1, num_joints)
    return steering


def initial_robot_orientation(num_envs):
    """
    Assign random yaw angles to initial pose of the robot
    """

    # Generate random yaw angles
    yaw_angles = torch.rand(num_envs) * 2*np.pi

    # Compute the quaternions
    w = torch.cos(yaw_angles / 2.0)
    x = torch.zeros_like(w)
    y = torch.zeros_like(w)
    z = torch.sin(yaw_angles / 2.0)

    # Ensure the sign of the quaternion
    is_neg = w < 0.0
    w[is_neg] = -1 * w[is_neg]
    x[is_neg] = -1 * x[is_neg]
    y[is_neg] = -1 * y[is_neg]
    z[is_neg] = -1 * z[is_neg]

    return torch.stack((w, x, y, z), dim=1)


def transform_to_inclined_frame(positions, velocities, orientations, rotation_matrix):
    """
    Transform positions, velocities, and orientations from world frame to inclined surface frame.
    
    Args:
        positions: torch.Tensor of shape (num_envs, 3) - world frame positions
        velocities: torch.Tensor of shape (num_envs, 3) - world frame linear velocities
        orientations: torch.Tensor of shape (num_envs, 4) - quaternions (w, x, y, z)
        rotation_matrix: np.ndarray of shape (3, 3) - rotation matrix from world to inclined frame
    
    Returns:
        tuple: (transformed_positions, transformed_velocities, yaw_angles)
    """
    # Convert rotation matrix to torch tensor
    R = torch.tensor(rotation_matrix, dtype=torch.float32, device=positions.device)
    
    # Transform positions and velocities
    transformed_positions = torch.matmul(positions, R.T)
    transformed_velocities = torch.matmul(velocities, R.T)
    
    # Extract yaw angles from quaternions in the inclined frame
    yaw_angles = torch.zeros(orientations.shape[0], device=orientations.device)
    
    for i in range(orientations.shape[0]):
        # Convert quaternion to rotation matrix
        quat = orientations[i]  # (w, x, y, z)
        
        # Normalize quaternion to ensure it's unit length
        quat = quat / torch.norm(quat)
        w, x, y, z = quat[0], quat[1], quat[2], quat[3]
        
        # Rotation matrix from quaternion (standard formula)
        R_quat = torch.tensor([
            [1 - 2*(y**2 + z**2), 2*(x*y - w*z), 2*(x*z + w*y)],
            [2*(x*y + w*z), 1 - 2*(x**2 + z**2), 2*(y*z - w*x)],
            [2*(x*z - w*y), 2*(y*z + w*x), 1 - 2*(x**2 + y**2)]
        ], device=orientations.device, dtype=torch.float32)
        
        # Transform to inclined frame
        R_inclined = torch.matmul(R, R_quat)
        
        # Extract yaw angle (rotation around Z-axis in inclined frame)
        # This represents the vehicle's heading relative to the downhill direction
        yaw_angles[i] = torch.atan2(R_inclined[1, 0], R_inclined[0, 0])
    
    return transformed_positions, transformed_velocities, yaw_angles


def compute_state_variables(positions, velocities, yaw_angles, steering_angles, linear_accelerations, _):
    """
    Compute the 7 state variables for the hybrid neural ODE model (excluding Z position).
    """
    # Extract x, y positions in the flat ground frame
    x_pos = positions[:, 0]
    y_pos = positions[:, 1]

    # Compute velocity magnitude in the flat ground frame
    velocity_magnitude = torch.sqrt(velocities[:, 0]**2 + velocities[:, 1]**2)

    # Compute side slip angle (angle between heading and velocity direction)
    velocity_direction = torch.atan2(velocities[:, 1], velocities[:, 0])
    side_slip = velocity_direction - yaw_angles

    # Normalize side slip angle to [-pi, pi]
    side_slip = torch.atan2(torch.sin(side_slip), torch.cos(side_slip))

    # Compute yaw rate (will be updated later)
    yaw_rate = torch.zeros_like(yaw_angles)

    # Stack all state variables (excluding Z position)
    states = torch.stack([
        x_pos, y_pos, yaw_angles, steering_angles, 
        velocity_magnitude, side_slip, yaw_rate
    ], dim=1)

    return states


def run_simulator(sim: sim_utils.SimulationContext, scene: InteractiveScene):
    """
    Runs the simulation loop and collects data for hybrid neural ODE model.
    """
    # Extract scene entities
    robot = scene["robot"]
    throttle_ids = robot.find_joints(".*_throttle")[0]
    steer_ids    = robot.find_joints(".*_steer")[0]

    # Define simulation stepping
    sim_dt = sim.get_physics_dt()
    count = 0

    

    # Initialize data collection lists
    timestamps = []
    states_data = []
    inputs_data = []
    
    # Store previous yaw angles for yaw rate computation
    prev_yaw = None

    # --- Initialize current velocity and steering angle at first reset ---
    current_velocity = None
    current_steering = None

    # Simulation loop
    while simulation_app.is_running():
        # Reset    
        if count % MAX_COUNT == 0:
            count = 0

            # robot
            root_state = robot.data.default_root_state.clone()
            root_state[:, :3] += scene.env_origins

            random_orientations = initial_robot_orientation(scene.num_envs)
            random_orientations = random_orientations.to(sim.device)
            root_state[:, 3:7] = random_orientations

            robot.write_root_pose_to_sim(root_state[:, :7])
            robot.write_root_velocity_to_sim(root_state[:, 7:])

            # Initialize current velocity and steering angle using the functions
            current_velocity = generate_target_velocity(
                MIN_VEL, MAX_VEL, scene.num_envs, len(throttle_ids)
            )
            current_steering = generate_target_steering(
                MIN_STEER, MAX_STEER, scene.num_envs, len(steer_ids)
            )
            
            # Reset previous yaw
            prev_yaw = None
            
            # clear internal buffers
            scene.reset()
            print("[INFO]: Resetting scene state...")

        # --- Randomize desired acceleration and steering velocity ---
        desired_acceleration = torch.empty_like(current_velocity).uniform_(-1.0, 1.0)  # m/s^2
        desired_steer_vel = torch.empty_like(current_steering).uniform_(-0.3, 0.3)    # rad/s

        # --- Euler integration for velocity and steering angle ---
        current_velocity = current_velocity + desired_acceleration * sim_dt
        current_velocity = torch.clamp(current_velocity, MIN_VEL, MAX_VEL)

        current_steering = current_steering + desired_steer_vel * sim_dt
        current_steering = torch.clamp(current_steering, MIN_STEER, MAX_STEER)

        # --- Use these as control outputs ---
        target_velocity = current_velocity.to(sim.device)
        target_rpms = target_velocity / (WHEEL_DIAMETER * 0.5)  # w = v / r
        robot.set_joint_velocity_target(target_rpms, joint_ids=throttle_ids)

        target_steering = current_steering.to(sim.device)
        robot.set_joint_position_target(target_steering, joint_ids=steer_ids)

        # Write data to sim
        scene.write_data_to_sim()

        # Perform step
        sim.step()
        count += 1
        scene.update(sim_dt)

        # --- Data Collection for Hybrid Neural ODE ---
        # Get current robot state in the world frame (flat ground frame)
        root_pos = robot.data.root_pos_w  # (num_envs, 3) - position in world frame
        root_quat = robot.data.root_quat_w  # (num_envs, 4) - quaternion orientation (w, x, y, z)
        root_lin_vel = robot.data.root_lin_vel_w  # (num_envs, 3) - linear velocity in world frame
        root_ang_vel = robot.data.root_ang_vel_w  # (num_envs, 3) - angular velocity in world frame
        root_lin_acc = robot.data.body_lin_acc_w  # (num_envs, 3) - linear acceleration in world frame
        
        # Extract yaw angles directly from world frame quaternions
        yaw_angles = torch.atan2(
            2 * (root_quat[:, 0] * root_quat[:, 3] + root_quat[:, 1] * root_quat[:, 2]),
            1 - 2 * (root_quat[:, 2]**2 + root_quat[:, 3]**2)
        )

        # Compute yaw rate
        if prev_yaw is not None:
            delta_yaw = yaw_angles - prev_yaw
          
            delta_yaw = torch.atan2(torch.sin(delta_yaw), torch.cos(delta_yaw))
            yaw_rate = delta_yaw / sim_dt
        else:
            yaw_rate = torch.zeros_like(yaw_angles)

        # Compute state variables in the flat ground frame
        states = compute_state_variables(
            root_pos, root_lin_vel, yaw_angles, target_steering[:, 0], root_lin_acc, None
        )

        # Update yaw rate in states
        states[:, 6] = yaw_rate
        
        # Prepare control inputs
        inputs = torch.stack([
            desired_acceleration[:, 0],  # longitudinal acceleration
            desired_steer_vel[:, 0]      # steering rate
        ], dim=1)
        
        # Store data
        timestamps.append(sim.current_time)
        states_data.append(states.cpu().numpy())
        inputs_data.append(inputs.cpu().numpy())
        
        # Update previous yaw
        prev_yaw = yaw_angles.clone()

        if (count == MAX_COUNT):
            break

    # Convert lists to numpy arrays
    timestamps_arr = np.array(timestamps)
    states_arr = np.array(states_data)  # (timesteps, num_envs, 7) - Excludes Z position
    inputs_arr = np.array(inputs_data)  # (timesteps, num_envs, 2)

    save_dir = SAVE_DIR
    filename = "hybrid_neural_ode_data12"

    # Save data in .npz format
    np.savez(os.path.join(save_dir, f'{filename}.npz'),
         timestamps=timestamps_arr,
         states=states_arr,
         inputs=inputs_arr,
         ground_plane_angle=GROUND_PLANE_ANGLE,
         rotation_matrix=G_R_P,
         num_envs=scene.num_envs)
    
    print(f"[INFO]: Hybrid neural ODE data saved to {os.path.join(save_dir, f'{filename}.npz')}")
    
    # Print summary statistics for verification
    print(f"[INFO]: Data collection summary:")
    print(f"  - Total timesteps: {len(timestamps_arr)}")
    print(f"  - Number of robots: {scene.num_envs}")
    print(f"  - Ground plane angle: {GROUND_PLANE_ANGLE}°")
    print(f"  - Simulation time: {timestamps_arr[-1]:.2f}s")
    print(f"  - States shape: {states_arr.shape}")
    print(f"  - Inputs shape: {inputs_arr.shape}")
    
    # Print sample data ranges for verification
    print(f"[INFO]: State variable ranges:")
    state_names = ['x_pos', 'y_pos', 'yaw', 'steering', 'velocity', 'side_slip', 'yaw_rate']
    for i, name in enumerate(state_names):
        min_val = np.min(states_arr[:, :, i])

        max_val = np.max(states_arr[:, :, i])
        print(f"  - {name}: [{min_val:.3f}, {max_val:.3f}]")
    
    print(f"[INFO]: Control input ranges:")
    input_names = ['acceleration', 'steering_rate']
    for i, name in enumerate(input_names):
        min_val = np.min(inputs_arr[:, :, i])
        max_val = np.max(inputs_arr[:, :, i])
        print(f"  - {name}: [{min_val:.3f}, {max_val:.3f}]")
    
    # Visualize the collected data
    # visualize_collected_data(timestamps_arr, states_arr, inputs_arr, scene.num_envs)


def visualize_collected_data(timestamps, states, inputs, num_envs):
    """
    Visualize the collected data over time for each robot (excluding Z position).
    """
    state_names = ['X Position [m]', 'Y Position [m]', 'Yaw [rad]', 
                   'Steering [rad]', 'Velocity [m/s]', 'Side Slip [rad]', 'Yaw Rate [rad/s]']
    input_names = ['Acceleration [m/s²]', 'Steering Rate [rad/s]']
    
    # Plot state variables
    fig, axes = plt.subplots(3, 3, figsize=(15, 12))
    axes = axes.flatten()
    
    for i in range(7):  # Exclude Z position
        ax = axes[i]
        for env in range(num_envs):
            ax.plot(timestamps, states[:, env, i], label=f'Robot {env+1}', alpha=0.7)
        ax.set_xlabel('Time [s]')
        ax.set_ylabel(state_names[i])
        ax.set_title(f'State: {state_names[i]}')
        ax.legend()
        ax.grid(True)
    
    # Remove empty subplot if any
    if len(axes) > 7:
        for i in range(7, len(axes)):
            axes[i].remove()
    
    plt.tight_layout()
    plt.show()
    
    # Plot control inputs
    fig, axes = plt.subplots(1, 2, figsize=(12, 5))
    
    for i in range(2):
        ax = axes[i]
        for env in range(num_envs):
            ax.plot(timestamps, inputs[:, env, i], label=f'Robot {env+1}', alpha=0.7)
        ax.set_xlabel('Time [s]')
        ax.set_ylabel(input_names[i])
        ax.set_title(f'Control Input: {input_names[i]}')
        ax.legend()
        ax.grid(True)
    
    plt.tight_layout()
    plt.show()
    
    # Plot trajectory in flat ground frame
    plt.figure(figsize=(10, 8))
    for env in range(num_envs):
        plt.plot(states[:, env, 0], states[:, env, 1], label=f'Robot {env+1}', alpha=0.7)
        plt.scatter(states[0, env, 0], states[0, env, 1], marker='o', s=100, label=f'Start {env+1}')
        plt.scatter(states[-1, env, 0], states[-1, env, 1], marker='s', s=100, label=f'End {env+1}')
    
    plt.xlabel('X Position [m]')
    plt.ylabel('Y Position [m]')
    plt.title('Robot Trajectories in Flat Ground Frame')
    plt.legend()
    plt.grid(True)
    plt.axis('equal')
    plt.show()
def main():
    """
    Main function.
    """

    # Load kit helper with memory-optimized settings
    sim_cfg = sim_utils.SimulationCfg(device=args_cli.device)
    sim_cfg.gravity = G_TRANSFORM
    
    # Optimize physics settings for memory usage
    sim_cfg.physx.solver_type = 1  # Use TGS solver (less memory intensive)
    sim_cfg.physx.min_position_iteration_count = 1
    sim_cfg.physx.max_position_iteration_count = 4
    sim_cfg.physx.min_velocity_iteration_count = 0
    sim_cfg.physx.max_velocity_iteration_count = 1
    
    # Reduce GPU memory usage
    sim_cfg.physx.gpu_max_rigid_contact_count = 1024 * 1024  # Reduce from default
    sim_cfg.physx.gpu_max_rigid_patch_count = 80 * 1024     # Reduce from default
    sim_cfg.physx.gpu_heap_capacity = 64 * 1024 * 1024      # Reduce from default
    sim_cfg.physx.gpu_temp_buffer_capacity = 16 * 1024 * 1024  # Reduce from default

    sim = SimulationContext(sim_cfg)

    # Set main camera
    sim.set_camera_view([2.5, 0.0, 4.0], [0.0, 0.0, 2.0])

    # Design scene with reduced environment spacing to save memory
    scene_cfg = NonPlanarSceneCfg(num_envs=args_cli.num_envs, env_spacing=2.0, replicate_physics=False)
    scene = InteractiveScene(scene_cfg)

    # Play the simulator
    sim.reset()

    # Now we are ready!
    print("[INFO]: Setup complete...")

    # Run the simulator
    run_simulator(sim, scene)

    print("[INFO]: Closing the simulator...")

if __name__ == "__main__":
    # run the main execution
    main()
    
    # close sim app
    simulation_app.close()




#./isaaclab.sh -p ~/ros2_ws/src/f1tenth-offroad-livox/07_sim/isaac_sim_roboracer/roboracer_assets/test/create_robot_scene.py --num_envs 5
