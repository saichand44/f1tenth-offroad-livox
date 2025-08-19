import argparse
import os

from isaaclab.app import AppLauncher

# create argparser
parser = argparse.ArgumentParser(description="Creating a non-planar track scene with vehicle")
parser.add_argument("--num_envs", type=int, default=1, help="Number of environments to spawn.")

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
from roboracer_assets.utils import Data

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

    # Lights
    dome_light = AssetBaseCfg(
        prim_path="/World/Light", 
        spawn=sim_utils.DomeLightCfg(intensity=3000.0, color=(0.75, 0.75, 0.75))
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


def run_simulator(sim: sim_utils.SimulationContext, scene: InteractiveScene):
    """
    Runs the simulation loop
    """
    # Extract scene entities
    robot = scene["robot"]
    throttle_ids = robot.find_joints(".*_throttle")[0]
    steer_ids    = robot.find_joints(".*_steer")[0]

    # Define simulation stepping
    sim_dt = sim.get_physics_dt()
    count = 0

    # initialize the data to record
    data = Data('timestamps',   # simulation timestamp
                
                # related to the environment and robot
                'ground_plane_inclination', # gound plane inclination in deg
                'g_original',   # gravity vector in ground inertial frame   
                'g_transform',  # gravity vector in incline inertial frame
                'g_R_p',    # rotation matrix from ground inertial to incline inertial
                'joint_names',  # joint names of implicit actuators
                'body_names',  # body names of articulations

                # related to the robots
                'target_velocity',    # target velocity to vehicle TODO: current input is rpm, need to change to car speed
                'target_steering',    # targte steering input to vehicle

                # related to robots and timestamps
                'root_pose',    # pose of the vehicle (position and quaternion orientation in (w, x, y, z))
                'root_velocity',    # pose of the vehicle (linear velocity (x, y, z) and angular velocity (x, y, z))
                'joint_velocity',   # joint velocities of implicit actuators
                'root_acceleration',    # linear acc of the vehicle (x, y, z)
                )

    data.ground_plane_inclination.append(GROUND_PLANE_ANGLE)
    data.g_original.append(G_ORIGINAL)
    data.g_transform.append(G_TRANSFORM)
    data.g_R_p.append(G_R_P)
    data.body_names.append(robot.data.body_names)
    data.joint_names.append(robot.joint_names)

    # --- Initialize current velocity and steering angle at first reset ---
    current_velocity = None
    current_steering = None
    desired_accelerations = []
    desired_steering_velocities = []

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
            desired_acceleration = torch.empty_like(current_velocity).uniform_(-1.0, 1.0)  # m/s^2
            desired_steer_vel   = torch.empty_like(current_steering).uniform_(-0.3, 0.3)
            # clear internal buffers
            scene.reset()
            print("[INFO]: Resetting scene state...")

        # --- Randomize desired acceleration and steering velocity (same shape as current) ---
           # rad/s
        desired_acceleration = torch.empty_like(current_velocity).uniform_(-1.0, 1.0)  # m/s^2
        desired_steer_vel = torch.empty_like(current_steering).uniform_(-0.3, 0.3)    # rad/s

        desired_accelerations.append(desired_acceleration.cpu().numpy())
        desired_steering_velocities.append(desired_steer_vel.cpu().numpy())


        # --- Euler integration for velocity and steering angle ---
        current_velocity = current_velocity + desired_acceleration * sim_dt
        current_velocity = torch.clamp(current_velocity, MIN_VEL, MAX_VEL)

        current_steering = current_steering + desired_steer_vel * sim_dt
        current_steering = torch.clamp(current_steering, MIN_STEER, MAX_STEER)

        # --- Use these as control outputs ---
        target_velocity = current_velocity.to(sim.device)
        target_rpms = target_velocity / (WHEEL_DIAMETER * 0.5)  # w = v / r
        print(target_velocity.cpu().numpy().shape)
        data.target_velocity.append(target_velocity.cpu().numpy())
        
        robot.set_joint_velocity_target(target_rpms, joint_ids=throttle_ids)

        target_steering = current_steering.to(sim.device)
        data.target_steering.append(target_steering.cpu().numpy())
        robot.set_joint_position_target(target_steering, joint_ids=steer_ids)

        # Write data to sim
        scene.write_data_to_sim()

        # Perform step
        sim.step()
        count += 1
        scene.update(sim_dt)

        data.root_pose.append(scene.get_state()['articulation']['robot']['root_pose'])
        data.root_velocity.append(scene.get_state()['articulation']['robot']['root_velocity'])
        data.joint_velocity.append(scene.get_state()['articulation']['robot']['joint_velocity'])
        data.timestamps.append(sim.current_time)
        data.root_acceleration.append(robot.data.body_lin_acc_w)

        if (count == MAX_COUNT):
            break

    # save the data
    save_dir = SAVE_DIR
    filename = 'data_record5'
    data.unpack_and_save(num_robots = args_cli.num_envs, save_dir=save_dir, filename=filename)

    # Save raw control data directly (workaround)
    print("[INFO]: Saving raw control data directly...")
    np.savez(os.path.join(save_dir, 'raw_control_data2.npz'), 
             velocity=np.array(data.target_velocity), 
             steering=np.array(data.target_steering),
             acceleration=np.array(desired_accelerations),
             steering_velocity=np.array(desired_steering_velocities),
             timestamps=np.array(data.timestamps))
    print(f"[INFO]: Raw control data saved to {os.path.join(save_dir, 'raw_control_data.npz')}")

    # --- Plotting the inputs over time for each robot ---
    # Convert lists to numpy arrays for plotting
    target_velocity_arr = np.array(data.target_velocity)  # shape: (timesteps, num_envs, num_joints)
    target_steering_arr = np.array(data.target_steering)  # shape: (timesteps, num_envs, num_joints)
    print(f"target_velocity_arr shape: {target_velocity_arr.shape}")
    print(f"target_steering_arr shape: {target_steering_arr.shape}")
    timestamps_arr = np.array(data.timestamps)

    # Plot velocity for each robot (first joint)
    plt.figure(figsize=(12, 5))
    for env in range(target_velocity_arr.shape[1]):
        plt.plot(timestamps_arr, target_velocity_arr[:, env, 0], label=f'Robot {env+1}')
    plt.xlabel('Time [s]')
    plt.ylabel('Velocity [m/s]')
    plt.title('Velocity Input Over Time')
    plt.legend()
    plt.grid()

    # Plot steering angle for each robot (first joint)
    plt.figure(figsize=(12, 5))
    for env in range(target_steering_arr.shape[1]):
        plt.plot(timestamps_arr, target_steering_arr[:, env, 0], label=f'Robot {env+1}')
    plt.xlabel('Time [s]')
    plt.ylabel('Steering Angle [rad]')
    plt.title('Steering Angle Input Over Time')
    plt.legend()
    plt.grid()

    plt.show()
def main():
    """
    Main function.
    """

    # Load kit helper
    sim_cfg = sim_utils.SimulationCfg(device=args_cli.device)
    sim_cfg.gravity = G_TRANSFORM

    sim = SimulationContext(sim_cfg)

    # Set main camera
    sim.set_camera_view([2.5, 0.0, 4.0], [0.0, 0.0, 2.0])

    # Design scene
    scene_cfg = NonPlanarSceneCfg(num_envs=args_cli.num_envs, env_spacing=0.0, replicate_physics=False)
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