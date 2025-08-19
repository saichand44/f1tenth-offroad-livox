import argparse
import os

# Set environment variables to reduce GPU memory usage
os.environ["CUDA_LAUNCH_BLOCKING"] = "1"
os.environ["PYTORCH_CUDA_ALLOC_CONF"] = "max_split_size_mb:128"

from isaaclab.app import AppLauncher

# create argparser
parser = argparse.ArgumentParser(description="Creating a non-planar track scene with vehicle controlled by MPPI")
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
from isaaclab.markers import VisualizationMarkers

from roboracer_assets.mushr import MUSHR_CFG

# F1TENTH Planning imports for MPPI
from f1tenth_planning.control.controllers.mppi.dynamic_mppi import Dynamic_MPPI_Planner
from f1tenth_planning.control.config.controller_config import dynamic_mppi_config
from f1tenth_planning.control.config.dynamics_config import f1tenth_params

# Nominal, min. max velocities and steering for each robot
MIN_VEL, MAX_VEL = 0.0, 5.0 
MIN_STEER, MAX_STEER = -0.8, 0.8 # radians 
GROUND_PLANE_ANGLE = -20.0 # degrees
SAVE_DIR = os.path.dirname(os.path.abspath(__file__))
MAX_COUNT = 50000
WHEEL_DIAMETER = 0.1 # in meters

def get_gravity_vec(angle_in_deg, g_original):
    """
    Compute the new gravity vector that corresponds to the rotation of ground plane
    by "angle" in deg in positive Y axis
    """
    angle_in_rad = np.deg2rad(angle_in_deg)
    Ry = np.array([[np.cos(angle_in_rad), 0, np.sin(angle_in_rad)],
                   [0, 1, 0],
                   [-np.sin(angle_in_rad), 0, np.cos(angle_in_rad)]])
    g_transform = Ry.T @ g_original
    g_transform = g_transform.flatten()

    return (float(g_transform[0]), float(g_transform[1]), float(g_transform[2])), Ry.T

G_ORIGINAL = np.array([0.0, 0.0, -9.81]).reshape(3, 1)
G_TRANSFORM, G_R_P = get_gravity_vec(GROUND_PLANE_ANGLE, G_ORIGINAL)

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

def compute_state_variables(positions, velocities, yaw_angles, steering_angles, linear_accelerations, yaw_rate):
    """
    Compute the 7 state variables for the MPPI model in the correct format.
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

    # Stack all state variables in MPPI expected format:
    # [pose_x, pose_y, pose_theta, delta, linear_vel_x, beta, ang_vel_z]
    states = torch.stack([
        x_pos,              # pose_x
        y_pos,              # pose_y  
        yaw_angles,         # pose_theta
        steering_angles,    # delta
        velocity_magnitude, # linear_vel_x
        side_slip,          # beta
        yaw_rate           # ang_vel_z
    ], dim=1)

    return states

class SimpleRaceline:
    """Simple raceline class to replace f1tenth_gym Track functionality."""
    def __init__(self, xs, ys, yaws, vxs):
        self.xs = xs
        self.ys = ys 
        self.yaws = yaws
        self.vxs = vxs

def load_raceline_from_csv(csv_path):
    """
    Load raceline data from CSV file.
    
    Expected CSV format (with semicolon delimiter and 3 header rows):
    # s_m; x_m; y_m; psi_rad; kappa_radpm; vx_mps; ax_mps2
    """
    try:
        # Load CSV data, skipping the first 3 header rows
        data = np.loadtxt(csv_path, delimiter=';', skiprows=3)
        
        # Extract the columns we need:
        # Column 1: x_m (x position)
        # Column 2: y_m (y position)  
        # Column 3: psi_rad (yaw angle)
        # Column 5: vx_mps (velocity)
        xs = data[:, 1]    # x positions
        ys = data[:, 2]    # y positions
        yaws = data[:, 3]  # yaw angles
        vxs = data[:, 5]   # velocities
        
        return SimpleRaceline(xs, ys, yaws, vxs)
        
    except Exception as e:
        print(f"[ERROR]: Failed to load raceline from {csv_path}: {e}")
        return None

def create_waypoint_markers(sim: SimulationContext, waypoints_track, marker_scale=0.1):
    """
    Create visual markers for waypoints in Isaac Sim using primitive spheres.
    
    Args:
        sim: SimulationContext object
        waypoints_track: Track object containing raceline waypoints
        marker_scale: Scale of the waypoint markers
    
    Returns:
        None (creates primitive spheres directly)
    """
    try:
        print("[INFO]: Creating waypoint spheres using primitive method...")
        
        # Create waypoint spheres using USD primitives
        import omni.usd
        from pxr import UsdGeom, Gf
        
        stage = omni.usd.get_context().get_stage()
        
        # Create a parent prim for all waypoints
        waypoints_prim_path = "/World/Visuals/Waypoints"
        UsdGeom.Xform.Define(stage, waypoints_prim_path)
        
        # Create sphere for each waypoint (sample every 5th to avoid clutter)
        sample_rate = max(1, len(waypoints_track.raceline.xs) // 100)  # Show ~100 waypoints max
        for i in range(0, len(waypoints_track.raceline.xs), sample_rate):
            x = float(waypoints_track.raceline.xs[i])
            y = float(waypoints_track.raceline.ys[i])
            
            sphere_path = f"{waypoints_prim_path}/waypoint_{i}"
            sphere_geom = UsdGeom.Sphere.Define(stage, sphere_path)
            
            # Set sphere properties
            sphere_geom.CreateRadiusAttr(marker_scale)
            sphere_geom.AddTranslateOp().Set(Gf.Vec3d(x, y, 0.2))  # Slightly above ground
            
            # Set color (green)
            sphere_geom.CreateDisplayColorAttr().Set([Gf.Vec3f(0.0, 1.0, 0.0)])
        
        print(f"[INFO]: Created waypoint spheres for {len(waypoints_track.raceline.xs)} waypoints (showing every {sample_rate}th)")
        return None
        
    except Exception as e:
        print(f"[WARNING]: Could not create waypoint visualization: {e}")
        return None


def create_trajectory_line(sim: SimulationContext, waypoints_track):
    """
    Create a line visualization showing the trajectory path using simple spheres.
    
    Args:
        sim: SimulationContext object
        waypoints_track: Track object containing raceline waypoints
    """
    try:
        print("[INFO]: Creating trajectory line using spheres...")
        
        import omni.usd
        from pxr import UsdGeom, Gf
        
        stage = omni.usd.get_context().get_stage()
        
        # Create a parent prim for trajectory line
        line_prim_path = "/World/Visuals/TrajectoryLine"
        UsdGeom.Xform.Define(stage, line_prim_path)
        
        # Create small red spheres along the trajectory (sample every 20th point)
        sample_rate = max(1, len(waypoints_track.raceline.xs) // 200)  # Show ~200 points max
        for i in range(0, len(waypoints_track.raceline.xs), sample_rate):
            x = float(waypoints_track.raceline.xs[i])
            y = float(waypoints_track.raceline.ys[i])
            
            sphere_path = f"{line_prim_path}/line_point_{i}"
            sphere_geom = UsdGeom.Sphere.Define(stage, sphere_path)
            
            # Set small sphere properties for line effect
            sphere_geom.CreateRadiusAttr(0.03)  # Very small radius for line effect
            sphere_geom.AddTranslateOp().Set(Gf.Vec3d(x, y, 0.05))  # Close to ground
            
            # Set color (red)
            sphere_geom.CreateDisplayColorAttr().Set([Gf.Vec3f(1.0, 0.0, 0.0)])
        
        print(f"[INFO]: Created trajectory line with {len(waypoints_track.raceline.xs)//sample_rate} points")
        return None
        
    except Exception as e:
        print(f"[WARNING]: Could not create trajectory line: {e}")
        return None

def create_velocity_arrows(sim: SimulationContext, waypoints_track, arrow_scale=0.2):
    """
    Create arrow markers showing velocity direction at waypoints using oriented cylinders.
    
    Args:
        sim: SimulationContext object
        waypoints_track: Track object containing raceline waypoints
        arrow_scale: Scale of the velocity arrows
    """
    try:
        print("[INFO]: Creating velocity direction markers using cylinders...")
        
        import omni.usd
        from pxr import UsdGeom, Gf
        import numpy as np
        
        stage = omni.usd.get_context().get_stage()
        
        # Create a parent prim for velocity arrows
        arrows_prim_path = "/World/Visuals/VelocityArrows"
        UsdGeom.Xform.Define(stage, arrows_prim_path)
        
        # Sample every N waypoints to avoid clutter
        sample_rate = max(1, len(waypoints_track.raceline.xs) // 50)  # Show ~50 arrows
        
        for i in range(0, len(waypoints_track.raceline.xs), sample_rate):
            x = float(waypoints_track.raceline.xs[i])
            y = float(waypoints_track.raceline.ys[i])
            yaw = float(waypoints_track.raceline.yaws[i])
            
            # Create cylinder to represent arrow
            cylinder_path = f"{arrows_prim_path}/arrow_{i}"
            cylinder_geom = UsdGeom.Cylinder.Define(stage, cylinder_path)
            
            # Set cylinder properties
            cylinder_geom.CreateRadiusAttr(arrow_scale * 0.1)
            cylinder_geom.CreateHeightAttr(arrow_scale)
            
            # Position the cylinder
            cylinder_geom.AddTranslateOp().Set(Gf.Vec3d(x, y, 0.3))
            
            # Rotate cylinder to match velocity direction (yaw)
            # USD cylinder default is along Z-axis, we want it along velocity direction
            rotation_matrix = Gf.Matrix4d()
            rotation_matrix.SetRotate(Gf.Rotation(Gf.Vec3d(0, 0, 1), np.degrees(yaw)))
            cylinder_geom.AddTransformOp().Set(rotation_matrix)
            
            # Set color (blue)
            cylinder_geom.CreateDisplayColorAttr().Set([Gf.Vec3f(0.0, 0.0, 1.0)])
        
        print(f"[INFO]: Created {len(waypoints_track.raceline.xs)//sample_rate} velocity direction arrows")
        return None
        
    except Exception as e:
        print(f"[WARNING]: Could not create velocity arrows: {e}")
        return None

def initialize_mppi_planner():
    """
    Initialize the MPPI planner with Spielberg track waypoints.
    """
    # Load track waypoints - update this path to your trajectory_log.csv
    trajectory_path = os.path.join(os.path.dirname(__file__), "examples", "control", "trajectory_log.csv")
    
    # If the above path doesn't work, try these alternatives:
    if not os.path.exists(trajectory_path):
        trajectory_path = os.path.join(SAVE_DIR, "trajectory_log.csv")
    if not os.path.exists(trajectory_path):
        # You may need to update this path based on your file structure
        print(f"[WARNING]: Could not find trajectory_log.csv at {trajectory_path}")
        print("[WARNING]: Please update the trajectory_path in initialize_mppi_planner()")
        return None
    
    # Load raceline data from CSV
    raceline = load_raceline_from_csv(trajectory_path)
    if raceline is None:
        return None
    
    # Create a simple track-like object that the MPPI planner can use
    class SimpleTrack:
        def __init__(self, raceline):
            self.raceline = raceline
    
    waypoints_track = SimpleTrack(raceline)

    # Create MPPI planner with same configuration as the example
    config = dynamic_mppi_config()
    config.Q = np.array([25.0, 25.0, 0.0, 1.0, 0.1, 0.0, 0.0])
    
    planner = Dynamic_MPPI_Planner(
        track=waypoints_track, 
        params=f1tenth_params()
    )
    
    print(f"[INFO]: MPPI planner initialized with {len(raceline.xs)} waypoints")
    return planner, waypoints_track

def run_simulator(sim: sim_utils.SimulationContext, scene: InteractiveScene):
    """
    Runs the simulation loop with MPPI control and collects data.
    """
    # Initialize MPPI planner
    mppi_result = initialize_mppi_planner()
    if mppi_result is None:
        print("[ERROR]: Failed to initialize MPPI planner. Exiting...")
        return
    
    mppi_planner, waypoints_track = mppi_result
    
    # Create waypoint visualization
    print("[INFO]: Creating waypoint visualization...")
    waypoint_markers = create_waypoint_markers(sim, waypoints_track, marker_scale=0.05)
    trajectory_line = create_trajectory_line(sim, waypoints_track)
    velocity_arrows = create_velocity_arrows(sim, waypoints_track, arrow_scale=0.15)
    
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
    mppi_outputs_data = []
    
    # Store previous yaw angles for yaw rate computation
    prev_yaw = None

    # Initialize current velocity and steering angle
    current_velocity = torch.tensor([[1.0]], device=sim.device, dtype=torch.float32)  # Start with some initial velocity
    current_steering = torch.tensor([[0.0]], device=sim.device, dtype=torch.float32)  # Start with zero steering

    # Position robot at track start
    print(f"[INFO]: Positioning robot at track start: x={waypoints_track.raceline.xs[0]:.3f}, y={waypoints_track.raceline.ys[0]:.3f}, yaw={waypoints_track.raceline.yaws[0]:.3f}")

    # Simulation loop
    while simulation_app.is_running():
        # Reset    
        if count % MAX_COUNT == 0:
            count = 0

            # Reset robot at track starting position
            root_state = robot.data.default_root_state.clone()
            root_state[:, :3] += scene.env_origins
            
            # Set robot position to track start
            root_state[0, 0] = waypoints_track.raceline.xs[0]   # x position
            root_state[0, 1] = waypoints_track.raceline.ys[0]   # y position
            root_state[0, 2] = 0.1  # z position (slightly above ground)
            
            # Set robot orientation to track start yaw
            start_yaw = waypoints_track.raceline.yaws[0]
            quat_w = np.cos(start_yaw / 2.0)
            quat_x = 0.0
            quat_y = 0.0  
            quat_z = np.sin(start_yaw / 2.0)
            root_state[0, 3:7] = torch.tensor([quat_w, quat_x, quat_y, quat_z])

            robot.write_root_pose_to_sim(root_state[:, :7])
            robot.write_root_velocity_to_sim(root_state[:, 7:])

            # Initialize current velocity and steering angle
            current_velocity = torch.tensor([[waypoints_track.raceline.vxs[0]]], device=sim.device, dtype=torch.float32)
            current_steering = torch.tensor([[0.0]], device=sim.device, dtype=torch.float32)
            
            # Reset previous yaw
            prev_yaw = None
            
            # clear internal buffers
            scene.reset()
            print("[INFO]: Resetting scene state...")

        # --- Get current robot state ---
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

        # Compute state variables for MPPI
        states = compute_state_variables(
            root_pos, root_lin_vel, yaw_angles, current_steering[:, 0], root_lin_acc, yaw_rate
        )

        # --- MPPI Planning ---
        try:
            # Create state dictionary for MPPI (using first robot)
            mppi_state = {
                "pose_x": float(states[0, 0].cpu()),
                "pose_y": float(states[0, 1].cpu()),
                "pose_theta": float(states[0, 2].cpu()),
                "delta": float(states[0, 3].cpu()),
                "linear_vel_x": float(states[0, 4].cpu()),
                "beta": float(states[0, 5].cpu()),
                "ang_vel_z": float(states[0, 6].cpu())
            }
            
            # Plan using MPPI
            (steer_vel, acceleration), mppi_info = mppi_planner.plan(mppi_state)
            
            # Convert MPPI outputs to torch tensors with explicit float32 dtype
            desired_steer_vel = torch.tensor([[float(steer_vel)]], device=sim.device, dtype=torch.float32)
            desired_acceleration = torch.tensor([[float(acceleration)]], device=sim.device, dtype=torch.float32)
            
            print(f"[MPPI] Step {count}: pos=({mppi_state['pose_x']:.2f}, {mppi_state['pose_y']:.2f}), "
                  f"vel={mppi_state['linear_vel_x']:.2f}, steer_vel={steer_vel:.3f}, accel={acceleration:.3f}")
            
        except Exception as e:
            print(f"[ERROR] MPPI planning failed at step {count}: {e}")
            # Fallback to zero control with explicit float32 dtype
            desired_steer_vel = torch.zeros((1, 1), device=sim.device, dtype=torch.float32)
            desired_acceleration = torch.zeros((1, 1), device=sim.device, dtype=torch.float32)
            mppi_info = {}

        # --- Euler integration for velocity and steering angle ---
        current_velocity = current_velocity + desired_acceleration * sim_dt
        current_velocity = current_velocity 
        current_velocity = torch.clamp(current_velocity, MIN_VEL, MAX_VEL)

        current_steering = current_steering + desired_steer_vel * sim_dt
        current_steering = torch.clamp(current_steering, MIN_STEER, MAX_STEER)

        # --- Apply control to robot ---
        # Ensure correct datat types (float32) to match Isaac Lab expectations
        target_velocity = current_velocity.to(device=sim.device, dtype=torch.float32)
        target_rpms = target_velocity / (WHEEL_DIAMETER * 0.5)  # w = v / r
        robot.set_joint_velocity_target(target_rpms.float(), joint_ids=throttle_ids)

        target_steering = current_steering.to(device=sim.device, dtype=torch.float32)
        robot.set_joint_position_target(target_steering.float(), joint_ids=steer_ids)

        # Write data to sim
        scene.write_data_to_sim()

        # Perform step
        sim.step()
        count += 1
        scene.update(sim_dt)

        # --- Data Collection ---
        # Store data
        timestamps.append(sim.current_time)
        states_data.append(states.cpu().numpy())
        
        # Store control inputs
        inputs = torch.stack([
            desired_acceleration[:, 0],  # longitudinal acceleration
            desired_steer_vel[:, 0]      # steering rate
        ], dim=1)
        inputs_data.append(inputs.cpu().numpy())
        
        # Store MPPI specific outputs
        mppi_outputs = {
            'predicted_steering_angle': mppi_info.get('steering_angle', 0.0),
            'predicted_velocity': mppi_info.get('velocity', 0.0),
            'steering_vel_cmd': float(desired_steer_vel[0, 0].cpu()),
            'acceleration_cmd': float(desired_acceleration[0, 0].cpu())
        }
        mppi_outputs_data.append(mppi_outputs)
        
        # Update previous yaw
        prev_yaw = yaw_angles.clone()

        if (count == MAX_COUNT):
            break

    # Convert lists to numpy arrays
    timestamps_arr = np.array(timestamps)
    states_arr = np.array(states_data)  # (timesteps, num_envs, 7)
    inputs_arr = np.array(inputs_data)  # (timesteps, num_envs, 2)

    save_dir = SAVE_DIR
    filename = "isaac_sim_mppi_data"

    # Save data in .npz format
    np.savez(os.path.join(save_dir, f'{filename}.npz'),
         timestamps=timestamps_arr,
         states=states_arr,
         inputs=inputs_arr,
         mppi_outputs=mppi_outputs_data,
         ground_plane_angle=GROUND_PLANE_ANGLE,
         rotation_matrix=G_R_P,
         num_envs=scene.num_envs)
    
    print(f"[INFO]: MPPI data saved to {os.path.join(save_dir, f'{filename}.npz')}")
    
    # Print summary statistics
    print(f"[INFO]: Data collection summary:")
    print(f"  - Total timesteps: {len(timestamps_arr)}")
    print(f"  - Number of robots: {scene.num_envs}")
    print(f"  - Simulation time: {timestamps_arr[-1]:.2f}s")
    print(f"  - States shape: {states_arr.shape}")
    print(f"  - Inputs shape: {inputs_arr.shape}")
    
    # Print sample data ranges for verification
    print(f"[INFO]: State variable ranges:")
    state_names = ['pose_x', 'pose_y', 'pose_theta', 'delta', 'linear_vel_x', 'beta', 'ang_vel_z']
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
