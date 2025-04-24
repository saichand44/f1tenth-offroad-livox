import argparse

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

import isaaclab.sim as sim_utils
from isaaclab.assets import ArticulationCfg, AssetBaseCfg, RigidObjectCfg
from isaaclab.scene import InteractiveScene, InteractiveSceneCfg
from isaaclab.terrains import TerrainImporterCfg
from isaaclab.terrains.trimesh.mesh_terrains_cfg import MeshPlaneTerrainCfg
from isaaclab.sim import SimulationContext
from isaaclab.utils import configclass
from isaaclab.sim.converters import MeshConverter

from roboracer_assets.non_planar_terrain import NON_PLANAR_TERRAIN_CFG
from roboracer_assets.mushr import MUSHR_CFG

# Nominal, min. max velocities and steering for each robot
MIN_VEL, MAX_VEL = 2.0, 10.0 
MIN_STEER, MAX_STEER = -1.3, 1.3 # radians 
GROUND_PLANE_ANGLE = 30.0 # degrees

@configclass
class NonPlanarSceneCfg(InteractiveSceneCfg):
    """
    Configuration for a scene with non-planar track and the vehicle
    """

    # Ground-plane
    ground = AssetBaseCfg(prim_path="/World/defaultGroundPlane", spawn=sim_utils.GroundPlaneCfg())

    # Lights
    dome_light = AssetBaseCfg(
        prim_path="/World/Light", spawn=sim_utils.DomeLightCfg(intensity=3000.0, color=(0.75, 0.75, 0.75))
    )

    # Non-Planar track
    # MeshConverter(NON_PLANAR_TERRAIN_CFG)
    # terrain = TerrainImporterCfg(
    #     prim_path="/World/NonPlanarSurface",
    #     terrain_type = "usd",
    #     usd_path=NON_PLANAR_TERRAIN_CFG.usd_dir + NON_PLANAR_TERRAIN_CFG.usd_file_name + ".usd",

    #     # This parameter is used only when the terrain_type is “generator” or “plane”.
    #     physics_material=sim_utils.RigidBodyMaterialCfg(
    #         friction_combine_mode="average",
    #         restitution_combine_mode="average",
    #         static_friction=0.0,
    #         dynamic_friction=0.0,
    #     ),
    #     debug_vis=True
    # )

    # Non-Planar track

    # Articulation
    robot: ArticulationCfg = MUSHR_CFG.replace(prim_path="{ENV_REGEX_NS}/Vehicle")

def generate_target_velocity(min_vel, max_vel, num_envs, num_joints):
    """
    Generate a random target velocity for each robot within the given range.
    """
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

def get_gravity_vec(angle_in_deg):
    """
    Compute the new gravity vector that corresponds to the rotation of ground plane
    by "angle" in deg in positive Y axis
    """
    angle_in_rad = np.deg2rad(angle_in_deg)
    print(f"angle_in_rad: {angle_in_rad}")
    g_original = np.array([0.0, 0.0, -9.81]).reshape(3, 1)
    Ry = np.array([[np.cos(angle_in_rad), 0, np.sin(angle_in_rad)],
                   [0, 1, 0],
                   [-np.sin(angle_in_rad), 0, np.cos(angle_in_rad)]])
    print(f"g_original:{g_original}")
    print(f"Ry:{Ry}")
    g_transform = Ry @ g_original
    g_transform = g_transform.flatten()

    return (g_transform[0], g_transform[1], g_transform[2])

def run_simulator(sim: sim_utils.SimulationContext, scene: InteractiveScene):
    """
    Runs the simulation loop
    """
    # Extract scene entities
    # note: we only do this here for readability.
    robot = scene["robot"]
    
    throttle_ids = robot.find_joints(".*_throttle")[0]
    steer_ids    = robot.find_joints(".*_steer")[0]

    # Define simulation stepping
    sim_dt = sim.get_physics_dt()
    count = 0

    # Simulation loop
    while simulation_app.is_running():
        # Reset    
        if count % 2500 == 0:
            # reset counter
            count = 0

            # robot
            # -- root state
            root_state = robot.data.default_root_state.clone()
            root_state[:, :3] += scene.env_origins

            robot.write_root_pose_to_sim(root_state[:, :7])
            robot.write_root_velocity_to_sim(root_state[:, 7:])

            # 1) Throttle: spin all throttle joints at full forward
            target_velocities = generate_target_velocity(MIN_VEL, MAX_VEL, scene.num_envs, len(throttle_ids))
            target_velocities = target_velocities.to(sim.device)
            print(f"target_velocities:\n{target_velocities}")
            robot.set_joint_velocity_target(target_velocities,
                                            joint_ids=throttle_ids)

            # 2) Steering: Apply steering
            target_steerings = generate_target_steering(MIN_STEER, MAX_STEER, scene.num_envs, len(steer_ids))
            target_steerings = target_steerings.to(sim.device)
            print(f"target_steerings:\n{target_steerings}")
            robot.set_joint_position_target(target_steerings,
                                            joint_ids=steer_ids)
            
            # clear internal buffers
            scene.reset()
            print("[INFO]: Resetting scene state...")

        # Write data to sim
        scene.write_data_to_sim()

        # Perform step
        sim.step()

        # Increment counter
        count += 1
        
        # Update buffers
        scene.update(sim_dt)

def main():
    """
    Main function.
    """

    # Load kit helper
    sim_cfg = sim_utils.SimulationCfg(device=args_cli.device)
    g_transform = get_gravity_vec(GROUND_PLANE_ANGLE)
    print(f"g_transform:\n {g_transform}")
    sim_cfg.gravity = g_transform

    sim = SimulationContext(sim_cfg)

    # Set main camera
    sim.set_camera_view([2.5, 0.0, 4.0], [0.0, 0.0, 2.0])

    # Design scene
    scene_cfg = NonPlanarSceneCfg(num_envs=args_cli.num_envs, env_spacing=2.0, replicate_physics=False)
    scene = InteractiveScene(scene_cfg)

    # Play the simulator
    sim.reset()

    # Now we are ready!
    print("[INFO]: Setup complete...")

    # Run the simulator
    run_simulator(sim, scene)

if __name__ == "__main__":
    # run the main execution
    main()
    # close sim app
    simulation_app.close()