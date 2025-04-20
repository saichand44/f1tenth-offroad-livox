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

import isaaclab.sim as sim_utils
from isaaclab.assets import ArticulationCfg, AssetBaseCfg
from isaaclab.scene import InteractiveScene, InteractiveSceneCfg
from isaaclab.terrains import TerrainImporterCfg
from isaaclab.sim import SimulationContext
from isaaclab.utils import configclass
from isaaclab.sim.converters import MeshConverter

from roboracer_assets.leatherback import ROBORACER_LEATHERBACK_CFG
from roboracer_assets.non_planar_terrain import NON_PLANAR_TERRAIN_CFG

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
    MeshConverter(NON_PLANAR_TERRAIN_CFG)
    terrain = TerrainImporterCfg(
        collision_group = -1,
        prim_path="/World/NonPlanarSurface",
        terrain_type = "usd",
        usd_path=NON_PLANAR_TERRAIN_CFG.usd_dir + NON_PLANAR_TERRAIN_CFG.usd_file_name + ".usd",
        physics_material=sim_utils.RigidBodyMaterialCfg(
            friction_combine_mode="average",
            restitution_combine_mode="average",
            static_friction=1.0,
            dynamic_friction=1.0,
        ),
        debug_vis=False
    )

    # Articulation
    robot: ArticulationCfg = ROBORACER_LEATHERBACK_CFG.replace(prim_path="{ENV_REGEX_NS}/Vehicle")

def run_simulator(sim: sim_utils.SimulationContext, scene: InteractiveScene):
    """
    Runs the simulation loop
    """
    # Extract scene entities
    # note: we only do this here for readability.
    robot = scene["robot"]

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

            # -- joint state
            joint_pos, joint_vel = robot.data.default_joint_pos.clone(), robot.data.default_joint_vel.clone()
            robot.write_joint_state_to_sim(joint_pos, joint_vel)

            # clear internal buffers
            scene.reset()
            print("[INFO]: Resetting scene state...")

        # Apply action to robot  #TODO: experiment with random action to joint pose and joint vel
        robot.set_joint_position_target(robot.data.default_joint_pos)

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