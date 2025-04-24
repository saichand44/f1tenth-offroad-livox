import os
import isaaclab.sim as sim_utils
from isaaclab.assets import RigidObjectCfg
from isaaclab.sim.converters import MeshConverter, MeshConverterCfg
from isaaclab.sim.schemas import schemas_cfg

from . import ROBORACER_ASSETS_DATA_DIR

# Overall configuration for mesh converter
# non_planar_terrain_path = f"{ROBORACER_ASSETS_DATA_DIR}/terrains/bump_track.obj"
non_planar_terrain_path = f"{ROBORACER_ASSETS_DATA_DIR}/terrains/on_camber_banked_track.obj"
usd_filename = non_planar_terrain_path.split("/")[-1].split(".")[0]
dest_usd_path = f"{ROBORACER_ASSETS_DATA_DIR}/terrains/{usd_filename}/"

# NON_PLANAR_TERRAIN_CFG = MeshConverterCfg(
#     mass_props=None,
#     rigid_props=None,
#     collision_props=sim_utils.CollisionPropertiesCfg(collision_enabled=True),
#     asset_path=non_planar_terrain_path,
#     force_usd_conversion=True,
#     usd_dir=dest_usd_path,
#     usd_file_name=usd_filename,
#     make_instanceable=True,
#     collision_approximation="meshSimplification",
#     translation=(0.0, 0.0, 0.0),
#     rotation=(0.5, 0.5, -0.5, -0.5),
#     scale=(1.0, 1.0, 1.0),
# )

NON_PLANAR_TERRAIN_CFG = MeshConverterCfg(
    mass_props=None,
    rigid_props=None,
    collision_props=sim_utils.CollisionPropertiesCfg(collision_enabled=True),
    asset_path=non_planar_terrain_path,
    force_usd_conversion=True,
    usd_dir=dest_usd_path,
    usd_file_name=usd_filename,
    make_instanceable=False,
    collision_approximation="meshSimplification",
    translation=(0.0, 0.0, 0.0),
    rotation=(0.5, 0.5, -0.5, -0.5),
    scale=(1.0, 1.0, 1.0),
)

# RIGID_TERRAIN_CFG: RigidObjectCfg = RigidObjectCfg(
#     prim_path="/World/envs/env_.*/NonPlanarSurface",
#     spawn=sim_utils.UsdFileCfg(
#         usd_path=NON_PLANAR_TERRAIN_CFG.usd_dir + NON_PLANAR_TERRAIN_CFG.usd_file_name + ".usd",
#         rigid_props=sim_utils.RigidBodyPropertiesCfg(
#             rigid_body_enabled=True,
#         ),
#         collision_props=sim_utils.CollisionPropertiesCfg(
#             collision_enabled=True,
#         )
#     ),
#     collision_group=0,
#     debug_vis=False
# )