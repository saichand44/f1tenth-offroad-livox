import os
import isaaclab.sim as sim_utils
from isaaclab.sim.converters import MeshConverter, MeshConverterCfg
from isaaclab.sim.schemas import schemas_cfg

from . import ROBORACER_ASSETS_DATA_DIR

# Overall configuration for mesh converter
non_planar_terrain_path = f"{ROBORACER_ASSETS_DATA_DIR}/terrains/bump_track.obj"
usd_filename = non_planar_terrain_path.split("/")[-1].split(".")[0]
dest_usd_path = f"{ROBORACER_ASSETS_DATA_DIR}/terrains/"

NON_PLANAR_TERRAIN_CFG = MeshConverterCfg(
    mass_props=None,
    rigid_props=None,
    collision_props=sim_utils.CollisionPropertiesCfg(collision_enabled=True),
    asset_path=non_planar_terrain_path,
    force_usd_conversion=True,
    usd_dir=dest_usd_path,
    usd_file_name=usd_filename,
    make_instanceable=True,
    collision_approximation="convexDecomposition",
)