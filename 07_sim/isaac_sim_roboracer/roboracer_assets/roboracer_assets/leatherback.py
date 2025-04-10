import isaaclab.sim as sim_utils
from isaaclab.actuators import ImplicitActuatorCfg, DCMotorCfg
from isaaclab.assets import ArticulationCfg

from . import ROBORACER_ASSETS_DATA_DIR

# 4WD actuator configuration
ROBORACER_LEATHERBACK_4WD_ACTUATOR_CFG = {
    "steering_joints": ImplicitActuatorCfg(
        joint_names_expr=["Knuckle__Upright__Front_Left", 
                          "Knuckle__Upright__Front_Right"], 
        velocity_limit=10.0,    # F1Tenth steering
        effort_limit=2.5,
        stiffness=120.0,
        damping=8.0,
        friction=0.0,
    ),
    "throttle_joints": DCMotorCfg(
        joint_names_expr=["Wheel__Upright__Rear_Left",
                          "Wheel__Upright__Rear_Right",
                          "Wheel__Knuckle__Front_Left",
                          "Wheel__Knuckle__Front_Right"],  # Matches all throttle joints (all four wheels)
        saturation_effort=1.0,
        effort_limit=0.25,   # Adjusted for the 3s VXL-3s motor/ESC
        velocity_limit=400.0,  # Reduced speed compared to a 4s system
        stiffness=0.0,
        damping=1100.0,
        friction=0.0,
    ),
    "suspension": ImplicitActuatorCfg(
        joint_names_expr=["Shock__Front_Right",
                          "Shock__Front_Left",
                          "Shock__Rear_Right",
                          "Shock__Rear_Left"],  # Matches all suspension joints
        effort_limit=None,
        velocity_limit=None,
        stiffness=5e7,       # Lower stiffness for a more springy suspension
        damping=0.0,         # Lower shock oil weight implies minimal damping
        friction=0.6,
    ),
}

# Initial state configuration
_ZERO_INIT_STATES = ArticulationCfg.InitialStateCfg(
    pos=(0.0, 0.0, 0.0),
    joint_pos={
        "Knuckle__Upright__Front_Left": 0.0,
        "Knuckle__Upright__Front_Right": 0.0,
        "Wheel__Upright__Rear_Left": 0.0,
        "Wheel__Upright__Rear_Right": 0.0,
        "Wheel__Knuckle__Front_Left": 0.0,
        "Wheel__Knuckle__Front_Right": 0.0,
    },
)

# Overall configuration tying together the asset, physics, and initial state
ROBORACER_LEATHERBACK_CFG = ArticulationCfg(
    spawn=sim_utils.UsdFileCfg(
        # usd_path=USD_PATH,
        usd_path=f"{ROBORACER_ASSETS_DATA_DIR}/vehicles/leatherback.usd",
        rigid_props=sim_utils.RigidBodyPropertiesCfg(
            rigid_body_enabled=True,
            max_linear_velocity=1000.0,
            max_angular_velocity=100000.0,
            max_depenetration_velocity=100.0,
            max_contact_impulse=0.0,
            enable_gyroscopic_forces=True,
        ),
        articulation_props=sim_utils.ArticulationRootPropertiesCfg(
            enabled_self_collisions=False,
            solver_position_iteration_count=4,
            solver_velocity_iteration_count=0,
            sleep_threshold=0.005,
            stabilization_threshold=0.001,
        ),
    ),
    init_state=_ZERO_INIT_STATES,
    actuators=ROBORACER_LEATHERBACK_4WD_ACTUATOR_CFG,
)
class LeatherbackRoboracer:
    """
    Leatherback roboracer robot integration
    """
    def __init__(self, prim_path="/World/Leatherback_Roboracer", **kwargs):
        self.prim_path = prim_path
        self.cfg = ROBORACER_LEATHERBACK_CFG.replace(**kwargs)
        self.asset_path = f"{ROBORACER_ASSETS_DATA_DIR}/vehicles/leatherback.usd"

    def spawn(self):
        """Spawn the F1Tenth model into the simulation at the specified prim path."""
        return sim_utils.spawn_articulation(self.cfg, prim_path=self.prim_path)
