import isaaclab.sim as sim_utils
from isaaclab.actuators import ImplicitActuatorCfg, DCMotorCfg
from isaaclab.assets import ArticulationCfg

from . import ROBORACER_ASSETS_DATA_DIR

# 4WD actuator configuration [TODO: NEED TO BE CONFIGURED TO F1TENTH]
ROBORACER_LEATHERBACK_4WD_ACTUATOR_CFG = {
    "steering_joints": ImplicitActuatorCfg(
        joint_names_expr=["Knuckle__Upright__Front_Left", 
                          "Knuckle__Upright__Front_Right"], 
        velocity_limit=10.0,
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
        stiffness=5e6,       # Lower stiffness for a more springy suspension
        damping=0.0,         # Lower shock oil weight implies minimal damping
        friction=0.0,
    ),
}

# Initial state configuration
_ZERO_INIT_STATES = ArticulationCfg.InitialStateCfg(
    pos=(0.6, 0.0, 0.0),
    joint_pos={
        "Knuckle__Upright__Front_Left": 0.6,
        "Knuckle__Upright__Front_Right": 0.6,
        "Wheel__Upright__Rear_Left": 0.0,
        "Wheel__Upright__Rear_Right": 0.0,
        "Wheel__Knuckle__Front_Left": 0.0,
        "Wheel__Knuckle__Front_Right": 0.0,
        "Shock__Front_Right": 0.03,
        "Shock__Front_Left": 0.03,
        "Shock__Rear_Right": -0.03,
        "Shock__Rear_Left": -0.03,
    },
    joint_vel={
        "Wheel__Upright__Rear_Left": 2.0,
        "Wheel__Upright__Rear_Right": 2.0,
        "Wheel__Knuckle__Front_Left": 2.0,
        "Wheel__Knuckle__Front_Right": 2.0,
    },
)

# Overall configuration tying together the asset, physics, and initial state
ROBORACER_LEATHERBACK_CFG = ArticulationCfg(
    prim_path="/World/envs/env_.*/Vehicle",
    spawn=sim_utils.UsdFileCfg(
        # usd_path=USD_PATH,
        usd_path=f"{ROBORACER_ASSETS_DATA_DIR}/vehicles/leatherback.usd",
        rigid_props=sim_utils.RigidBodyPropertiesCfg(
            # TODO: Explore the paramaeters and experiment
            rigid_body_enabled=True,
            disable_gravity=False,
            linear_damping=0.0,
            angular_damping=0.0,
            max_linear_velocity=1000.0,
            max_angular_velocity=1000.0,
            max_depenetration_velocity=1.0,  # TODO: need to experiment
            enable_gyroscopic_forces=True,  # TODO: need to experiment
            retain_accelerations=True,  # TODO: need to experiment
        ),
        articulation_props=sim_utils.ArticulationRootPropertiesCfg(
            enabled_self_collisions=False,
            solver_position_iteration_count=4,
            solver_velocity_iteration_count=0,
        ),
    ),
    init_state=_ZERO_INIT_STATES,
    actuators=ROBORACER_LEATHERBACK_4WD_ACTUATOR_CFG,
)
