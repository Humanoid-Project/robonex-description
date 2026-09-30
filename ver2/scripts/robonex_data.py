import json
import os

from robonex_common.joints import ALL_MOTORS
from robonex_common.motors import MOTOR_PHYSICS


DEG = 3.141592653589793 / 180.0

_CONST_PATH = os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))), "ver2_constants.json")
CONSTANTS = json.load(open(_CONST_PATH)) if os.path.exists(_CONST_PATH) else {}
SPAWN_HEIGHT = CONSTANTS.get("zero_pose_base_height", 0.95)
MUJOCO_SPAWN_HEIGHT = CONSTANTS.get("mujoco_spawn_height", 0.956)
HOME_HEIGHT = CONSTANTS.get("home_base_height")
DEFAULT_JOINT_POS = CONSTANTS.get("default_actuated_pos", {})
HOME_PASSIVE_JOINT_POS = CONSTANTS.get("home_passive_pos", {})
HELD_JOINTS = (
    "neck_pitch_joint",
    "l_shoulder_pitch_joint", "l_shoulder_roll_joint", "l_shoulder_yaw_joint", "l_elbow_joint",
    "r_shoulder_pitch_joint", "r_shoulder_roll_joint", "r_shoulder_yaw_joint", "r_elbow_joint",
)

DAMPING = 0.2


MOTOR_MODEL_BY_JOINT = {joint.model_name: joint.motor_model for joint in ALL_MOTORS}


def motor_physics_for(joint_name):
    return MOTOR_PHYSICS.get(MOTOR_MODEL_BY_JOINT.get(joint_name))


JOINT_ORDER = [
    "l_hip_yaw_joint", "l_hip_pitch_joint", "l_hip_roll_joint",
    "l_knee_joint", "l_ankle_roll_joint", "l_ankle_pitch_joint",
    "r_hip_yaw_joint", "r_hip_pitch_joint", "r_hip_roll_joint",
    "r_knee_joint", "r_ankle_roll_joint", "r_ankle_pitch_joint",
]

COLLISION_BOX = CONSTANTS.get("collision_box", {})

FEET = ("l_foot", "r_foot")

FOOT_FRICTION = 0.6
FOOT_FRICTION_RANGE = (0.4, 0.8)
BODY_FRICTION = 1.0

TIMESTEP = 0.001

PACKAGE = "robonex_description"
