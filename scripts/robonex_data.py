import json
import os

from robonex_common.joints import ALL_MOTORS, AUXILIARY_JOINTS
from robonex_common.motors import MOTOR_PHYSICS

from model_io import CONSTANTS_PATH as _CONST_PATH


DEG = 3.141592653589793 / 180.0

CONSTANTS = json.load(open(_CONST_PATH)) if os.path.exists(_CONST_PATH) else {}
SPAWN_HEIGHT = CONSTANTS.get("zero_pose_base_height", 0.95)
MUJOCO_SPAWN_HEIGHT = CONSTANTS.get("mujoco_spawn_height", 0.956)
HOME_HEIGHT = CONSTANTS.get("home_base_height")
DEFAULT_JOINT_POS = CONSTANTS.get("default_actuated_pos", {})
HOME_PASSIVE_JOINT_POS = CONSTANTS.get("home_passive_pos", {})
UPPER_BODY_JOINTS = tuple(joint.model_name for joint in AUXILIARY_JOINTS)
UPPER_BODY_DEFAULT_POS = CONSTANTS.get("upper_body_default_pos", {})

MOTOR_MODEL_BY_JOINT = {joint.model_name: joint.motor_model for joint in ALL_MOTORS}


def motor_physics_for(joint_name):
    return MOTOR_PHYSICS.get(MOTOR_MODEL_BY_JOINT.get(joint_name))


def actuated_joints(model_joints, loop_actuated):
    return [name for name in (*loop_actuated, *UPPER_BODY_JOINTS) if name in model_joints]


COLLISION_BOX = CONSTANTS.get("collision_box", {})

FEET = ("l_foot", "r_foot")

FOOT_FRICTION = 0.6
