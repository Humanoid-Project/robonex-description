import json
import os

from robonex_common.actuators import CONTROL_GAINS_BY_JOINT
from robonex_common.joints import MOTOR_BY_ID
from robonex_common.motors import MOTOR_PHYSICS

from model_io import CONSTANTS_PATH, VARIANT

SIDES = (("left", "l"), ("right", "r"))
LEG_ROLES = ("hip_yaw", "hip_pitch", "hip_roll", "knee_pitch", "ankle_upper", "ankle_lower")
ARM_ROLES = ("shoulder_pitch", "shoulder_roll", "shoulder_yaw", "elbow", "wrist_roll")
COMMON_ROLE = {"wrist_roll": "wrist_yaw"}

MOTOR_IDS = {}
for _side, _ in SIDES:
    _base = 0 if _side == "left" else 6
    for _i, _role in enumerate(LEG_ROLES):
        MOTOR_IDS["%s_%s_joint" % (_side, _role)] = _base + 1 + _i
    _base = 16 if _side == "left" else 21
    for _i, _role in enumerate(ARM_ROLES):
        MOTOR_IDS["%s_%s_joint" % (_side, _role)] = _base + _i
MOTOR_IDS["neck_yaw_joint"] = 26
MOTOR_IDS["neck_pitch_joint"] = 27


def common_name(name):
    for side, short in SIDES:
        prefix = side + "_"
        if name.startswith(prefix):
            role = name[len(prefix):-len("_joint")]
            return "%s_%s_joint" % (short, COMMON_ROLE.get(role, role))
    return name


for _name, _motor_id in MOTOR_IDS.items():
    _spec = MOTOR_BY_ID[_motor_id]
    if _spec.model_name != common_name(_name):
        raise SystemExit("ver3 joint %s -> motor ID %d, but robonex-common names ID %d %s"
                         % (_name, _motor_id, _motor_id, _spec.model_name))

LEG_JOINTS = tuple(name for name, motor_id in sorted(MOTOR_IDS.items(), key=lambda kv: kv[1]) if motor_id <= 12)
UPPER_BODY_JOINTS = tuple(name for name, motor_id in sorted(MOTOR_IDS.items(), key=lambda kv: kv[1]) if motor_id > 12)
FIXED_ZERO_BY_VARIANT = {"full": frozenset(), "legs": frozenset(UPPER_BODY_JOINTS)}
FIXED_ZERO_JOINTS = FIXED_ZERO_BY_VARIANT[VARIANT]


def motor_spec(name):
    return MOTOR_BY_ID[MOTOR_IDS[name]]


def control_gains(name):
    return CONTROL_GAINS_BY_JOINT[common_name(name)]


def motor_physics_for(name):
    if name not in MOTOR_IDS:
        return None
    return MOTOR_PHYSICS.get(motor_spec(name).motor_model)


def common_limits():
    return {name: [motor_spec(name).lower, motor_spec(name).upper] for name in MOTOR_IDS}


CONSTANTS = json.load(open(CONSTANTS_PATH)) if os.path.exists(CONSTANTS_PATH) else {}
SPAWN_HEIGHT = CONSTANTS.get("zero_pose_base_height", 0.95)
MUJOCO_SPAWN_HEIGHT = CONSTANTS.get("mujoco_spawn_height", 0.956)
HOME_HEIGHT = CONSTANTS.get("home_base_height")
DEFAULT_JOINT_POS = CONSTANTS.get("default_actuated_pos", {})
HOME_PASSIVE_JOINT_POS = CONSTANTS.get("home_passive_pos", {})
UPPER_BODY_DEFAULT_POS = CONSTANTS.get("upper_body_default_pos", {})
COLLISION_BOX = CONSTANTS.get("collision_box", {})

FEET = ("left_ankle_pitch_link", "right_ankle_pitch_link")
FOOT_FRICTION = 0.6
