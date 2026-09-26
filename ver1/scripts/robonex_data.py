from robonex_common.joints import JOINT_BY_MODEL_NAME
from robonex_common.motors import MOTOR_PHYSICS


DEG = 3.141592653589793 / 180.0


def motor_physics_for(joint_name):
    joint = JOINT_BY_MODEL_NAME.get(joint_name)
    return MOTOR_PHYSICS[joint.motor_model] if joint else None


COLLISION_BOX = {
    "base_link":        ((0.2608, 0.3500, 0.4039), (0.0000,  0.0000, -0.0005)),
    "l_hip_yaw_link":   ((0.1150, 0.0927, 0.1180), (0.0000,  0.0013, -0.0590)),
    "r_hip_yaw_link":   ((0.1150, 0.0927, 0.1180), (0.0000, -0.0013, -0.0590)),
    "l_hip_pitch_link": ((0.0891, 0.1198, 0.1315), (-0.0046,  0.0599, -0.0258)),
    "r_hip_pitch_link": ((0.0891, 0.1202, 0.1315), (-0.0046, -0.0601, -0.0257)),
    "l_hip_roll_link":  ((0.1147, 0.1553, 0.3999), (-0.0348, -0.0110, -0.1600)),
    "r_hip_roll_link":  ((0.1147, 0.1553, 0.3999), (-0.0348,  0.0110, -0.1600)),
    "l_knee_link":      ((0.2044, 0.0500, 0.4350), (0.0027,  0.0250, -0.1525)),
    "r_knee_link":      ((0.2044, 0.0500, 0.4350), (0.0027, -0.0250, -0.1525)),
    "l_ankle_link":     ((0.0400, 0.0400, 0.0900), (-0.0200,  0.0000, -0.0250)),
    "r_ankle_link":     ((0.0400, 0.0400, 0.0900), (-0.0200,  0.0000, -0.0250)),
    "l_foot":           ((0.2014, 0.1176, 0.0200), (0.0416,  0.0200, -0.0554)),
    "r_foot":           ((0.2014, 0.1176, 0.0200), (0.0416,  0.0215, -0.0554)),
}

FEET = ("l_foot", "r_foot")

FOOT_FRICTION = 0.6
FOOT_FRICTION_RANGE = (0.4, 0.8)
