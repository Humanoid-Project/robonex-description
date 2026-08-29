DEG = 3.141592653589793 / 180.0

SPAWN_HEIGHT = 1.0789
MUJOCO_SPAWN_HEIGHT = 1.085

MOTOR_KP = 40.0
MOTOR_KD = 2.0

DAMPING = 0.2

MOTOR_PHYSICS = {
    "rs02": {"armature": 0.0042, "frictionloss": 0.1},
    "rs03": {"armature": 0.02,   "frictionloss": 0.2},
}

MOTOR_MODEL = {
    "l_hip_yaw_joint":     "rs02",
    "r_hip_yaw_joint":     "rs02",
    "l_ankle_upper_joint": "rs02",
    "l_ankle_lower_joint": "rs02",
    "r_ankle_upper_joint": "rs02",
    "r_ankle_lower_joint": "rs02",
    "l_hip_pitch_joint":   "rs03",
    "r_hip_pitch_joint":   "rs03",
    "l_hip_roll_joint":    "rs03",
    "r_hip_roll_joint":    "rs03",
    "l_knee_pitch_joint":  "rs03",
    "r_knee_pitch_joint":  "rs03",
}


def motor_physics_for(joint_name):
    """armature/frictionloss for a crank joint, or None if it isn't motor-driven."""
    model = MOTOR_MODEL.get(joint_name)
    return MOTOR_PHYSICS[model] if model else None


JOINT_ORDER = [
    "l_hip_yaw_joint", "l_hip_pitch_joint", "l_hip_roll_joint",
    "l_knee_joint", "l_ankle_roll_joint", "l_ankle_pitch_joint",
    "r_hip_yaw_joint", "r_hip_pitch_joint", "r_hip_roll_joint",
    "r_knee_joint", "r_ankle_roll_joint", "r_ankle_pitch_joint",
]

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
BODY_FRICTION = 1.0

TIMESTEP = 0.001

PACKAGE = "robonex_description"
