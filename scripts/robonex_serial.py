DEG = 3.141592653589793 / 180.0

SPAWN_HEIGHT = 1.0789
MUJOCO_SPAWN_HEIGHT = 1.085

# What the real RobStride motors are driven with, in MIT mode, on the bench
# (Robstride-Motor-Test/scripts/motor_run/motor_pose_run.py: HOLD_KP/HOLD_KD).
# Their position feedback is the output shaft, so these are N.m/rad and
# N.m.s/rad at the joint the motor turns - directly comparable to MuJoCo's
# <position kp kv>, which sits on those same crank joints.
MOTOR_KP = 40.0
MOTOR_KD = 2.0

# The serial-equivalent model actuates the joints the linkages DRIVE, not the
# crank joints the motors actually turn, so the gains have to come through the
# transmission with them. Torque divides by the ratio r = d(joint)/d(motor) and
# so does the angle, hence stiffness scales by 1/r^2:
#
#     k_joint = n_motors * MOTOR_KP / r^2
#
# r measured off the MuJoCo model (gazebo/README.md has the sweep): knee 0.70 at
# the home pose, each ankle motor ~1.0 into roll and pitch, and the ankle is
# driven by two motors at once. Same convention as the effort limits below,
# where the ankle carries 2x17 N.m and the knee 60/0.70.
#
# The knee ratio is only 0.70 at the home pose - it runs 0.21..1.23 across the
# travel - so its gain is as configuration-dependent as its effort limit, and
# just as approximate.
SERIAL_GAINS = {
    "l_hip_yaw_joint":    (40.0, 2.00),
    "r_hip_yaw_joint":    (40.0, 2.00),
    "l_hip_pitch_joint":  (40.0, 2.00),
    "r_hip_pitch_joint":  (40.0, 2.00),
    "l_hip_roll_joint":   (40.0, 2.00),
    "r_hip_roll_joint":   (40.0, 2.00),
    "l_knee_joint":       (80.5, 4.02),
    "r_knee_joint":       (80.5, 4.02),
    "l_ankle_roll_joint":  (77.7, 3.88),
    "r_ankle_roll_joint":  (77.7, 3.88),
    "l_ankle_pitch_joint": (79.8, 3.99),
    "r_ankle_pitch_joint": (79.8, 3.99),
}

DAMPING = 0.2

# effort (N.m), velocity (rad/s), lower (deg), upper (deg).
#
# Hips are direct drive, so they carry the motor's own numbers and the ranges
# measured by hand on the real leg.
#
# Knee and ankle are outputs of a linkage, so everything comes through the
# transmission ratio r = d(joint)/d(motor), measured off this model:
#     effort   = n_motors * motor_peak / r
#     velocity = motor_no_load * r
#     range    = swept, driving the crank actuators to their own limits
# r is 0.705 for the knee at the home pose (it runs 0.21..1.23 across the
# travel, so knee effort/velocity are only right near home) and ~1.01 into
# ankle roll, ~1.00 into ankle pitch, with two RS02s driving each ankle axis.
#
# CAVEAT: the ankle ranges below are set by the rod-end stops, because the
# ankle CRANK limits are still the URDF's +/-180 placeholder. Once those are
# measured they may bind first and shrink these - re-run the sweep then.
SERIAL_JOINTS = {
    "l_hip_yaw_joint":    (17.0, 42.9,  -40.0,  40.0),
    "r_hip_yaw_joint":    (17.0, 42.9,  -40.0,  40.0),
    "l_hip_pitch_joint":  (60.0, 20.9,  -50.0,  50.0),
    "r_hip_pitch_joint":  (60.0, 20.9,  -50.0,  50.0),
    "l_hip_roll_joint":   (60.0, 20.9,  -60.0,   5.0),
    "r_hip_roll_joint":   (60.0, 20.9,   -5.0,  60.0),
    "l_knee_joint":        (85.1, 14.7,  -42.0,   3.4),
    "r_knee_joint":        (85.1, 14.7,   -3.4,  42.0),
    "l_ankle_roll_joint":  (33.5, 43.5,  -15.6,  14.8),
    "r_ankle_roll_joint":  (33.5, 43.5,  -14.8,  15.6),
    "l_ankle_pitch_joint": (34.0, 42.9,  -19.3,  16.3),
    "r_ankle_pitch_joint": (34.0, 42.9,  -19.3,  16.3),
}

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

FOOT_FRICTION = 1.0
BODY_FRICTION = 1.0

TIMESTEP = 0.001

PACKAGE = "robonex_description"
