# RoboNex URDF

## Summary

| | |
|---|---|
| Total mass | 20.513910 kg |
| Links | 25 |
| Joints | 24 (20 revolute, 4 fixed) |
| Actuated joints | 12 |
| Passive joints | 8 |

## Motor specifications:

| | RS02 | RS03 |
|---|---|---|
| Rated / peak torque | 6 / 17 N·m | 20 / 60 N·m |
| No-load speed | 410 rpm (42.9 rad/s) | 200 rpm (20.9 rad/s) |
| Gear ratio | 7.75 : 1 | 9 : 1 |
| Torque constant | 1.22 N·m/Arms | 2.36 N·m/Arms |
| Rated / peak phase current | 7 / 23 Apk | 13 / 43 Apk |
| Voltage | 48 VDC (24–60) | 48 VDC (24–60) |
| Mass | 380 g ±3 | 880 g ±20 |
| Encoder | 14-bit absolute | 14-bit absolute |

## Link masses

| Link | Mass (kg) | Centre of mass (m) | Ixx | Iyy | Izz |
|---|---:|---|---:|---:|---:|
| `base_link` | 6.80147 | 0.032843, 0.000297, −0.045593 | 0.176944 | 0.148721 | 0.087102 |
| `l_hip_yaw_link` | 0.97680 | −0.000038, 0.023069, −0.059394 | 0.001085 | 0.001442 | 0.001013 |
| `r_hip_yaw_link` | 0.97680 | 0.000038, −0.023069, −0.059394 | 0.001085 | 0.001442 | 0.001013 |
| `l_hip_pitch_link` | 1.04436 | −0.003785, 0.060976, −0.032577 | 0.001614 | 0.001239 | 0.001181 |
| `r_hip_pitch_link` | 1.04436 | −0.003774, −0.061283, −0.032564 | 0.001616 | 0.001239 | 0.001182 |
| `l_hip_roll_link` | 1.71574 | −0.034473, −0.032586, −0.198268 | 0.018321 | 0.017112 | 0.004122 |
| `r_hip_roll_link` | 1.71574 | −0.034561, 0.032600, −0.198312 | 0.018327 | 0.017121 | 0.004119 |
| `l_knee_crank_link` | 0.09538 | 0.013738, 0.029543, 0.030625 | 0.000108 | 0.000119 | 0.000047 |
| `r_knee_crank_link` | 0.09538 | 0.013738, −0.029543, 0.030625 | 0.000108 | 0.000119 | 0.000047 |
| `l_knee_coupler_link` | 0.05790 | 0.023792, 0.010000, −0.062632 | 0.000161 | 0.000186 | 0.000029 |
| `r_knee_coupler_link` | 0.05790 | 0.023792, −0.010000, −0.062632 | 0.000161 | 0.000186 | 0.000029 |
| `l_knee_link` | 1.71687 | 0.001621, 0.025016, −0.172019 | 0.017178 | 0.019528 | 0.002923 |
| `r_knee_link` | 1.71687 | 0.001621, −0.025013, −0.172019 | 0.017175 | 0.019525 | 0.002923 |
| `l_ankle_crank_link_a` | 0.08456 | 0.021493, 0.024513, 0.004270 | 0.000020 | 0.000052 | 0.000060 |
| `l_ankle_crank_link_b` | 0.08456 | 0.021493, −0.024513, 0.004270 | 0.000020 | 0.000052 | 0.000060 |
| `r_ankle_crank_link_a` | 0.08456 | 0.021493, 0.024513, 0.004270 | 0.000020 | 0.000052 | 0.000060 |
| `r_ankle_crank_link_b` | 0.08456 | 0.021493, −0.024513, 0.004270 | 0.000020 | 0.000052 | 0.000060 |
| `l_ankle_coupler_link_a` | 0.07410 | 0, 0, −0.073500 | 0.000121 | 0.000121 | 0.000007 |
| `r_ankle_coupler_link_a` | 0.07410 | 0, 0, −0.073500 | 0.000121 | 0.000121 | 0.000007 |
| `l_ankle_coupler_link_b` | 0.08512 | 0, 0, −0.129000 | 0.000660 | 0.000660 | 0.000007 |
| `r_ankle_coupler_link_b` | 0.08512 | 0, 0, −0.129000 | 0.000660 | 0.000660 | 0.000007 |
| `l_ankle_link` | 0.13119 | −0.020126, 0, −0.025484 | 0.000089 | 0.000089 | 0.000033 |
| `r_ankle_link` | 0.13119 | −0.020126, 0, −0.025484 | 0.000089 | 0.000089 | 0.000033 |
| `l_foot` | 0.78964 | 0.036405, 0.020073, −0.039754 | 0.001441 | 0.004139 | 0.005020 |
| `r_foot` | 0.78964 | 0.036405, 0.021573, −0.039754 | 0.001441 | 0.004139 | 0.005020 |

## Actuated joints

| Joint | Motor | Parent | Axis | Effort (N·m) | Velocity (rad/s) | Min (°) | Max (°) |
|---|---|---|---|---:|---:|---:|---:|
| `l_hip_yaw_joint` | RS02 | `base_link` | 0 0 1 | 17.0 | 42.9 | | |
| `r_hip_yaw_joint` | RS02 | `base_link` | 0 0 1 | 17.0 | 42.9 | | |
| `l_hip_pitch_joint` | RS03 | `l_hip_yaw_link` | 0 −1 0 | 60.0 | 20.9 | | |
| `r_hip_pitch_joint` | RS03 | `r_hip_yaw_link` | 0 −1 0 | 60.0 | 20.9 | | |
| `l_hip_roll_joint` | RS03 | `l_hip_pitch_link` | 1 0 0 | 60.0 | 20.9 | | |
| `r_hip_roll_joint` | RS03 | `r_hip_pitch_link` | 1 0 0 | 60.0 | 20.9 | | |
| `l_knee_pitch_joint` | RS03 | `l_hip_roll_link` | 0 1 0 | 60.0 | 20.9 | | |
| `r_knee_pitch_joint` | RS03 | `r_hip_roll_link` | 0 −1 0 | 60.0 | 20.9 | | |
| `l_ankle_upper_joint` | RS02 | `l_knee_link` | 0 1 0 | 17.0 | 42.9 | | |
| `l_ankle_lower_joint` | RS02 | `l_knee_link` | 0 −1 0 | 17.0 | 42.9 | | |
| `r_ankle_lower_joint` | RS02 | `r_knee_link` | 0 1 0 | 17.0 | 42.9 | | |
| `r_ankle_upper_joint` | RS02 | `r_knee_link` | 0 −1 0 | 17.0 | 42.9 | | |

## Passive joints

| Joint | Parent → Child | Driven by |
|---|---|---|
| `l_knee_joint` | `l_hip_roll_link` → `l_knee_link` | knee four-bar |
| `r_knee_joint` | `r_hip_roll_link` → `r_knee_link` | knee four-bar |
| `l_knee_coupler_joint_a` | `l_knee_crank_link` → `l_knee_coupler_link` | knee four-bar |
| `r_knee_coupler_joint_a` | `r_knee_crank_link` → `r_knee_coupler_link` | knee four-bar |
| `l_ankle_roll_joint` | `l_knee_link` → `l_ankle_link` | ankle differential (roll) |
| `r_ankle_roll_joint` | `r_knee_link` → `r_ankle_link` | ankle differential (roll) |
| `l_ankle_pitch_joint` | `l_ankle_link` → `l_foot` | ankle differential (pitch) |
| `r_ankle_pitch_joint` | `r_ankle_link` → `r_foot` | ankle differential (pitch) |