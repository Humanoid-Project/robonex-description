import os

import mujoco
import numpy as np

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))


class Loops:
    def __init__(self, variant="edu", path=None):
        self.model = mujoco.MjModel.from_xml_path(path or os.path.join(ROOT, "mujoco", "robot", variant, "scene_fixed.xml"))
        self.model.opt.jacobian = mujoco.mjtJacobian.mjJAC_DENSE
        self.data = mujoco.MjData(self.model)
        m = self.model
        self.names = [mujoco.mj_id2name(m, mujoco.mjtObj.mjOBJ_JOINT, j) for j in range(m.njnt)]
        self.qadr = {n: m.jnt_qposadr[j] for j, n in enumerate(self.names)}
        self.dadr = {n: m.jnt_dofadr[j] for j, n in enumerate(self.names)}

    def q(self, name):
        return float(self.data.qpos[self.qadr[name]])

    def set(self, values):
        for n, v in values.items():
            self.data.qpos[self.qadr[n]] = v

    def residual_rows(self):
        return [r for r in range(self.data.nefc) if self.data.efc_type[r] == mujoco.mjtConstraint.mjCNSTR_EQUALITY]

    def solve(self, fixed, free, tol=1e-12, iters=100, damping=1e-9):
        self.set(fixed)
        cols = [self.dadr[n] for n in free]
        adr = [self.qadr[n] for n in free]
        res = np.inf
        for it in range(iters):
            mujoco.mj_forward(self.model, self.data)
            rows = self.residual_rows()
            r = np.array([self.data.efc_pos[i] for i in rows])
            res = np.abs(r).max() if len(r) else 0.0
            if res < tol:
                break
            Jm = self.data.efc_J.reshape(self.data.nefc, self.model.nv)[rows][:, cols]
            dq = -np.linalg.solve(Jm.T @ Jm + damping * np.eye(len(cols)), Jm.T @ r)
            step = np.abs(dq).max()
            if step > 0.2:
                dq *= 0.2 / step
            for a, d in zip(adr, dq):
                self.data.qpos[a] += d
        mujoco.mj_forward(self.model, self.data)
        return res, it

    def jac_cols(self, names):
        mujoco.mj_forward(self.model, self.data)
        rows = self.residual_rows()
        return self.data.efc_J.reshape(self.data.nefc, self.model.nv)[rows][:, [self.dadr[n] for n in names]]

    def body_R(self, name):
        return self.data.xmat[mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_BODY, name)].reshape(3, 3).copy()

    def body_p(self, name):
        return self.data.xpos[mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_BODY, name)].copy()


def leg_passive(s):
    return ["%s_knee_joint" % s, "%s_knee_coupler_joint_a" % s, "%s_ankle_roll_joint" % s, "%s_ankle_pitch_joint" % s] + [
        "%s_ankle_coupler_joint_%s_%s" % (s, ab, ax) for ab in "ab" for ax in "xyz"]
