import os, re, numpy as np, mujoco
ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
src = open(os.path.join(ROOT, "urdf", "robonex.urdf"), encoding="utf-8").read()
src = src.replace('<robot name="robonex">', '<robot name="robonex">\n<mujoco><compiler meshdir="%s" strippath="true" discardvisual="false" fusestatic="false"/></mujoco>' % os.path.join(ROOT, "meshes").replace("\\", "/"), 1)
m = mujoco.MjModel.from_xml_string(src)
d = mujoco.MjData(m); mujoco.mj_forward(m, d)
print("bodies", m.nbody - 1, "joints", m.njnt, "geoms", m.ngeom, "meshes", m.nmesh, "total mass %.5f" % sum(m.body_mass))
for n in ("base_link", "l_hip_pitch_link", "l_knee_link", "l_ankle_link", "l_foot", "r_foot"):
    b = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_BODY, n)
    print("  %-18s xpos %s" % (n, np.round(d.xpos[b] * 1000, 2)))
# lowest mesh vertex of the feet (world, z) at zero pose
for foot in ("l_foot", "r_foot"):
    b = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_BODY, foot)
    zs = []
    for g in range(m.ngeom):
        if m.geom_bodyid[g] == b and m.geom_type[g] == mujoco.mjtGeom.mjGEOM_MESH:
            mid = m.geom_dataid[g]; v = m.mesh_vert[m.mesh_vertadr[mid]:m.mesh_vertadr[mid] + m.mesh_vertnum[mid]]
            R = d.geom_xmat[g].reshape(3, 3); zs.append((v @ R.T + d.geom_xpos[g])[:, 2].min())
    print("  %s sole (lowest vertex) z = %.2f mm" % (foot, min(zs) * 1000))
try:
    r = mujoco.Renderer(m, 480, 640)
    import PIL.Image as Image
    cam = mujoco.MjvCamera(); cam.lookat[:] = [0, 0, -0.4]; cam.distance = 2.0
    imgs = []
    for az, el in ((180, 0), (90, 0), (135, -20)):
        cam.azimuth, cam.elevation = az, el
        opt = mujoco.MjvOption(); opt.geomgroup[:] = 1
        r.update_scene(d, cam, opt); imgs.append(r.render())
    Image.fromarray(np.concatenate(imgs, axis=1)).save(os.path.join(os.path.dirname(os.path.abspath(__file__)), "render_zero_pose.png"))
    print("rendered render_zero_pose.png")
except Exception as e:
    print("render failed:", type(e).__name__, e)
