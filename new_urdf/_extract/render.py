import os, numpy as np, mujoco, PIL.Image as Image, PIL.ImageDraw as D
ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
src = open(os.path.join(ROOT, "urdf", "robonex.urdf"), encoding="utf-8").read()
mesh = os.path.join(ROOT, "meshes").replace("\\", "/")
src = src.replace('<robot name="robonex">', '<robot name="robonex">\n<mujoco><compiler meshdir="%s" discardvisual="false" fusestatic="false"/></mujoco>' % mesh, 1)
m = mujoco.MjModel.from_xml_string(src); d = mujoco.MjData(m); mujoco.mj_forward(m, d)
m.vis.headlight.ambient[:] = 0.55; m.vis.headlight.diffuse[:] = 0.6
m.mat_rgba[:] = m.mat_rgba  # keep
for g in range(m.ngeom):
    mid = m.geom_dataid[g]
    name = mujoco.mj_id2name(m, mujoco.mjtObj.mjOBJ_MESH, mid) if mid >= 0 else ""
    m.geom_rgba[g] = [0.85, 0.72, 0.35, 1] if name.startswith("rs0") else [0.55, 0.58, 0.64, 1]
r = mujoco.Renderer(m, 480, 400)
opt = mujoco.MjvOption(); opt.frame = mujoco.mjtFrame.mjFRAME_WORLD
cam = mujoco.MjvCamera(); cam.lookat[:] = [0, 0, -0.25]; cam.distance = 2.0
tiles = []
for label, az, el in (("front view", 180, 0), ("side view from +Y (left)", 90, 0), ("iso view", 150, -20)):
    cam.azimuth, cam.elevation = az, el
    r.update_scene(d, cam, opt); img = Image.fromarray(r.render()).convert("RGB")
    D.Draw(img).text((8, 8), label + "   axes: red X fwd, green Y left, blue Z up   amber = RS02/RS03", fill=(255, 255, 255))
    tiles.append(np.asarray(img))
Image.fromarray(np.concatenate(tiles, axis=1)).save(os.path.join(os.path.dirname(os.path.abspath(__file__)), "render_zero_pose.png"))
print("ok")
