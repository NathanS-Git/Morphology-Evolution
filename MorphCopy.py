import numpy as np
import mujoco_py
import xml.etree.ElementTree as et


def MorphologyGeneration():

    root = et.Element("mujoco")
    et.SubElement(root, "compiler", angle="degree", coordinate="local", inertiafromgeom="true")
    et.SubElement(root, "option", integrator="RK4", timestep="0.01")

    default = et.SubElement(root, "default")
    et.SubElement(default, "joint", armature="1", damping="1", limited="true")
    et.SubElement(default, "geom", conaffinity="0", condim="3.0", density="5.0", friction="1 0.5 0.5", margin="0.01", rgba="0.4 0.6 1 1")

    asset = et.SubElement(root, "asset")
    et.SubElement(asset, "texture", builtin="gradient", height="100", rgb1="1 1 1", rgb2="0 0 0", type="skybox", width="100")
    et.SubElement(asset, "texture", builtin="flat", height="1278", mark="cross", markrgb="1 1 1", name="texgeom", random="0.01", rgb1="0.8 0.6 0.4", rgb2="0.8 0.6 0.4", type="cube", width="127")
    et.SubElement(asset, "texture", builtin="checker", height="100", name="texplane", rgb1="0 0 0", rgb2="0.8 0.8 0.8", type="2d", width="100")
    et.SubElement(asset, "material", name="MatPlane", reflectance="0.5", shininess="1", specular="1", texrepeat="10 10", texture="texplane")
    et.SubElement(asset, "material", name="geom", texture="texgeom", texuniform="true")

    worldbody = et.SubElement(root, "worldbody")
    et.SubElement(worldbody, "light", cutoff="100", diffuse="1 1 1", dir="-0 0 -1.3", directional="true", exponent="1", pos="0 0 0", specular=".1 .1 .1")
    et.SubElement(worldbody, "geom", conaffinity="1", condim="3", material="MatPlane", name="floor", pos="0 0 0", rgba="1 1 1 1", size="10 10 10", type="plane")

    torso = et.SubElement(worldbody, "body", name="torso", pos="0 0 0.1")
    et.SubElement(torso, "camera", name="track", mode="trackcom", pos="0 -3 0.3", xyaxes="1 0 0 0 0 1")
    et.SubElement(torso, "geom", name="torso_geom", pos="0 0 0", size="0.1", fromto="-0.1 0 0 0.5 0 0", type="capsule")
    et.SubElement(torso, "joint", armature="0", damping="0", limited="false", margin="0.01", name="root", pos="0 0 0", type="free")

    # Main body size
    # Numbers of limbs
    # Number of segments per limb
    # Length of each limb segment
    # Initial position for each limb
    # Joint types (fixed(none), hinge, ball) / and rotation axis / and rotation range
    # General leg diameter

    for l in range(1):
        if l == 0:
            hip_pos = "0 0 0"
            hip_axis = "0 0 1"
            hip_range = "-30 70"
            hip_fromto = "0 0 0 0 0.4 0"

            thigh_pos = "0 0.4 0"
            thigh_axis = "0 0 1"
            thigh_range = "30 70"
            thigh_fromto = "0 0 0 0 0.5 0"

            calf_pos = "0 0.5 0"
            calf_axis = "1 0 0"
            calf_range = "30 70"
            calf_fromto = "0 0 0 0 0.4 0"

            diameter = "0.08"
        elif l == 1:
            hip_pos = "-0.2 0.2 0"
            hip_range = "-30 30"
            hip_fromto = "0 0 0 -0.2 0.2 0.0"

            thigh_pos = "0.4 0.2 0"
            thigh_range = "-30 30"
            thigh_fromto = "0.4 0 0 0.4 0.2 0"

            calf_pos = "0.4 0.2 0"
            calf_range = "-30 30"
            calf_fromto = "0.4 0 0 0.4 0.2 0"

            diameter = "0.08"
        elif l == 2:
            hip_pos = "-0.2 0.2 0"
            hip_range = "-30 30"
            hip_fromto = "0 0 0 -0.2 0.2 0.0"

            thigh_pos = "0.4 0.2 0"
            thigh_range = "-30 30"
            thigh_fromto = "0.4 0 0 0.4 0.2 0"

            calf_pos = "0.4 0.2 0"
            calf_range = "-30 30"
            calf_fromto = "0.4 0 0 0.4 0.2 0"

            diameter = "0.08"
        else:
            hip_pos = "-0.2 0.2 0"
            hip_range = "-30 30"
            hip_fromto = "0 0 0 -0.2 0.2 0.0"

            thigh_pos = "0.4 0.2 0"
            thigh_range = "-30 30"
            thigh_fromto = "0.4 0 0 0.4 0.2 0"

            calf_pos = "0.4 0.2 0"
            calf_range = "-30 30"
            calf_fromto = "0.4 0 0 0.4 0.2 0"

            diameter = "0.08"

        hip = et.SubElement(torso, "body", pos=hip_pos)
        et.SubElement(hip, "joint", axis=hip_axis, pos="0 0 0", range=hip_range, type="hinge")
        et.SubElement(hip, "geom", fromto=hip_fromto, size=diameter, type="capsule")
        
        thigh = et.SubElement(hip, "body", pos=thigh_pos)
        et.SubElement(thigh, "joint", axis=thigh_axis, pos="0 0 0", range=thigh_range, type="hinge")
        et.SubElement(thigh, "geom", fromto=thigh_fromto, size=diameter, type="capsule")
        
        calf = et.SubElement(thigh, "body", pos=calf_pos)
        et.SubElement(calf, "joint", axis=calf_axis, pos="0 0 0", range=calf_range, type="hinge")
        et.SubElement(calf, "geom", fromto=calf_fromto, size=diameter, type="capsule")

    tree = et.ElementTree(root)
    et.indent(tree)
    tree.write("morphology.xml")


if (__name__ == "__main__"):
    MorphologyGeneration()
    model = mujoco_py.load_model_from_path('morphology.xml')
    sim = mujoco_py.MjSim(model)
    viewer = mujoco_py.MjViewer(sim)
    while True:
        viewer.render()
        sim.step()