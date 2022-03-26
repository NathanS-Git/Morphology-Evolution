import numpy as np
import mujoco_py
import xml.etree.ElementTree as et
import time
import os


def MorphologyGeneration(morphology,gen=0):

    # GENERATE MORPHOLOGY XML FILE USING GIVEN CONFIGURATION

    # Boiler plate code

    root = et.Element("mujoco")
    et.SubElement(root, "compiler", angle="degree", coordinate="local", inertiafromgeom="true")
    et.SubElement(root, "option", integrator="RK4", timestep="0.01")

    default = et.SubElement(root, "default")
    et.SubElement(default, "joint", armature="1", damping="1", limited="true")
    et.SubElement(default, "geom", conaffinity="0", condim="3.0", density="5.0", friction="1 0.5 0.5", margin="0.01", rgba="0.4 0.6 1 1")
    et.SubElement(default, "motor", ctrllimited="true", ctrlrange="-1 1")

    visual = et.SubElement(root, "visual")
    et.SubElement(visual, "quality", shadowsize="16384")
    et.SubElement(visual, "map", shadowclip="10")

    asset = et.SubElement(root, "asset")
    et.SubElement(asset, "texture", builtin="gradient", height="100", rgb1="1 1 1", rgb2="0.2 0.2 0.2", type="skybox", width="100")
    et.SubElement(asset, "texture", builtin="flat", height="1278", mark="cross", markrgb="1 1 1", name="texgeom", random="0.01", rgb1="0.8 0.6 0.4", rgb2="0.8 0.6 0.4", type="cube", width="127")
    et.SubElement(asset, "texture", builtin="checker", height="100", name="texplane", rgb1="0 0 0", rgb2="0.6 0.6 0.6", type="2d", width="100")
    et.SubElement(asset, "material", name="MatPlane", reflectance="0.5", shininess="1", specular="1", texrepeat="1 1", texture="texplane")
    et.SubElement(asset, "material", name="geom", texture="texgeom", texuniform="true")

    worldbody = et.SubElement(root, "worldbody")
    et.SubElement(worldbody, "light", cutoff="1", diffuse="1 1 1", dir="-0 0 -1.3", directional="true", exponent="1", pos="0 0 1.3", specular=".1 .1 .1")
    et.SubElement(worldbody, "geom", conaffinity="1", condim="3", material="MatPlane", name="floor", pos="0 0 0", rgba="1 1 1 1", size="0 0 10", type="plane")

    # Main root node defined below

    main_body = morphology[0]

    if main_body['type'] == "capsule":
        body_length = main_body['length']
        torso_position = str(-body_length/2)+" 0 0 "+str(body_length/2)+" 0 0"

    torso = et.SubElement(worldbody, "body", pos="0 0 0.5", name="torso")
    et.SubElement(torso, "camera", mode="track", pos="0 -3 3", xyaxes="1 0 0 0 1 1", fovy="45")
    et.SubElement(torso, "camera", mode="trackcom", pos="0 -2 1", xyaxes="1 0 0 0 1 2", fovy="45")
    if main_body['type'] == "capsule":
        et.SubElement(torso, "geom", size=str(main_body['size']), fromto=torso_position, type=main_body['type'])
    else: # Sphere
        et.SubElement(torso, "geom", size=str(main_body['size']), type=main_body['type'])

    et.SubElement(torso, "joint", armature="0", damping="0", limited="false", margin="0.01", pos="0 0 0", type="free")

    actuators = et.SubElement(root, "actuator")

    # Segments defined below

    for limb in morphology[1:]:
        seg = torso
        for segment in limb:
            seg = et.SubElement(seg, "body", pos=' '.join([str(pos) for pos in segment['pos']])) # Recursive definition
            # Joint type
            if segment['joint_type'] == "hinge":
                id = segment['name']
                et.SubElement(actuators, "motor", joint=id, gear="100")
                if segment['restricted'] == False:
                    et.SubElement(seg, "joint", name=id, limited=str(segment['restricted']).lower(), axis=' '.join([str(pos) for pos in segment['rotation_axis']]), type=segment['joint_type'])
                else:
                    et.SubElement(seg, "joint", name=id, limited=str(segment['restricted']).lower(), axis=' '.join([str(pos) for pos in segment['rotation_axis']]), range=' '.join([str(pos) for pos in segment['joint_range']]), type=segment['joint_type'])
            elif segment['joint_type'] == "ball":
                id = segment['name']
                et.SubElement(actuators, "motor", joint=id, gear="100")
                if segment['restricted'] == False:
                    et.SubElement(seg, "joint", name=id, limited=str(segment['restricted']).lower(), type=segment['joint_type'])
                else:
                    et.SubElement(seg, "joint", name=id, limited=str(segment['restricted']).lower(), range=' '.join([str(pos) for pos in segment['joint_range']]), type=segment['joint_type'])
            elif segment['joint_type'] == "fixed":
                pass
            et.SubElement(seg, "geom", conaffinity="1", fromto=' '.join([str(pos) for pos in segment['from_to']]), size=str(segment['size']), type="capsule")

    tree = et.ElementTree(root)
    et.indent(tree) # Beautify the file

    extension = 0
    name = "Morphology_"+time.strftime("%Y-%b-%d %H:%M", time.localtime())+" Gen:"+str(gen)+" "+str(extension)
    while os.path.exists(f"./morphology/{name}.xml"): 
        extension += 1 # If you're creating many, append a value to the end
        name = "Morphology_"+time.strftime("%Y-%b-%d %H:%M", time.localtime())+" Gen:"+str(gen)+" "+str(extension)
    
    if not os.path.exists("./morphology"):
        os.makedirs("./morphology")
    
    tree.write(f"./morphology/{name}.xml")
    
    return f"./morphology/{name}.xml"


if (__name__ == "__main__"):
    pass