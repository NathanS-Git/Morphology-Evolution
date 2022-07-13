import numpy as np
import xml.etree.ElementTree as et
import time
import os

def gen_XML(morphology, generation=0):

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

    torso = et.SubElement(worldbody, "body", pos="0 0 0.5", name="torso")
    et.SubElement(torso, "camera", mode="track", pos="0 -3 3", xyaxes="1 0 0 0 1 1", fovy="45")
    et.SubElement(torso, "camera", mode="trackcom", pos="0 -2 1", xyaxes="1 0 0 0 1 2", fovy="45")
    if main_body['type'] == "capsule":
        et.SubElement(torso, "geom", size=str(main_body['size']), fromto=str(-main_body['length']/2)+" 0 0 "+str(main_body['length']/2)+" 0 0", type=main_body['type'])
    else: # Sphere
        et.SubElement(torso, "geom", size=str(main_body['size']), type=main_body['type'])

    et.SubElement(torso, "joint", armature="0", damping="0", limited="false", margin="0.01", pos="0 0 0", type="free")

    actuators = et.SubElement(root, "actuator")

    # Segments defined below

    for limb in morphology[1:]:
        seg = torso
        last_pos = 0

        seg = et.SubElement(seg, "body", pos="0 0 0")
        # Visually connect a fixed arm to wherever the initial point of the leg is positioned
        if main_body['type'] == "capsule":
            # If the main body is a capsule put the starting point of the visual leg to the closest point on the capsule
            x,_,_ = limb[0]['initial_pos']
            # Abusing the fact that a capsule is always initialized along the x axis
            closest_point = min(x,main_body['length']/2) if x > 0 else max(x,-main_body['length']/2)
            et.SubElement(seg, "geom", conaffinity="1", fromto=str(closest_point)+" 0 0 "+" ".join(str(pos) for pos in limb[0]['initial_pos']), size=str(limb[0]['size']), type="capsule")

        else: # Sphere
            et.SubElement(seg, "geom", conaffinity="1", fromto="0 0 0 "+" ".join(str(pos) for pos in limb[0]['initial_pos']), size=str(limb[0]['size']), type="capsule")
        
        for segment in limb:
            try: # If this is the first segment for this leg, specify the starting coordinates
                starting_x,starting_y,starting_z = segment['initial_pos']
                last_pos = (starting_x,starting_y,starting_z)
                normalize_denom = np.sqrt(starting_x*starting_x+starting_y*starting_y)
                try:
                    direction_unit_vector = (starting_x/normalize_denom,starting_y/normalize_denom,0)
                except ZeroDivisionError:
                    direction_unit_vector = (0,1,0)
            except KeyError:
                last_pos = to

            seg = et.SubElement(seg, "body", pos=" ".join(str(pos) for pos in last_pos)) # Recursive definition
            # Joint type
            if segment['joint_type'] == "hinge":
                id = str(int(time.time()*10000000))
                et.SubElement(actuators, "motor", joint=id, gear="100")
                if segment['restricted'] == False:
                    et.SubElement(seg, "joint", name=id, limited=str(segment['restricted']).lower(), axis=' '.join(str(pos) for pos in segment['rotation_axis']), type=segment['joint_type'])
                else:
                    et.SubElement(seg, "joint", name=id, limited=str(segment['restricted']).lower(), axis=' '.join(str(pos) for pos in segment['rotation_axis']), range=' '.join([str(pos) for pos in segment['joint_range']]), type=segment['joint_type'])
            elif segment['joint_type'] == "ball":
                id = str(int(time.time()*10000000))
                et.SubElement(actuators, "motor", joint=id, gear="100")
                if segment['restricted'] == False:
                    et.SubElement(seg, "joint", name=id, limited=str(segment['restricted']).lower(), type=segment['joint_type'])
                else:
                    et.SubElement(seg, "joint", name=id, limited=str(segment['restricted']).lower(), range=' '.join(str(pos) for pos in segment['joint_range']), type=segment['joint_type'])
            elif segment['joint_type'] == "fixed":
                pass
                
            to = tuple(pos*segment['length'] for pos in direction_unit_vector)
            et.SubElement(seg, "geom", conaffinity="1", fromto="0 0 0 "+" ".join(str(pos) for pos in to), size=str(segment['size']), type="capsule")

    tree = et.ElementTree(root)
    et.indent(tree) # Beautify the file

    extension = 0
    name = f"{main_body['custom_name']} Gen:{generation} {extension}"
    while os.path.exists(f"./morphology/{name}.xml"):
        extension += 1
        name = f"{main_body['custom_name']} Gen:{generation} {extension}"

    if not os.path.exists("./morphology"):
        os.makedirs("./morphology")
    
    tree.write(f"./morphology/{name}.xml")
    
    return f"./morphology/{name}.xml"