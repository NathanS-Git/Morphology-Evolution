import numpy as np
import mujoco_py
import xml.etree.ElementTree as et


def MorphologyGeneration():

    # GENERATE MORPHOLOGY CONFIGURATION

    # Main body size
    # Numbers of limbs
    # Number of segments per limb
    # Length of each limb segment
    # Initial position for each limb
    # Joint types (fixed(none), hinge, ball) / and rotation axis / and rotation range
    # General leg diameter

    legs = np.random.randint(1,7)
    diameter = "0.08" # Leg diameter
    #diameter = np.random.normal(0.4, 0.4)
    morphology = []
    main_body = {}
    
    main_body['type'] = np.random.choice(["sphere","capsule"])
    if main_body['type'] == "sphere":
        main_body['size'] = min(np.abs(np.random.normal(0.25,0.1)),1)
    else: # Capsule
        main_body['size'] = diameter
        main_body['length'] = np.random.random()

    morphology.append(main_body)
    for leg in range(legs):
        limb_seg = np.random.randint(1,5)
        limb = []
        if main_body['type'] == "sphere":
            # Find a random point on the sphere to initialize a limb at
            starting_x = np.random.normal()
            starting_y = np.random.normal()
            starting_z = np.random.normal()
            denominator = 1/np.sqrt(np.square(starting_x)+np.square(starting_y)+np.square(starting_z))
            starting_x *= denominator*main_body['size']*0.99
            starting_y *= denominator*main_body['size']*0.99
            starting_z *= denominator*main_body['size']*0.99
            
            # Add a fixed limb to connect to the outside of the sphere for visual purposes
            seg = {}
            starting_x *= 1.05
            starting_y *= 1.05
            starting_z *= 1.05
            
            seg['pos'] = "0 0 0"
            seg['from_to'] = "0 0 0 "+str(starting_x)+" "+str(starting_y)+" "+str(starting_z)
            seg['joint_type'] = "fixed"
            limb.append(seg)

        else: # Capsule
            starting_x = np.random.uniform(-main_body['length']/2,main_body['length']/2)
            starting_y = 0
            starting_z = 0
        limb_seg_x = 0
        limb_seg_y = 0
        for segment in range(limb_seg):
            seg = {}
            
            seg['pos'] = str(starting_x+limb_seg_x)+" "+str(limb_seg_y+starting_y)+" "+str(starting_z)
            starting_x = 0
            starting_y = 0
            starting_z = 0

            #limb_seg_length = np.random.uniform(0.1,0.4)
            limb_seg_x = np.random.uniform(-0.5,0.5)
            limb_seg_y = np.random.uniform(-0.5,0.5)
            seg['from_to'] = "0 0 0 "+str(limb_seg_x)+" "+str(limb_seg_y)+" 0"

            seg['joint_type'] = np.random.choice(["ball", "hinge"])

            seg['name'] = "leg:"+str(leg)+" segment:"+str(segment)

            if seg['joint_type'] != "ball":
                seg['rotation_axis'] = str(np.random.random())+" "+str(np.random.random())+" "+str(np.random.random())
            seg['restricted'] = np.random.choice(["false","true"])
            if seg['restricted'] == "true":
                lower = np.random.randint(-120,119)
                upper = np.random.randint(lower+1,120)
                if seg['joint_type'] == "ball":
                    seg['joint_range'] = "0 "+str(lower)
                else: # Joint type is hinge
                    seg['joint_range'] = str(lower)+" "+str(upper)

            limb.append(seg)
        morphology.append(limb)

    

    # GENERATE MORPHOLOGY XML FILE USING CONFIGURATION

    # Boiler plate code

    root = et.Element("mujoco")
    et.SubElement(root, "compiler", angle="degree", coordinate="local", inertiafromgeom="true")
    et.SubElement(root, "option", integrator="RK4", timestep="0.01")

    default = et.SubElement(root, "default")
    et.SubElement(default, "joint", armature="1", damping="1", limited="true")
    et.SubElement(default, "geom", conaffinity="0", condim="3.0", density="5.0", friction="1 0.5 0.5", margin="0.01", rgba="0.4 0.6 1 1")

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
    body_size = main_body['size']
    if main_body['type'] == "capsule":
        body_length = main_body['length']
        torso_position = str(-body_length/2)+" 0 0 "+str(body_length/2)+" 0 0"

    torso = et.SubElement(worldbody, "body", pos="0 0 0.5", name="torso")
    et.SubElement(torso, "camera", mode="track", pos="0 -3 3", xyaxes="1 0 0 0 1 1", fovy="45")
    et.SubElement(torso, "camera", mode="trackcom", pos="0 -2 1", xyaxes="1 0 0 0 1 2", fovy="45")
    if main_body['type'] == "capsule":
        et.SubElement(torso, "geom", size=str(body_size), fromto=torso_position, type=main_body['type'])
    else: # Sphere
        et.SubElement(torso, "geom", size=str(body_size), type=main_body['type'])

    et.SubElement(torso, "joint", armature="0", damping="0", limited="false", margin="0.01", pos="0 0 0", type="free")

    actuators = et.SubElement(root, "actuator")

    # Segments defined below

    for limb in morphology[1:]:
        seg = torso
        for segment in limb:
            seg = et.SubElement(seg, "body", pos=segment['pos']) # Recursive definition
            # Joint type
            if segment['joint_type'] == "hinge":
                id = segment['name']
                et.SubElement(actuators, "motor", joint=id, gear="100")
                if segment['restricted'] == "false":
                    et.SubElement(seg, "joint", name=id, limited=segment['restricted'], axis=segment['rotation_axis'], type=segment['joint_type'])
                else:
                    et.SubElement(seg, "joint", name=id, limited=segment['restricted'], axis=segment['rotation_axis'], range=segment['joint_range'], type=segment['joint_type'])
            elif segment['joint_type'] == "ball":
                id = segment['name']
                et.SubElement(actuators, "motor", joint=id, gear="100")
                if segment['restricted'] == "false":
                    et.SubElement(seg, "joint", name=id, limited=segment['restricted'], type=segment['joint_type'])
                else:
                    et.SubElement(seg, "joint", name=id, limited=segment['restricted'], range=segment['joint_range'], type=segment['joint_type'])
            elif segment['joint_type'] == "fixed":
                pass
            et.SubElement(seg, "geom", conaffinity="1", fromto=segment['from_to'], size=diameter, type="capsule")

    tree = et.ElementTree(root)
    et.indent(tree)
    tree.write("morphology.xml")
    
    return morphology


if (__name__ == "__main__"):   
    MorphologyGeneration()

    model = mujoco_py.load_model_from_path('morphology.xml')
    sim = mujoco_py.MjSim(model)
    viewer = mujoco_py.MjViewer(sim)
    
    while True:
        #print(sim.data.ctrl)
        for motor in range(len(sim.data.ctrl)): 
            sim.data.ctrl[motor] = np.random.random()*2-1
        sim.step()
        print("\n",sim.get_state())
        viewer.render()