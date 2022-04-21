import numpy as np


def micro_mutation(morphology,body_p=0.1,joint_range_p=0.75,joint_plane_p=0.75,len_change_p=0.75):
    ''' Micro mutations:
    DOF of joints
    Joint hinge plane
    Minor length changes
    Main body size adjustment
    '''

    if np.random.random() < body_p:
        # Main body size adjustment
        if morphology[0]['type'] == "sphere":
            morphology[0]['size'] = min(np.abs(np.random.normal(0.25,0.1)),1)
        else: # Capsule
            morphology[0]['length'] = min(np.abs(np.random.normal(0.75,0.1)),1)
    
    for leg in morphology[1:]: # Ignore morphology body type
        for seg in leg:
            # Modify joint range
            if np.random.random() < joint_range_p and seg['restricted']:
                lower_joint_range,upper_joint_range = seg['joint_range']
                lower_joint_range += np.random.normal()*10
                upper_joint_range += np.random.normal()*10
                if lower_joint_range >= upper_joint_range:
                    lower_joint_range,upper_joint_range = upper_joint_range,lower_joint_range
                if seg['joint_type'] == "ball":
                    lower_joint_range = 0
                seg['joint_range'] = (lower_joint_range,upper_joint_range)

            # Modify joint hinge plane
            if np.random.random() < joint_plane_p and seg['joint_type'] == "hinge":
                x,y,z = seg['rotation_axis']
                seg['rotation_axis'] = (x*(1+np.random.normal()),y*(1+np.random.normal()),z*(1+np.random.normal()))

            # Minor length changes
            if np.random.random() < len_change_p:
                seg['length'] *= min(np.abs(np.random.normal(0.75,0.1)),1)


def macro_mutation(morphology,len_change_p=0.3,joint_change_p=0.1,seg_rem_p=0.01,seg_add_p=0.01,leg_rem_p=0.1,leg_add_p=0.1,joint_low=1,joint_high=5):
    ''' Macro mutations:
    Joint type change # Evaluated per segment
    Remove/add entire leg segments # Evaluated per leg
    Remove/add entire leg's # Evaluated once per morphology
    Major length changes # Evaluated per segment
    '''
    diameter = 0.08

    if np.random.random() < leg_rem_p and len(morphology[1:]) > 1: # Remove random leg
        
        index = np.random.randint(len(morphology)-1)
        # Never remove main body (it's important)
        del morphology[index+1]
    

    if np.random.random() < leg_add_p: # Add leg
        
        limb_seg = np.random.randint(joint_low,joint_high)
        limb = []

        if morphology[0]['type'] == "sphere":
            radius = morphology[0]['size']
        else: # Capsule
            radius = morphology[0]['length']/2

        # Find a random point on the sphere to initialize a limb at
        starting_x = np.random.normal()
        starting_y = np.random.normal()
        starting_z = np.random.normal()
        denominator = 1/np.sqrt(np.square(starting_x)+np.square(starting_y)+np.square(starting_z))
        starting_x *= denominator*radius*1.05
        starting_y *= denominator*radius*1.05
        starting_z *= denominator*radius*1.05
        
        for limb_seg_i in range(limb_seg):
            
            seg = {}
            
            if limb_seg_i == 0:
                # Initial joint position, defines where whole leg will appear relative to morphology body
                seg['initial_pos'] = (starting_x, starting_y, starting_z)

            # Size & Length
            seg['size'] = diameter
            seg['length'] = np.random.random()*0.5

            # Joint type
            seg['joint_type'] = np.random.choice(["ball", "hinge"])

            # Rotation plane/axis
            if seg['joint_type'] != "ball":
                seg['rotation_axis'] = (np.random.random(), np.random.random(), np.random.random())
            
            # Joint restrictions
            seg['restricted'] = np.random.choice([False,True])
            if seg['restricted']:
                lower = np.random.randint(-120,119)
                upper = np.random.randint(lower+1,120)
                if seg['joint_type'] == "ball":
                    seg['joint_range'] = (0, lower)
                else: # Joint type is hinge
                    seg['joint_range'] = (lower,upper)

            limb.append(seg)
        morphology.append(limb)
    

    for leg in morphology[1:]: # Ignore morphology body type
        if np.random.random() < seg_rem_p and len(leg) > 1:
            # Remove random segment
            index = np.random.randint(len(leg)-1)
            # Never remove first segment (it's important)
            del leg[index+1]

        if np.random.random() < seg_add_p: # Add segment
            seg = {}

            # Size & Length
            seg['size'] = diameter
            seg['length'] = np.random.random()*0.5

            # Joint type
            seg['joint_type'] = np.random.choice(["ball", "hinge"])

            # Rotation plane/axis
            if seg['joint_type'] != "ball":
                seg['rotation_axis'] = (np.random.random(), np.random.random(), np.random.random())
            
            # Joint restrictions
            seg['restricted'] = np.random.choice([False,True])
            if seg['restricted']:
                lower = np.random.randint(-120,119)
                upper = np.random.randint(lower+1,120)
                if seg['joint_type'] == "ball":
                    seg['joint_range'] = (0, lower)
                else: # Joint type is hinge
                    seg['joint_range'] = (lower,upper)

            leg.append(seg)

        for seg in leg:

            # Major length change
            if np.random.random() < len_change_p:
                seg['length'] *= np.abs(np.random.normal(0.75,0.15))

            # Joint type/attribute change
            if np.random.random() < joint_change_p:
                if seg['joint_type'] == "ball":
                    seg['joint_type'] = "hinge"
                else:
                    seg['joint_type'] = "ball"
                
                # Rotation plane/axis
                if seg['joint_type'] == "hinge":
                    seg['rotation_axis'] = (np.random.random(), np.random.random(), np.random.random())

                # Change restrictions
                seg['restricted'] = np.random.choice([False,True])
                if seg['restricted']:
                    lower = np.random.randint(-120,119)
                    upper = np.random.randint(lower+1,120)
                    if seg['joint_type'] == "ball":
                        seg['joint_range'] = (0, lower)
                    else: # Joint type is hinge
                        seg['joint_range'] = (lower,upper)