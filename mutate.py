import numpy as np


def micro_mutation(morphology,joint_range_p=0.5,joint_plane_p=0.5,len_change_p=0.5):
    """ Micro mutations:
    DOF of joints
    Joint hinge plane
    Minor length changes
    """
    for leg in morphology[1:]:
        for seg in leg:

            # Modify joint range
            if np.random.random() > joint_range_p:
                lower_joint_range,upper_joint_range = seg['joint_range']
                lower_joint_range += np.random.normal()*10
                upper_joint_range += np.random.normal()*10
                if lower_joint_range >= upper_joint_range:
                    lower_joint_range,upper_joint_range = upper_joint_range,lower_joint_range
                
                seg['joint_range'] = (lower_joint_range,upper_joint_range)

            # Modify joint hinge plane
            if np.random.random() > joint_plane_p:
                if seg['joint_type'] == "hinge":
                    x,y,z = seg['rotation_axis']
                    seg['rotation_axis'] = (x*(1+np.random.normal()),y*(1+np.random.normal()),z*(1+np.random.normal()))

            # Minor length changes
            if np.random.random() > len_change_p:
                fx,fy,fz,tx,ty,tz = seg['from_to']
                seg['from_to'] = (fx,fy,fz,tx+np.random.normal(),ty+np.random.normal(),tz)


def macro_mutation(morphology):
    """ Macro mutations:
    Joint type change
    Remove/add entire leg segments
    Remove/add entire leg's
    Major length changes
    """