import numpy as np
import time

from morphology_gen import GenXML as morphgen

def pop_init(pop_size=10, limb_low=1, limb_high=6, body_types=["capsule","sphere"], joint_low=1, joint_high=5):
    ''' Population initialization
    Initializes the starting morphology types. '''

    population = []

    for _ in range(pop_size):

        # GENERATE MORPHOLOGY CONFIGURATION

        # Main body size
        # Numbers of limbs
        # Number of segments per limb
        # Length of each limb segment
        # Initial position for each limb
        # Joint types (fixed(none), hinge, ball) / and rotation axis / and rotation range
        # General leg diameter

        legs = np.random.randint(limb_low,limb_high)
        diameter = 0.08 # General leg diameter

        morphology = []
        main_body = {}

        # Assign name to all morphology for this specific simulation
        main_body['custom_name'] = time.strftime("%Y-%b-%d %H:%M:%S", time.localtime())

        # Choose main body type
        main_body['type'] = np.random.choice(body_types)
        if main_body['type'] == "sphere":
            main_body['size'] = min(np.abs(np.random.normal(0.25,0.1)),1)
        else: # Capsule
            main_body['size'] = diameter
            main_body['length'] = min(np.abs(np.random.normal(0.75,0.1)),1)

        morphology.append(main_body)

        for _ in range(legs):
            limb_seg = np.random.randint(joint_low,joint_high)
            limb = []
            if main_body['type'] == "sphere":
                radius = main_body['size']
            else: # Capsule
                radius = main_body['length']/2

            # Find a random point on a sphere surrounding the main body to initialize a limb at
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
                    # Initial joint position defines where whole leg will appear relative to morphology body
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
        
        morphology_file = morphgen(morphology)
        
        population.append((morphology_file,morphology))

    return population
