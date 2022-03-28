import numpy as np

from MorphologyGeneration import MorphologyGeneration as morphgen

def pop_init(pop_size=5, limb_low=1, limb_high=7, body_types=["sphere","capsule"], joint_low=1, joint_high=5):
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

        # Choose main body type
        main_body['type'] = np.random.choice(body_types)
        if main_body['type'] == "sphere":
            main_body['size'] = min(np.abs(np.random.normal(0.25,0.1)),1)
        else: # Capsule
            main_body['size'] = diameter
            main_body['length'] = np.random.random()

        morphology.append(main_body)

        for _ in range(legs):
            limb_seg = np.random.randint(joint_low,joint_high)
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
                
                seg['pos'] = (0,0,0)
                seg['from_to'] = (0,0,0,starting_x,starting_y,starting_z)
                seg['joint_type'] = "fixed"
                seg['size'] = diameter

                limb.append(seg)

            else: # Capsule
                starting_x = np.random.uniform(-main_body['length']/2,main_body['length']/2)
                starting_y = 0
                starting_z = 0
            
            limb_seg_x = 0
            limb_seg_y = 0
            
            for _ in range(limb_seg):
                seg = {}
                
                # Size & Initial starting position
                seg['size'] = diameter
                seg['pos'] = (starting_x+limb_seg_x, limb_seg_y+starting_y, starting_z)
                
                starting_x = 0
                starting_y = 0
                starting_z = 0

                # Length
                limb_seg_x = np.random.uniform(-0.5,0.5)
                limb_seg_y = np.random.uniform(-0.5,0.5)
                seg['from_to'] = (0,0,0,limb_seg_x, limb_seg_y, 0)

                # Joint type
                seg['joint_type'] = np.random.choice(["ball", "hinge"])

                # Identifier
                seg['name'] = str(int(time.time()*1000000))

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