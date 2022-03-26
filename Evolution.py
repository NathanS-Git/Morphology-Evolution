from re import L
import numpy as np
import copy
import time
from MorphologyGeneration import MorphologyGeneration as morphgen
from ViewMorphology import view
from Evaluate import eval_morphology


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
        diameter = 0.08 # Leg diameter
        #diameter = np.random.normal(0.4, 0.4)
        morphology = []
        main_body = {}

        main_body['type'] = np.random.choice(body_types)
        if main_body['type'] == "sphere":
            main_body['size'] = min(np.abs(np.random.normal(0.25,0.1)),1)
        else: # Capsule
            main_body['size'] = diameter
            main_body['length'] = np.random.random()

        morphology.append(main_body)
        for leg in range(legs):
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
            
            for segment in range(limb_seg):
                seg = {}
                
                seg['size'] = diameter
                seg['pos'] = (starting_x+limb_seg_x, limb_seg_y+starting_y, starting_z)
                starting_x = 0
                starting_y = 0
                starting_z = 0

                #limb_seg_length = np.random.uniform(0.1,0.4)
                limb_seg_x = np.random.uniform(-0.5,0.5)
                limb_seg_y = np.random.uniform(-0.5,0.5)
                seg['from_to'] = (0,0,0,limb_seg_x, limb_seg_y, 0)

                seg['joint_type'] = np.random.choice(["ball", "hinge"])

                seg['name'] = str(int(time.time()*1000000))

                if seg['joint_type'] != "ball":
                    seg['rotation_axis'] = (np.random.random(), np.random.random(), np.random.random())
                
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
        

def tournament(fit_vals,n_ts=3):
    ''' Parent and survival selection by tournament.
    It samples 2*n_ts individuals from the population, splits them
    and makes each half compete. The best become parents, while the worst
    are replaced by the parents offspring.

    fit_vals -> the fitness values of each individual within the population (it assumes indicies are perserved)
    n_ts -> individuals to select per tournament
    '''
    
    selected_to_mate = []
    selected_to_die = []
    fit_index = list(enumerate(fit_vals)) # Retain index values for each fitness value
    tournament = np.random.choice(fit_index, 2*n_ts) # Sample the population

    # Tournament 1
    selected_to_mate.append(min(tournament[:n_ts], key=lambda x: x[1])[0])
    selected_to_die.append(max(tournament[:n_ts], key=lambda x: x[1])[0])
    # Tournament 2
    selected_to_mate.append(min(tournament[n_ts:], key=lambda x: x[1])[0])
    selected_to_die.append(max(tournament[n_ts:], key=lambda x: x[1])[0])

    # Returns the index values of each parent and loser
    return selected_to_mate,selected_to_die


def parent_selection(fitness):
    """ Simply obtain the two best morphology """
    parent1,parent2 = sorted(enumerate(fitness))[:2]
    return parent1,parent2


def breed(parent1, parent2):
    """ Merge two morphology into two unique children 
    Randomly swap the legs of the two parents to create two opposing children
    """
    child1, child2 = [copy.deepcopy(parent1[0])], [copy.deepcopy(parent2[0])]
    for leg in copy.deecopy(parent1[1:]):
        if np.random.random() > 0.5:
            child1.append(leg)
        else:
            child2.append(leg)

    for leg in copy.deepcopy(parent2[1:]):
        if np.random.random() > 0.5:
            child1.append(leg)
        else:
            child2.append(leg)
    
    return child1,child2


def survival_selection(fitness):
    """  """
    pass
    


if (__name__ == "__main__"):
    pop = pop_init()
    pop_size = len(pop)
    gen = 0
    gen_count = 10
    while True:
        gen += 1
        fitness = []
        print("Generation {}".format(gen))
        for file_path,morphology in pop:
            fitness.append(eval_morphology(file_path,5e4*gen))
        

        
            
            
    