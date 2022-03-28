import numpy as np
import copy
import time
import traceback

from MorphologyGeneration import MorphologyGeneration as morphgen
from ViewMorphology import view
from Evaluate import eval_morphology


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


def micro_mutation(morphology,body_p=0.1,joint_range_p=0.75,joint_plane_p=0.75,len_change_p=0.75):
    """ Micro mutations:
    DOF of joints
    Joint hinge plane
    Minor length changes
    Main body size adjustment
    """

    if np.random.random() > body_p:
        # Main body size adjustment
        if morphology[0]['type'] == "sphere":
            morphology[0]['size'] = min(np.abs(np.random.normal(0.25,0.1)),1)
        else: # Capsule
            morphology[0]['length'] = min(np.abs(np.random.normal(0.75,0.1)),1)
    
    for leg in morphology[1:]: # Ignore morphology body type
        for seg in leg:
            # Modify joint range
            if np.random.random() > joint_range_p and seg['restricted']:
                lower_joint_range,upper_joint_range = seg['joint_range']
                lower_joint_range += np.random.normal()*10
                upper_joint_range += np.random.normal()*10
                if lower_joint_range >= upper_joint_range:
                    lower_joint_range,upper_joint_range = upper_joint_range,lower_joint_range
                seg['joint_range'] = (lower_joint_range,upper_joint_range)

            # Modify joint hinge plane
            if np.random.random() > joint_plane_p and seg['joint_type'] == "hinge":
                x,y,z = seg['rotation_axis']
                seg['rotation_axis'] = (x*(1+np.random.normal()),y*(1+np.random.normal()),z*(1+np.random.normal()))

            # Minor length changes
            if np.random.random() > len_change_p:
                seg['length'] *= min(np.abs(np.random.normal(0.75,0.1)),1)


def macro_mutation(morphology):
    """ Macro mutations:
    Joint type change
    Remove/add entire leg segments
    Remove/add entire leg's
    Major length changes
    """
    for leg in morphology[1:]: # Ignore morphology body type
        for seg in leg:
            pass

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
    fit = sorted(enumerate(fitness),key=lambda x: x[1])
    parent1_i,_ = fit[0]
    parent2_i,_ = fit[1]
    return parent1_i,parent2_i


def breed(parent1, parent2):
    """ Merge two morphology into two unique children 
    Randomly swap the legs of the two parents to create two opposing children
    """
    parent1_clone = copy.deepcopy(parent1)
    parent2_clone = copy.deepcopy(parent2)

    # Each child inherits their parents body type
    child1, child2 = [parent1_clone[0]], [parent2_clone[0]]

    # Build list of available legs to inherit
    inheritable_legs = [leg for leg in parent1_clone[1:]] + [leg for leg in parent2_clone[1:]]
    leg_amount = len(inheritable_legs)
    np.random.shuffle(inheritable_legs)

    # Share the legs amongst the children
    child1_leg_amount = np.random.randint(1, leg_amount)
    
    # Child 1 gets the legs they were assigned
    child1 += inheritable_legs[:child1_leg_amount]
    # Child 2 gets the leftovers
    child2 += inheritable_legs[child1_leg_amount:]
    
    return child1,child2


def survival_selection(fitness):
    """ Simply obtain the two worst morphology """
    fit = sorted(enumerate(fitness),key=lambda x: x[1])
    old1_i, _ = fit[-1]
    old2_i, _ = fit[-2]
    return old1_i,old2_i


if (__name__ == "__main__"):

    pop = pop_init(1)
    pop_size = len(pop)
    gen = 0

    gen_count = 10
    episode_increase_per_gen = 5e4 # 5e4
    while True:
        gen += 1
        fitness = []
        print("Generation {}".format(gen))
        for file_path,morphology in pop:
            try:
                fitness.append(np.random.randint(-1000,1000))
                #fitness.append(eval_morphology(file_path,gen*episode_increase_per_gen))
            except Exception:
                print("Bad morphology. Something went wrong.")
                traceback.print_exc()
                fitness.append(-float('int'))
        
        parent1_i, parent2_i = parent_selection(fitness)
        child1, child2 = breed(pop[parent1_i][1], pop[parent2_i][1])

        try:
            micro_mutation(child1)
            micro_mutation(child2)
        except KeyError:
            traceback.print_exc()
            exit()
        
        old1_i, old2_i = survival_selection(fitness)

        pop[old1_i] = (morphgen(child1,gen),child1)
        pop[old2_i] = (morphgen(child2,gen),child2)

        print("Best fitness: {:.2f}\t Avg fitness: {:.2f}\t Worst fitness: {:.2f}".format(max(fitness),sum(fitness)/len(fitness),min(fitness)))  
