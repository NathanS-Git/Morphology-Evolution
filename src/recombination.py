import copy
import numpy as np


def breed(parent1, parent2):
    """ Merge two morphology into two unique children 
    Randomly swap the legs of the two parents to create two opposing children.
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
