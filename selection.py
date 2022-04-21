import numpy as np


def parent_selection(fitness):
    ''' Simply obtain the two best morphology '''
    fit = sorted(enumerate(fitness),key=lambda x: x[1])
    parent1_i,_ = fit[-1]
    parent2_i,_ = fit[-2]
    return parent1_i,parent2_i


def survival_selection(fitness, n=2):
    ''' Simply obtain the n worst morphology '''
    fit = sorted(enumerate(fitness),key=lambda x: x[1])

    return [index for index,_ in fit[:n]]

