from doctest import IGNORE_EXCEPTION_DETAIL
import numpy as np
import copy
import time
import traceback
import pickle
import os

from morphology_gen import GenXML as morphgen
from evaluate import eval_morphology
from initialization import pop_init
from mutate import micro_mutation, macro_mutation
from selection import parent_selection, survival_selection
from recombination import breed


if (__name__ == '__main__'):

    if os.path.exists("Backup.dat"):
        print("RECOVERING...")
        # I was dealing with breaker issues so this allows for easy recovery
        file = open("Backup.dat",'rb')
        begin_gen,pop = pickle.load(file)
        file.close()
    else:
        pop = pop_init()
        begin_gen = 1

    micro_mutation_p = 1 # Micro mutation probability
    macro_mutation_p = 1 # Macro mutation probability
    replacements_per = 6 # Replacements per generation (Should be even)
    gen_count = 100 # Number of generations to evolve to
    episode_increase_per_gen = 5e4
    static_eps_eval = 5e5
    for gen in range(begin_gen,gen_count):
        
        fitness = []
        
        # Save info to file after each generation for recovery purposes
        file = open("Backup.dat",'wb')
        pickle.dump((gen,pop),file)
        file.close()

        for file_path, morphology in pop:
            try:
                #fitness.append(np.random.randint(-1000,1000))
                #fitness.append(eval_morphology(file_path,gen*episode_increase_per_gen))
                fitness.append(eval_morphology(file_path,static_eps_eval))
                #fitness.append(np.random.random())
            except Exception:
                print("Bad morphology. Something went wrong.")
                traceback.print_exc()
                fitness.append(float('-inf'))
        
        parent1_i, parent2_i = parent_selection(fitness)
        children = []
        for _ in range(replacements_per//2): # Six children
            children += breed(pop[parent1_i][1], pop[parent2_i][1])

        for child in children:
            if np.random.random() < micro_mutation_p:
                micro_mutation(child)
            if np.random.random() < macro_mutation_p:
                macro_mutation(child)

        replacements_i = survival_selection(fitness, replacements_per)

        c_i = 0
        for r_i in replacements_i:
            pop[r_i] = (morphgen(children[c_i],gen),children[c_i])
            c_i += 1

        print("Next Gen Population : {}".format([n for n,m in pop]))
        print("Generation: {}\t Best fitness: {:.2f}\t Avg fitness: {:.2f}\t Worst fitness: {:.2f}".format(gen,max(fitness),sum(fitness)/len(fitness),min(fitness)))  