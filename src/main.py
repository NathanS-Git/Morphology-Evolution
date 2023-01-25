from doctest import IGNORE_EXCEPTION_DETAIL
import numpy as np
import traceback
import pickle
import os

import morphology_gen
import evaluate
import initialization
import mutate
import selection
import recombination


if (__name__ == "__main__"):

    if os.path.exists("Backup.dat"):
        print("RECOVERING...")
        # I was dealing with breaker issues so this allows for easy recovery
        file = open("Backup.dat",'rb')
        begin_gen,pop = pickle.load(file)
        file.close()
    else:
        pop = initialization.pop_init()
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
                #fitness.append(eval_morphology(file_path,gen*episode_increase_per_gen))
                fitness.append(evaluate.eval_morphology(file_path,static_eps_eval))
            except Exception:
                print("Bad morphology. Something went wrong.")
                traceback.print_exc()
                fitness.append(float('-inf'))
        
        parent1_i, parent2_i = selection.parent_selection(fitness)
        children = []
        for _ in range(replacements_per//2): # Six children
            children += recombination.breed(pop[parent1_i][1], pop[parent2_i][1])

        for child in children:
            if np.random.random() < micro_mutation_p:
                mutate.micro_mutation(child)
            if np.random.random() < macro_mutation_p:
                mutate.macro_mutation(child)

        replacements_i = selection.survival_selection(fitness, replacements_per)

        c_i = 0
        for r_i in replacements_i:
            pop[r_i] = (morphology_gen.gen_XML(children[c_i],gen),children[c_i])
            c_i += 1

        print(f"Next Gen Population : {[n for n,m in pop]}")
        print(f"Generation: {gen}\t Best fitness: {max(fitness):.2f}\t Avg fitness: {sum(fitness)/len(fitness):.2f}\t Worst fitness: {min(fitness):.2f}")  