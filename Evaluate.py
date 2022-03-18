import numpy as np
import mujoco_py
import time

import MorphologyGeneration

if (__name__ == "__main__"):   
    MorphologyGeneration.MorphologyGeneration()

    model = mujoco_py.load_model_from_path('morphology.xml')
    sim = mujoco_py.MjSim(model)
    viewer = mujoco_py.MjViewer(sim)
    
    while True:
        #print(sim.data.ctrl)
        for motor in range(len(sim.data.ctrl)): 
            sim.data.ctrl[motor] = np.random.random()*2-1
        sim.step()
        print("\n",sim.get_state())
        viewer.render()