# Evolving an optimal morphology for walking

This project utilizes MuJoCo through a slightly modified environment based on OpenAI's gym to train and evaluate different generated morphologies. The morphology are designed by a genetic algorithm, and evaluated with a reinforcement learning algorithm called TD3.

&nbsp;

## Overview

You can read the paper associated with the project [here](docs/paper.pdf), however, reading it won't be necessary as I'll provide the data here.

&nbsp;


### **Timeline of some test populations:**

The below population examples were evaluated over an increasing amount of episodes. It should also be noted the colors are not consistent between frames.

- Evolution of a test population *without* macro mutations

<img src="docs/Gen0.gif" width="500">

- Evolution of a test population *with* macro mutations

<img src="docs/Trial1.gif" width="500">

&nbsp;

### **Human designed morphology for comparison:**

Human designed 'Ant' morphology

- Model

<img src="docs/template.png" width="300">

- Score

<img src="docs/Ant.png" width="400">