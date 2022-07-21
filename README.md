# Evolving an optimal morphology for walking

This is my final undergraduate project. It essentially combines all my machine learning knowledge into a succinct project. This utilizes MuJoCo through a slightly modified environment based on OpenAI's gym to train and evaluate different generated morphologies. The morphology are designed by a genetic algorithm, and evaluated with a reinforcement learning algorithm called TD3.

&nbsp;

## Examples

You can read the paper associated with the project [here](docs/paper.pdf), however, reading it won't be necessary as I'll provide the data here.

&nbsp;

### **Examples of generated morphology:**
<img src="docs/Gen-0_0.png" width="200">
<img src="docs/Gen-0_1.png" width="200">
<img src="docs/Gen-0_2.png" width="200">
<img src="docs/Gen-0_3.png" width="200">
<img src="docs/Gen-0_6.png" width="200">
<img src="docs/Gen-0_7.png" width="200">
<img src="docs/Gen-0_8.png" width="200">
<img src="docs/Gen-0_9.png" width="200">
<img src="docs/Gen-6_2.png" width="200">
<img src="docs/Gen-8_2.png" width="200">
<img src="docs/Gen-9_0.png" width="200">
<img src="docs/Gen-9_1.png" width="200">
<img src="docs/Gen-9_2.png" width="200">
<img src="docs/Gen-9_3.png" width="200">
<img src="docs/Gen-9_4.png" width="200">
<img src="docs/Gen-9_5.png" width="200">

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

&nbsp;

## Future

Despite this originally being just a project for my undergrad, I have plans on coming back to this and improving it. Things such as modifying the generation and modification of morphology to encourage more human-like configurations, as well as large performance boosts through multithreading the training, among other things. 