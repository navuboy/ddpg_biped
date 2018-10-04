# Reinforcement Learning for Bipedal walking robot.
<p align= "center">
  <img src="walker_controller/src/biped.gif/">
</p>

## Planar Bipedal walking robot in Gazebo environment using Deep Deterministic Policy Gradient(DDPG).
The autonomous walking of the bipedal walking robot is achieved using reinforcement learning algorithm called <b>Deep Deterministic Policy Gradient(DDPG)<sup>1</sup></b>. DDPG is one of the algorithms for <b>learning controls in continuous action spaces</b>.
<p>The project details & the results of the experiment have been documented in the research manuscript, <b><a href="https://arxiv.org/abs/1807.05924v2">Bipedal walking robot using Deep Deterministic Policy Gradient</a></b></p>
This project was developed at the <a href="https://sites.google.com/site/compintellab/home"><b>Computational Intelligence Laboratory, IISc, Bangalore</b></a>.

## Dependencies & Packages:
- <a href="http://releases.ubuntu.com/16.04/">Ubuntu 16.04</a>
- <a href="http://wiki.ros.org/kinetic">ROS Kinetic</a>
- <a href="http://gazebosim.org/">Gazebo 7</a>
- <a href="https://www.tensorflow.org/">TensorFlow: 1.1.0 [with GPU support]</a> 
- <a href="https://gym.openai.com/docs/">gym: 0.9.3</a>
- Python 3.6

## File setup:
- ***walker_gazebo*** contains the robot model(both **.stl** files & **.urdf** file) and also the gazebo **launch** file.

- ***walker_controller*** contains the reinforcement learning implementation of **DDPG algorithm** for control of the bipedal walking robot.
 
 ## Initial Learning Phase
<p align= "center">
  <img src="walker_controller/src/training_1.gif/" height="350" width="600">
</p>

  ## Post Learning Phase - Stable Bipedal walking
<p align= "center">
  <img src="walker_controller/src/trained.gif/" height="350" width="600">
</p>

**<a href="https://goo.gl/1hwqJy*">Project video</a>**

**Note:** A stable bipedal walking was acheived after training the model using a Nvidia GeForce GTX 1050 Ti GPU enabled system for over 41 hours. The visualization for the horizontal boom(attached to the waist) in turned off.

## References:
<ol>
  <li>Lillicrap, Timothy P., et al.<b><a href="https://arxiv.org/abs/1509.02971"> Continuous control with deep reinforcement learning.</a></b> arXiv preprint arXiv:1509.02971 (2015).</li>
<li>Silver, David, et al.<b><a href="http://proceedings.mlr.press/v32/silver14.pdf"> Deterministic Policy Gradient Algorithms.</a></b> ICML (2014).</li>
</ol> 

## Project Collaborator(s): 
**<a href="https://github.com/ioarun">Arun Kumar</a>** 
