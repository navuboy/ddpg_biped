# RL4BIPED - Reinforcement Learning for Bipedal walking robot
Planar Bipedal walking robot in Gazebo environment using Deep Deterministic Policy Gradient(DDPG).

The autonomous walking of the bipedal walking robot is achieved using reinforcement learning algorithm called <b>Deep Deterministic Policy Gradient(DDPG)</b>.DDPG is one of the algorithms for <b>learning controls in continuous action spaces</b>.
<p>The results of the experiment have been documented in the research manuscript: <b>https://arxiv.org/abs/1807.05924v2</b></p>

**Packages and Platforms:** 
- Ubuntu 16.04 
- ROS Kinetic (http://wiki.ros.org/kinetic)
- Gazebo 7 (http://gazebosim.org/)
- TensorFLow: 1.1.0 (with GPU support)
- gym: 0.9.3
- Python 2.7

## File setup:
- ***walker_gazebo*** contains the robot model(both **.stl** files & **.urdf** file) and also the gazebo launch file.

- ***walker_controller*** contains the reinforcement learning implementation of ****DDPG algorithm**** for control of the bipedal walking robot.
 
 ## Initial Learning Phase
<p align= "center">
  <img src="walker_controller/src/training_1.gif/">
</p>

  ## Post Learning Phase - Stable Bipedal walking
<p align= "center">
  <img src="walker_controller/src/trained.gif/">
</p>

**Project video link: https://goo.gl/1hwqJy**
<p>Note: A stable bipedal walking was acheived after training the model using a Nvidia GeForce GTX 1050 Ti GPU enable system for over 41 hours.</p>
