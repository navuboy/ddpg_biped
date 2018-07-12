# RL4BIPED
Planar Bipedal walking robot in Gazebo environment using Deep Deterministic Policy Gradient(DDPG).
<p>
The autonomous walking of the bipedal walking robot is achieved using reinforcement learning algorithm called **Deep Deterministic Policy Gradient(DDPG)**.DDPG is one of the algorithms for **learning controls in continuous action spaces**.

Platforms and Package: ROS Kinetic, Gazebo, Nvidia GeForce GTX 1050 Ti GPU, Solidworks, TensorFLow, Numpy
</p>

***walker_gazebo*** contains the robot model(both **.stl** files & **.urdf** file) and also the gazebo launch file.

***walker_controller*** contains the reinforcement learning implementation of ****DDPG algorithm**** for control of the bipedal walking robot.

**Initial Learning Phase**
<p align= "center">
  <img src="walker_controller/src/training_1.gif/">
</p>

**Post Learning Phase - Stable Bipedal walking**
<p align= "center">
  <img src="walker_controller/src/trained.gif/">
</p>

**Project video link: https://www.youtube.com/watch?v=Q4N78P7cink**
