# RL4BIPED - Reinforcement Learning for Bipedal walking robot
Planar Bipedal walking robot in Gazebo environment using Deep Deterministic Policy Gradient(DDPG).
<p>
The autonomous walking of the bipedal walking robot is achieved using reinforcement learning algorithm called <b>Deep Deterministic Policy Gradient(DDPG)</b>.DDPG is one of the algorithms for <b>learning controls in continuous action spaces</b>.

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

**Project video link: https://youtu.be/Q4N78P7cink**
<p>Note: A stable bipedal walking was acheived after training the model using a Nvidia GeForce GTX 1050 Ti GPU enable system for over 41 hours.</p>
