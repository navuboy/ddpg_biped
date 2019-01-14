- **cd to the walker_gazebo file(walker_gazebo/launch/) & launch the robot model in gazebo:**
```
$ cd ddpg_biped/walker_gazebo/launch/
$ roslaunch walker_gazebo.launch
```
- **Run gym_ddpg3.py after launching the model in Gazebo environment.**
```
$ cd ddpg_biped/walker_controller/src/ddpg2/
$ python3 gym_ddpg3.py
```
The <strong>reward function</strong> that is defined/shaped for the reinforcement problem is mentioned below (in <strong>gym_ddpg3.py</strong>):
```python 
    reward = -0.1  # when it used to run, used to be -0.1
    current_time = time.time()
    if (robot_state.outer_ring_inner_ring_theta - robot_state.last_outer_ring_inner_ring_theta) <= -0.09: #-0.001forward motion
         
        delta_time = current_time - robot_state.last_time
       
        reward += -((robot_state.outer_ring_inner_ring_theta - robot_state.last_outer_ring_inner_ring_theta))*10
    robot_state.last_time = current_time   
    robot_state.last_outer_ring_inner_ring_theta = robot_state.outer_ring_inner_ring_theta

    if robot_state.waist_z < -0.10: 
        reward += -100
        robot_state.done = True
        robot_state.fall = 1
  
    if robot_state.outer_ring_inner_ring_theta < -9.0:
        reward += 100
        robot_state.done = True
        robot_state.fall = 1
        print ('REACHED TO THE END!')
 ```
