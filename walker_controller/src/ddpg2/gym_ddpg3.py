# ddpg that works.

import filter_env
from ddpg import *
import gc
gc.enable()


import roslib
import rospy
import rostopic
import random
import time
import math
import csv
from std_srvs.srv import Empty
from gazebo_msgs.srv import SetModelConfiguration

from control_msgs.msg import JointControllerState
from sensor_msgs.msg import JointState
from gazebo_msgs.msg import LinkStates
from gazebo_msgs.msg import ContactsState
from std_msgs.msg import Float64
from std_msgs.msg import String
from sensor_msgs.msg import Joy

import threading
from scipy.interpolate import interp1d

ENV_NAME = 'BipedalWalker-v2'
EPISODES = 500000
TEST = 10

# src_file = 'walking-data-preprocessed-3.csv'
# src_file = "along-z-5-preprocessed.csv"
src_file = "navneet-walks-for-biped.csv"
reward_file = "reward_file.csv"
trajectory_file = "trajectory_file.csv"
# src_file = "walking-data-march.csv"
# src_file = "walking-data-ga-2.csv"

pubHipR = rospy.Publisher('/waist_thighR_position_controller/command', Float64, queue_size=10)
pubHipL = rospy.Publisher('/waist_thighL_position_controller/command', Float64, queue_size=10)
pubKneeR = rospy.Publisher('/thighR_shankR_position_controller/command', Float64, queue_size=10)
pubKneeL = rospy.Publisher('/thighL_shankL_position_controller/command', Float64, queue_size=10)
reset_simulation = rospy.ServiceProxy('/gazebo/reset_world', Empty)
reset_joints = rospy.ServiceProxy('/gazebo/set_model_configuration', SetModelConfiguration)
unpause = rospy.ServiceProxy('/gazebo/unpause_physics', Empty)
pause = rospy.ServiceProxy('/gazebo/pause_physics', Empty)

fall = 0


rospy.init_node('walker_control_script')
rate = rospy.Rate(50)

class RobotState(object):
    def __init__(self):
        self.waist_z = 0.0
        self.waist_y = 0.0
        self.outer_ring_inner_ring_theta = 0.0
        self.hipr_theta = 0.0
        self.hipr_theta_dot = 0.0
        self.hipl_theta = 0.0
        self.hipl_theta_dot = 0.0
        self.kneer_theta = 0.0
        self.kneer_theta_dot = 0.0
        self.kneel_theta = 0.0
        self.kneel_theta_dot = 0.0
        self.vel_y = 0.0
        self.vel_z = 0.0
        self.footr_contact = 0
        self.footl_contact = 0
        self.robot_state = [self.vel_y, self.vel_z, self.hipr_theta, self.hipr_theta_dot, self.hipl_theta, self.hipl_theta_dot, \
        self.kneer_theta, self.kneer_theta_dot, self.kneel_theta, self.kneel_theta_dot, self.footr_contact, self.footl_contact]

        self.latest_reward = 0.0
        self.best_reward = -100000000000000.0
        self.episode = 0
        self.last_outer_ring_inner_ring_theta = 0.0
        self.last_time = 0.0

        self.fall = 0
        self.done = False
        self.count_of_1 = 0
        self.avg_reward = 0.0

class Publisher(threading.Thread):
    def __init__(self, pubHipR, pubHipL, pubKneeR, pubKneeL, rate):
        threading.Thread.__init__(self)
        self.counter = 0
        self.pubHipR = pubHipR
        self.pubHipL = pubHipL
        self.pubKneeR = pubKneeR
        self.pubKneeL = pubKneeL
        self.rate = rate
        

    def run(self):
        publisher(self.pubHipR, self.pubHipL, self.pubKneeR, self.pubKneeL, self.rate, self.counter)

robot_state = RobotState()


def reset():
    # ['waist_thighR', 'waist_thighL', 'thighR_shankR', 'thighL_shankL', 'outer_ring_inner_ring', 'inner_ring_boom', 'boom_waist']
    rospy.wait_for_service('gazebo/reset_world')
    try:
        reset_simulation()
    except(rospy.ServiceException) as e:
        print ('reset_world failed!')


    rospy.wait_for_service('gazebo/set_model_configuration')

    try:
        reset_joints("walker", "robot_description", ['boom_waist', 'outer_ring_inner_ring', 'thighL_shankL', 'thighR_shankR', 'waist_thighL', 'waist_thighR'], [0.0,0.0,0.0, 0.0, 0.0, 0.0])
        robot_state.last_outer_ring_inner_ring_theta = 0.0
    except (rospy.ServiceException) as e:
        print ('reset_joints failed!')

    rospy.wait_for_service('/gazebo/pause_physics')
    try:
        pause()
    except (rospy.ServiceException) as e:
        print ('rospause failed!')

    set_robot_state()

    # print "called reset()"
     


def set_robot_state():
    robot_state.robot_state = [robot_state.vel_y, robot_state.vel_z, robot_state.hipr_theta, robot_state.hipr_theta_dot, robot_state.hipl_theta, robot_state.hipl_theta_dot, \
        robot_state.kneer_theta, robot_state.kneer_theta_dot, robot_state.kneel_theta, robot_state.kneel_theta_dot, robot_state.footr_contact, robot_state.footl_contact]

def take_action(action):
    rospy.wait_for_service('/gazebo/unpause_physics')
    
    try:
        unpause()
    except (rospy.ServiceException) as e:
        print ('/gazebo/pause_physics service call failed')

    pubHipR.publish(action[0])
    pubKneeR.publish(action[1])
    pubHipL.publish(action[2])
    pubKneeL.publish(action[3])
    
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
    rate.sleep()
    return reward, robot_state.done


def callbackJointStates(data):
    # ['boom_waist', 'outer_ring_inner_ring', 'thighL_shankL', 'thighR_shankR', 'waist_thighL', 'waist_thighR']
    # if vel == 0 ['waist_thighR', 'waist_thighL', 'thighR_shankR', 'thighL_shankL', 'outer_ring_inner_ring', 'boom_waist']
    robot_state.data = data

    if len(data.velocity)!=0:

        robot_state.vel_z = data.velocity[0]
        robot_state.vel_y = data.velocity[1]
        robot_state.kneel_theta_dot = data.velocity[2]
        robot_state.kneer_theta_dot = data.velocity[3]
        robot_state.hipl_theta_dot = data.velocity[4]
        robot_state.hipr_theta_dot = data.velocity[5]

        robot_state.waist_z = data.position[0]
        robot_state.waist_y = data.position[1]
        robot_state.outer_ring_inner_ring_theta = data.position[1]
        robot_state.kneel_theta = data.position[2]
        robot_state.kneer_theta = data.position[3]
        robot_state.hipl_theta = data.position[4]
        robot_state.hipr_theta = data.position[5]
    else:
        robot_state.vel_z = 0
        robot_state.vel_y = 0
        robot_state.kneel_theta_dot = 0
        robot_state.kneer_theta_dot = 0
        robot_state.hipl_theta_dot = 0
        robot_state.hipr_theta_dot = 0

        robot_state.waist_z = 0
        robot_state.waist_y = 0
        robot_state.outer_ring_inner_ring_theta = 0
        robot_state.kneel_theta = 0
        robot_state.kneer_theta = 0
        robot_state.hipl_theta = 0
        robot_state.hipr_theta = 0
    
    
    set_robot_state()
    # rate.sleep()


def callbackSub(data):
    set_robot_state()
    
def callbackContactShankR(data):
    if not data.states:
        robot_state.footr_contact = 0
    else:
        robot_state.footr_contact = 1


def callbackContactShankL(data):
    if not data.states:
        robot_state.footl_contact = 0
    else:
        robot_state.footl_contact = 1

def listener():
    print ('listener')
    
    rospy.Subscriber("/joint_states", JointState, callbackJointStates)
    rospy.Subscriber("/footR_contact_sensor_state", ContactsState, callbackContactShankR)
    rospy.Subscriber("/footL_contact_sensor_state", ContactsState, callbackContactShankL)
    

def publisher(pubHipR, pubHipL, pubKneeR, pubKneeL, rate, counter):

    while not rospy.is_shutdown():

        # writing rewards in the csv file
        file = open(reward_file, 'wt')
        writer = csv.writer(file)
        writer.writerow(['avg_reward'])
        
        env = [12, 4] # [state_dim, action_dim]
        agent = DDPG(env)
        for episode in range(EPISODES):
            reset()
            state = robot_state.robot_state
            # Train
            
            for steps in range(1600):
                action = agent.noise_action(state)
                reward,done = take_action(action)
                next_state = robot_state.robot_state
                agent.perceive(state,action,reward,next_state,done)
                state = next_state
                if done:
                    robot_state.done = False
                    break
            robot_state.episode = episode
            # Testing:
            if episode % 100 == 0 and episode > 100:
                traj_file = open(trajectory_file, 'wt')
                traj_writer = csv.writer(traj_file, delimiter='\t')
                traj_writer.writerow(['z', 'y', 'vel_z', 'vel_y', 'hr', 'hl', 'hr_dot', 'hl_dot', 'kr', 'kl', 'kr_dot', 'kl_dot', 'fr', 'fl'])

                print ('testing')
                total_reward = 0
                count_of_1 = 0
                for i in range(TEST):
                    reset()
                    state = robot_state.robot_state
                    for steps in range(1600):
                        action = agent.action(state) # direct action for test
                        reward,done = take_action(action)
                        state = robot_state.robot_state
                        traj_writer.writerow([robot_state.waist_z, robot_state.waist_y, robot_state.vel_z, robot_state.vel_y, robot_state.hipr_theta, robot_state.hipl_theta, robot_state.hipr_theta_dot, robot_state.hipl_theta_dot, robot_state.kneer_theta, robot_state.kneel_theta, robot_state.kneer_theta_dot, robot_state.kneel_theta_dot, robot_state.footr_contact, robot_state.footl_contact])
                        traj_file.flush()
                        if reward == 1:
                            count_of_1 += 1
                        total_reward += reward
                        if done:
                            robot_state.done = False
                            break
                ave_reward = total_reward/TEST
                robot_state.latest_reward = ave_reward
                if ave_reward > robot_state.best_reward:
                    robot_state.best_reward = ave_reward
                robot_state.avg_reward = ave_reward
                writer.writerow([ave_reward])
                file.flush()
            
                print ('episode: '),episode,'Evaluation Average Reward:',ave_reward
                # print "best_reward: ", robot_state.best_reward
   
def main():

    # Create new threads
    thread = Publisher(pubHipR, pubHipL, pubKneeR, pubKneeL, rate)

    # Start new Threads
    thread.start()
    listener()

if __name__ == '__main__':
    main()
