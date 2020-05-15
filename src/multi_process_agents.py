#!/usr/bin/env python

import multiprocessing

import rospy
from geometry_msgs.msg import Twist
from math import pow, atan2, sqrt

import numpy as np

from turtle_instance_laser_death_star import TurtleBot
'''
Description: This python file is responsible for creating multiple turtlebot instances
by using multiprocessing. 
'''
k = 0
sp_x = [1,9,1,9]
sp_y = [1,9,9,1]

goal_x = [9,1,9,1]
goal_y = [9,1,1,9]

def multi_agents(agent_name,agent_obj,p_name):
        '''
Description: Creates number of robots that user inputs. 
'''
    try:
        agent_obj = TurtleBot(agent_name)
        # agent_obj.start_point(sp_x,sp_y)
        agent_obj.begin()
        # agent_obj.move2goal_rvo(goal_x,goal_y)

    except rospy.ROSInterruptException:
        pass

user_input = int(input("Type no. of agents : "))
agent_names, agent_obj,p_name = [None] * (user_input), [None] * (user_input), [None] * (user_input)

#Equal distribution for start_point


#Temp fix for non-responding frist turtle

l=0
for j in range(user_input):
        i=j+2
        agent_names[j] = "robot_" + str(j)
        agent_obj[j] = "x" + str(j)
        

for i in agent_names:
    p_name[k] = "p"+str(k)
    p_name[k] = multiprocessing.Process(target=multi_agents, args=(agent_names[k], agent_obj[k], p_name, ))
    k += 1

for i in p_name:
    i.start()
    rospy.sleep(10)
