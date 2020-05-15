#!/usr/bin/env python

import multiprocessing

import rospy
from geometry_msgs.msg import Twist
from math import pow, atan2, sqrt

import numpy as np

from turtle_instance_laser_death_star import TurtleBot

k = 0
sp_x = [1,9,1,9]
sp_y = [1,9,9,1]

goal_x = [9,1,9,1]
goal_y = [9,1,1,9]

def multi_agents(agent_name,agent_obj,goal_x,goal_y,p_name):
    try:
        agent_obj = TurtleBot(agent_name)
        # agent_obj.start_point(sp_x,sp_y)
        agent_obj.begin()
        # agent_obj.move2goal_rvo(goal_x,goal_y)

    except rospy.ROSInterruptException:
        pass

user_input = int(input("Type no. of agents : "))
agent_names, agent_obj,p_name = [None] * (user_input), [None] * (user_input), [None] * (user_input)
sp_x,sp_y,goal_x,goal_y = [None] * (user_input),[None] * (user_input),[None] * (user_input),[None] * (user_input)

#Equal distribution for start_point
r = 5
c = [44,-24]
_angle = 2*(np.pi/(10))
_pad_angle_sp = 0
_pad_angle_goal = 0

#Temp fix for non-responding frist turtle
#define turtle 0 outside loop
# agent_names[0] = "robot_" + str(0)
# agent_obj[0] = "x" + str(0)
# sp_x[0] = c[0] + r*np.cos(_angle*0 + _pad_angle_sp)
# sp_y[0] = c[1] + r*np.sin(_angle*0 + _pad_angle_sp)
# goal_x[0] = c[0] - r*np.cos(_angle*0 + _pad_angle_goal)
# goal_y[0] = c[1] - r*np.sin(_angle*0 + _pad_angle_goal)
# p_name[0] = "p"+str(0)
# p_name[0] = multiprocessing.Process(target=multi_agents, args=(agent_names[k], agent_obj[k], sp_x[k], sp_y[k], goal_x[k], goal_y[k], p_name, ))
# print("AGENT: "+agent_names[0])
# print(goal_x[0],goal_y[0])
l=0
for j in range(user_input):
        i=j+2
        agent_names[j] = "robot_" + str(j)
        agent_obj[j] = "x" + str(j)
        # sp_x[j] = c[0] + r*np.cos(_angle*i + _pad_angle_sp)
        # sp_y[j] = c[1] + r*np.sin(_angle*i + _pad_angle_sp)
        # if(j<=48):
        #         goal_x[j] = c[0]
        #         goal_y[j] = c[1] +j
        # else:
                
        #         goal_x[j] = c[0]+5
        #         goal_y[j] = c[1] +l
        #         l = l+1
        # # goal_x[j] = c[0] - r*np.cos(_angle*i + _pad_angle_goal)
        # # goal_y[j] = c[1] - r*np.sin(_angle*i + _pad_angle_goal)

        # print("AGENT: "+agent_names[j])
        # print(goal_x[j],goal_y[j])

#print(agent_names)

for i in agent_names:
    p_name[k] = "p"+str(k)
    p_name[k] = multiprocessing.Process(target=multi_agents, args=(agent_names[k], agent_obj[k],goal_x[k], goal_y[k], p_name, ))
    k += 1

for i in p_name:
    i.start()
    rospy.sleep(10)
