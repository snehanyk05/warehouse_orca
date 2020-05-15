#!/usr/bin/env python
#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Twist, PoseStamped, PoseWithCovarianceStamped
from tf.transformations import euler_from_quaternion
from nav_msgs.msg import Odometry, Path
from sensor_msgs.msg import LaserScan
import sys
from std_msgs.msg import String
from rvo.msg import Information
from death_star.srv import smartPlan
from warehouse_manager.srv import Robot_Task_Complete, Robot_Task_Request
from random import randint
import numpy as np
import time
import math
import os
GET_TB3_DIRECTION = 0
TB3_DRIVE_FORWARD = 1
TB3_RIGHT_TURN = 2
TB3_LEFT_TURN = 3
TB3_TURN_AROUND = 4

from math import pow, atan2, sqrt, cos, sin, atan, asin

class TurtleBot:

    #global all_agents_pose_dict
    all_agents_pose_dict = {}
    all_wps = {}
    def __init__(self, agent_name):
		# Creates a node with name of the agent
        self.agent_name = agent_name
        # print(self.agent_name)
        rospy.init_node(self.agent_name)

		
        self.velocity_publisher = rospy.Publisher('/'+self.agent_name+'/cmd_vel', Twist, queue_size=10)
        self.init_pose_publisher = rospy.Publisher('/'+self.agent_name+'/initpose', PoseStamped, queue_size=10)
        self.goal_publisher = rospy.Publisher('/'+self.agent_name+'/finalgoal', PoseStamped, queue_size=10)

        self.publish_information = rospy.Publisher("/common_information", Information, queue_size=10)
        self.publish_obstacles_wp = rospy.Publisher("/map_update", Num, queue_size=10)

        self.pub_pose = Odometry()
        self.inf = Information()
        self.start_time=time.time()
        self.plan_time = time.time()
        self.track_time = time.time()
        self.state_description = 0
        self.turtlebot3_state_num = 0
        self.busy = False
        self.replan = False
        self.result = {}
        self.x = 0
        self.y = 0
        self.path_received = False
        self.tb3_pose_ = 0.0
        self.prev_tb3_pose_ = 0.0
        self.escape_range_ = math.radians(30)
        self.check_forward_dist_ = 0.7
        self.check_side_dist_    = 0.6
        self.scan_data_ = []
        ### Subscriber ###

        # self.update_pose is called when a message of type Pose is received.
        self.pose_subscriber = rospy.Subscriber('/'+self.agent_name+'/base_pose_ground_truth', Odometry, self.update_pose)
        self.scan_subscriber = rospy.Subscriber('/'+self.agent_name+'/base_scan', LaserScan, self.clbk_laser)
        # self.nav_path_subscriber = rospy.Subscriber('/'+self.agent_name+'/nav_path', Path, self.found_path)
        
        # self.recieve_from_information_channel is called when a message of type information is received.
        rospy.Subscriber("/common_information", Information, self.recieve_from_information_channel)
        self.nav_path = Path()
        self.odom = Odometry()
        self.theta = 0;
        self.rate = rospy.Rate(10)
        
        
        self.previous_pose = Odometry()
#-----------------------------------------------------------------------------------------#
# Functions related to topics
    def clbk_laser(self, msg):
        scan_angle = [0, 30, 330]
        # self.scan_data_ = []
        self.scan_data_ = {
            'right':  min(min(msg.ranges[0:143]), 10),
            'fright': min(min(msg.ranges[144:287]), 10),
            'front':  min(min(msg.ranges[288:431]), 10),
            'fleft':  min(min(msg.ranges[432:575]), 10),
            'left':   min(min(msg.ranges[576:719]), 10),
        }
        # for num in range(0,3):
        #     # print(num)
        #     if (math.isinf((msg.ranges[scan_angle[num]]))):
            
        #         self.scan_data_.append(msg.range_max)
            
        #     else:
            
        #         self.scan_data_.append(msg.ranges[scan_angle[num]])
            
        
        
        # else
        # {

        # }
    # print(msg)
    # --------------------------------------------------------
        # regions = {
        #     'right':  min(min(msg.ranges[0:143]), 10),
        #     'fright': min(min(msg.ranges[144:287]), 10),
        #     'front':  min(min(msg.ranges[288:431]), 10),
        #     'fleft':  min(min(msg.ranges[432:575]), 10),
        #     'left':   min(min(msg.ranges[576:719]), 10),
        # }
        
        self.take_action()

    def take_action_1(self, regions):
        # msg = Twist()
        linear_x = 0
        angular_z = 0


            # self.state_description = 'case 1 - nothing'
            # state_description = 'case 2 - front'
            # state_description = 'case 3 - fright'
            # state_description = 'case 4 - fleft'
            # state_description = 'case 5 - front and fright'
            # state_description = 'case 6 - front and fleft'
            # state_description = 'case 7 - front and fleft and fright'
            # state_description = 'case 8 - fleft and fright'
        # if(regions['front'] > 1):
        #     print("In here front")
        #     if(regions['fleft'] > 1):
        #         print("In here fleft")
        #         if(regions['fright'] > 1):
        #             print("In here fright")
        #             if(regions['left'] > 1):
        #                 print("In here left")
        #                 if(regions['right'] > 1):
        #                     print("In here right")
            
        if regions['front'] >= 0.85 and regions['fleft'] >= 0.85 and regions['fright'] >= 0.85 and regions['right'] >= 0.85 and regions['left'] >= 0.85:
            # print('case 1 - nothing')
            self.state_description = 1
        elif regions['right'] <= 0.85:
            # print('case 2 - right')
            self.state_description = 2
        elif regions['left'] <= 0.85:
            # print('case 3 - left')
            self.state_description = 3 
        elif regions['front'] <= 0.85 and regions['fleft'] >= 0.85 and regions['fright'] >= 0.85:
            # print('case 4 - front')
            self.state_description = 4
        elif regions['front'] >= 0.85 and regions['fleft'] >= 0.85 and regions['fright'] <= 0.85:
            # print('case 5 - fright')
            self.state_description = 5
        elif regions['front'] >= 0.85 and regions['fleft'] <= 0.85 and regions['fright'] >= 0.85:
            # print('case 6 - fleft')
            self.state_description = 6
        elif regions['front'] <= 0.85 and regions['fleft'] >= 0.85 and regions['fright'] <= 0.85:
            # print('case 7 - front and fright')
            self.state_description = 7
        elif regions['front'] <= 0.85 and regions['fleft'] <= 0.85 and regions['fright'] >= 0.85:
            # print('case 8 - front and fleft')
            self.state_description = 8
        elif regions['front'] <= 0.85 and regions['fleft'] <= 0.85 and regions['fright'] <= 0.85:
            # print('case 9 - f/ront and fleft and fright')
            self.state_description = 9
        elif regions['front'] >= 0.85 and regions['fleft'] <= 0.85 and regions['fright'] <= 0.85:
            # print('case 10 - fleft and fright')
            self.state_description = 10
        else:
            # state_description = 'unknown case'
            self.state_description = 11
            print("case 11 - UNNNNNKNOWNNN")
            rospy.loginfo(self.agent_name+'; '+str(self.state_description)+":::::::")
            rospy.loginfo(regions)
            exit(0)

        # rospy.loginfo(state_description)

        # self.vel_msg = Twist()
        # self.vel_msg.linear.x = linear_x
        # self.vel_msg.angular.z = angular_z
        # self.velocity_publisher.publish(self.vel_msg)
        # self..linear.x = linear_x
        # msg.angular.z = angular_z
        # self.velocity_publisher.publish(msg)
    def getDirection(self):
        # print(self.scan_data_)
        if (self.scan_data_['front'] > self.check_forward_dist_):
            
                if ((self.scan_data_['left'] < self.check_side_dist_) or (self.scan_data_['fleft'] < self.check_side_dist_)):
                    # print("obstacle on left turn right")
                    self.prev_tb3_pose_ = self.tb3_pose_
                    self.turtlebot3_state_num = TB3_RIGHT_TURN
                
                elif ((self.scan_data_['right'] < self.check_side_dist_) or (self.scan_data_['fright'] < self.check_side_dist_)):
                    # print("obstacle on right turn left")
                    self.prev_tb3_pose_ = self.tb3_pose_
                    self.turtlebot3_state_num = TB3_LEFT_TURN
                
                else:
                
                    self.turtlebot3_state_num = TB3_DRIVE_FORWARD
                
            

        if (self.scan_data_['front'] < self.check_forward_dist_):
                
                if ((self.scan_data_['left'] < self.check_side_dist_) and (self.scan_data_['right'] < self.check_side_dist_) and (self.scan_data_['fright'] < self.check_side_dist_) and (self.scan_data_['fleft'] < self.check_side_dist_)):
                
                    self.prev_tb3_pose_ = self.tb3_pose_
                    self.turtlebot3_state_num = TB3_TURN_AROUND
        
                elif ((self.scan_data_['left'] < self.check_side_dist_) and (self.scan_data_['right'] < self.check_side_dist_)):
                
                    self.prev_tb3_pose_ = self.tb3_pose_
                    self.turtlebot3_state_num = TB3_TURN_AROUND
                elif  ((self.scan_data_['fright'] < self.check_side_dist_) and (self.scan_data_['fleft'] < self.check_side_dist_)): 
                    self.prev_tb3_pose_ = self.tb3_pose_
                    self.turtlebot3_state_num = TB3_TURN_AROUND
                elif ((self.scan_data_['fright'] < self.check_side_dist_) and (self.scan_data_['right'] < self.check_side_dist_)): 
                    self.prev_tb3_pose_ = self.tb3_pose_
                    self.turtlebot3_state_num = TB3_LEFT_TURN 
                elif ((self.scan_data_['fleft'] < self.check_side_dist_) and (self.scan_data_['left'] < self.check_side_dist_)): 
                    self.prev_tb3_pose_ = self.tb3_pose_
                    self.turtlebot3_state_num = TB3_RIGHT_TURN    
                else:
                    self.prev_tb3_pose_ = self.tb3_pose_
                    self.turtlebot3_state_num = TB3_RIGHT_TURN
            
            
    def take_action(self):
    
    
        self.getDirection()
        
        if(self.turtlebot3_state_num == TB3_DRIVE_FORWARD):
                # updatecommandVelocity(LINEAR_VELOCITY, 0.0);
                # self.turtlebot3_state_num = GET_TB3_DIRECTION
                self.state_description = 1
                

        elif(self.turtlebot3_state_num == TB3_RIGHT_TURN):
                if (math.fabs(self.prev_tb3_pose_ - self.tb3_pose_) >= self.escape_range_):
                    # self.turtlebot3_state_num = GET_TB3_DIRECTION;
                    self.take_action()
                else:
                    self.state_description = 2
                    # updatecommandVelocity(0.0, -1 * ANGULAR_VELOCITY);
                

        elif(self.turtlebot3_state_num == TB3_LEFT_TURN):
                if (math.fabs(self.prev_tb3_pose_ - self.tb3_pose_) >= self.escape_range_):
                    self.take_action()
                else:
                    self.state_description = 3
                    # updatecommandVelocity(0.0, ANGULAR_VELOCITY);
        elif(self.turtlebot3_state_num == TB3_TURN_AROUND):
                self.state_description = 4        

        else:
                self.state_description = 5
        # print("In state desc "+str(self.state_description))       
    

    def state_velocities(self):
        linear_x = 0
        angular_z = 0
        
        if self.state_description == 1:
            linear_x = 0
            angular_z = 0
        elif self.state_description == 2:
            linear_x = 0
            angular_z = -1.5708
        elif self.state_description == 3:
            linear_x = 0
            angular_z = 1.5708
        elif self.state_description == 4:
            linear_x = 0
            angular_z = -3.14159
        else: 
            rospy.loginfo("Invalid")
            linear_x = 0
            angular_z = 0
            # exit(0)
        # print(self.state_description)
        self.vel_msg = Twist()
        self.vel_msg.linear.x = linear_x
        self.vel_msg.angular.z = angular_z

    
    def state_velocities_1(self):
        linear_x = 0
        angular_z = 0
        
        if self.state_description == 1:
            linear_x = 0
            angular_z = 0
        elif self.state_description == 2:
            linear_x = 0
            angular_z = 0.1
        elif self.state_description == 3:
            linear_x = 0
            angular_z = -0.2


        elif self.state_description == 4:
            linear_x = 0
            angular_z = 0.3
        elif self.state_description == 5:
            linear_x = 0
            angular_z = 0.3
        elif self.state_description == 6:
            linear_x = 0
            angular_z = -0.3
        elif self.state_description == 7:
            linear_x = 0
            angular_z = 0.3
        elif self.state_description == 8:
            linear_x = 0
            angular_z = -0.3
        elif self.state_description == 9:
            linear_x = 0
            angular_z = 0.3
        elif self.state_description == 10:
            linear_x = 0.3
            angular_z = 0
        else: 
            rospy.loginfo("Invalid")
            linear_x = 0
            angular_z = -0.3
            # exit(0)
        # print(self.state_description)
        self.vel_msg = Twist()
        self.vel_msg.linear.x = linear_x
        self.vel_msg.angular.z = angular_z


    def checkStuckInRadius(self):
    
        if(self.euclidean_distance(self.previous_pose) <= 4):
            # print("HERE IN CHECK RAD")
            self.count_stuck = self.count_stuck + 1

            if(self.count_stuck > 5):
                # print("Changing angular velocity for "+ self.agent_name)
                self.vel_msg = Twist()
                self.vel_msg.linear.x = 0
                self.vel_msg.angular.z = -0.7
                self.velocity_publisher.publish(self.vel_msg)
                self.count_stuck = 0
                self.track_time = time.time();
        else:
            print("HERE IN ELSE RAD")
            self.previous_pose = self.odom
            self.count_stuck = 0
    # def found_path(self, data):
    #     """Callback function which is called when a new message of type Pose is
    #     received by the subscriber."""
        
    #     # print(data)
    #     self.path_received = True
    #     self.nav_path = data
    #     print("found path")
    def update_pose(self, data):
        """Callback function which is called when a new message of type Pose is
        received by the subscriber."""
        # print(data)
        # print("POSSSSSSSSSSSE:",self.pose)
        
        self.odom = data
        # if(time.time() - self.track_time > 30 ):
        
        #     self.checkStuckInRadius()
        
        rot_q = data.pose.pose.orientation
        (roll,pitch,theta) = \
            euler_from_quaternion ([rot_q.x,rot_q.y,rot_q.z,rot_q.w])
        self.theta = theta
        self.odom.pose.pose.position.x = round(self.odom.pose.pose.position.x, 4)
        self.odom.pose.pose.position.y = round(self.odom.pose.pose.position.y, 4)

        # ----------------------------------------------------------
        siny = 2.0 * (data.pose.pose.orientation.w * data.pose.pose.orientation.z + data.pose.pose.orientation.x * data.pose.pose.orientation.y);
        cosy = 1.0 - 2.0 * (data.pose.pose.orientation.y * data.pose.pose.orientation.y + data.pose.pose.orientation.z * data.pose.pose.orientation.z);  

        self.tb3_pose_ = atan2(siny, cosy);

    def publish_to_information_channel(self,t):
        i = Information()
        i.wp = self.nav_path.poses
        i.agent_name = t
        i.agent_pose_x = self.odom.pose.pose.position.x
        i.agent_pose_y = self.odom.pose.pose.position.y
        i.agent_heading = self.heading
        # i.agent_vel_mag = self.vel_msg.linear.x
        var_exists = 'self.vel_msg' in locals() or 'self.vel_msg' in globals()
        if(var_exists):
            i.agent_vel_mag = self.vel_msg.linear.x            
        else:
            # print('here')
            i.agent_vel_mag = 0
        #print("Published the turtle node name on topic /common_information")
        self.publish_information.publish(i)
        self.rate.sleep()

    def recieve_from_information_channel(self,data):
        self.inf = data
        self.name_temp = self.inf.agent_name
        self.x_temp = self.inf.agent_pose_x
        self.y_temp = self.inf.agent_pose_y
        self.heading_temp = self.inf.agent_heading
        self.vel_mag_temp = self.inf.agent_vel_mag
        self.all_wps.update({self.name_temp: self.inf.wp})
        self.pose_updated = [self.x_temp, self.y_temp, self.heading_temp, self.vel_mag_temp]
        self.all_agents_pose_dict.update({self.name_temp: self.pose_updated})
# end
#-----------------------------------------------------------------------------------------#

#-----------------------------------------------------------------------------------------#
# Helper functions
    def euclidean_distance(self, goal_pose):
        # print("EUCLID"+self.agent_name)
        # print(goal_pose.pose.pose.position, self.odom.pose.pose.position  )
        """Euclidean distance between current pose and the goal."""
        return sqrt(pow((goal_pose.pose.pose.position.x - self.odom.pose.pose.position.x), 2) +
                    pow((goal_pose.pose.pose.position.y - self.odom.pose.pose.position.y), 2))
    def euclidean_distance_pose(self, pose):
        # print("EUCLID"+self.agent_name)
        # print(goal_pose.pose.pose.position, self.odom.pose.pose.position  )
        """Euclidean distance between current pose and the goal."""
        return sqrt(pow((pose.pose.position.x - self.odom.pose.pose.position.x), 2) +
                    pow((pose.pose.position.y - self.odom.pose.pose.position.y), 2))
    # sets heading towards given direction
    def set_heading(self,theta):
        rospy.sleep(0.1)

    # sets heading towards given co-ordinates
    def set_goal_heading(self,x,y):
        rospy.sleep(0.1)
        self.heading = atan2(y - self.odom.pose.pose.position.y, x - self.odom.pose.pose.position.x)
        # self.turtle_teleport(self.pose.x,self.pose.y,self.heading)
        #print(self.heading)
        return



    def in_VO(self,h):
        for i in self.VO:
            if(self.VO[i][0] < h[i] < self.VO[i][1]):
                return True
                break
        return False

    def in_RVO(self,h):
        #use sets for optimized code using "if h in self.RVO"
        # print("THETA AND RVO "+self.agent_name, h,self.RVO)
        for i in self.RVO:
            if(self.RVO[i][0] < h < self.RVO[i][1]):
                return True
                break
        return False

    def update_RVO(self,v_mag,turtle_name=0,r=2):
        rr = 2
        #calc the relative velocity of the agent and choosen other agent
        self._rel_heading = {}
        self.point_to_agent_heading = {}
        self._omega = {}
        self.VX = {}
        self.VO = {}
        self.RVO = {}
        self.time_to_collision = {}
        rospy.sleep(0.01)

        self.present_temp_h = round(self.theta,rr)

        #neighbouring region is a circle of 3 units
        self.NR = 3

        self._least_distance = 10
        # print (self.all_agents_pose_dict)
        # k, value in list(kwargs.items())
        for i, value in list(self.all_agents_pose_dict.items()):
            if(i != self.agent_name):
                #calc distance between agent and oher agent/obstacle
                self._distance = round(sqrt(pow((value[0] - self.odom.pose.pose.position.x), 2) + pow((value[1] - self.odom.pose.pose.position.y), 2)),rr)

                #if it lies in the NR, consider it in calculating RVO
                if(self._distance < self.NR):
                    #calc the relative velocity of the agent and choosen other agent
                    self._rel_v_x =  v_mag * cos(self.theta) - value[3] * cos(value[2])
                    self._rel_v_y =  v_mag * sin(self.theta) - value[3] * sin(value[2])
                    self._rel_heading[i] = round(atan2(self._rel_v_y,self._rel_v_x),rr)

                    # VO finder :: Should output a range of headings into an 2D array
                    self.point_to_agent_heading[i] = round(atan2((value[1] - self.odom.pose.pose.position.y),(value[0] - self.odom.pose.pose.position.x)),rr)
                    #can also use np.clip
                    try:
                        # print("DIST "+self.agent_name ,self._distance)
                        if(self._distance > 0 ):
                            self._omega[i] = round(asin(r/self._distance),rr)
                        else:
                            self._omega[i] = round(np.pi/2,rr)
                    except ValueError:
                        self._omega[i] = round(np.pi/2,rr)

                    #time to collision
                    # should know distance and relative velocity in the direction of the obstacle
                    # if negative, it means collision will not occur

                    c1 = self._rel_v_x - (v_mag * (cos(self.theta))) <= 0
                    c2 = self._rel_v_x - (v_mag * (cos(value[2])))<= 0
                    c3 = self._rel_v_y - (v_mag * (sin(self.theta)))<= 0
                    c4 = self._rel_v_y - (v_mag * (sin(value[2])))<= 0

                    self.time_to_collision[i] = np.inf
                    if(c1 | c2 | c3 | c4):
                        temp = abs(v_mag * (cos(self.theta) - cos(value[2])))
                        if temp != 0:
                            self.time_to_collision[i] = abs(value[0] - self.odom.pose.pose.position.x)/temp
                        
                            

                    #Instead of checking if v_A is in VO, im checking if v_AB is inside something called "VX"
                    #But for RVO, we are adding v_A and v_B to VX.
                    # This is computationally easier
                    self.VX[i] = (np.asarray([self.point_to_agent_heading[i] - self._omega[i],self.point_to_agent_heading[i] + self._omega[i]]))
                    #####find v_A by adding v_B to VX (both mag and dir)
                    #self.VO[i] = self.VX[i] + value[2]
                    self.num = v_mag * sin(self.present_temp_h) + value[3] * sin(value[2])
                    self.den = v_mag * cos(self.present_temp_h) + value[3] * cos(value[2])

                    self.RVO[i] = (self.VX[i] + atan2(self.num,self.den))/2
                    #Uncomment the below line if you want the code to behave like VO
                    #self.RVO[i] = self.VX[i]

                    if (self._distance < self._least_distance):
                        self._least_distance = self._distance

        self.vel_msg.linear.x = (self._least_distance/4.5)/self.NR
        #print(self.time_to_collision)

                    #print(self.agent_name)
                    #print("A2A heading:")
                    #print(self._rel_heading[i])
                    #print("Omega:")
                    #print(self._omega[i])


    # Returns True when called if the agent is on collision course
    def collision(self):
        if(self.in_RVO(self.theta) == True):
            #if True, return True. Else, False.
            return True
        return False

    #Returns a new velocity that is outside VO
    def choose_new_velocity_VO(self):
        #Find the nearest heading that is outside the VO
        self.desired_heading = atan2(self.goal_pose.pose.pose.position.y - self.odom.pose.pose.position.y, self.goal_pose.pose.pose.position.x - self.odom.pose.pose.position.x)
        self._headings_array = np.round(np.arange(-np.pi,np.pi,0.01),2)

        # if not available, self.inside will return None.
        self.best_min = None

        #Find the nearest heading that is outside the VO
        self.temp_array_marginals = np.array([])
        # print(self.VO)
        for i in self.VO:
            self.temp_array_marginals = np.append(self.temp_array_marginals, self.VO[i])
            #self.temp_temp_temp = self.VO[i][0]
        self._h = np.round(self.temp_array_marginals, 2)
        for i in range(len(self._h)):
            if(i%2==0):
                k = self._h[i]
                while(k <= np.round(self._h[i+1],2)):
                    self._headings_array = np.delete(self._headings_array, np.where(self._headings_array == np.round(k,2)))
                    k+=0.01
        # print("===")
        # print("RVO is :")
        # print(self._h)
        self.idx = np.abs(self._headings_array - self.desired_heading -0.1).argmin()
        self.best_min = self._headings_array[self.idx]
        # print("desired heading :")
        # print(self.desired_heading)
        # print("choosen direction is")
        # print(self.best_min)
        # print("===")
        #rospy.sleep(1)
        return self.best_min

    #Returns a new velocity that is outside RVO
    def choose_new_velocity_RVO(self):
        rr = 2
        incr = 0.01
        self.desired_heading = atan2(self.goal_pose.pose.pose.position.y - self.odom.pose.pose.position.y, self.goal_pose.pose.pose.position.x - self.odom.pose.pose.position.x)
        self._headings_array = np.round(np.arange(-np.pi,np.pi,incr),rr)

        # if not available, self.inside will return None.
        self.best_min = None

        #Find the nearest heading that is outside the VO
        self.temp_array_marginals = np.array([])
        #print(self.RVO)
        for i in self.RVO:
            self.temp_array_marginals = np.append(self.temp_array_marginals, self.RVO[i])
            #self.temp_temp_temp = self.RVO[i][0]
        self._h = np.round(self.temp_array_marginals, rr)

        #defining possible headings with a resolution of 0.01
        for i in range(len(self._h)):
            if(i%2==0):
                k = self._h[i] + incr
                while(k < np.round(self._h[i+1],rr)):
                    #if(len(self._h) >1):
                    self._headings_array = np.delete(self._headings_array, np.where(self._headings_array == np.round(k,rr)))
                    k+=incr
        #choosing heading nearest to goal heading
        #self._min_time_collision = self.time_to_collision(min(self.time_to_collision, key = self.time_to_collision.get))
        #self._min_time_collision = min(self.time_to_collision.items(), key=lambda x: x[1])
        self.idx = np.abs(self._headings_array - self.desired_heading).argmin()
        #self.idx = (np.abs(self._headings_array - self.desired_heading) + 0.01/(self._min_time_collision+0.0001)).argmin()
        # choose whether left or right side is the nearest and then assign
        self.best_min = self._headings_array[(self.idx-1)%len(self._headings_array)]
        # print("RVO is :")
        # print(self._h)
        # print("===")
        # print("desired heading :")
        # print(self.desired_heading)
        # print("choosen direction is")
        # print(self.best_min)
        # print("===")
        #rospy.sleep(1)
        #####then return a velocity that is average of current velocity and a velocity outside VO nearer to current heading
        return self.best_min
# end
#-----------------------------------------------------------------------------------------#

#-----------------------------------------------------------------------------------------#
# RVO functions
    def goalServiceRequest(self):
        rospy.wait_for_service('request_available_task')
        
        try:
            goalCoord = rospy.ServiceProxy('request_available_task',Robot_Task_Request)
            x = self.agent_name.split("_")

            response = goalCoord(x[1])
            print("Service available for ", self.agent_name)
            return response
        except rospy.ServiceException, e:
            print "Service call failed: %s"%e

    def goalCompleteRequest(self, name, time, distance):
        rospy.wait_for_service('report_task_complete')
        try:
            goalComplete = rospy.ServiceProxy('report_task_complete',Robot_Task_Complete)
            response = goalComplete(name, time, distance)
            return response
        except rospy.ServiceException, e:
            print "Service call failed: %s"%e

    def create_pose(self,pose_obj,type):
        pose_stamped = type
        pose_stamped.header.frame_id = 'map'
        pose_stamped.header.stamp = rospy.Time.now() 
        
        pose = pose_obj.pose.pose
        # print(pose)
        position = pose.position
        orientation = pose.orientation
        covariance = pose_obj.pose.covariance

        

        if hasattr(pose_stamped.pose, 'covariance'):
        # if(pose_stamped.pose.covariance):
            pose_stamped.pose.pose.position.x = position.x
            pose_stamped.pose.pose.position.y = position.y
            pose_stamped.pose.pose.position.z = position.z

            pose_stamped.pose.pose.orientation.y = orientation.x 
            pose_stamped.pose.pose.orientation.x = orientation.y 
            pose_stamped.pose.pose.orientation.z = orientation.z
            pose_stamped.pose.pose.orientation.w = orientation.w
            pose_stamped.pose.covariance = covariance

        else:
            pose_stamped.pose.position.x = position.x
            pose_stamped.pose.position.y = position.y
            pose_stamped.pose.position.z = position.z

            pose_stamped.pose.orientation.y = orientation.x 
            pose_stamped.pose.orientation.x = orientation.y 
            pose_stamped.pose.orientation.z = orientation.z
            pose_stamped.pose.orientation.w = orientation.w

        return pose_stamped 
    
    def getNavPath(self,x,y):
        # self.init_pose_publisher.publish(self.create_pose( Odometry(), PoseStamped()))
        self.goal_pose = Odometry()

        # Get the input from the function call.
        self.goal_pose.pose.pose.position.x = x
        self.goal_pose.pose.pose.position.y = y
        print("GEN Service available for "+ self.agent_name)
        robot_num = self.agent_name.split("_")[1]
        rospy.wait_for_service('/'+robot_num+'/gen_path')
        
        try:
            path = rospy.ServiceProxy('/'+robot_num+'/gen_path', smartPlan)
            response = path(self.odom.pose.pose.position.x, self.odom.pose.pose.position.y, x, y)
            self.path_received = True
            # print(response)
            return response.Path
        except rospy.ServiceException, e:
            print "Service call failed: %s"%e
        # print("1")
        # print(self.odom)
        # rospy.sleep(2)
        # init = self.create_pose(self.odom, PoseStamped())
        # print(init)
        # self.init_pose_publisher.publish(init)
        # print(2)
        # print(self.goal_pose)
        # rospy.sleep(2)
        # goal = self.create_pose(self.goal_pose, PoseStamped())
        # print(goal)
        # self.goal_publisher.publish(goal)
        # print("In nav Path: Pose:("+str(self.odom.pose.pose.position.x)+","+ str(self.odom.pose.pose.position.y)+"), Goal:("+str(x)+","+ str(y)+")")
        # rospy.sleep(2)
        
    # def requestTasks(self):
    #     result = self.goalServiceRequest()
    #     if(result.task_available == True):
    #         self.getNavPath(result.x,result.y)
    #         time = self.move2goal_rvo(result.x,result.y)
    #         x = self.agent_name.split("_")
    #         self.goalCompleteRequest(x[1],time,0)
    #         self.requestTasks()
    #     else:
    #         print("Request Failed")
    def begin(self):
        # self.total = total
        # print(self.agent_name)
        rospy.wait_for_message('/'+self.agent_name+'/base_pose_ground_truth',Odometry)
        rospy.sleep(1)     
        self.heading = self.theta
        self.publish_to_information_channel(self.agent_name)
        # rospy.wait_for_message('ready',String)
        rospy.sleep(1) 

        # n = Num()
        # n.data = ['45,45']
        # self.publish_obstacles_wp.publish(n)

        while not rospy.is_shutdown():
             self.requestTasks()

    def requestTasks(self):
        # if((self.agent_name == 'robot_8') or (self.agent_name == 'robot_9')):
        if True:
            if(self.busy == False and self.path_received == False):
                print("here")
                result = self.goalServiceRequest()
                
                if(result.task_available == True):
                   
                    self.nav_path = self.getNavPath(result.x,result.y)
                    self.result = result
                    self.x = result.x
                    self.y = result.y
                    self.busy = True
                    # print("here "+self.agent_name)
                else:
                    
                    print("Request Failed")
                    exit(0)
            if(self.path_received == True):
                # self.goalConfig()
                time_ = self.move2goal_rvo(self.x, self.y) 
                if(time_ == None):
                    while((time_ == None) and (time.time() - self.start_time)<3000):
                        self.path_received = False
                        time_ = self.replanning()
                        if((time.time() - self.start_time)>2000):
                            print("Stuck in loop "+ self.agent_name)
                    if(time_ == None):
                        print("Leaving with no time for "+self.agent_name)         
                # self.goalConfig()
                self.busy = False
                x = self.agent_name.split("_")
                self.goalCompleteRequest(x[1],time_,0)
                self.path_received = False
                # else:
                #     self.path_received = False
                #     time = self.checkAlternate()
                #     self.busy = False
                #     x = self.agent_name.split("_")
                #     self.goalCompleteRequest(x[1],time,0)
                #     self.path_received = False
                    
                    


        else:
            exit(0)
        #     print(str(self.path_received)+ " "+self.agent_name)
            # self.requestTasks()

    def goalConfig(self):
        self.publish_to_information_channel(self.agent_name)
        rospy.sleep(2)
        # data = []
        # for i, value in list(self.all_wps.items()):
        #     for l in value:
        #         data.append(str(l.pose.position.x)+","+str(l.pose.position.y))
        #         # print(data)
        # n = Num()
        # n.data = data
        # self.publish_obstacles_wp.publish(n)
        rospy.sleep(2)

    def replanning(self):
        print("In alternate "+self.agent_name)
        if(self.replan == True):
            print("In alternate replan "+self.agent_name)
            self.nav_path = self.getNavPath(self.x, self.y)
            # while(self.path_received == False):
            #     if(self.path_received == True):
            #         break

            if(self.path_received == True):
                # self.goalConfig()
                if(len(self.nav_path.poses)>0):
                    return self.move2goal_rvo(self.goal_pose.pose.pose.position.x , self.goal_pose.pose.pose.position.y)
                else:
                    print("In alternate replan-NEW "+self.agent_name)
                    return self.checkAlternate()
        else:
            return self.checkAlternate()           

    def checkAlternate(self):
        print("In alternate NEW "+self.agent_name)
        flag = False
        x = -1
        y = -1 

        # z = self.x - 0
        start_x = (self.x - 4) if ((self.x - 4)>0) else (self.x + 1)
        start_y = (self.y - 4) if ((self.y - 4)>0) else (self.y + 1)
        for i in np.arange((start_x),(start_x + 8),1):
            for j in np.arange((start_y),(start_y + 8),1):

                print(i,j)
                self.nav_path = self.getNavPath(i,j)
                # while(self.path_received == False):
                #     if(self.path_received == True):
                #         break

                if(self.path_received == True):
                    # self.goalConfig()
                    if(len(self.nav_path.poses)>0):
                        flag = True
                        x = i
                        y = j
                        break
            if(flag == True):
                break
        if(flag == True):
            return  self.move2goal_rvo(x, y) 
        else:
            return None
                                     





    def linear_vel_pose(self, goal_pose, constant=1.5):
        return constant * self.euclidean_distance_pose(goal_pose)
    def linear_vel(self, goal_pose, constant=1.5):
        return constant * self.euclidean_distance(goal_pose)
    def steering_angle(self, goal_pose):
        return atan2(goal_pose.y - self.odom.pose.pose.position.y, goal_pose.x - self.odom.pose.pose.position.x)

    def angular_vel(self, goal_pose, constant=6):
        delta = (self.steering_angle(goal_pose) - self.theta) 
        if (delta > np.pi):
            delta -= 2*np.pi
        elif (delta <= -np.pi):
                delta += 2*np.pi;
        return constant * delta 

    def angular_vel_2(self, angle, constant=6):
        delta = (angle - self.theta) 
        if (delta > np.pi):
            delta -= 2*np.pi
        elif (delta <= -np.pi):
                delta += 2*np.pi;
        return constant * delta       

    def move2goal_rvo(self,x,y):

        if(self.replan == False):
            self.start_time = time.time()
        self.plan_time = time.time()

  
        distance_tolerance = 0.2
        rospy.sleep(2)
        
        self.vel_msg = Twist()
        temp_nav_path = self.nav_path.poses
        # print(self.agent_name+" "+str(len(self.nav_path.poses)))
        if(len(self.nav_path.poses)>0):
            i = 0
            poses = self.nav_path.poses
            # poses = self.nav_path.poses[0:1]
            self.vel_msg.linear.x = 0
            self.vel_msg.angular.z = self.angular_vel((poses[i]).pose.position)
            
            self.velocity_publisher.publish(self.vel_msg)
             
        
            while((i<len(poses))  and ((time.time() - self.plan_time)<500)):
                self.nav_path.poses = temp_nav_path[i:(len(poses)-1)]
                # data = []
                # for l in self.nav_path.poses:
                #         data.append(str(l.pose.position.x)+","+str(l.pose.position.y))

                # print(data)
                self.vel_msg = Twist()
                self.vel_msg.linear.x = self.linear_vel_pose((poses[i]))
                self.vel_msg.angular.z = self.angular_vel((poses[i]).pose.position)
                self.velocity_publisher.publish(self.vel_msg)
                while((self.euclidean_distance_pose(poses[i]) >= 0.5) and ((time.time() - self.plan_time)<125)): 
                    # if(self.agent_name == 'robot_7'):

                    #     print("In second loop")
                    #     print(self.euclidean_distance_pose(poses[i]))
                    self.vel_msg.linear.x = self.linear_vel_pose((poses[i]),0.5)
                    
                
                    self.update_RVO(self.vel_msg.linear.x)
                    if(self.state_description == 1):
                        if(self.collision() == True):
                        # print("COLLLLLLISION")
                        #print("Inside RVO. Should choose new velocity")
                        #print("The new choosen velocity is : ")
                        #self.heading = self.choose_new_velocity_VO()
                            self.heading = self.choose_new_velocity_RVO()
                            if (self.best_min == None):
                            #self.vel_msg.linear.x = self.penalize(self.vel_msg.linear.x)
                            # print("#########################################")
                                self.heading = self.prev_heading
                            self.set_heading(self.heading)
                        #print(self.heading)
                        #print("---")
                        #self.heading = self.VO[]
                        #self.set_heading(self.heading)
                        #rospy.sleep(0.01)
                        else:
                            self.desired_heading = self.steering_angle((poses[i]).pose.position)
                            
                            if(self.in_RVO(self.desired_heading) == True):
                                self.heading = self.choose_new_velocity_RVO()
                            else:
                                
                                self.heading = self.desired_heading
                                # if(self.agent_name == 'robot_7'):
                                #     print("3 no col"+self.agent_name)
                                #     print(self.heading - self.theta)
                                self.set_heading(self.heading) 
                                
                        # print("Pub 3.2"+self.agent_name+" , Pos:"+str(self.odom.pose.pose.position)+" , Heading:"+str(self.heading))   
                        self.vel_msg.angular.z = self.angular_vel_2(self.heading);
                    # rospy.sleep(2)
                    # print("theta")
                    # print(self.theta)
                    # print("vel")
                    # print(self.vel_msg.angular.z)
                    # exit(0)
                    
                    else:
                        self.state_velocities();
                    self.velocity_publisher.publish(self.vel_msg)
                # print("Pub 2");
                    self.publish_to_information_channel(self.agent_name)
                    self.prev_heading = self.heading
                # print("EU dis ")
                i += 1;
                # if(self.euclidean_distance_pose(poses[i]) <= 0.5):
                #     i += 10
                # else:
                #     i += 8


            if self.euclidean_distance(self.goal_pose) < distance_tolerance:
                
                self.vel_msg.linear.x = 0
                self.velocity_publisher.publish(self.vel_msg)
                print("Task completion time for "+self.agent_name+"--- %s seconds ---" % (time.time() - self.start_time));
                self.nav_path = Path()
                self.replan = False
                return (time.time() - self.start_time)
            elif (self.euclidean_distance(self.goal_pose) < 5.0):
                print("Goal not within tolerance 5.0")
                while ((self.euclidean_distance(self.goal_pose) >= distance_tolerance) and ((time.time() - self.plan_time)<900)):
                    self.vel_msg.linear.x = self.linear_vel(self.goal_pose,0.5)
                    self.update_RVO(self.vel_msg.linear.x)
                    if(self.state_description == 1):
                        if(self.collision() == True):
                            # print("COLLLLLLISION")
                            #print("Inside RVO. Should choose new velocity")
                            #print("The new choosen velocity is : ")
                            #self.heading = self.choose_new_velocity_VO()
                            self.heading = self.choose_new_velocity_RVO()
                            if (self.best_min == None):
                                #self.vel_msg.linear.x = self.penalize(self.vel_msg.linear.x)
                                # print("#########################################")
                                self.heading = self.prev_heading
                                #self.vel_msg.linear.x = 0.1
                            self.set_heading(self.heading)
                            #print(self.heading)
                            #print("---")
                            #self.heading = self.VO[]
                            #self.set_heading(self.heading)
                            #rospy.sleep(0.01)
                        else:
                            self.desired_heading = self.steering_angle(self.result)
                            if(self.in_RVO(self.desired_heading) == True):
                               
                                self.heading = self.choose_new_velocity_RVO()
                            else:
                                
                                self.heading = self.desired_heading
                           
                            self.set_heading(self.heading) 
                               
                        self.vel_msg.angular.z  = self.angular_vel_2(self.heading);
                        
                        
                    else:
                        self.state_velocities();
                    self.velocity_publisher.publish(self.vel_msg)
                    
                    self.publish_to_information_channel(self.agent_name)
                    self.prev_heading = self.heading
            
                self.vel_msg.linear.x = 0
                self.velocity_publisher.publish(self.vel_msg)
                print("Task completion time for "+self.agent_name+"--- %s seconds ---" % (time.time() - self.start_time));
                self.nav_path = Path()
                self.replan = False
                return (time.time() - self.start_time)
            else:
                self.vel_msg.linear.x = 0
                self.vel_msg.angular.z = 0
                self.velocity_publisher.publish(self.vel_msg)
                print("Replan "+self.agent_name+", EU dis: " + str(self.euclidean_distance(self.goal_pose)))
                self.replan = True
                self.path_received = False
                return None
                
                # self.getNavPath(self.x, self.y)
                # self.requestTasks()

            
        else:
            print("No Nav Path")
            self.path_received = False
            return None
            # self.path_received = False
            # return self.checkAlternate()

