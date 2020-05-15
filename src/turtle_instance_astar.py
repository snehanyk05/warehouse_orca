import rospy
from geometry_msgs.msg import Twist, PoseStamped, PoseWithCovarianceStamped
from turtlesim.msg import Pose
from turtlesim.srv import TeleportAbsolute
from tf.transformations import euler_from_quaternion
from nav_msgs.msg import Odometry, Path
from sensor_msgs.msg import LaserScan
from warehouse_manager.srv import Robot_Task_Complete, Robot_Task_Request
from std_msgs.msg import String
from rvo.msg import Information

from random import randint
import numpy as np
import time
import os

from math import pow, atan2, sqrt, cos, sin, atan, asin

class TurtleBot:

    #global all_agents_pose_dict
    all_agents_pose_dict = {}

    def __init__(self, agent_name):
		# Creates a node with name of the agent
        self.agent_name = agent_name
        print(self.agent_name)
        rospy.init_node(self.agent_name)

		### Publisher ###

		# Publisher which will publish velocity to the topic '/turtleX/cmd_vel'.
        self.velocity_publisher = rospy.Publisher('/'+self.agent_name+'/cmd_vel', Twist, queue_size=10)
        self.init_pose_publisher = rospy.Publisher('/'+self.agent_name+'/initialpose', PoseWithCovarianceStamped, queue_size=10)
        self.goal_publisher = rospy.Publisher('/'+self.agent_name+'/move_base_simple/goal', PoseStamped, queue_size=10)


        # Publisher which will publish Pose to the topic '/turtleX/cmd_vel'.
        # self.turtle_teleport = rospy.ServiceProxy(self.agent_name + '/teleport_absolute', TeleportAbsolute)

        # Publisher which will publish to the topic '/common_information'
        self.publish_information = rospy.Publisher("/common_information", Information, queue_size=10)

        self.pub_pose = Odometry()
        self.inf = Information()

        ### Subscriber ###

        # self.update_pose is called when a message of type Pose is received.
        self.pose_subscriber = rospy.Subscriber('/'+self.agent_name+'/base_pose_ground_truth', Odometry, self.update_pose)
        self.nav_path_subscriber = rospy.Subscriber('/'+self.agent_name+'/nav_path', Path, self.found_path)
        # self.recieve_from_information_channel is called when a message of type information is received.
        rospy.Subscriber("/common_information", Information, self.recieve_from_information_channel)
        self.nav_path = Path()
        self.odom = Odometry()
        self.theta = 0;
        self.rate = rospy.Rate(10)
        self.start_time=time.time()

#-----------------------------------------------------------------------------------------#
# Functions related to topics
    def found_path(self, data):
        """Callback function which is called when a new message of type Pose is
        received by the subscriber."""
        
        print(data)
        self.nav_path = data
        print("found path")
        # print("POSSSSSSSSSSSE:",self.pose)
        
    def update_pose(self, data):
        """Callback function which is called when a new message of type Pose is
        received by the subscriber."""
        # print(data)
        # print("POSSSSSSSSSSSE:",self.pose)
        self.odom = data
        rot_q = data.pose.pose.orientation
        (roll,pitch,theta) = \
            euler_from_quaternion ([rot_q.x,rot_q.y,rot_q.z,rot_q.w])
        self.theta = theta
        self.odom.pose.pose.position.x = round(self.odom.pose.pose.position.x, 4)
        self.odom.pose.pose.position.y = round(self.odom.pose.pose.position.y, 4)

    def publish_to_information_channel(self,t):
        i = Information()
        i.agent_name = t
        i.agent_pose_x = self.odom.pose.pose.position.x
        i.agent_pose_y = self.odom.pose.pose.position.y
        i.agent_heading = self.heading
        i.agent_vel_mag = self.vel_msg.linear.x

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

    # sets heading towards given direction
    def set_heading(self,theta):
        #theta_rad = (theta * np.pi)/180
        rospy.sleep(0.1)
        # print(theta)

        # self.vel_msg = Twist()
        # self.vel_msg.linear.x = self.odom.pose.pose.position.x
        # self.vel_msg.linear.y = self.odom.pose.pose.position.y
        # self.vel_msg.angular.z = theta
        # self.velocity_publisher.publish(self.vel_msg)
        # self.turtle_teleport(self.pose.x,self.pose.y,theta)

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
        for i in self.all_agents_pose_dict:
            if(i != self.agent_name):
                #calc distance between agent and oher agent/obstacle
                self._distance = round(sqrt(pow((self.all_agents_pose_dict[i][0] - self.odom.pose.pose.position.x), 2) + pow((self.all_agents_pose_dict[i][1] - self.odom.pose.pose.position.y), 2)),rr)

                #if it lies in the NR, consider it in calculating RVO
                if(self._distance < self.NR):
                    #calc the relative velocity of the agent and choosen other agent
                    self._rel_v_x =  v_mag * cos(self.theta) - self.all_agents_pose_dict[i][3] * cos(self.all_agents_pose_dict[i][2])
                    self._rel_v_y =  v_mag * sin(self.theta) - self.all_agents_pose_dict[i][3] * sin(self.all_agents_pose_dict[i][2])
                    self._rel_heading[i] = round(atan2(self._rel_v_y,self._rel_v_x),rr)

                    # VO finder :: Should output a range of headings into an 2D array
                    self.point_to_agent_heading[i] = round(atan2((self.all_agents_pose_dict[i][1] - self.odom.pose.pose.position.y),(self.all_agents_pose_dict[i][0] - self.odom.pose.pose.position.x)),rr)
                    #can also use np.clip
                    try:
                        print("DIST "+self.agent_name ,self._distance)
                        self._omega[i] = round(asin(r/self._distance),rr)
                    except ValueError:
                        self._omega[i] = round(np.pi/2,rr)

                    #time to collision
                    # should know distance and relative velocity in the direction of the obstacle
                    # if negative, it means collision will not occur

                    c1 = self._rel_v_x - (v_mag * (cos(self.theta))) <= 0
                    c2 = self._rel_v_x - (v_mag * (cos(self.all_agents_pose_dict[i][2])))<= 0
                    c3 = self._rel_v_y - (v_mag * (sin(self.theta)))<= 0
                    c4 = self._rel_v_y - (v_mag * (sin(self.all_agents_pose_dict[i][2])))<= 0

                    self.time_to_collision[i] = np.inf
                    if(c1 | c2 | c3 | c4):
                        self.time_to_collision[i] = abs(self.all_agents_pose_dict[i][0] - self.odom.pose.pose.position.x)/abs(v_mag * (cos(self.theta) - cos(self.all_agents_pose_dict[i][2])))

                    #Instead of checking if v_A is in VO, im checking if v_AB is inside something called "VX"
                    #But for RVO, we are adding v_A and v_B to VX.
                    # This is computationally easier
                    self.VX[i] = (np.asarray([self.point_to_agent_heading[i] - self._omega[i],self.point_to_agent_heading[i] + self._omega[i]]))
                    #####find v_A by adding v_B to VX (both mag and dir)
                    #self.VO[i] = self.VX[i] + self.all_agents_pose_dict[i][2]
                    self.num = v_mag * sin(self.present_temp_h) + self.all_agents_pose_dict[i][3] * sin(self.all_agents_pose_dict[i][2])
                    self.den = v_mag * cos(self.present_temp_h) + self.all_agents_pose_dict[i][3] * cos(self.all_agents_pose_dict[i][2])

                    self.RVO[i] = (self.VX[i] + atan2(self.num,self.den))/2
                    #Uncomment the below line if you want the code to behave like VO
                    #self.RVO[i] = self.VX[i]

                    if (self._distance < self._least_distance):
                        self._least_distance = self._distance

        self.vel_msg.linear.x = (self._least_distance/4.5)/self.NR
        # self.vel_msg.linear.x = 0.75
        # print(self.vel_msg.linear.x)

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
        print(self.VO)
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
        print("===")
        print("RVO is :")
        print(self._h)
        self.idx = np.abs(self._headings_array - self.desired_heading -0.1).argmin()
        self.best_min = self._headings_array[self.idx]
        print("desired heading :")
        print(self.desired_heading)
        print("choosen direction is")
        print(self.best_min)
        print("===")
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
        print("RVO is :")
        print(self._h)
        print("===")
        print("desired heading :")
        print(self.desired_heading)
        print("choosen direction is")
        print(self.best_min)
        print("===")
        #rospy.sleep(1)
        #####then return a velocity that is average of current velocity and a velocity outside VO nearer to current heading
        return self.best_min
# end
#-----------------------------------------------------------------------------------------#

#-----------------------------------------------------------------------------------------#
# RVO functions
    def goalServiceRequest(self):
        rospy.wait_for_service('request_available_task')
        #print("Service available for ", name)
        try:
            goalCoord = rospy.ServiceProxy('request_available_task',Robot_Task_Request)
            x = self.agent_name.split("_")

            response = goalCoord(x[1])
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
        self.goal_pose = Odometry()

        # Get the input from the function call.
        self.goal_pose.pose.pose.position.x = x
        self.goal_pose.pose.pose.position.y = y
        print("1")
        # print(self.odom)
        rospy.sleep(1)
        init = self.create_pose(self.odom, PoseWithCovarianceStamped())
        self.init_pose_publisher.publish(init)
        print(2)
        # print(self.goal_pose)
        goal = self.create_pose(self.goal_pose, PoseStamped())
        self.goal_publisher.publish(goal)
        

    def requestTasks(self):
        result = self.goalServiceRequest()
        if(result.task_available == True):
            self.getNavPath(result.x,result.y)
            time = self.move2goal_rvo(result.x,result.y)
            x = self.agent_name.split("_")
            self.goalCompleteRequest(x[1],time,0)
            self.requestTasks()
        else:
            print("Request Failed")
  
    def move2goal_rvo(self,x,y):
        self.start_time = time.time()
        
        # print("In move to path")
        # print(self.nav_path)
        # Please, insert a number slightly greater than 0 (e.g. 0.01).
        distance_tolerance = 0.2

        # Setting the direction and velocity
        # self.desired_heading = atan2(self.goal_pose.pose.pose.position.y - self.odom.pose.pose.position.y, self.goal_pose.pose.pose.position.x - self.odom.pose.pose.position.x)
        # self.set_heading(self.desired_heading)
        rospy.sleep(2)
        # for point in self.nav_path:
        if(len(self.nav_path.poses)>0):
        # print("Pub 1,"+self.agent_name+" :"+str(self.desired_heading));
        # print(self.odom.pose.pose.position)
            i = 0
            poses = self.nav_path.poses
            # for i in range(len(poses)):
            print("Prev Goal")
            print(poses[i])
            print('Next Goal')
            print(poses[i+1])
                    # rospy.sleep(0.1)
                    # self.desired_heading = atan2((poses[i+1]).pose.position.y - (poses[i]).pose.position.y, (poses[i+1]).pose.position.x - (poses[i]).pose.position.x)
            self.desired_heading = atan2((poses[i]).pose.position.y - self.odom.pose.pose.position.y, (poses[i]).pose.position.x - self.odom.pose.pose.position.x)
                    
            self.heading = self.desired_heading
                    # print("Pub 1"+self.agent_name+" :"+str(self.heading))
                    # print(self.odom.pose.pose.position)
            self.vel_msg = Twist()
            self.vel_msg.linear.x = 0.1
            self.vel_msg.angular.z = self.heading
            self.velocity_publisher.publish(self.vel_msg)
                    
                    
            while((i<len(poses))  and (self.euclidean_distance(self.goal_pose) >= distance_tolerance)):
                        self.vel_msg.linear.x = 0.5
                        self.update_RVO(self.vel_msg.linear.x)

                        if(self.collision() == True):
                            print("COLLLLLLISION")
                            #print("Inside RVO. Should choose new velocity")
                            #print("The new choosen velocity is : ")
                            #self.heading = self.choose_new_velocity_VO()
                            self.heading = self.choose_new_velocity_RVO()
                            if (self.best_min == None):
                                #self.vel_msg.linear.x = self.penalize(self.vel_msg.linear.x)
                                print("#########################################")
                                self.heading = self.prev_heading
                                #self.vel_msg.linear.x = 0.1
                            self.set_heading(self.heading)
                            #print(self.heading)
                            #print("---")
                            #self.heading = self.VO[]
                            #self.set_heading(self.heading)
                            #rospy.sleep(0.01)
                        else:
                            # self.desired_heading = atan2((poses[i+1]).pose.position.y - (poses[i]).pose.position.y, (poses[i+1]).pose.position.x - (poses[i]).pose.position.x)
                            self.desired_heading = atan2((poses[i]).pose.position.y - self.odom.pose.pose.position.y, (poses[i]).pose.position.x - self.odom.pose.pose.position.x)
                            print("POSE")
                            print(poses[i])
                            if(self.in_RVO(self.desired_heading) == True):
                                # print("2")
                                #print("desired heading still inside. Continue prev heading")
                                #self.vel_msg.linear.x = 0
                                #self.heading = self.prev_heading
                                self.heading = self.choose_new_velocity_RVO()
                            else:
                                # print("3")
                                self.heading = self.desired_heading
                            # print("Pub 3.1"+self.agent_name+" , Pos:"+str(self.odom.pose.pose.position)+" , Heading:"+str(self.heading))    
                            self.set_heading(self.heading) 
                            # print("Pub 3.2"+self.agent_name+" , Pos:"+str(self.odom.pose.pose.position)+" , Heading:"+str(self.heading))   
                        self.vel_msg.angular.z = self.heading - self.theta;
                            # print("3:"+str(self.heading))
                            # exit(0)
                        self.velocity_publisher.publish(self.vel_msg)
                        # print("Pub 2");
                        self.publish_to_information_channel(self.agent_name)
                        self.prev_heading = self.heading
                        i += 1
            # print("-----")

        # Stopping the agent after the movement is over.
            if self.euclidean_distance(self.goal_pose) < distance_tolerance:
                self.vel_msg.linear.x = 0
                self.velocity_publisher.publish(self.vel_msg)
                print("Task completion time for "+self.agent_name+"--- %s seconds ---" % (time.time() - self.start_time));
                return (time.time() - self.start_time)
            else:
                print("Goal not within tolerance")
        else:
            print("No Nav Path")
        # If we press control + C, the node will stop.
        #rospy.spin()