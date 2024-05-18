#! /usr/bin/env python
import rospy
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist, Point
from nav_msgs.msg import Odometry
from tf import transformations
from std_srvs.srv import *
import math
import numpy as np
from tf.transformations import euler_from_quaternion


class ControlPID:
    def __init__(self, dt, kp = 1, ki = 0, kd = 0):
        self.dt = dt
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.integral = 0
        self.error_actual = 0
        self.pred = 0
        ##Inicialiace points
        self.x_list=0.0
        self.y_list=0.0
        #errosself.
        self.error_ang=0.0

    #get the points 
    def points_callback(self,x,y):
        self.x_list=x
        self.y_list=y
    #get the velocity for the distance we need 
    def get_pid(self, current, goal):
        error_p = goal - current
        self.integral = self.integral + self.ki*error_p*self.dt
        ed = (error_p - self.error_actual)/self.dt
        self.error_actual = error_p
        self.pred = current + (error_p*self.kp) + self.integral + (ed*self.kd)
        return self.pred
    
    def get_pid_d(self,goal):
        error_p = goal 
        self.integral = self.integral + self.ki*error_p*self.dt
        ed = (error_p - self.error_actual)/self.dt
        self.error_actual = error_p
        self.pred = (error_p*self.kp) + self.integral + (ed*self.kd)
        return self.pred
    #get angular error
    def e_ang (self,z_pose,x,y):
        self.error_ang = np.arctan2(self.y_list - y, self.x_list - x)
        a = self.error_ang - z_pose
        #find best possible angle 
        a -= 2*np.pi if a > np.pi else -2*np.pi if a < -np.pi else 0
        return a
    #get distance error
    def e_dis(self,x,y):
        error_dist = np.sqrt((self.x_list - x) ** 2 + (self.y_list -y) ** 2)
        return error_dist

class move_foward:
    def __init__(self):
        # Initialize ROS node
        rospy.init_node('bug_node')
        #start sus and publishers and services 
        rospy.Subscriber('/odom',Odometry,self.odom_callback)
        rospy.Subscriber('/scan',LaserScan,self.callbackScan)
        srv = rospy.Service('go_to_point_switch', SetBool, self.go_to_point_switch)
        self.vel_pub=rospy.Publisher('/puzzlebot_1/base_controller/cmd_vel', Twist, queue_size=10)
        #start variables
        self.x_odom = 0.0
        self.y_odom = 0.0
        self.quaternion =0.0
        self.pos_z=0.0
        self.active_ = False
        self.state_ = 0
        # goal
        self.goal_x=[1,1,0,0]
        self.goal_y=[0,1,1,0]
        self.goal_z = 0
        # maybe this could be in the controller 
        self.theta_tolerance = np.pi / 45  # +/- 4 degree allowed maybe implement this to messure the angular tolerance
        self.rate = rospy.Rate(100)
        self.vel = Twist()
        ##Init_classes
        self.turnPID = ControlPID(0.09,kp = 0.8)
        self.fowaPID = ControlPID(0.09,kp = 0.2,kd = 0.005)
        

    def callbackScan(self,scan):
        self.regions = {
        'right': min(min(scan.ranges[0:143]), 10),
        'fright':min(min(scan.ranges[144:287]), 10),
        'front': min(min(scan.ranges[288:431]), 10),
        'fleft': min(min(scan.ranges[432:575]), 10),
        'left':  min(min(scan.ranges[576:713]), 10),
        }
        #rospy.loginfo(self.regions)
    
    def odom_callback(self, odom):
        self.x_odom = odom.pose.pose.position.x
        self.y_odom = odom.pose.pose.position.y
        self.quaternion = odom.pose.pose.orientation
        (_,_,self.pos_z) = euler_from_quaternion([self.quaternion.x, self.quaternion.y, self.quaternion.z, self.quaternion.w])
    
    def change_state(self,state):
        self.state_ = state
        #print ('State changed to [%s]' % self.state_)
    
    #active the function!
    def go_to_point_switch(self,req):
        self.active_ = req.data
        res = SetBoolResponse()
        res.success = True
        res.message = 'Done!'
        return res
    
    def done():
        twist_msg = Twist()
        twist_msg.linear.x = 0
        twist_msg.angular.z = 0
        pub.publish(twist_msg)
    
    ##Here i have to add PID
    ##Maybe i dont need this 
    def fix_yaw(self,des_pos):
        global pub
        #get the velocity with PID
        goal_theta = np.arctan2(des_pos.y - position_.y, des_pos.x - position_.x)
        err_theta = goal_theta - self.pos_z


        twist_msg = Twist()
        if np.abs(err_theta) > self.theta_tolerance:
            twist_msg.angular.z = -0.3 if err_theta > 0 else 0.3

        pub.publish(twist_msg)

        # state change conditions
        if np.abs(err_theta) <= self.theta_tolerance:
            print ('Yaw error: [%s]' % err_theta)
            self.change_state(1)
    
    def go_straight_ahead(self,des_pos):
        global yaw_, pub, yaw_precision_, state_
        desired_yaw = np.atan2(des_pos.y - position_.y, des_pos.x - position_.x)
        err_yaw = desired_yaw - yaw_
        err_pos = np.sqrt(pow(des_pos.y - position_.y, 2) +
                            pow(des_pos.x - position_.x, 2))

        if err_pos > dist_precision_:
            twist_msg = Twist()
            twist_msg.linear.x = 0.3
            pub.publish(twist_msg)
        else:
            print ('Position error: [%s]' % err_pos)
            self.change_state(2)

        # state change conditions
        if math.fabs(err_yaw) > yaw_precision_:
            print ('Yaw error: [%s]' % err_yaw)
            change_state(0)


    

    def run(self):
        error_a=0.0
        while not rospy.is_shutdown():
            if not self.active_:
                continue
            else:
                if self.state_ == 0:
                    #This state is for moving the angle
                    ##Send the points 
                    self.turnPID.points_callback(self.goal_x[1],self.goal_y[1])
                    error_a=self.turnPID.e_ang(self.pos_z,self.x_odom,self.y_odom)
                    #Here i need to set
                    ##Send the velocity
                    if abs(error_a)>0.3:
                        self.vel.angular.z=self.turnPID.get_pid(self.pos_z,error_a)
                        self.vel.linear.x=0.0
                        self.vel_pub.publish(self.vel)
                    else: #eq=error_a=0 this means tollerance so i got to adjust that
                        #this means we have to change state
                        self.vel.angular.z=0.0
                        self.vel.linear.x=0.0
                        self.vel_pub.publish(self.vel)
                        self.change_state(1)
        
                elif self.state_ == 1: ##This is for straight
                    error_d=self.fowaPID.e_dis(self.x_odom,self.y_odom)
                    error_a=self.turnPID.e_ang(self.pos_z,self.x_odom,self.y_odom)
                    self.vel.linear.x = self.fowaPID.get_pid_d(error_d)  
                    #here i have to send 2 variables the foward and the straight for PID
                    #this function calculate the d_v and if e_a is rising change to state 0
                    if abs(error_a)<=0.3 and abs(error_d)>=0.1 :
                        self.vel.angular.z=0.0
                        self.vel.linear.x = self.fowaPID.get_pid_d(error_d)  
                        self.vel_pub.publish(self.vel)
               
                    self.go_straight_ahead(self.x)
                    self.change_state(2)



                elif self.state_ == 2:
                    self.done()
                else:
                    rospy.logerr('Unknown state!')

            self.rate.sleep()




