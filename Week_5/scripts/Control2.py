#!/usr/bin/env python
import rospy
from std_msgs.msg import Float32
from geometry_msgs.msg import Twist
import numpy as np
from nav_msgs.msg import Odometry


class Controller:
    def __init__(self):
        #Parameters
         # PID distancia
        self.kp_dist = 0.09  
        self.ki_dist = 0.0
        self.kd_dist = 0.0  
        self.error_dist=0.0
        self.integral_dist=0.0
        self.prev_error_dist=0.0
        self.velocidad_l=0
        # PID angulo
        self.kp_ang = 1  
        self.ki_ang = 0.05
        self.kd_ang = 0.02  
        self.error_ang=0.0
        self.velocidad_a=0
        self.integral_ang=0.0
        self.prev_error_ang=0.0
        # Variables de estado
        self.x = 0.0
        self.y = 0.0
        self.ori_z = 0.0
        self.ori_z_ant= 0.0
        #Lista de puntos 
        self.index = 0
        self.x_list = [1, 1, 2, 0, 0]
        self.y_list = [0, 1, 1, 0, 0]
        self.first=True
        self.last_index = None
        self.last_error_ang_w = None
        
        # Inicializar nodos
        rospy.init_node("controller")
        rospy.Subscriber("/odom", Odometry, self.odom_callback)
        self.pose_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        self.msg = Twist()
        self.msg.angular.z =0.0
        self.msg.linear.x = 0.0
        self.rate = rospy.Rate(10)
    
    def odom_callback(self, data):
        self.x = data.pose.pose.position.x
        self.y = data.pose.pose.position.y
        self.ori_z = data.pose.pose.orientation.z*np.pi
        #2pi?

    def wrap_to_Pi(self,theta):
        result = np.fmod((theta + np.pi),(2 * np.pi))
        if(result < 0):
                result += 2 * np.pi
        return result - np.pi
        #return theta
    
    def error_a_l(self,index):
        last_error_ang_w = np.arctan2(self.y_list[index] - self.y, self.x_list[index] - self.x)
        if last_error_ang_w>=3.14:
            if index != self.last_index:
                self.last_index = index
                self.last_error_ang_w = np.arctan2(self.y_list[index] - self.y, self.x_list[index] - self.x)
        else:
            self.last_error_ang_w = np.arctan2(self.y_list[index] - self.y, self.x_list[index] - self.x)
        
        error_ang_w = self.last_error_ang_w
        
        error_ang = abs(error_ang_w) - self.ori_z
        print_info = "%3f  |%3f | %3f " %(error_ang_w,error_ang, self.wrap_to_Pi(error_ang))
        rospy.loginfo(print_info)

        rospy.loginfo(self.ori_z)
        error_dist = np.sqrt((self.x_list[index] - self.x) ** 2 + (self.y_list[index] - self.y) ** 2)
        
        return error_ang, error_dist
   
   
    def pid_controller(self, dt, error_ang, error_dist, prev_error_dist, prev_error_ang): 
        
        #PID Dist
        #if self.error_ang< 0.1 and self.error_ang>-0.1:
        self.integral_dist += error_dist * dt
        derivative_dist = (error_dist - prev_error_dist) / dt
        self.velocidad_l = self.kp_dist * self.error_dist + self.ki_dist * self.integral_dist + self.kd_dist * derivative_dist
       
        #PID Ang     
        self.integral_ang += error_ang * dt
        derivative_ang = (error_ang - prev_error_ang) / dt
        self.velocidad_a = self.kp_ang * error_ang + self.ki_ang * self.integral_ang + self.kd_ang * derivative_ang
        #if (error_ang<0.005 and error_ang>-0.005):
        #     self.velocidad_l = self.kp_dist * self.error_dist + self.ki_dist * self.integral_dist + self.kd_dist * derivative_dist
       
       
        return self.velocidad_l, self.velocidad_a


    def run(self):
        while not rospy.is_shutdown():
            if self.first:
                self.current_time = rospy.get_time() 
                self.previous_time = rospy.get_time()
                self.first = False
            
            else:
                self.current_time = rospy.get_time() 
                dt = (self.current_time - self.previous_time) #get dt
                self.previous_time = self.current_time
                
                self.error_ang, self.error_dist = self.error_a_l(self.index)
               
                #PID controller 
                self.velocidad_l, self.velocidad_a = self.pid_controller(dt, self.error_ang, self.error_dist, self.prev_error_dist, self.prev_error_ang)
                self.prev_error_dist = self.error_dist
                self.prev_error_ang = self.error_ang
                
                if self.error_ang< 0.001 and self.error_ang>-0.001:
                    self.velocidad_a=0.0
                    self.error_ang = 0.0
                    #self.index += 1

                if self.error_dist < 0.01 and self.error_dist>-0.01:
                    self.velocidad_l=0.0
                    self.error_dist = 0

                if self.error_ang ==0.0  and self.error_dist == 0.0  and self.index < len(self.x_list)-1:
                    self.index += 1
               

                if self.index == len(self.x_list)-1:
                    self.msg.angular.z= 0.0
                    self.msg.linear.x = 0.0
               
                else:
                    self.msg.angular.z= self.velocidad_a
                    self.msg.linear.x = self.velocidad_l

                    print_info = "%3f  |%3f | %3f  " %(self.index,self.error_dist, self.error_ang)
                    rospy.loginfo(self.index)
                    print_info2 = "%3f |%3f  |%3f  "%(self.ori_z,self.x,self.y)
                    #rospy.loginfo(print_info2)
                    
                    self.pose_pub.publish(self.msg)
                    self.rate.sleep()



if __name__== "__main__":
    controller = Controller()
    try:
        controller.run()
    except rospy.ROSInterruptException:
        None
