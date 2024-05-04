#!/usr/bin/env python
import rospy 
import numpy as np
from std_msgs.msg import Float32
from geometry_msgs.msg import Twist, PoseStamped
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TransformStamped, Quaternion
from tf import TransformBroadcaster
from tf.transformations import quaternion_from_euler
from std_msgs.msg import Float64MultiArray


class LocalizationNode:
    def __init__(self):
        # Initialize the robot's pose
        rospy.init_node("localisation")
        self.rate = rospy.Rate(100) 
        self.previous_time = rospy.Time.now()

        # Define the wheelbase and radius of the robot
        self.first = True
        self.wheelbase = 0.19 
        self.radius = 0.05 
        self.wr_speed = 0.0
        self.wl_speed = 0.0
        self.vel=0.0
        self.w=0.0
        self.theta = 0.0
        self.x=0.0
        self.y=0.0
        self.kr=1
        self.kl=1
        self.odom_msg = Odometry()
        self.snt_tnf=TransformBroadcaster()
        self.pose_robot=PoseStamped()
        self.H_matrix=np.zeros((3,3),dtype=float)
        self.covariance_matrix=np.zeros((3,3),dtype=float)
        self.Qk_matrix=np.zeros((3,3),dtype=float)
        self.cov_delta_q=np.array([[self.kr * abs(self.wr_speed), 0], [0, self.kl * abs(self.wl_speed)]],dtype=float)
       
        self.odom_pub = rospy.Publisher("/odom", Odometry, queue_size=10)
        rospy.Subscriber("/wr", Float32, self.wr_callback)
        rospy.Subscriber("/wl", Float32, self.wl_callback)
         
    def wr_callback(self, v_r):
        self.wr_speed = v_r.data

    def wl_callback(self, v_l):
        self.wl_speed = v_l.data
        
    def get_positon(self,wr,wl,dt):
        self.vel=self.radius*(wr+wl)/2
        self.w=self.radius*(wr-wl)/self.wheelbase
        x_dot= self.vel*np.cos(self.theta) #vel
        y_dot=self.vel*np.sin(self.theta) #vel
        theta_dot=self.w
        #Linealizado
        #x_dot= -vel*np.sin(self.theta) #vel
        #y_dot=vel*np.cos(self.theta) #vel
        #theta_dot=w
        self.x += x_dot*dt
        print(self.w)
        self.y += y_dot*dt
        self.theta += theta_dot*dt
        self.wrap_to_Pi(self.theta)
        return    
      
    def get_odometry(self,current_time,x,y,theta,covarianza):
        self.odom_msg.header.stamp = current_time
        self.odom_msg.header.frame_id = "odom" #or odom
        self.odom_msg.child_frame_id = "base_link"
        self.odom_msg.pose.pose.position.x = x
        self.odom_msg.pose.pose.position.y = y
        quaternion=Quaternion(*quaternion_from_euler(0,0,theta))
        self.odom_msg.pose.pose.orientation= quaternion

        #
        co = np.zeros((6,6),dtype=float)
        co[:2,:2] = covarianza[:2,:2]
        co[-1,:2] = covarianza[-1,:2]
        co[:2,-1] = covarianza[:2,-1]
        co[-1,-1] = covarianza[-1,-1]
        self.odom_msg.pose.covariance = co.reshape(36).tolist()
        self.odom_msg.twist.twist.linear.x=self.vel
        self.odom_msg.twist.twist.angular.z= self.w
        return self.odom_msg

    def transform(self,odom):
        tnf=TransformStamped()
        tnf.header.stamp=rospy.Time.now()
        tnf.header.frame_id= "odom"
        tnf.child_frame_id="base_link"
        tnf.transform.translation.x = odom.pose.pose.position.x
        tnf.transform.translation.y = odom.pose.pose.position.y
        tnf.transform.translation.z = 0.0
        #rotation
        tnf.transform.rotation.x = odom.pose.pose.orientation.x
        tnf.transform.rotation.y = odom.pose.pose.orientation.y
        tnf.transform.rotation.z = odom.pose.pose.orientation.z
        tnf.transform.rotation.w = odom.pose.pose.orientation.w
        self.snt_tnf.sendTransform(
        (tnf.transform.translation.x, tnf.transform.translation.y, tnf.transform.translation.z),
        (tnf.transform.rotation.x, tnf.transform.rotation.y, tnf.transform.rotation.z, tnf.transform.rotation.w),
        rospy.Time.now(),
        "base_link",
        tnf.header.frame_id
    )
    
    def wrap_to_Pi(self,theta):
        result = np.fmod((theta + np.pi),(2 * np.pi))
        if(result < 0):
                result += 2 * np.pi
        return result - np.pi
    
    def get_covariance(self, dt):
         gra_wk = 0.5 * self.radius * dt * np.array([[np.cos(self.theta), np.cos(self.theta)],[np.sin(self.theta), np.sin(self.theta)],[2.0/self.wheelbase, -2.0/self.wheelbase]])
         
         self.cov_delta_q = np.array([[self.kr * abs(self.wr_speed), 0.0], [0.0, self.kl * abs(self.wl_speed)]])
         
         self.Qk_matrix = np.matmul(np.matmul(gra_wk,self.cov_delta_q), np.transpose(gra_wk))
         
         self.H_matrix = np.array([[1.0,0.0, -dt * self.vel * np.sin(self.theta)],[0.0,1.0,dt * self.vel * np.cos(self.theta)],[0.0,0.0,1.0]])
         
         self.covariance_matrix = np.matmul(np.matmul(self.H_matrix, self.covariance_matrix),np.transpose(self.H_matrix)) + self.Qk_matrix
         
         return self.covariance_matrix

    def calculate_odometry(self):
        current_time = rospy.Time.now()  # Get current time
    
        if self.first:
            self.previous_time = current_time
            self.first = False
        else:
            dt = (current_time - self.previous_time).to_sec()  # get dt
            self.previous_time = current_time

            #Get the covariance 
            covariance_matrix=self.get_covariance(dt)
            
            #get the pose of the robot
            self.get_positon(self.wr_speed,self.wl_speed,dt)

            # Create Odometry message
            odom_msg = self.get_odometry(current_time,self.x,self.y,self.theta,covariance_matrix)
            
            # Publish Odometry message
            self.odom_pub.publish(odom_msg)
    
            # Publish transform
            self.transform(odom_msg)
    
    def run(self):
           # Run the node
           while not rospy.is_shutdown():
               self.calculate_odometry()
               self.rate.sleep()

if __name__ == "__main__":
    try:
        node = LocalizationNode()
        node.run()
    except rospy.ROSInterruptException:
        pass
