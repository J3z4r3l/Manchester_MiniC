#!/usr/bin/env python
import rospy 
import numpy as np
from std_msgs.msg import Float32
from sensor_msgs.msg import JointState
from geometry_msgs.msg import Twist, PoseStamped
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TransformStamped
from tf import TransformBroadcaster

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
        self.theta = 0
        self.snt_tnf=TransformBroadcaster()
        self.pose_robot=PoseStamped()

        self.odom_pub = rospy.Publisher("/odom", Odometry, queue_size=10)
        rospy.Subscriber("/wr", Float32, self.wr_callback)
        rospy.Subscriber("/wl", Float32, self.wl_callback)
        rospy.Subscriber("/pose", PoseStamped, self.pose_stamped)
         
    def wr_callback(self, v_r):
        self.wr_speed = v_r.data

    def wl_callback(self, v_l):
        self.wl_speed = v_l.data
    
    def pose_stamped(self, msg):
    # Update robot's pose
        self.pose_robot = msg
    
    def get_odometry(self,current_time):
        odom_msg = Odometry()
        odom_msg.header.stamp = current_time
        odom_msg.header.frame_id = "odom" #or odom
        odom_msg.child_frame_id = "base_link"
        odom_msg.pose.pose.position.x = self.pose_robot.pose.position.x
        odom_msg.pose.pose.position.y = self.pose_robot.pose.position.y
        odom_msg.pose.pose.orientation.x = self.pose_robot.pose.orientation.x
        odom_msg.pose.pose.orientation.y = self.pose_robot.pose.orientation.y
        odom_msg.pose.pose.orientation.z = self.pose_robot.pose.orientation.z
        odom_msg.pose.pose.orientation.w = self.pose_robot.pose.orientation.w
        return odom_msg

    def transform(self,odom):
        tnf=TransformStamped()
        tnf.header.stamp=rospy.Time.now()
        tnf.header.frame_id= "odom"
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


    
    def calculate_odometry(self):
        current_time = rospy.Time.now()  # Get current time
    
        if self.first:
            self.previous_time = current_time
            self.first = False
        else:
            dt = (current_time - self.previous_time).to_sec()  # get dt
            self.previous_time = current_time
    
            # Create Odometry message
            odom_msg = self.get_odometry(current_time)
            
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
