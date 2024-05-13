#!/usr/bin/env python
import rospy
from tf import TransformBroadcaster  
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point, Quaternion
from std_msgs.msg import Float32
import numpy as np
from tf.transformations import quaternion_from_euler

class TrianglePublisher:
    def __init__(self):
        rospy.init_node('triangle_publisher', anonymous=True)
        self.marker_pub = rospy.Publisher('triangle_marker', Marker, queue_size=10)
        self.rate = rospy.Rate(100)  # 1 Hz
        self.half_dist = 0.02616352319  # equals y
        self.x_base_pose =2.091621188
        self.rotation = 0#np.pi*0.34
        self.theta = 0.0
        self.x = 0.0
        self.y = 0.0
        self.tf_broadcaster = TransformBroadcaster()  # Inicializa el TransformBroadcaster

    def publish_triangle(self):
        while not rospy.is_shutdown():
            marker = Marker()
            marker.header.frame_id = "odom"  # Frame de referencia (ajusta  tu caso)
            marker.header.stamp = rospy.Time.now()
            marker.ns = "triangle"
            marker.id = 0
            marker.type = Marker.TRIANGLE_LIST
            marker.action = Marker.ADD
            marker.scale.x = 1.0
            marker.scale.y = 1.0
            marker.scale.z = 1.0
            marker.color.a = 1.0
            marker.color.r = 1.0
            marker.color.g = 0.0
            marker.color.b = 0.0

            p1 = Point()  # distance 1
            p1.x = self.x_base_pose
            p1.y = self.half_dist - 0.10  # l_l
            p1.z = 0.0
            p2 = Point()  # top
            p2.x = self.x_base_pose + 0.5
            p2.y = self.half_dist  # l/2
            p2.z = 0.0
            p3 = Point()  # distance 2
            p3.x = self.x_base_pose
            p3.y = self.half_dist + 0.10  # l
            p3.z = 0.0
            print(self.half_dist)
            marker.points.append(p1)
            marker.points.append(p2)
            marker.points.append(p3)

            rotation_quaternion = quaternion_from_euler(0, 0, self.rotation)
            marker.pose.orientation.x = rotation_quaternion[0]
            marker.pose.orientation.y = rotation_quaternion[1]
            marker.pose.orientation.z = rotation_quaternion[2]
            marker.pose.orientation.w = rotation_quaternion[3]

            # Publica el marcador
            self.marker_pub.publish(marker)

            self.rate.sleep()

if __name__ == '__main__':
    try:
        triangle_publisher = TrianglePublisher()
        triangle_publisher.publish_triangle()
    except rospy.ROSInterruptException:
        pass
