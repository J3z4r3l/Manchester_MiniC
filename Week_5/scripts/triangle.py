#!/usr/bin/env python

import rospy
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point, Quaternion

def publish_triangle():
    rospy.init_node('triangle_publisher', anonymous=True)
    marker_pub = rospy.Publisher('triangle_marker', Marker, queue_size=10)
    rate = rospy.Rate(1)  # 1 Hz
    half_dist=0.015 #equals y
    x_base_pose=3.39
    rotation=0.026

    while not rospy.is_shutdown():
        marker = Marker()
        marker.header.frame_id = "odom"  # Frame de referencia (ajusta  tu caso)
        marker.header.stamp = rospy.Time.now()
        marker.ns = "triangle"
        marker.id = 0
        marker.type = Marker.TRIANGLE_LIST
        marker.action = Marker.ADD
        marker.scale.x = 1.0  # 
        marker.scale.y = 1.0
        marker.scale.z = 1.0
        marker.color.a = 1.0
        marker.color.r = 1.0
        marker.color.g = 0.0
        marker.color.b = 0.0

        p1 = Point() ###disstance 1
        p1.x = x_base_pose
        p1.y = half_dist-0.10#l_l 
        p1.z = 0.0
        p2 = Point() ##top
        p2.x = x_base_pose+0.2
        p2.y = half_dist#l/2
        p2.z = 0.0
        p3 = Point()#distance 2
        p3.x = x_base_pose
        p3.y = half_dist+0.10 #l
        p3.z = 0.0

        marker.points.append(p1)
        marker.points.append(p2)
        marker.points.append(p3)

        marker.pose.orientation.x = 0.0
        marker.pose.orientation.y = 0.0
        marker.pose.orientation.z = rotation
        marker.pose.orientation.w = 1.0

        marker_pub.publish(marker)
        rate.sleep()

if __name__ == '__main__':
    try:
        publish_triangle()
    except rospy.ROSInterruptException:
        pass
