#!/usr/bin/env python
import rospy
from std_msgs.msg import String, Header
from nav_msgs.msg import Odometry
import numpy as np

pub = rospy.Publisher('odom/odom0', Odometry)

if __name__ == '__main__':
    rospy.init_node('odom0')
    r = rospy.Rate(1)
    i = 0
    while not rospy.is_shutdown():
        o = Odometry()
        o.header.seq = i
        i+=1
        o.header.stamp = rospy.get_rostime()
        o.header.frame_id = "map"
        o.child_frame_id = "odom"
        o.pose.covariance = [1,0,0,0,0,0,
                                  0,1,0,0,0,0,
                                  0,0,1,0,0,0,
                                  0,0,0,0,0,0,
                                  0,0,0,0,0,0,
                                  0,0,0,0,0,0,
                                  ]

        pub.publish(o)
        r.sleep()
