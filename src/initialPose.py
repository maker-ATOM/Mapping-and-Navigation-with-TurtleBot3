#!/usr/bin/env python3

import rospy
from tf2_msgs.msg import TFMessage
from geometry_msgs.msg import PoseWithCovarianceStamped
from nav_msgs.msg import Odometry

if __name__ == '__main__':

    # -------------------------------------------------------------------#

    rospy.init_node("initialPose")
    initialPose_pub = rospy.Publisher("/initialpose", PoseWithCovarianceStamped, queue_size=1)

    robotPose = Odometry()
    initialPose = PoseWithCovarianceStamped()

    robotPose = rospy.wait_for_message('/odom', Odometry)

    initialPose.pose.pose = robotPose.pose.pose
    initialPose.header.frame_id = "map"
    # How much error is present in my pose estimation
    initialPose.pose.covariance = [0.25, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.25, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
                                    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
                                    0.0, 0.0, 0.0, 0.0, 0.0, 0.0]

    rospy.sleep(1)
    initialPose_pub.publish(initialPose)