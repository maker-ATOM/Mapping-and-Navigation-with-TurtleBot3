#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import JointState
from tf2_msgs.msg import TFMessage
from rosgraph_msgs.msg import Clock
from geometry_msgs.msg import Twist
from gazebo_msgs.msg import LinkStates
from gazebo_msgs.msg import ModelStates
from dynamic_reconfigure.msg import ConfigDescription
from dynamic_reconfigure.msg import Config
from gazebo_msgs.msg import PerformanceMetrics
from gazebo_msgs.msg import LinkState
from gazebo_msgs.msg import ModelState
from sensor_msgs.msg import Imu
from nav_msgs.msg import Odometry
from rosgraph_msgs.msg import Log
from sensor_msgs.msg import LaserScan


def callback(data):
    pass



if __name__ == '__main__':

    # -------------------------------------------------------------------#

    rospy.init_node("dummy_node")
    rospy.Subscriber("/clock", Clock, callback)
    rospy.Subscriber("/cmd_vel", Twist, callback)
    rospy.Subscriber("/imu", Imu, callback)
    rospy.Subscriber("/joint_states", JointState, callback)
    rospy.Subscriber("/odom", Odometry, callback)
    rospy.Subscriber("/rosout", Log, callback)
    rospy.Subscriber("/rosout_agg", Log, callback)
    rospy.Subscriber("/scan", LaserScan, callback)
    rospy.Subscriber("/tf", TFMessage, callback)

    jointState = JointState()
    tranforms = TFMessage()

    rate = rospy.Rate(50)

# -------------------------------------------------------------------#

    while not rospy.is_shutdown():
        pass


# -------------------------------------------------------------------#

    rospy.spin()
