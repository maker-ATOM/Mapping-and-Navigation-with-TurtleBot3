#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist

import tf
from statistics import mean

object_dist = 0
obstacle_threshold = 0.4
turn_threshold = 1
raw_ld_data = 0

yaw = 0

def lidar_callback(ld_data):
    global raw_ld_data
    raw_ld_data = ld_data.ranges

def odom_callback(odom_data):
    global yaw
    (r, p, y) = tf.transformations.euler_from_quaternion(
        [odom_data.pose.pose.orientation.x, odom_data.pose.pose.orientation.y, odom_data.pose.pose.orientation.z, odom_data.pose.pose.orientation.w])
    yaw = y

if __name__ == '__main__':

    # -------------------------------------------------------------------#

    rospy.init_node("data_viz_node")

    lidar_topic = "/scan"
    rospy.Subscriber(lidar_topic, LaserScan, lidar_callback)

    odom_topic = "/odom"
    rospy.Subscriber(odom_topic, Odometry, odom_callback)

    cmdvel_topic = "/cmd_vel"
    pub = rospy.Publisher(cmdvel_topic, Twist, queue_size = 10)

    botVel = Twist()

    rate = rospy.Rate(50)

    rospy.sleep(1)
    botVel.linear.x = 0.15

# -------------------------------------------------------------------#

    while not rospy.is_shutdown():

        a = raw_ld_data[0:20]
        b = raw_ld_data[339:359]

        object_dist = round(min(a + b),2)

        print(max(raw_ld_data[0:180]), max(raw_ld_data[180:359]), botVel.angular.z)

        if object_dist < obstacle_threshold:
            botVel.linear.x = 0
            if max(raw_ld_data[0:180]) > max(raw_ld_data[180:359]):
                botVel.angular.z = 0.15
            else:
                botVel.angular.z = -0.15
        
        if object_dist > turn_threshold:
            botVel.linear.x = 0.15
            botVel.angular.z = 0

        pub.publish(botVel)
        rate.sleep()


# -------------------------------------------------------------------#

    rospy.spin()
