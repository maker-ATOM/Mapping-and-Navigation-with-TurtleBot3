#!/usr/bin/env python

import rospy
from geometry_msgs.msg import PoseArray, Pose
from nav_msgs.msg import Odometry
from tf.transformations import quaternion_from_euler, euler_from_quaternion
from sensor_msgs.msg import LaserScan

import random as rn
import math


def odom_callback(botPose):
    global robotPose
    robotPose = botPose

# def scan_callback(botLaserScan):
#     global robotLaserScan
#     robotLaserScan = botLaserScan

if __name__ == '__main__':
    # --------------------------------- SETUP ------------------------------------------#

    particle_count = 1000
    robotPose = Odometry()
    # robotLaserScan = LaserScan()
    i = 0

    rospy.init_node("localizer_node")
    localizer_pub_PoseArray = rospy.Publisher("particle_cloud_PoseArray", PoseArray, queue_size=1)
    localizer_pub_Origin = rospy.Publisher("particle_cloud_Origin", PoseArray, queue_size=1)
    # localizer_pub_LaserScan = rospy.Publisher("particle_cloud_LaserScan", LaserScan, queue_size=1)
    rospy.Subscriber("/odom", Odometry, odom_callback)
    # rospy.Subscriber("/scan", LaserScan, scan_callback)

    particle_cloud_PoseArray = PoseArray()
    particle_cloud_PoseArray.header.frame_id = 'map'

    particle_cloud_Origin = PoseArray()
    particle_cloud_Origin.header.frame_id = 'map'

    # particle_cloud_LaserScan = LaserScan()
    # particle_cloud_LaserScan.header.frame_id = "base_scan"
    # particle_cloud_LaserScan.angle_min = 0.0
    # particle_cloud_LaserScan.angle_max = 6.28
    # particle_cloud_LaserScan.angle_increment = 0.0175
    # particle_cloud_LaserScan.range_min = 0.12
    # particle_cloud_LaserScan.range_max = 7.0
    # custom_intensities = []
    # custom_ranges = []
    # for i in range(359):
    #     custom_ranges.append(0)
    # particle_cloud_LaserScan.ranges = custom_ranges
    # for i in range(359):
    #     custom_intensities.append(0)
    # particle_cloud_LaserScan.intensities = custom_intensities
   
    for i in range(particle_count):
        p = Pose()

        p.position.x = rn.randrange(-222, 222, 1) / 100
        p.position.y = rn.randrange(-222, 222, 1) / 100
        # p.position.x = 0.5
        # p.position.y = 1
        p.position.z = 0
 
        roll = 0
        pitch = 0
        yaw = rn.randrange(-314, 314, 1)/100
        # yaw = 1.57
   
        q = quaternion_from_euler(roll, pitch, yaw)
        p.orientation.x = q[0]
        p.orientation.y = q[1]
        p.orientation.z = q[2]
        p.orientation.w = q[3]

        particle_cloud_Origin.poses.append(p)
        particle_cloud_PoseArray.poses.append(p)


    rate = rospy.Rate(20)
    rospy.sleep(1)

    # --------------------------------- LOOP -------------------------------------------#
    while not rospy.is_shutdown():
        for i in range(0, particle_count):
            pfp = Pose()    # particle final pose

            # robot pose
            xr = robotPose.pose.pose.position.x
            yr = robotPose.pose.pose.position.y
            (rollr, pitchr, yawr) = euler_from_quaternion([robotPose.pose.pose.orientation.x, robotPose.pose.pose.orientation.y, robotPose.pose.pose.orientation.z, robotPose.pose.pose.orientation.w])
            
            # origin pose
            xp = particle_cloud_Origin.poses[i].position.x
            yp = particle_cloud_Origin.poses[i].position.y
            (rollo, pitcho, yawo) = euler_from_quaternion([particle_cloud_Origin.poses[i].orientation.x, particle_cloud_Origin.poses[i].orientation.y, particle_cloud_Origin.poses[i].orientation.z, particle_cloud_Origin.poses[i].orientation.w])
            
            # rotational transformation
            xn = xr * math.cos(yawo) - yr * math.sin(yawo)
            yn = xr * math.sin(yawo) + yr * math.cos(yawo)

            pfp.position.x = xp + xn
            pfp.position.y = yp + yn

            q = quaternion_from_euler(0, 0, yawr + yawo)
            pfp.orientation.x = q[0]
            pfp.orientation.y = q[1]
            pfp.orientation.z = q[2]
            pfp.orientation.w = q[3]

            particle_cloud_PoseArray.poses[i] = pfp

            # LaserScan updation
            # for i in range(0, 359):
            #     particle_cloud_LaserScan.ranges[i] = robotLaserScan.ranges[i]

        localizer_pub_Origin.publish(particle_cloud_Origin)
        localizer_pub_PoseArray.publish(particle_cloud_PoseArray)
        # localizer_pub_LaserScan.publish(particle_cloud_LaserScan)

    # ----------------------------------------------------------------------------------#

    rospy.spin()
