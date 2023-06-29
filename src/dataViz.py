#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import JointState
from tf2_msgs.msg import TFMessage

def js_callback(js_data):
    global jointState
    jointState =js_data

def tf_callback(tf_data):
    global tranforms
    tranforms = tf_data

if __name__ == '__main__':

    # -------------------------------------------------------------------#

    rospy.init_node("data_viz_node")
    rospy.Subscriber("/joint_states", JointState, js_callback)
    rospy.Subscriber("/tf", TFMessage, tf_callback)

    jointState = JointState()
    tranforms = TFMessage()

    rate = rospy.Rate(50)

# -------------------------------------------------------------------#

    while not rospy.is_shutdown():
        print(tranforms)

        rate.sleep()


# -------------------------------------------------------------------#

    rospy.spin()
