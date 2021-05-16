#!/usr/bin/env python

## Simple talker demo that listens to std_msgs/Strings published 
## to the 'chatter' topic

import rospy
from std_msgs.msg import String
from geometry_msgs.msg import Twist
from std_msgs.msg import UInt8

last_twist = Twist()

linear_gain = 20
angular_gain = 15

def callback(data):
    rospy.loginfo(rospy.get_caller_id() + 'I heard %s', data.data)

def twist_callback(data):
    global last_twist
    rospy.loginfo(rospy.get_caller_id() + 'I heard twist message %s', data)
    last_twist = data


def listener():

    # In ROS, nodes are uniquely named. If two nodes with the same
    # name are launched, the previous one is kicked off. The
    # anonymous=True flag means that rospy will choose a unique
    # name for our 'listener' node so that multiple listeners can
    # run simultaneously.
    rospy.init_node('listener', anonymous=True)

    rospy.Subscriber('chatter', String, callback)
    rospy.Subscriber('cmd_vel', Twist, twist_callback)

    # spin() simply keeps python from exiting until this node is stopped
#    rospy.spin()
    pub_left = rospy.Publisher('left_motor', UInt8, queue_size=10)
    pub_right = rospy.Publisher('right_motor', UInt8, queue_size=10)
    rate = rospy.Rate(10) # 10hz

    center_point = 90

    while not rospy.is_shutdown():
        hello_str = "hello world %s" % rospy.get_time()
        rospy.loginfo(hello_str)
        rospy.loginfo("Got this as a last twist message")
        rospy.loginfo(last_twist)

        left_motor_speed = center_point + linear_gain*last_twist.linear.x + angular_gain*last_twist.angular.z
        right_motor_speed = center_point + linear_gain*last_twist.linear.x - angular_gain*last_twist.angular.z
        message = "Setting left" + str(left_motor_speed) + 'right' + str(right_motor_speed)
        rospy.logerr(message)
        pub_left.publish(left_motor_speed)
        pub_right.publish(right_motor_speed)
        rate.sleep()


if __name__ == '__main__':
    listener()
