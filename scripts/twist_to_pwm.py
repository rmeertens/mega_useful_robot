#!/usr/bin/env python
import rospy
from std_msgs.msg import String
from geometry_msgs.msg import Twist
from std_msgs.msg import UInt8

last_twist = Twist()
last_seconds = 0

linear_gain = 80
angular_gain = 45

log_each = 10
log_now = 0

def twist_callback(data):
    global last_twist
    global last_seconds
    rospy.loginfo(rospy.get_caller_id() + 'I heard twist message %s', data)
    last_twist = data
    last_seconds = rospy.get_rostime().secs



def listener():
    global log_each
    global log_now
    global last_seconds

    rospy.init_node('listener', anonymous=False)

    rospy.Subscriber('cmd_vel', Twist, twist_callback)

    pub_left = rospy.Publisher('left_motor', UInt8, queue_size=10)
    pub_right = rospy.Publisher('right_motor', UInt8, queue_size=10)
    rate = rospy.Rate(10) # 10hz

    center_point = 90

    while not rospy.is_shutdown():
        if rospy.get_rostime().secs - last_seconds < 4.0: 
            left_add = linear_gain*last_twist.linear.x + angular_gain*last_twist.angular.z
            right_add = linear_gain*last_twist.linear.x - angular_gain*last_twist.angular.z
        else:
            rospy.logerr("no message received in the last 4 seconds")
            left_add = 0
            right_add = 0
        
        # Note: dirty hack to either make the motors move, or not, no inbetween
        if abs(left_add) > 15:
            left_motor_speed = center_point + left_add
        else:
            left_motor_speed = center_point

        if abs(right_add) > 15:
            right_motor_speed = center_point + right_add
        else:
            right_motor_speed = center_point

        log_now += 1
        if log_now%log_each == 0:
            message = "Setting left" + str(left_motor_speed) + 'right' + str(right_motor_speed)
            rospy.logerr(message)

        pub_left.publish(left_motor_speed)
        pub_right.publish(right_motor_speed)
        rate.sleep()


if __name__ == '__main__':
    listener()
