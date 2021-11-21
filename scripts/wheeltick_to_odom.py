#!/usr/bin/env python
import rospy
from std_msgs.msg import String
from geometry_msgs.msg import Twist
from std_msgs.msg import UInt8, Int16

import tf
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Point, Pose, Quaternion, Twist, Vector3

from math import cos, sin

previousLeftEncoderCounts = 0 
previousRightEncoderCounts = 0 


DistancePerCount = (3.14159265 * 0.13) / 2300
lengthBetweenTwoWheels = 0.40

last_time_encoder = None

tick_x = 0
tick_y = 0

def wheel_callback(data, topic_name):
    global last_time_encoder
    global tick_x 
    global tick_y

#    rospy.loginfo(rospy.get_caller_id() + 'I heard wheel callback from %s message %s', topic_name, data)
    last_time_encoder = rospy.get_rostime().secs

    if topic_name == 'left': 
        tick_x = data.data
    else: 
        tick_y = data.data

def wrap_int16(offset, prev_value, new_value): 
    int16_max = 32767 
    if prev_value > 20000 and new_value < -20000: 
        offset += int16_max 
    elif prev_value < -20000 and new_value > 20000: 
        offset -= int16_max
    return offset


def listener():
    global tick_x 
    global tick_y

    rospy.init_node('wheeltick_to_odom', anonymous=False)

    rospy.Subscriber('/left_wheeltick_sensor', Int16, wheel_callback, callback_args='left')
    rospy.Subscriber('/right_wheeltick_sensor', Int16, wheel_callback, callback_args= 'right')


    odom_pub = rospy.Publisher("odom", Odometry, queue_size=50)
    odom_broadcaster = tf.TransformBroadcaster()

    rate = rospy.Rate(20) # 20hz

    _PreviousLeftEncoderCounts = None
    _PreviousRightEncoderCounts = None
     
    x = 0
    y = 0
    th = 0 

    offset_left = 0
    offset_right = 0 

    last_time = rospy.get_rostime() # gets the current time in float seconds
    while not rospy.is_shutdown():
        current_time = rospy.get_rostime()

        if not _PreviousLeftEncoderCounts: 
            _PreviousLeftEncoderCounts = tick_x;
            _PreviousRightEncoderCounts = tick_y;
            last_time = current_time
                
            continue

        
        dt = current_time - last_time

        offset_left = wrap_int16(offset_left, tick_x, _PreviousLeftEncoderCounts) 
        offset_right = wrap_int16(offset_right, tick_y, _PreviousRightEncoderCounts) 

        tick_x -= offset_left
        tick_y -= offset_right

        # extract the wheel velocities from the tick signals count
        deltaLeft = tick_x - _PreviousLeftEncoderCounts
        deltaRight = tick_y - _PreviousRightEncoderCounts

        v_left = (deltaLeft * DistancePerCount) / (current_time - last_time).to_sec()
        v_right = (deltaRight * DistancePerCount) / (current_time - last_time).to_sec()


        vx = ((v_right + v_left) / 2);
        vy = 0;
        vth = ((v_right - v_left)/lengthBetweenTwoWheels);

        dt = (current_time - last_time).to_sec();
        delta_x = (vx * cos(th)) * dt;
        delta_y = (vx * sin(th)) * dt;
        delta_th = vth * dt;

        x += delta_x;
        y += delta_y;
        th += delta_th;

        print('odometry', x, y, th)

        # since all odometry is 6DOF we'll need a quaternion created from yaw
        odom_quat = tf.transformations.quaternion_from_euler(0, 0, th)

        # first, we'll publish the transform over tf
        odom_broadcaster.sendTransform(
            (x, y, 0.),
            odom_quat,
            current_time,
            "base_link",
            "odom"
        )

        # next, we'll publish the odometry message over ROS
        odom = Odometry()
        odom.header.stamp = current_time
        odom.header.frame_id = "odom"

        # set the position
        odom.pose.pose = Pose(Point(x, y, 0.), Quaternion(*odom_quat))

        # set the velocity
        odom.child_frame_id = "base_link"
        odom.twist.twist = Twist(Vector3(vx, vy, 0), Vector3(0, 0, vth))

        # publish the message
        odom_pub.publish(odom)
        #geometry_msgs::Quaternion odom_quat = tf::createQuaternionMsgFromYaw(th);

        #geometry_msgs::TransformStamped odom_trans;
        #odom_trans.header.stamp = current_time;
        #odom_trans.header.frame_id = "odom";
        #odom_trans.child_frame_id = "base_link";

        #odom_trans.transform.translation.x = x;
        #odom_trans.transform.translation.y = y;
        #odom_trans.transform.translation.z = 0.0;
        #odom_trans.transform.rotation = odom_quat;

        #// send the transform
        #odom_broadcaster.sendTransform(odom_trans);

        #// Odometry message
        #nav_msgs::Odometry odom;
        #odom.header.stamp = current_time;
        #odom.header.frame_id = "odom";

        #// set the position
        #odom.pose.pose.position.x = x;
        #odom.pose.pose.position.y = y;
        #odom.pose.pose.position.z = 0.0;
        #odom.pose.pose.orientation = odom_quat;

        #// set the velocity
        #odom.child_frame_id = "base_link";
        #odom.twist.twist.linear.x = vx;
        #odom.twist.twist.linear.y = vy;
        #odom.twist.twist.angular.z = vth;

        #// publish the message
        #odom_pub.publish(odom);
        _PreviousLeftEncoderCounts = tick_x;
        _PreviousRightEncoderCounts = tick_y;

        #last_time = current_time;

        last_time = current_time
        rate.sleep()


if __name__ == '__main__':
    print("starting listener")
    listener()
