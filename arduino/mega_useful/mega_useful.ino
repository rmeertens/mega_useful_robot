
#include <ros.h>
#include <std_msgs/Empty.h>
#include <std_msgs/UInt8.h>
#include <std_msgs/Int16.h>
#include <Servo.h>

bool debug_messages = false;

ros::NodeHandle  nh;
int left_pin = 8;
int right_pin = 9;

int delaytime = 15; // Time delay between reading messages
int initialised = 0;
unsigned long time_since_start = 0;

char showreceived[] = "received";

Servo leftservo;  // create servo object to control a servo
Servo rightservo;  // create servo object to control a servo


void leftmotormessage( const std_msgs::UInt8& motor_msg) {
  if (debug_messages) {
    nh.logfatal(showreceived);
  }

  if (motor_msg.data <= 180) {
    leftservo.write(motor_msg.data);
  }

}

void rightmotormessage( const std_msgs::UInt8& motor_msg) {
  if (debug_messages) {
    nh.logfatal(showreceived);
  }
  if (motor_msg.data <= 180) {
    rightservo.write(motor_msg.data);
  }

}

ros::Subscriber<std_msgs::UInt8> sub_left("left_motor", &leftmotormessage );
ros::Subscriber<std_msgs::UInt8> sub_right("right_motor", &rightmotormessage );

void setup() {
  pinMode(LED_BUILTIN, OUTPUT);

  leftservo.attach(left_pin, 1000, 2000);  // 1000 and 2000 are values for the ESCS I'm using
  leftservo.write(90); // Value goes between 0 and 180 for a servo. 90 means neutral if your esc can go forwards and backwards

  rightservo.attach(right_pin, 1000, 2000);
  rightservo.write(90);

  nh.initNode();
  
  nh.subscribe(sub_left);
  nh.subscribe(sub_right);
}


// Make sure the servos are set correctly to zero before going on ROS
void loop() {
  if (initialised > 3) {
    nh.spinOnce();
  }
  else {
    initialised += 1;
    delay(1000);
  }
}
