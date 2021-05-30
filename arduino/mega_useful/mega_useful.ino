
#include <ros.h>
#include <std_msgs/Empty.h>
#include <std_msgs/UInt8.h>
#include <std_msgs/Int16.h>
#include <Servo.h>

bool debug_messages = false;

ros::NodeHandle  nh;
std_msgs::Int16 odometry_msg_left;
std_msgs::Int16 odometry_msg_right;
ros::Publisher pub_odometry_left( "left_wheeltick_sensor", &odometry_msg_left);
ros::Publisher pub_odometry_right( "right_wheeltick_sensor", &odometry_msg_right);

int left_pin = 8;
int right_pin = 9;


int left_odometry_pin_green = 2;
int left_odometry_pin_white = 6;

int right_odometry_pin_green = 3;
int right_odometry_pin_white = 7;


volatile int16_t counter_left = 0;
volatile int16_t counter_right = 0;
unsigned long odometry_timer;


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

  
  pinMode(left_odometry_pin_green, INPUT_PULLUP);
  pinMode(right_odometry_pin_green, INPUT_PULLUP);

  attachInterrupt(digitalPinToInterrupt(left_odometry_pin_green), ai0, RISING);
  attachInterrupt(digitalPinToInterrupt(right_odometry_pin_green), ai1, RISING);
//      nh.getHardware()->setBaud(115200);

  nh.initNode();
  
  nh.subscribe(sub_left);
  nh.subscribe(sub_right);
  nh.advertise(pub_odometry_left);
  nh.advertise(pub_odometry_right);
}


// Make sure the servos are set correctly to zero before going on ROS
void loop() {
  if (initialised > 3) {
    if ((millis() - odometry_timer) > 50) { // publish wheel odometry every 50ms

      if (debug_messages) {
        char str[20];
        itoa( counter_left, str, 10 );
        itoa( counter_right, str+10, 10 );

        nh.logfatal(str);
      }
      
      odometry_msg_left.data = counter_left;
      odometry_msg_right.data = counter_right;
      
      pub_odometry_left.publish(&odometry_msg_left);
      pub_odometry_right.publish(&odometry_msg_right);
      odometry_timer = millis(); // reset the timer

    }
    nh.spinOnce();
  }
  else {
    initialised += 1;
    delay(1000);
  }
}


void ai0() {
  if (digitalRead(left_odometry_pin_white) == LOW) {
    counter_left++;
  } else {
    counter_left--;
  }
}

void ai1() {
  if (digitalRead(right_odometry_pin_white) == LOW) {
    counter_right--;
  } else {
    counter_right++;
  }
}
