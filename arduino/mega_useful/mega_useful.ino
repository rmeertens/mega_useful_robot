
#include <ros.h>
#include <std_msgs/Empty.h>
#include <std_msgs/UInt8.h>
#include <std_msgs/Int16.h>
#include <Servo.h>

ros::NodeHandle  nh;
std_msgs::Int16 odometry_msg; 
ros::Publisher pub_odometry( "left_wheeltick_sensor", &odometry_msg);


int left_pin = 8;
int right_pin = 9;

int left_odometry_pin_white = 2;
int left_odometry_pin_green = 3;
volatile int16_t counter = 0;
unsigned long odometry_timer;


int delaytime = 15; // Time delay between reading messages
int initialised = 0;
unsigned long time_since_start = 0;

char showreceived[] = "received";

Servo leftservo;  // create servo object to control a servo
Servo rightservo;  // create servo object to control a servo


void leftmotormessage( const std_msgs::UInt8& motor_msg) {
  nh.logfatal(showreceived);
  if (motor_msg.data <= 180) {
    leftservo.write(motor_msg.data);
  }

}

void rightmotormessage( const std_msgs::UInt8& motor_msg) {
  nh.logfatal(showreceived);
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

  pinMode(left_odometry_pin_white, INPUT_PULLUP);
  pinMode(left_odometry_pin_green, INPUT_PULLUP);

  attachInterrupt(digitalPinToInterrupt(left_odometry_pin_white), ai0, RISING);
  attachInterrupt(digitalPinToInterrupt(left_odometry_pin_green), ai1, RISING);

  nh.initNode();
  nh.subscribe(sub_left);
  nh.subscribe(sub_right);
  nh.advertise(pub_odometry);
}


// Make sure the servos are set correctly to zero before going on ROS
void loop() {
  if (initialised > 3) {
    if((millis() - odometry_timer) > 50){ // publish wheel odometry every 50ms

      char str[8];
      itoa( counter, str, 10 );

      nh.logfatal(str);

      odometry_msg.data = counter;
     // odometry_msg.header.stamp = nh.now();
      pub_odometry.publish(&odometry_msg);
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
  // ai0 is activated if DigitalPin nr 2 is going from LOW to HIGH
  // Check pin 3 to determine the direction
  if (digitalRead(left_odometry_pin_green) == LOW) {
    counter++;
  } else {
    counter--;
  }
}

void ai1() {
  // ai0 is activated if DigitalPin nr 3 is going from LOW to HIGH
  // Check with pin 2 to determine the direction
  if (digitalRead(left_odometry_pin_white) == LOW) {
    counter--;
  } else {
    counter++;
  }
}
