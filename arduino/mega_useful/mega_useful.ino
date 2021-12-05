
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

#define encoder0PinA 2      // encoder 1
#define encoder0PinB 3

#define encoder1PinA 18     // encoder 2
#define encoder1PinB 19

volatile long encoder0Pos = 0;    // encoder 1
volatile long encoder1Pos = 0;    // encoder 2

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


  pinMode(encoder0PinA, INPUT_PULLUP);    // encoder pins
  pinMode(encoder0PinB, INPUT_PULLUP);

  pinMode(encoder1PinA, INPUT_PULLUP); 
  pinMode(encoder1PinB, INPUT_PULLUP);

  attachInterrupt(digitalPinToInterrupt(encoder0PinA), doEncoderA, CHANGE);
  attachInterrupt(digitalPinToInterrupt(encoder0PinB), doEncoderB, CHANGE); 

  attachInterrupt(digitalPinToInterrupt(encoder1PinA), doEncoderC, CHANGE);
  attachInterrupt(digitalPinToInterrupt(encoder1PinB), doEncoderD, CHANGE); 

  
  nh.initNode();
  
  nh.subscribe(sub_left);
  nh.subscribe(sub_right);

  nh.advertise(pub_odometry_left);
  nh.advertise(pub_odometry_right);

  // Serial.begin(115200); 
}


// Make sure the servos are set correctly to zero before going on ROS
void loop() {
  if (initialised > 3) {

    if ((millis() - odometry_timer) > 50) { // publish wheel odometry every 50ms

      if (debug_messages) {
        char str[20];
        itoa( encoder0Pos, str, 10 );
        itoa( encoder1Pos, str+10, 10 );

        nh.logfatal(str);
        // Serial.println(str);
      }
      
      odometry_msg_left.data = encoder0Pos;
      odometry_msg_right.data = encoder1Pos;
      
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



// void loop() {
//   if (initialised > 3) {

//     if ((millis() - odometry_timer) > 50) { // publish wheel odometry every 50ms

//       if (debug_messages) {
//         char str[20];
//         // itoa( counter_left, str, 10 );
//         // itoa( counter_right, str+10, 10 );

//         Serial.print(encoder0Pos); 
//         Serial.print(" - "); 
//         Serial.print(encoder1Pos); 
      
//         Serial.println();
//         // Serial.println(str);
//       }
    
//     }
//     delay(50);
//   }
//   else {
//     initialised += 1;
//     delay(1000);
//   }
// }





// ************** encoders interrupts **************

// ************** encoder 1 *********************


void doEncoderA(){  

  // look for a low-to-high on channel A
  if (digitalRead(encoder0PinA) == HIGH) { 
    // check channel B to see which way encoder is turning
    if (digitalRead(encoder0PinB) == LOW) {  
      encoder0Pos = encoder0Pos + 1;         // CW
    } 
    else {
      encoder0Pos = encoder0Pos - 1;         // CCW
    }
  }
  else   // must be a high-to-low edge on channel A                                       
  { 
    // check channel B to see which way encoder is turning  
    if (digitalRead(encoder0PinB) == HIGH) {   
      encoder0Pos = encoder0Pos + 1;          // CW
    } 
    else {
      encoder0Pos = encoder0Pos - 1;          // CCW
    }
  }
 
}

void doEncoderB(){  

  // look for a low-to-high on channel B
  if (digitalRead(encoder0PinB) == HIGH) {   
   // check channel A to see which way encoder is turning
    if (digitalRead(encoder0PinA) == HIGH) {  
      encoder0Pos = encoder0Pos + 1;         // CW
    } 
    else {
      encoder0Pos = encoder0Pos - 1;         // CCW
    }
  }
  // Look for a high-to-low on channel B
  else { 
    // check channel B to see which way encoder is turning  
    if (digitalRead(encoder0PinA) == LOW) {   
      encoder0Pos = encoder0Pos + 1;          // CW
    } 
    else {
      encoder0Pos = encoder0Pos - 1;          // CCW
    }
  }
  

}

// ************** encoder 2 *********************

void doEncoderC(){  

  // look for a low-to-high on channel A
  if (digitalRead(encoder1PinA) == HIGH) { 
    // check channel B to see which way encoder is turning
    if (digitalRead(encoder1PinB) == LOW) {  
      encoder1Pos = encoder1Pos - 1;         // CW
    } 
    else {
      encoder1Pos = encoder1Pos + 1;         // CCW
    }
  }
  else   // must be a high-to-low edge on channel A                                       
  { 
    // check channel B to see which way encoder is turning  
    if (digitalRead(encoder1PinB) == HIGH) {   
      encoder1Pos = encoder1Pos - 1;          // CW
    } 
    else {
      encoder1Pos = encoder1Pos + 1;          // CCW
    }
  }
 
}

void doEncoderD(){  

  // look for a low-to-high on channel B
  if (digitalRead(encoder1PinB) == HIGH) {   
   // check channel A to see which way encoder is turning
    if (digitalRead(encoder1PinA) == HIGH) {  
      encoder1Pos = encoder1Pos - 1;         // CW
    } 
    else {
      encoder1Pos = encoder1Pos + 1;         // CCW
    }
  }
  // Look for a high-to-low on channel B
  else { 
    // check channel B to see which way encoder is turning  
    if (digitalRead(encoder1PinA) == LOW) {   
      encoder1Pos = encoder1Pos - 1;          // CW
    } 
    else {
      encoder1Pos = encoder1Pos + 1;          // CCW
    }
  }
  

}