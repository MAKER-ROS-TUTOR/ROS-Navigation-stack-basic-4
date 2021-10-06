// last update 4-10-21 21:00
/*
 * Author: Automatic Addison
 * Website: https://automaticaddison.com
 * Description: ROS node that publishes the accumulated ticks for each wheel
 * (/right_ticks and /left_ticks topics) at regular intervals using the 
 * built-in encoder (forward = positive; reverse = negative). 
 * The node also subscribes to linear & angular velocity commands published on 
 * the /cmd_vel topic to drive the robot accordingly.
 * Reference: Practical Robotics in C++ book (ISBN-10 : 9389423465)
 */
 
#include <ros.h>
#include <std_msgs/Int32.h>
#include <geometry_msgs/Twist.h>
//#include <rospy_tutorials/Floats.h>

#define MAX_INT32  2147483647
#define MIN_INT32 -2147483648 
// Handles startup and shutdown of ROS
ros::NodeHandle nh;
 
////////////////// Tick Data Publishing Variables and Constants ///////////////
 
// Encoder output to Arduino Interrupt pin. Tracks the tick count.

#define ENC_IN_RIGHT_A 19
#define ENC_IN_RIGHT_B 18 
// Other encoder output to Arduino to keep track of wheel direction
// Tracks the direction of rotation.
#define ENC_IN_LEFT_A 20
#define ENC_IN_LEFT_B 21

//rospy_tutorials::Floats right_joint_state, left_joint_state;
 
// True = Forward; False = Reverse
boolean Direction_left = true;
boolean Direction_right = true;
 
// Minumum and maximum values for 32-bit integers
// Range of 2147483647
const long encoder_minimum = MIN_INT32;
const long encoder_maximum = MAX_INT32;
 
// Keep track of the number of wheel ticks
std_msgs::Int32 right_wheel_tick_count;
ros::Publisher rightPub("right_ticks", &right_wheel_tick_count);
 
std_msgs::Int32 left_wheel_tick_count;
ros::Publisher leftPub("left_ticks", &left_wheel_tick_count);
 
// Time interval for measurements in milliseconds
const int interval = 30;
long previousMillis = 0;
long currentMillis = 0;
 
////////////////// Motor Controller Variables and Constants ///////////////////
 
// Motor A connections

const int in1 = 5;
const int in2 = 6;
  
// Motor B connections
 
const int in3 = 7;
const int in4 = 8;
 
// How much the PWM value can change each cycle
const int PWM_INCREMENT = 1;
 
// Number of ticks per wheel revolution. We won't use this in this code.
const int TICKS_PER_REVOLUTION = 39000;
 
// Wheel radius in meters
const double WHEEL_RADIUS = 0.1;
 
// Distance from center of the left tire to the center of the right tire in m
const double WHEEL_BASE = 0.48;
 
// Number of ticks a wheel makes moving a linear distance of 1 meter
// This value was measured manually.
const double TICKS_PER_METER = 62000; // Originally 2880
 
// Proportional constant, which was measured by measuring the 
// PWM-Linear Velocity relationship for the robot.
const int K_P = 278;
 
// Y-intercept for the PWM-Linear Velocity relationship for the robot
const int b = 52;
 
// Correction multiplier for drift. Chosen through experimentation.
const int DRIFT_MULTIPLIER = 120;
 
// Turning PWM output (0 = min, 255 = max for PWM values)
const int PWM_TURN = 80;
 
// Set maximum and minimum limits for the PWM values
const int PWM_MIN = 80; // about 0.1 m/s
const int PWM_MAX = 100; // about 0.172 m/s
 
// Set linear velocity and PWM variable values for each wheel
double velLeftWheel = 0;
double velRightWheel = 0;
double pwmLeftReq = 0;
double pwmRightReq = 0;
 
// Record the time that the last velocity command was received
double lastCmdVelReceived = 0;
 
/////////////////////// Tick Data Publishing Functions ////////////////////////
 
// Increment the number of ticks
void right_wheel_tickA() {

  if (digitalRead(ENC_IN_RIGHT_A) != digitalRead(ENC_IN_RIGHT_B))
  {
         Direction_right = true; // Forward
        

         if (right_wheel_tick_count.data == encoder_maximum) {
             right_wheel_tick_count.data = encoder_minimum;
         }
         else {
             right_wheel_tick_count.data++;  
         }    
              
  }else{
    
         Direction_right = false; // Reverse
        

         if (right_wheel_tick_count.data == encoder_minimum) {
             right_wheel_tick_count.data = encoder_maximum;
         }
         else {
             right_wheel_tick_count.data--;  
         }   
    
  }
    
  
}
void right_wheel_tickB() {

  if (digitalRead(ENC_IN_RIGHT_A) == digitalRead(ENC_IN_RIGHT_B))
  {
         Direction_right = true; // Forward
        

         if (right_wheel_tick_count.data == encoder_maximum) {
             right_wheel_tick_count.data = encoder_minimum;
         }
         else {
             right_wheel_tick_count.data++;  
         }    
              
  }else{
    
         Direction_right = false; // Reverse
         

         if (right_wheel_tick_count.data == encoder_minimum) {
             right_wheel_tick_count.data = encoder_maximum;
         }
         else {
             right_wheel_tick_count.data--;  
         }   
    
  }
    
  
}
 
// Increment the number of ticks
void left_wheel_tickA() {
  
  if (digitalRead(ENC_IN_LEFT_A) != digitalRead(ENC_IN_LEFT_B))
  {
         Direction_left = true; // Forward
         

         if (left_wheel_tick_count.data == encoder_maximum) {
             left_wheel_tick_count.data = encoder_minimum;
         }
         else {
             left_wheel_tick_count.data++;  
         }    
              
  }else{
    
         Direction_left = false; // Reverse
          

         if (left_wheel_tick_count.data == encoder_minimum) {
             left_wheel_tick_count.data = encoder_maximum;
         }
         else {
             left_wheel_tick_count.data--;  
         }   
    
  }
 
}
void left_wheel_tickB() {
  
  if (digitalRead(ENC_IN_LEFT_A) == digitalRead(ENC_IN_LEFT_B))
  {
         Direction_left = true; // Forward
         

         if (left_wheel_tick_count.data == encoder_maximum) {
             left_wheel_tick_count.data = encoder_minimum;
         }
         else {
             left_wheel_tick_count.data++;  
         }    
              
  }else{
    
         Direction_left = false; // Reverse
          

         if (left_wheel_tick_count.data == encoder_minimum) {
             left_wheel_tick_count.data = encoder_maximum;
         }
         else {
             left_wheel_tick_count.data--;  
         }   
    
  }
 
}
 
/////////////////////// Motor Controller Functions ////////////////////////////
 
// Calculate the left wheel linear velocity in m/s every time a 
// tick count message is rpublished on the /left_ticks topic. 

void calc_vel_left_wheel(){
   
  // Previous timestamp
  static double prevTime = 0;
   
  // Variable gets created and initialized the first time a function is called.
  static long prevLeftCount = 0;
 
  // Manage rollover and rollunder when we get outside the 32-bit integer range 
  long numOfTicks = ( MAX_INT32 + left_wheel_tick_count.data - prevLeftCount) % MAX_INT32;
 
  // If we have had a big jump, it means the tick count has rolled over.
  if (numOfTicks > 10000) {
        numOfTicks = 0 - ( MAX_INT32 - numOfTicks);
  }
 
  // Calculate wheel velocity in meters per second
  velLeftWheel = numOfTicks/TICKS_PER_METER/((millis()/1000)-prevTime);
 
  // Keep track of the previous tick count
  prevLeftCount = left_wheel_tick_count.data;
 
  // Update the timestamp
  prevTime = (millis()/1000);
 
}
 
// Calculate the right wheel linear velocity in m/s every time a 
// tick count message is published on the /right_ticks topic. 
void calc_vel_right_wheel(){
   
  // Previous timestamp
  static double prevTime = 0;
   
  // Variable gets created and initialized the first time a function is called.
  static int prevRightCount = 0;
 
  // Manage rollover and rollunder when we get outside the 16-bit integer range 
  int numOfTicks = (65535 + right_wheel_tick_count.data - prevRightCount) % 65535;
 
  if (numOfTicks > 10000) {
        numOfTicks = 0 - (65535 - numOfTicks);
  }
 
  // Calculate wheel velocity in meters per second
  velRightWheel = numOfTicks/TICKS_PER_METER/((millis()/1000)-prevTime);
 
  prevRightCount = right_wheel_tick_count.data;
   
  prevTime = (millis()/1000);
 
}
 
// Take the velocity command as input and calculate the PWM values.
void calc_pwm_values(const geometry_msgs::Twist& cmdVel) {
   
  // Record timestamp of last velocity command received
  lastCmdVelReceived = (millis()/1000);
   
  // Calculate the PWM value given the desired velocity 
  pwmLeftReq = K_P * cmdVel.linear.x + b;
  pwmRightReq = K_P * cmdVel.linear.x + b;
 
  // Check if we need to turn 
  if (cmdVel.angular.z != 0.0) {
 
    // Turn left
    if (cmdVel.angular.z > 0.0) {
      pwmLeftReq = -PWM_TURN;
      pwmRightReq = PWM_TURN;
    }
    // Turn right    
    else {
      pwmLeftReq = PWM_TURN;
      pwmRightReq = -PWM_TURN;
    }
  }
  // Go straight
  else {
     
    // Remove any differences in wheel velocities 
    // to make sure the robot goes straight
    static double prevDiff = 0;
    static double prevPrevDiff = 0;
    double currDifference = velLeftWheel - velRightWheel; 
    double avgDifference = (prevDiff+prevPrevDiff+currDifference)/3;
    prevPrevDiff = prevDiff;
    prevDiff = currDifference;
 
    // Correct PWM values of both wheels to make the vehicle go straight
    pwmLeftReq -= (int)(avgDifference * DRIFT_MULTIPLIER);
    pwmRightReq += (int)(avgDifference * DRIFT_MULTIPLIER);
  }
 
  // Handle low PWM values
  if (abs(pwmLeftReq) < PWM_MIN) {
    pwmLeftReq = 0;
  }
  if (abs(pwmRightReq) < PWM_MIN) {
    pwmRightReq = 0;  
  }  
}
 
void set_pwm_values() {
 
  // These variables will hold our desired PWM values
  static int pwmLeftOut = 0;
  static int pwmRightOut = 0;
 
  // If the required PWM is of opposite sign as the output PWM, we want to
  // stop the car before switching direction
  static bool stopped = false;
  
  if ((pwmLeftReq * velLeftWheel < 0 && pwmLeftOut != 0) ||
      (pwmRightReq * velRightWheel < 0 && pwmRightOut != 0)) {
    pwmLeftReq = 0;
    pwmRightReq = 0;
  }
 
  
 
  // Increase the required PWM if the robot is not moving
  if (pwmLeftReq != 0 && velLeftWheel == 0) {
    pwmLeftReq *= 1.5;
  }
  if (pwmRightReq != 0 && velRightWheel == 0) {
    pwmRightReq *= 1.5;
  }
 
  // Calculate the output PWM value by making slow changes to the current value
  if (abs(pwmLeftReq) > pwmLeftOut) {
    pwmLeftOut += PWM_INCREMENT;
  }
  else if (abs(pwmLeftReq) < pwmLeftOut) {
    pwmLeftOut -= PWM_INCREMENT;
  }
  else{}
   
  if (abs(pwmRightReq) > pwmRightOut) {
    pwmRightOut += PWM_INCREMENT;
  }
  else if(abs(pwmRightReq) < pwmRightOut) {
    pwmRightOut -= PWM_INCREMENT;
  }
  else{}
 
  // Conditional operator to limit PWM output at the maximum 
  pwmLeftOut = (pwmLeftOut > PWM_MAX) ? PWM_MAX : pwmLeftOut;
  pwmRightOut = (pwmRightOut > PWM_MAX) ? PWM_MAX : pwmRightOut;
 
  // PWM output cannot be less than 0
  pwmLeftOut = (pwmLeftOut < 0) ? 0 : pwmLeftOut;
  pwmRightOut = (pwmRightOut < 0) ? 0 : pwmRightOut;
 
  // Set the PWM value on the pins
 // analogWrite(enA, pwmLeftOut); 
 // analogWrite(enB, pwmRightOut); 

  // Set the direction of the motors
  if (pwmLeftReq > 0) { // Left wheel forward
    analogWrite(in1, pwmLeftOut);
    analogWrite(in2, 0);
  }
  else if (pwmLeftReq < 0) { // Left wheel reverse
    analogWrite(in1, 0);
    analogWrite(in2, pwmLeftOut);
  }
  else if (pwmLeftReq == 0 && pwmLeftOut == 0 ) { // Left wheel stop
    analogWrite(in1, 0);
    analogWrite(in2, 0);
  }
  else { // Left wheel stop
    analogWrite(in1, 0);
    analogWrite(in2, 0); 
  }
  
 
  if (pwmRightReq > 0) { // Right wheel forward
    analogWrite(in3, pwmRightOut);
    analogWrite(in4, 0);
  }
  else if(pwmRightReq < 0) { // Right wheel reverse
    analogWrite(in3, 0);
    analogWrite(in4, pwmRightOut);
  }
  else if (pwmRightReq == 0 && pwmRightOut == 0) { // Right wheel stop
    analogWrite(in3, 0);
    analogWrite(in4, 0);
  }
  else { // Right wheel stop
    analogWrite(in3, 0);
    analogWrite(in4, 0); 
  }

  
}
 
// Set up ROS subscriber to the velocity command
ros::Subscriber<geometry_msgs::Twist> subCmdVel("cmd_vel", &calc_pwm_values );
 
void setup() {
 
  // Set pin states of the encoder
  pinMode(ENC_IN_LEFT_A , INPUT);
  pinMode(ENC_IN_LEFT_B , INPUT);
  pinMode(ENC_IN_RIGHT_A , INPUT);
  pinMode(ENC_IN_RIGHT_B , INPUT);
 
  // Every time the pin goes high, this is a tick
  attachInterrupt(digitalPinToInterrupt(ENC_IN_LEFT_A), left_wheel_tickA, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENC_IN_RIGHT_A), right_wheel_tickA, CHANGE);

  attachInterrupt(digitalPinToInterrupt(ENC_IN_LEFT_B), left_wheel_tickB, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENC_IN_RIGHT_B), right_wheel_tickB, CHANGE);
   
  // Motor control pins are outputs
  
  pinMode(in1, OUTPUT);
  pinMode(in2, OUTPUT);
  pinMode(in3, OUTPUT);
  pinMode(in4, OUTPUT);
  
  // Turn off motors - Initial state
  analogWrite(in1, 0);
  analogWrite(in2, 0);
  analogWrite(in3, 0);
  analogWrite(in4, 0);
  
 

 // right_joint_state.data = (float *)malloc(sizeof(float)*2);
 // right_joint_state.data_length = 2;

 // left_joint_state.data = (float *)malloc(sizeof(float)*2);
 // left_joint_state.data_length = 2;
 
  // ROS Setup
 // ROS Setup
  nh.getHardware()->setBaud(115200);
  nh.initNode();
  nh.advertise(rightPub);
  nh.advertise(leftPub);
  nh.subscribe(subCmdVel);
     
}
void loop() {
   
  nh.spinOnce();
   
  // Record the time
  currentMillis = millis();
 
  // If the time interval has passed, publish the number of ticks,
  // and calculate the velocities.
  if (currentMillis - previousMillis > interval) {
     
    previousMillis = currentMillis;
 
    // Publish tick counts to topics
    leftPub.publish( &left_wheel_tick_count );
    rightPub.publish( &right_wheel_tick_count );
 
    // Calculate the velocity of the right and left wheels
    calc_vel_right_wheel();
    calc_vel_left_wheel();
     
  }
   
  // Stop the car if there are no cmd_vel messages
  if((millis()/1000) - lastCmdVelReceived > 1) {
    pwmLeftReq = 0;
    pwmRightReq = 0;
  }
 
  set_pwm_values();
}
