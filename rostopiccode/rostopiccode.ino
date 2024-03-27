#include <ros.h>
#include <std_msgs/Int16.h>
#include <geometry_msgs/Twist.h> 
// Handles startup and shutdown of ROS
ros::NodeHandle nh;
 
////////////////// Tick Data Publishing Variables and Constants ///////////////
 
// Encoder output to Arduino Interrupt pin. Tracks the tick count.
#define leftENCODER_A 18
#define leftENCODER_B 19


#define rightENCODER_A 2
#define rightENCODER_B 3
 
// Other encoder output to Arduino to keep track of wheel direction
// Tracks the direction of rotation.


 
// True = Forward; False = Reverse
boolean Direction_left = true;
boolean Direction_right = true;
 
// Minumum and maximum values for 16-bit integers
// Range of 65,535
const int encoder_minimum = -32768;
const int encoder_maximum = 32767;
 
// Keep track of the number of wheel ticks
std_msgs::Int16 right_wheel_tick_count;
ros::Publisher rightPub("right_ticks", &right_wheel_tick_count);
 
std_msgs::Int16 left_wheel_tick_count;
ros::Publisher leftPub("left_ticks", &left_wheel_tick_count);
 
// Time interval for measurements in milliseconds
const int interval = 30;
long previousMillis = 0;
long currentMillis = 0;
 
////////////////// Motor Controller Variables and Constants ///////////////////
 
// Motor A connections // TODO: Change BTS7960 convention with sense of L or R motor
// const int enA = 9;
// const int in1 = 5;
// const int in2 = 6;
int rightRPWM = 5;  
int rightLPWM = 6;
int rightRPWM_EN = 7;  
int rightLPWM_EN = 8;
int EN_right = rightLPWM; // integer for subsituting rpwm and lpwm
  
// Motor B connections //TODO: Change BTS7960 convention with sense of L or R motor
// const int enB = 10; 
// const int in3 = 9;
// const int in4 = 10;
int leftRPWM = 10;  
int leftLPWM = 11;
int leftRPWM_EN = 7;  
int leftLPWM_EN = 8;
int EN_left = leftLPWM;
 
// How much the PWM value can change each cycle
const int PWM_INCREMENT = 1;
 
// Number of ticks per wheel revolution. We won't use this in this code.
const int TICKS_PER_REVOLUTION = 620;
 
// Wheel radius in meters
const double WHEEL_RADIUS = 0.15; //Done todo:  Measure Wheel Radius 15cm
 
// Distance from center of the left tire to the center of the right tire in m
const double WHEEL_BASE = 0.558; //done todo: Measure wheel base
 
// Number of ticks a wheel makes moving a linear distance of 1 meter
// This value was measured manually.
const double TICKS_PER_METER_A = 387; // done today
const double TICKS_PER_METER_B = 369;

// Proportional constant, which was measured by measuring the 
// PWM-Linear Velocity relationship for the robot.
const int K_P = 836.5; //done TODO: CHange to new value
 
// Y-intercept for the PWM-Linear Velocity relationship for the robot
const int b = -49.95;  //done TODO: CHange to new value
 
// Correction multiplier for drift. Chosen through experimentation.
const int DRIFT_MULTIPLIER = 120; //TODO: CHange to new value
 
// Turning PWM output (0 = min, 255 = max for PWM values)
const int PWM_TURN = 80; //TODO: CHange to new value
 
// Set maximum and minimum limits for the PWM values
const int PWM_MIN = 60; // about 0.1 m/s //done TODO: CHange to new value
const int PWM_MAX = 110; // about 0.172 m/s //done TODO: CHange to new value
 
// Set linear velocity and PWM variable values for each wheel
double velLeftWheel = 0;
double velRightWheel = 0;
double pwmLeftReq = 0;
double pwmRightReq = 0;
 
// Record the time that the last velocity command was received
double lastCmdVelReceived = 0;
void calc_pwm_values(const geometry_msgs::Twist& cmdVel);
 
ros::Subscriber<geometry_msgs::Twist> subCmdVel("cmd_vel", &calc_pwm_values );
/////////////////////// Tick Data Publishing Functions ////////////////////////
 
// Increment the number of ticks


/////////////////////// Motor Controller Functions ////////////////////////////
 void doencoderA()
{
 if (digitalRead(rightENCODER_A) == HIGH)
 {
  if(digitalRead(rightENCODER_B) == LOW)
  {
    right_wheel_tick_count.data++;
  }
  else
  {
    right_wheel_tick_count.data--;
  }
 }
 else
 {
  if(digitalRead(rightENCODER_B) == HIGH)
  {
    right_wheel_tick_count.data++;
  }
  else
  {
    right_wheel_tick_count.data--;
  }
 }
}

void doencoderB()
{
 if (digitalRead(rightENCODER_B) == HIGH)
 {
  if(digitalRead(rightENCODER_A) == HIGH)
  {
    right_wheel_tick_count.data++;
  }
  else
  {
    right_wheel_tick_count.data--;
  }
 }
 else
 {
  if(digitalRead(rightENCODER_A) == LOW)
  {
    right_wheel_tick_count.data++;
  }
  else
  {
    right_wheel_tick_count.data--;
  }
 }
}

void doencoderC()
{
 if (digitalRead(leftENCODER_A) == HIGH)
 {
  if(digitalRead(leftENCODER_B) == LOW)
  {
    left_wheel_tick_count.data--;
  }
  else
  {
    left_wheel_tick_count.data++;
  }
 }
 else
 {
  if(digitalRead(leftENCODER_B) == HIGH)
  {
    left_wheel_tick_count.data--;
  }
  else
  {
    left_wheel_tick_count.data++;
  }
 }
}

void doencoderD()
{
 if (digitalRead(leftENCODER_B) == HIGH)
 {
  if(digitalRead(leftENCODER_A) == HIGH)
  {
    left_wheel_tick_count.data--;
  }
  else
  {
    left_wheel_tick_count.data++;
  }
 }
 else
 {
  if(digitalRead(leftENCODER_A) == LOW)
  {
    left_wheel_tick_count.data--;
  }
  else
  {
    left_wheel_tick_count.data++;
  }
 }
}

// Calculate the left wheel linear velocity in m/s every time a 
// tick count message is rpublished on the /left_ticks topic. 
void calc_vel_left_wheel(){
   
  // Previous timestamp
  static double prevTime = 0;
   
  // Variable gets created and initialized the first time a function is called.
  static int prevLeftCount = 0;
 
  // Manage rollover and rollunder when we get outside the 16-bit integer range 
  int numOfTicks = (65535 + left_wheel_tick_count.data - prevLeftCount) % 65535;
 
  // If we have had a big jump, it means the tick count has rolled over.
  if (numOfTicks > 10000) {
        numOfTicks = 0 - (65535 - numOfTicks);
  }
 
  // Calculate wheel velocity in meters per second
  velLeftWheel = numOfTicks/TICKS_PER_METER_A/((millis()/1000)-prevTime);
 
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
  velRightWheel = numOfTicks/TICKS_PER_METER_B/((millis()/1000)-prevTime);
 
  prevRightCount = right_wheel_tick_count.data;
   
  prevTime = (millis()/1000);
 
}
 
// Take the velocity command as input and calculate the PWM values.
void calc_pwm_values(const geometry_msgs::Twist& cmdVel) {
   
  // Record timestamp of last velocity command received
  lastCmdVelReceived = (millis()/1000);
  float linear_x= cmdVel.linear.x;
  float angular_z= cmdVel.angular.z;
  Serial.print("Linear Velocity (x): ");
  Serial.print(linear_x);
  Serial.print(", Angular Velocity (z): ");
  Serial.println(angular_z);
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
 
  // Set the direction of the motors
  if (pwmLeftReq > 0) { // Left wheel forward
     digitalWrite(leftLPWM_EN, HIGH);
     digitalWrite(leftRPWM_EN, LOW);
     EN_left = leftLPWM;


  }
  else if (pwmLeftReq < 0) { // Left wheel reverse
    // digitalWrite(in1, LOW);
    // digitalWrite(in2, HIGH);
    digitalWrite(leftLPWM_EN, LOW);
    digitalWrite(leftRPWM_EN, HIGH);
    EN_left = leftRPWM;
  }
  else if (pwmLeftReq == 0 && pwmLeftOut == 0 ) { // Left wheel stop
    // digitalWrite(in1, LOW);
    // digitalWrite(in2, LOW);
    digitalWrite(leftLPWM_EN, LOW);
    digitalWrite(leftRPWM_EN, LOW);
  }
  else { // Left wheel stop
    // digitalWrite(in1, LOW);
    // digitalWrite(in2, LOW); 
    digitalWrite(leftLPWM_EN, LOW);
    digitalWrite(leftRPWM_EN, LOW);
  }
 
  if (pwmRightReq > 0) { // Right wheel forward
    // digitalWrite(in3, HIGH);
    // digitalWrite(in4, LOW);
     digitalWrite(rightLPWM_EN, HIGH);
     digitalWrite(rightRPWM_EN, LOW);
     EN_right = rightLPWM;
  }
  else if(pwmRightReq < 0) { // Right wheel reverse
    // digitalWrite(in3, LOW);
    // digitalWrite(in4, HIGH);
    digitalWrite(rightLPWM_EN, LOW);
    digitalWrite(rightRPWM_EN, HIGH);
    EN_right = rightRPWM;
  }
  else if (pwmRightReq == 0 && pwmRightOut == 0) { // Right wheel stop
    // digitalWrite(in3, LOW);
    // digitalWrite(in4, LOW);
    digitalWrite(rightLPWM_EN, LOW);
    digitalWrite(rightRPWM_EN, LOW);
  }
  else { // Right wheel stop
    // digitalWrite(in3, LOW);
    // digitalWrite(in4, LOW);
    digitalWrite(rightLPWM_EN, LOW);
    digitalWrite(rightRPWM_EN, LOW); 
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
  analogWrite(EN_left, pwmLeftOut); 
  analogWrite(EN_right, pwmRightOut); 
}
 
// Set up ROS subscriber to the velocity command

 
void setup() {
 
  Serial.begin(115200);
  // Set pin states of the encoder
  
  pinMode(leftENCODER_A, INPUT_PULLUP);
  pinMode(leftENCODER_B, INPUT_PULLUP);
  pinMode(rightENCODER_A, INPUT_PULLUP);
  pinMode(rightENCODER_B, INPUT_PULLUP);
  // Every time the pin goes high, this is a tick
  
  // Attach interrupts to the encoder pins (rising edge)
  attachInterrupt(2, doencoderA, CHANGE);
  attachInterrupt(3, doencoderB, CHANGE);
  attachInterrupt(18, doencoderC, CHANGE);
  attachInterrupt(19, doencoderD, CHANGE);
   
  // Motor control pins are outputs
  pinMode(leftLPWM, OUTPUT);
  pinMode(leftRPWM, OUTPUT);
  pinMode(rightLPWM, OUTPUT);
  pinMode(rightRPWM, OUTPUT);
  pinMode(leftLPWM_EN, OUTPUT);
  pinMode(leftRPWM_EN, OUTPUT);
  pinMode(rightLPWM_EN, OUTPUT);
  pinMode(rightRPWM_EN, OUTPUT);
  
  // Turn off motors - Initial state
  digitalWrite(leftLPWM_EN, LOW);
  digitalWrite(leftRPWM_EN, LOW);
  digitalWrite(rightLPWM_EN, LOW);
  digitalWrite(rightRPWM_EN, LOW);
  
  // Set the motor speed
  analogWrite(EN_left, 0); 
  analogWrite(EN_right, 0);
 
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
