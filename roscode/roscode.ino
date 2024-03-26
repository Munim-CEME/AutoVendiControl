 
#include <ros.h>
#include <std_msgs/Int16.h>
#include <geometry_msgs/Twist.h> 
// Handles startup and shutdown of ROS
ros::NodeHandle nh;
//right mototr pins
int rightRPWM = 5;  // Digital/PWM pin 5 to the RPWM on the BTS7960
int rightLPWM = 6;  // Digital/PWM pin 6 to the LPWM on the BTS7960

//left motor pins
int leftRPWM = 10;  // Digital/PWM pin 10 to the RPWM on the BTS7960
int leftLPWM = 11;  // Digital/PWM pin 11 to the LPWM on the BTS7960

//right encoder pins
#define rightENCODER_A 2 // Encoder channel A connected to digital pin 2
#define rightENCODER_B 3 // Encoder channel B connected to digital pin 3

//left encoder pins
#define leftENCODER_A  18 // Encoder channel A connected to digital pin 18
#define leftENCODER_B  19 // Encoder channel B connected to digital pin 19

//variables
bool Direction_left = true;
bool Direction_right = true;

volatile int pulseCountA = 0;
volatile int pulseCountB = 0;

// Keep track of the number of wheel ticks
std_msgs::Int16 right_wheel_tick_count;
ros::Publisher rightPub("right_ticks", &rightencoderPos);
 
std_msgs::Int16 left_wheel_tick_count;
ros::Publisher leftPub("left_ticks", &leftencoderPos);


ros::Subscriber<geometry_msgs::Twist> subCmdVel("cmd_vel", &calc_pwm_values );

void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
  //motor outputs
  pinMode(5, OUTPUT);
  pinMode(6, OUTPUT);
  pinMode(10, OUTPUT);
  pinMode(11, OUTPUT);

  //encoder pullups
  pinMode(rightENCODER_A, INPUT_PULLUP);
  pinMode(rightENCODER_B, INPUT_PULLUP);
  pinMode(leftENCODER_A, INPUT_PULLUP);
  pinMode(leftENCODER_B , INPUT_PULLUP);

  //interrupt attachement
  attachInterrupt(digitalPinToInterrupt(rightENCODER_A), countPulseA, CHANGE);
  attachInterrupt(digitalPinToInterrupt(leftENCODER_A), countPulseB, CHANGE);
  // ROS Setup
  nh.getHardware()->setBaud(115200);
  nh.initNode();
  nh.advertise(rightPub);
  nh.advertise(leftPub);
  nh.subscribe(subCmdVel);
}

void loop() {
  // put your main code here, to run repeatedly:
 nh.spinOnce();
   
  // Record the time
  currentMillis = millis();
 
  // If the time interval has passed, publish the number of ticks,
  // and calculate the velocities.
  if (currentMillis - previousMillis > interval) {
     
    previousMillis = currentMillis;
 
    // Publish tick counts to topics
    leftPub.publish( &pulseCountA);
    rightPub.publish( &pulseCountB);
        
  }
}

void twistcallback(const geometry_msgs::Twist& cmdVel)
{
  float linear_x= cmdVel.linear.x;
  float angular_z= cmd.angular.z;
  Serial.print("Linear Velocity (x): ");
  Serial.print(linear_x);
  Serial.print(", Angular Velocity (z): ");
  Serial.println(angular_z);
}
void countPulseA() {
  pulseCountA.data++;
}
void countPulseB() {
  pulseCountB.data++;
}
