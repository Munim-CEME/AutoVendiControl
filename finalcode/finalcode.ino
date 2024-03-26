/*FINAL CODE*/

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

int pwmRightReq = 0;
int pwmLeftReq = 0;

volatile int rightencoderPos = 0;
volatile int leftencoderPos= 0;
volatile int lastEncoded= 0;

float velA = 0;
float velB = 0;
float cmd_vel= 0;

unsigned long currentMillis;
unsigned long previousMillis;
unsigned long lap_Time;

//data calculated from experimentation
int ticksA = 387;
int ticksB = 369;

float kp = 836.5;
float b = 49.95;

int PWM_MIN = 65;
int PWM_MAX = 110;

const int PWM_INCREMENT = 1;

const double WHEEL_RADIUS = 0.15; 
const double WHEEL_BASE = 0.558; 

void setup()
{
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
 // attachInterrupt(digitalPinToInterrupt(rightENCODER_A), rightencoderISR, CHANGE);
 // attachInterrupt(digitalPinToInterrupt(leftENCODER_A), leftencoderISR, CHANGE);
}
void loop()
{
  velA = cal_vel_A(rightencoderPos, lap_Time);
  velB = cal_vel_B(leftencoderPos, lap_Time);

  Serial.print(velA);
  Serial.println(velB);

  calc_pwm_values(cmd_vel);
  print();
  start();
    
}
///////////////////Function Definition///////////////////////////////
/////////////////////////ADD ENCODER DIRECTION PLUS TICKS CODE////////////////////
//vel of robot on run time
//calculate vel for right motor
float cal_vel_A(volatile int ticks, unsigned long milliseconds)
{
  float distance = ticks/ float(ticksA);
  float seconds = milliseconds/ 1000.0;

  float velocityA = distance/seconds;
  return velocityA;
}
//cal vel for left motor
float cal_vel_B(volatile int ticks, unsigned long milliseconds)
{
  float distance = ticks/ float(ticksB);
  float seconds = milliseconds/ 1000.0;

  float velocityB = distance/seconds;
  return velocityB;
}
//cal pwm
// Calculate the PWM value given the desired velocity
int calc_pwm_values(float vel) 
{
  pwmLeftReq =int( kp * vel - b);
  pwmRightReq = int(kp * vel - b);
 
  // Handle low PWM values
  if (abs(pwmLeftReq) < PWM_MIN) {
    pwmLeftReq = 0;
  }
  if (abs(pwmRightReq) < PWM_MIN) {
    pwmRightReq = 0;  
  }  
  return pwmLeftReq,pwmRightReq;
}
//make the robot move
void start()
{
   analogWrite(rightLPWM, pwmLeftReq);
   analogWrite(leftLPWM, pwmRightReq);
}
//print pwms
void print()
{
  Serial.print(pwmLeftReq);
  Serial.print(" , ");
  Serial.print(pwmRightReq);
}
//set pwm
