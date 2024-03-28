/*In this code we are getting pwm values according to the velocity of the robot.
  this code can print velocity , PWM and encoder ticks*/

int rightRPWM = 5;  // Digital/PWM pin 5 to the RPWM on the BTS7960
int rightLPWM = 6;  // Digital/PWM pin 6 to the LPWM on the BTS7960

int leftRPWM = 10;  // Digital/PWM pin 9 to the RPWM on the BTS7960
int leftLPWM = 11;  // Digital/PWM pin 10 to the LPWM on the BTS7960

// // Enable "Left" and "Right" movement
// #define rightENCODER_B = 18 // connect Digital/PWM pin 7 to L_EN on the BTS7960
// #define leftENCODER_B = 19  // connect Digital/PWM pin 8 to R_EN on the BTS7960

#define rightENCODER_A 2
#define leftENCODER_A 3

//////ticks per meter determin experimentally
int ticksA = 387;
int ticksB = 369;

int pwmRightReq = 0;
int pwmLeftReq = 0;

volatile int pulseCountA = 0;
volatile int pulseCountB = 0;

float velA = 0;
float velB = 0;

//m and b values for pwm and velocity relation using linear regression
float kp = 836.5;
float b = 49.95;

//pwm limit determine experimentally
int PWM_MIN = 65;

//for calculating the time
unsigned long currentMillis;
unsigned long previousMillis;
unsigned long lap_Time;

void setup() {

  Serial.begin(9600);
  
  pinMode(5, OUTPUT);
  pinMode(6, OUTPUT);
  pinMode(10, OUTPUT);
  pinMode(11, OUTPUT);
  
  pinMode(rightENCODER_A, INPUT_PULLUP);
  pinMode(leftENCODER_A, INPUT_PULLUP);

  // Attach interrupts to the encoder pins (rising edge)
  attachInterrupt(digitalPinToInterrupt(rightENCODER_A), countPulseA, RISING);
  attachInterrupt(digitalPinToInterrupt(leftENCODER_A), countPulseB, RISING);
  delay(3000);
}
void loop()
{

  if (Serial.available()>0)
  {
    Serial.print("Pulse Count L1: ");
    Serial.print(pulseCountA);
    Serial.print(",");
    Serial.print("Pulse Count R1: ");
    Serial.println(pulseCountB);
    float  a = Serial.parseFloat();
    calc_pwm_values(a);
    print();
    start();
    delay(5000);
    stop();
    delay(3000);
  }
}
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
//will stop 
void stop()
{
   analogWrite(rightLPWM, 0);
   analogWrite(leftLPWM, 0);
}
//count ticks of encoder A right motor
void countPulseA() {
  pulseCountA++;
}
//count ticks of encoder B left motor
void countPulseB() {
  pulseCountB++;
}
