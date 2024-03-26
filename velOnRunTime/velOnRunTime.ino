/*
  this code calculates the velocity of the robot in run time.
*/
int rightRPWM = 5;  // Digital/PWM pin 5 to the RPWM on the BTS7960
int rightLPWM = 6;  // Digital/PWM pin 6 to the LPWM on the BTS7960

int leftRPWM = 10;  // Digital/PWM pin 9 to the RPWM on the BTS7960
int leftLPWM = 11;  // Digital/PWM pin 10 to the LPWM on the BTS7960

// // Enable "Left" and "Right" movement
// int L_EN = 7;  // connect Digital/PWM pin 7 to L_EN on the BTS7960
// int R_EN = 8;  // connect Digital/PWM pin 8 to R_EN on the BTS7960

#define rightENCODER_A 2
#define leftENCODER_A 3

//////ticks per meter determin experimentally
int ticksA = 387;
int ticksB = 369;

volatile int pulseCountA = 0;
volatile int pulseCountB = 0;

float velA = 0;
float velB = 0;

//////for calculating the time
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
  currentMillis = millis();
  lap_Time= currentMillis - previousMillis;
  if(lap_Time >=1000)
  {
    start();
    velA = cal_vel_A(pulseCountA, lap_Time);
    velB = cal_vel_B(pulseCountB, lap_Time);

    Serial.print(velA);
    Serial.println(velB);

    pulseCountA = 0;
    pulseCountB = 0;

    previousMillis = currentMillis;
  }
 
}
//count encoder ticks of motorA
void countPulseA() 
{
  pulseCountA++;
}
void countPulseB() 
{
  pulseCountB++;
}
//forward movement
void start()
{
   analogWrite(rightLPWM, 70);
   analogWrite(leftLPWM, 70); 
}
//will stop 
void stop()
{
   analogWrite(rightLPWM, 0);
   analogWrite(leftLPWM, 0);
}
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
