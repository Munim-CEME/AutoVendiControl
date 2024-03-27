/*this code gives us the encoder tick values for all 4 encoders 
 without giving any pwm*/
int rightRPWM = 5;  // Digital/PWM pin 5 to the RPWM on the BTS7960
int rightLPWM = 6;  // Digital/PWM pin 6 to the LPWM on the BTS7960

int leftRPWM = 10;  // Digital/PWM pin 10 to the RPWM on the BTS7960
int leftLPWM = 11;  // Digital/PWM pin 11 to the LPWM on the BTS7960


#define leftENCODER_A 2
#define leftENCODER_B 3


#define rightENCODER_A 18
#define rightENCODER_B 19

volatile int pulseCountA = 0;
volatile int pulseCountB = 0;
volatile int pulseCountC = 0;
volatile int pulseCountD = 0;

//values found by moving bot 1meter 10 times
int ticksA = 387; //right encoder
int ticksB = 369; //left encoder


void setup() {

  Serial.begin(9600);
  
  pinMode(leftENCODER_A, INPUT_PULLUP);
  pinMode(leftENCODER_B, INPUT_PULLUP);
  pinMode(rightENCODER_A, INPUT_PULLUP);
  pinMode(rightENCODER_B, INPUT_PULLUP);

  // Attach interrupts to the encoder pins (rising edge)
  attachInterrupt(digitalPinToInterrupt(rightENCODER_A), countPulseA, RISING);
  attachInterrupt(digitalPinToInterrupt(rightENCODER_B), countPulseB, RISING);
  attachInterrupt(digitalPinToInterrupt(leftENCODER_A), countPulseC, RISING);
  attachInterrupt(digitalPinToInterrupt(leftENCODER_B), countPulseD, RISING);
}

void loop() 
{
  Serial.print("Pulse Count L1: ");
  Serial.print(pulseCountA);
  Serial.print(",");
  Serial.print("Pulse Count R1: ");
  Serial.print(pulseCountB);
  Serial.print("Pulse Count L2: ");
  Serial.print(pulseCountC);
  Serial.print(",");
  Serial.print("Pulse Count R2: ");
  Serial.println(pulseCountD);
  delay(10);
 
}
void countPulseA() {
  pulseCountA++;
}
void countPulseB() {
  pulseCountB++;
}
void countPulseC() {
  pulseCountC++;
}
void countPulseD() {
  pulseCountD++;
}
