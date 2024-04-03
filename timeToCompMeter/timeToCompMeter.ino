/*
  this code will give us time taken by given pwm to complete 1 meter
  distance. it takes the total ticks match the encoder tcks and stop.
  we have taken 10 readings from 65 to 110 to find PWM and vel rel.
*/

int rightRPWM = 5;  // Digital/PWM pin 5 to the RPWM on the BTS7960
int rightLPWM = 6;  // Digital/PWM pin 6 to the LPWM on the BTS7960

int leftRPWM = 10;  // Digital/PWM pin 9 to the RPWM on the BTS7960
int leftLPWM = 11;  // Digital/PWM pin 10 to the LPWM on the BTS7960

// // Enable "Left" and "Right" movement
// #define rightENCODER_B 18
// #define leftENCODER_B 19  

#define rightENCODER_A 2
#define leftENCODER_A 18

//////ticks per meter determin experimentally
int totalticks1 = 387;
int totalticks2 = 369;

volatile int pulseCountA = 0;
volatile int pulseCountB = 0;

//////for calculating the time
unsigned long currentMillis;
unsigned long previousMillis;

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
  delay(5000);

}

void loop()
{
  //prints the encoder ticks 
  Serial.print(pulseCountA);
  Serial.print(",");
  Serial.println(pulseCountB);
  Serial.println(pulseCountA-pulseCountB);
  delay(10);
  
  //measure the time when it will complete the total ticks
//  currentMillis = millis();
//   if(currentMillis - previousMillis >=10)
//   {
//     previousMillis = currentMillis;
//     if (pulseCountA >= totalticks1 && pulseCountB >= totalticks2)
//     {
//       stop();
//     }
//     else
//     {
//       start();
//     }
//     Serial.println(previousMillis);
//   }
  start();
  /*if(Serial.available()>0)
  {
    char c= Serial.read();
    if(c = 'b')
    {
      stop();
    }
  }*/
}

//count encoder ticks of right motor
void countPulseA()
{
  pulseCountA++;
}
//count encoder ticks of left motor
void countPulseB()
{
  pulseCountB++;
}
//forward movement
void start()
{
   analogWrite(rightLPWM, 80);
   analogWrite(leftLPWM, 80);
   
}
//will stop 
void stop()
{
   analogWrite(rightLPWM, 0);
   analogWrite(leftLPWM, 0);
}
