/*this code gives us the encoder tick values for all 4 encoders 
 without giving any pwm*/
int rightRPWM = 5;  // Digital/PWM pin 5 to the RPWM on the BTS7960
int rightLPWM = 6;  // Digital/PWM pin 6 to the LPWM on the BTS7960

int leftRPWM = 10;  // Digital/PWM pin 10 to the RPWM on the BTS7960
int leftLPWM = 11;  // Digital/PWM pin 11 to the LPWM on the BTS7960


#define leftENCODER_A 18
#define leftENCODER_B 19


#define rightENCODER_A 2
#define rightENCODER_B 3

volatile int right_wheel_tick_count= 0;
volatile int left_wheel_tick_count = 0;


void setup() {

  Serial.begin(9600);
  
  pinMode(leftENCODER_A, INPUT_PULLUP);
  pinMode(leftENCODER_B, INPUT_PULLUP);
  pinMode(rightENCODER_A, INPUT_PULLUP);
  pinMode(rightENCODER_B, INPUT_PULLUP);

  // Attach interrupts to the encoder pins (rising edge)
  attachInterrupt(digitalPinToInterrupt(rightENCODER_A), doencoderA, CHANGE);
  attachInterrupt(digitalPinToInterrupt(rightENCODER_B), doencoderB, CHANGE);
  attachInterrupt(digitalPinToInterrupt(leftENCODER_A), doencoderC, CHANGE);
  attachInterrupt(digitalPinToInterrupt(leftENCODER_B), doencoderD, CHANGE);
}

void loop()
{
  Serial.print(right_wheel_tick_count);
  Serial.print(" , ");
  Serial.println(left_wheel_tick_count);
  delay(10);
}
void doencoderA()
{
 if (digitalRead(rightENCODER_A) == HIGH)
 {
  if(digitalRead(rightENCODER_B) == LOW)
  {
    right_wheel_tick_count++;
  }
  else
  {
    right_wheel_tick_count--;
  }
 }
 else
 {
  if(digitalRead(rightENCODER_B) == HIGH)
  {
    right_wheel_tick_count++;
  }
  else
  {
    right_wheel_tick_count--;
  }
 }
}

void doencoderB()
{
 if (digitalRead(rightENCODER_B) == HIGH)
 {
  if(digitalRead(rightENCODER_A) == HIGH)
  {
    right_wheel_tick_count++;
  }
  else
  {
    right_wheel_tick_count--;
  }
 }
 else
 {
  if(digitalRead(rightENCODER_A) == LOW)
  {
    right_wheel_tick_count++;
  }
  else
  {
    right_wheel_tick_count--;
  }
 }
}

void doencoderC()
{
 if (digitalRead(leftENCODER_A) == HIGH)
 {
  if(digitalRead(leftENCODER_B) == LOW)
  {
    left_wheel_tick_count--;
  }
  else
  {
    left_wheel_tick_count++;
  }
 }
 else
 {
  if(digitalRead(leftENCODER_B) == HIGH)
  {
    left_wheel_tick_count--;
  }
  else
  {
    left_wheel_tick_count++;
  }
 }
}

void doencoderD()
{
 if (digitalRead(leftENCODER_B) == HIGH)
 {
  if(digitalRead(leftENCODER_A) == HIGH)
  {
    left_wheel_tick_count--;
  }
  else
  {
    left_wheel_tick_count++;
  }
 }
 else
 {
  if(digitalRead(leftENCODER_A) == LOW)
  {
    left_wheel_tick_count--;
  }
  else
  {
    left_wheel_tick_count++;
  }
 }
}
