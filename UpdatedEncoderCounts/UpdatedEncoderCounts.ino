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

volatile int right_wheel_tick_count = 0;
volatile int left_wheel_tick_count = 0;

void setup() {

  Serial.begin(9600);
  
  pinMode(leftENCODER_A, INPUT_PULLUP);
  pinMode(leftENCODER_B, INPUT_PULLUP);
  pinMode(rightENCODER_A, INPUT_PULLUP);
  pinMode(rightENCODER_B, INPUT_PULLUP);

  // Attach interrupts to the encoder pins (rising edge)
  attachInterrupt(digitalPinToInterrupt(rightENCODER_A), right_wheel_tick, RISING);
  attachInterrupt(digitalPinToInterrupt(leftENCODER_A), left_wheel_tick, RISING);

}

void loop() 
{
  Serial.print("right pos: ");
  Serial.print(right_wheel_tick_count);
  Serial.print(",");
  Serial.print("left pos: ");
  Serial.println(left_wheel_tick_count);
  delay(10);
 
}

// Adjust the interrupt service routines for quadrature encoders
void right_wheel_tick() {
  static int prevState = 0;
  int newState = digitalRead(rightENCODER_A) << 1 | digitalRead(rightENCODER_B);
  if ((prevState == 0b00 && newState == 0b01) || (prevState == 0b11 && newState == 0b10)) {
    right_wheel_tick_count++;
  } else if ((prevState == 0b01 && newState == 0b00) || (prevState == 0b10 && newState == 0b11)) {
    right_wheel_tick_count--;
  }
  prevState = newState;
}

void left_wheel_tick() {
  static int prevState = 0;
  int newState = digitalRead(leftENCODER_A) << 1 | digitalRead(leftENCODER_B);
  if ((prevState == 0b00 && newState == 0b01) || (prevState == 0b11 && newState == 0b10)) {
    left_wheel_tick_count++;
  } else if ((prevState == 0b01 && newState == 0b00) || (prevState == 0b10 && newState == 0b11)) {
    left_wheel_tick_count--;
  }
  prevState = newState;
}

