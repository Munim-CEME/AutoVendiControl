int RPWM = 5;  // Digital/PWM pin 5 to the RPWM on the BTS7960
int LPWM = 6;  // Digital/PWM pin 6 to the LPWM on the BTS7960

int RPWM1 = 9;  // Digital/PWM pin 5 to the RPWM on the BTS7960
int LPWM2 = 10;  // Digital/PWM pin 6 to the LPWM on the BTS7960

// // Enable "Left" and "Right" movement
// int L_EN = 7;  // connect Digital/PWM pin 7 to L_EN on the BTS7960
// int R_EN = 8;  // connect Digital/PWM pin 8 to R_EN on the BTS7960

#define ENCODER_A 2
#define ENCODER_B 3

volatile int pulseCountA = 0;
volatile int pulseCountB = 0;


void setup() {

  Serial.begin(9600);
  
  //enable "Right" and "Left" movement on the HBridge
  // Notice use of digitalWrite to simply turn it on and keep it on.
  // digitalWrite(R_EN, HIGH);  
  // digitalWrite(L_EN, HIGH);

    pinMode(ENCODER_A, INPUT_PULLUP);
  pinMode(ENCODER_B, INPUT_PULLUP);

  // Attach interrupts to the encoder pins (rising edge)
  attachInterrupt(digitalPinToInterrupt(ENCODER_A), countPulseA, RISING);
  attachInterrupt(digitalPinToInterrupt(ENCODER_B), countPulseB, RISING);
}



void loop() {
  // put your main code here, to run repeatedly:

  // Use an analogWrite(pin,  which tells it to send a modulated
  // signal (PWM) to specific pin at a specific "duty cycle".
  // Valid values are 0 to 255.  0 means always off(or no power) and 255 means always on(full power)
  // https://www.arduino.cc/reference/en/language/functions/analog-io/analogwrite/
  // analogWrite(LPWM, 50);
  // analogWrite(RPWM1,50 );
   // pulse a signal continually at the rate of 64 
  // // the analogWrite line above should start the motor turning in one direction at about 1/4 of power.
  // delay(5000); // wait 5 seconds, motor continues to move because the analogWrite is still pulsing
  // analogWrite(RPWM, 128); // pulse signal now at 128 (about half power... half of max 255).
  // delay(5000);

  // // after 5 seconds at half power, stop the motor moving
  // analogWrite(RPWM, 0);
  // delay(5000);
  // // now start moving in opposite direction.
  // analogWrite(LPWM, 64);
  // delay(5000);
  // analogWrite(LPWM, 128);
  // delay(5000);
  // analogWrite(LPWM, 0); // Stop moving in this direction
  // // at this point should be no movement.
  
  // delay(1000);
    Serial.print("Pulse Count L: ");
  Serial.print(pulseCountA);
  Serial.print(",");
  Serial.print("Pulse Count R: ");
  Serial.println(pulseCountB);

  // Add any additional code for your application
  delay(10);
}


void countPulseA() {
  pulseCountA++;
}

// Interrupt service routine for encoder B
void countPulseB() {
  pulseCountB++;
}
