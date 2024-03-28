// #define A A8
// #define B A9
// int a = 0;
// int b = 0;
// void setup() {
//   // put your setup code here, to run once:
//   Serial.begin(9600);
//   pinMode(A, INPUT_PULLUP);
//   pinMode(B, INPUT_PULLUP);

// }

// void loop() {
//   // put your main code here, to run repeatedly:
//   a = digitalRead(A);
//   b = digitalRead(B);
//   // Serial.println(a);
//   Serial.println(b);
//   // delay(100);


// }
// #define A A8
// #define B A9

// int a = 0;
// int b = 0;
// int pulseCount = 0;
// int lastStateA = 0;

// void setup() {
//   Serial.begin(9600);
//   pinMode(A, INPUT_PULLUP);
//   pinMode(B, INPUT_PULLUP);
// }

// void loop() {
//   a = digitalRead(A);

//   // Check for rising edge
//   if (a == HIGH && lastStateA == LOW) {
//     pulseCount++;
//     Serial.println(pulseCount);
//   }

//   lastStateA = a;

//   // You can also print the raw value if needed
//   // Serial.println(a);

//   // Add a delay to avoid printing too fast
//   delay(10);
// }
#define ENCODER_A 2
#define ENCODER_B 3

// Variables for pulse counting
volatile int pulseCountA = 0;
volatile int pulseCountB = 0;

void setup() {
  Serial.begin(9600);

  // Set encoder pins as inputs with pull-up resistors
  pinMode(ENCODER_A, INPUT_PULLUP);
  pinMode(ENCODER_B, INPUT_PULLUP);

  // Attach interrupts to the encoder pins (rising edge)
  attachInterrupt(digitalPinToInterrupt(ENCODER_A), countPulseA, RISING);
  attachInterrupt(digitalPinToInterrupt(ENCODER_B), countPulseB, RISING);
}

void loop() {
  // Your main code goes here
  // You can use pulseCountA and pulseCountB for pulse counts
  Serial.print("Pulse Count A: ");
  Serial.println(pulseCountA);
  Serial.print("Pulse Count B: ");
  Serial.println(pulseCountB);

  // Add any additional code for your application
  delay(10);  // Add a delay to avoid printing too fast
}

// Interrupt service routine for encoder A
void countPulseA() {
  pulseCountA++;
}

// Interrupt service routine for encoder B
void countPulseB() {
  pulseCountB++;
}
