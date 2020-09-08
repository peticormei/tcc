#include <AFMotor.h>

#define echoPin_L A0
#define triggerPin_L A1
#define echoPin_R A4
#define triggerPin_R A5

AF_DCMotor motor1(1);
AF_DCMotor motor2(2);
AF_DCMotor motor3(3);
AF_DCMotor motor4(4);

const byte MOTOR_2 = 18;
const byte MOTOR_3 = 19;

const byte BUMPER_BTN_L = 20;
const byte BUMPER_BTN_R = 21;

float duration, distance;

const float stepcount = 20.00;
const float wheeldiameter = 67.50;

volatile int counter_motor_2 = 0;
volatile int counter_motor_3 = 0;

volatile byte bumper_l = HIGH;
volatile byte bumper_r = HIGH;

void ISR_MOTOR_2() {
  counter_motor_2++;
}

void ISR_MOTOR_3() {
  counter_motor_3++;
}

void ISR_Bumper_L() {
  bumper_l = digitalRead(BUMPER_BTN_L);
}

void ISR_Bumper_R() {
  bumper_r = digitalRead(BUMPER_BTN_R);
}

float getDistance(int tPin, int ePin) { 
  digitalWrite(tPin, LOW);
  delayMicroseconds(2); 
  digitalWrite(tPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(tPin, LOW);

  uint32_t duration = pulseIn(ePin, HIGH);

  float distance = (duration / 2) / 29.1;

  return distance;
}

float sensorLimit(float distance) {
  if (distance >= 400.0 || distance <= 2.0) {
    return 0;
  }

  return distance;
}

void setup() {
  Serial.begin(9600);
  Serial.println("Motor test!");

  attachInterrupt(digitalPinToInterrupt (MOTOR_2), ISR_MOTOR_2, RISING);
  attachInterrupt(digitalPinToInterrupt (MOTOR_3), ISR_MOTOR_3, RISING);

  attachInterrupt(digitalPinToInterrupt (BUMPER_BTN_L), ISR_Bumper_L, CHANGE);
  attachInterrupt(digitalPinToInterrupt (BUMPER_BTN_R), ISR_Bumper_R, CHANGE);

  motor1.setSpeed(255);
  motor2.setSpeed(255);
  motor3.setSpeed(255);
  motor4.setSpeed(255);

  pinMode(BUMPER_BTN_L, INPUT);
  pinMode(BUMPER_BTN_R, INPUT);

  pinMode(echoPin_L, INPUT);
  pinMode(triggerPin_L, OUTPUT);

  pinMode(echoPin_R, INPUT);
  pinMode(triggerPin_R, OUTPUT);
}

void loop() {
  int distance_r = getDistance(triggerPin_R, echoPin_R);
  int distance_l = getDistance(triggerPin_L, echoPin_L);
  
  if (bumper_l == LOW && bumper_r == LOW) {
    motor1.run(RELEASE);
    motor2.run(RELEASE);
    motor3.run(RELEASE);
    motor4.run(RELEASE);

    counter_motor_2 = 0;
    counter_motor_3 = 0;
    
    Serial.print("Stop! ");

 } else if (bumper_l == HIGH && bumper_r == LOW) {
    motor1.run(BACKWARD);
    motor2.run(BACKWARD);
    motor3.run(FORWARD);
    motor4.run(FORWARD);

    counter_motor_2 = 0;
    counter_motor_3 = 0;

    Serial.print("Turn left! ");
   
 } else if (bumper_l == LOW && bumper_r == HIGH) {
    motor1.run(FORWARD);
    motor2.run(FORWARD);
    motor3.run(BACKWARD);
    motor4.run(BACKWARD);

    counter_motor_2 = 0;
    counter_motor_3 = 0;

    Serial.print("Turn right! ");
 } else {
    motor1.run(FORWARD);
    motor2.run(FORWARD);
    motor3.run(FORWARD);
    motor4.run(FORWARD);

    Serial.print("Forward! ");
  }

  Serial.print("Motor Left: ");
  Serial.print(counter_motor_2);
  Serial.print(" Motor Right: ");
  Serial.print(counter_motor_3);
  Serial.print(" HC-SR04 Rear: ");
  Serial.print(distance_l);
  Serial.print(" HC-SR04 Front: ");
  Serial.println(distance_r);
}
