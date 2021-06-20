#include <AFMotor.h>

#define echoPin_L A0
#define triggerPin_L A1

#define echoPin_F A2
#define triggerPin_F A3

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

byte old_state_2 = LOW;
  
void ISR_MOTOR_2() {
  byte encoder_state = digitalRead(MOTOR_2);
  if (encoder_state == HIGH && old_state_2 != encoder_state) {
    counter_motor_2++;
  }
  old_state_2 = encoder_state;
}

byte old_state_3 = LOW;

void ISR_MOTOR_3() {
  byte encoder_state = digitalRead(MOTOR_3);
  if (encoder_state == HIGH && old_state_3 != encoder_state) {
    counter_motor_3++;
  }
  old_state_3 = encoder_state;
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

  attachInterrupt(digitalPinToInterrupt (MOTOR_2), ISR_MOTOR_2, CHANGE);
  attachInterrupt(digitalPinToInterrupt (MOTOR_3), ISR_MOTOR_3, CHANGE);

  attachInterrupt(digitalPinToInterrupt (BUMPER_BTN_L), ISR_Bumper_L, CHANGE);
  attachInterrupt(digitalPinToInterrupt (BUMPER_BTN_R), ISR_Bumper_R, CHANGE);

  pinMode(BUMPER_BTN_L, INPUT);
  pinMode(BUMPER_BTN_R, INPUT);

  pinMode(echoPin_L, INPUT);
  pinMode(triggerPin_L, OUTPUT);

  pinMode(echoPin_R, INPUT);
  pinMode(triggerPin_R, OUTPUT);
}

void loop() {
  int distance_f = getDistance(triggerPin_F, echoPin_F);
  int distance_r = getDistance(triggerPin_R, echoPin_R);
  int distance_l = getDistance(triggerPin_L, echoPin_L);

  Serial.print("Motor Left: ");
  Serial.print(counter_motor_2);
  Serial.print(" Motor Right: ");
  Serial.print(counter_motor_3);
  
  Serial.print(" Bumper Left: ");
  Serial.print(bumper_l);
  Serial.print(" Bumper Right: ");
  Serial.print(bumper_r);

  Serial.print(" HC-SR04 Left: ");
  Serial.print(distance_l);
  Serial.print(" HC-SR04 Right: ");
  Serial.print(distance_r);
  Serial.print(" HC-SR04 Front: ");
  Serial.println(distance_f);
}
