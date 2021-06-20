#include <AFMotor.h>

AF_DCMotor motor1(1);
AF_DCMotor motor2(2);
AF_DCMotor motor3(3);
AF_DCMotor motor4(4);

const byte MOTOR_2 = 18;
const byte MOTOR_3 = 19;

const byte BUMPER_BTN_L = 20;
const byte BUMPER_BTN_R = 21;

//const float stepcount = 20.00;
//const float wheeldiameter = 67.50;

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

void setup() {
  Serial.begin(9600);
  Serial.println("Motor test!");

  attachInterrupt(digitalPinToInterrupt (MOTOR_2), ISR_MOTOR_2, RISING);
  attachInterrupt(digitalPinToInterrupt (MOTOR_3), ISR_MOTOR_3, RISING);

  attachInterrupt(digitalPinToInterrupt (BUMPER_BTN_L), ISR_Bumper_L, CHANGE);
  attachInterrupt(digitalPinToInterrupt (BUMPER_BTN_R), ISR_Bumper_R, CHANGE);
}

void loop() {
  motor4.run(FORWARD);
  motor1.run(FORWARD);
  motor2.run(FORWARD);
  motor3.run(BACKWARD);

  motor1.setSpeed(255);
  motor4.setSpeed(255);
  motor2.setSpeed(245);
  motor3.setSpeed(245);

  Serial.print("Motor Left: ");
  Serial.print(counter_motor_2);
  Serial.print(" Motor Right: ");
  Serial.print(counter_motor_3);
  Serial.print(" Bumper Left: ");
  Serial.print(bumper_l);
  Serial.print(" Bumper Right: ");
  Serial.println(bumper_r);
}
