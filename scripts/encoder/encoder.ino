#include <AFMotor.h>

#define echoPin_R A4
#define triggerPin_R A5

AF_DCMotor motor3(3);

const byte MOTOR_3 = 19;

volatile int counter_motor_3 = 0;

void ISR_MOTOR_3() {
  counter_motor_3++;
}

void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
  Serial.println("Motor test!");

  motor3.setSpeed(150);
  
  attachInterrupt(digitalPinToInterrupt (MOTOR_3), ISR_MOTOR_3, RISING);
}

void loop() {
  // put your main code here, to run repeatedly:
  motor3.run(FORWARD);
  Serial.println(counter_motor_3);
  
}
