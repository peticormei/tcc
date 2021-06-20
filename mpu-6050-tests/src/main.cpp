#include <Arduino.h>

#include <I2Cdev.h>
#include <MPU6050_6Axis_MotionApps20.h>

#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
  #include "Wire.h"
#endif

#include <AFMotor.h>

#include <PID_v1.h>

AF_DCMotor motor_left(1);
AF_DCMotor motor_right(2);

const byte MOTOR_LEFT = 18;
const byte MOTOR_RIGHT = 19;

const byte BUMPER_BTN_LEFT = 22;
const byte BUMPER_BTN_RIGHT = 23;

const int TRIGGER_PIN_LEFT = 50;
const int ECHO_PIN_LEFT = 51;

const int TRIGGER_PIN_RIGHT = 52;
const int ECHO_PIN_RIGHT = 53;

volatile byte bumper_l = HIGH;
volatile byte bumper_r = HIGH;

volatile int counter_motor_left = 0;
volatile int counter_motor_right = 0;

volatile int left_side = 0;
volatile int right_side = 0;

float encoders_distance = 0.0;

const float stepcount = 20.00;
const float wheeldiameter = 6.75;

const float wheelcircuference = PI * wheeldiameter;
const float wheelstepdistance = wheelcircuference / stepcount;

volatile byte old_state_left = LOW;

void ISR_MOTOR_LEFT() {
  byte encoder_state = digitalRead(MOTOR_LEFT);
  if (encoder_state == HIGH && old_state_left != encoder_state) {
    counter_motor_left++;
  }
  old_state_left = encoder_state;
}

volatile byte old_state_right = LOW;

void ISR_MOTOR_RIGHT() {
  byte encoder_state = digitalRead(MOTOR_RIGHT);
  if (encoder_state == HIGH && old_state_right != encoder_state) {
    counter_motor_right++;
  }
  old_state_right = encoder_state;
}

MPU6050 mpu(0x68);

#define INTERRUPT_PIN 2
#define LED_PIN 13 // (Arduino is 13, Teensy is 11, Teensy++ is 6)
bool blinkState = false;

// MPU control/status vars
bool dmpReady = false;  // set true if DMP init was successful
uint8_t mpuIntStatus;   // holds actual interrupt status byte from MPU
uint8_t devStatus;      // return status after each device operation (0 = success, !0 = error)
uint16_t packetSize;    // expected DMP packet size (default is 42 bytes)
uint16_t fifoCount;     // count of all bytes currently in FIFO
uint8_t fifoBuffer[64]; // FIFO storage buffer

// orientation/motion vars
Quaternion q;        // [w, x, y, z]         quaternion container
VectorInt16 aa;      // [x, y, z]            accel sensor measurements
VectorInt16 aaReal;  // [x, y, z]            gravity-free accel sensor measurements
VectorInt16 aaWorld; // [x, y, z]            world-frame accel sensor measurements
VectorFloat gravity; // [x, y, z]            gravity vector
float euler[3];      // [psi, theta, phi]    Euler angle container
float ypr[3];        // [yaw, pitch, roll]   yaw/pitch/roll container and gravity vector

// ISR
volatile bool mpuInterrupt = false;     // indicates whether MPU interrupt pin has gone high
void dmpDataReady()
{
  mpuInterrupt = true;
}

int turnSpeed = 130;
double heading, modifiedTargetHeading, motorOffsetOutput, motorOffsetTurnOutput;
double headingTarget = 0.0;

PID steeringPID;
// PID turnPID;

bool firstLoop = true;
double initialPose;
double offsetAngle;

bool debug = false;

// AUX
#define NONE 0
#define IMU_WARM 1

// STATE
#define PULL 4
#define CRASH 2
#define MOVING 3

// COMAND
#define GO 7
#define BACK 10
#define TURN 11
#define STOP 12

// QUADRANT
#define Q1 1
#define Q2 2
#define Q3 3
#define Q4 4

int Command = NONE;
int State = PULL;

const long interval = 500;
volatile long previousMillis = 0;

int turn_samples = 0;

void waitGyroStabilize()
{
  int secondsToWait = 15;

  Serial.println(F("Stabilizing..."));

  for (int i = 0; i < secondsToWait; i++) {
    digitalWrite(LED_PIN, HIGH);
    delay(500);
    digitalWrite(LED_PIN, LOW);
    delay(500);
  }
}

void setTargetHeading(double targetAngle) {
  headingTarget = initialPose + headingTarget + targetAngle;
  
  if (headingTarget > 180.0) {
    headingTarget -= 360.0;
  } else if (headingTarget < -180.0) {
    headingTarget += 360.0;
  }
}

void setup()
{
  #if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
    Wire.begin();
  #elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
    Fastwire::setup(400, true);
  #endif

  Serial.begin(115200);

  attachInterrupt(digitalPinToInterrupt(MOTOR_LEFT), ISR_MOTOR_LEFT, CHANGE);
  attachInterrupt(digitalPinToInterrupt(MOTOR_RIGHT), ISR_MOTOR_RIGHT, CHANGE);

  // initialize device
  Serial.println(F("Initializing I2C devices..."));
  mpu.initialize();
  pinMode(INTERRUPT_PIN, INPUT);

  // verify connection
  Serial.println(F("Testing device connections..."));
  Serial.println(mpu.testConnection() ? F("MPU6050 connection successful") : F("MPU6050 connection failed"));

  Serial.println(F("Initializing DMP..."));
  devStatus = mpu.dmpInitialize();

  // supply your own gyro offsets here, scaled for min sensitivity
  // Note - use the 'raw' program to get these.  
  // Expect an unreliable or long startup if you don't bother!!! 
  mpu.setXGyroOffset(-25);
  mpu.setYGyroOffset(112);
  mpu.setZGyroOffset(88);
  mpu.setZAccelOffset(1161);

  // make sure it worked (returns 0 if so)
  if (devStatus == 0) {
    // turn on the DMP, now that it's ready
    Serial.println(F("Enabling DMP..."));
    mpu.setDMPEnabled(true);

    // enable Arduino interrupt detection
    Serial.println(F("Enabling interrupt detection (Arduino external interrupt 0)..."));
    attachInterrupt(0, dmpDataReady, RISING);
    mpuIntStatus = mpu.getIntStatus();

    // set our DMP Ready flag so the main loop() function knows it's okay to use it
    Serial.println(F("DMP ready! Waiting for first interrupt..."));
    dmpReady = true;

    // get expected DMP packet size for later comparison
    packetSize = mpu.dmpGetFIFOPacketSize();
  } else {
    // ERROR!
    // 1 = initial memory load failed
    // 2 = DMP configuration updates failed
    // (if it's going to break, usually the code will be 1)
    Serial.print(F("DMP Initialization failed (code "));
    Serial.print(devStatus);
    Serial.println(F(")"));
  }

  pinMode(LED_PIN, OUTPUT);

  // bumper
  pinMode(BUMPER_BTN_LEFT, INPUT);
  pinMode(BUMPER_BTN_RIGHT, INPUT);

  // ultrasonic sensor pin setup
  pinMode(ECHO_PIN_LEFT, INPUT);
  pinMode(TRIGGER_PIN_LEFT, OUTPUT);
  pinMode(ECHO_PIN_RIGHT, INPUT);
  pinMode(TRIGGER_PIN_RIGHT, OUTPUT);

  steeringPID.Setup(&modifiedTargetHeading, &motorOffsetOutput, &headingTarget, 1.5, 4, 0.4, DIRECT);

  steeringPID.SetOutputLimits(-20.0, 20.0);
  steeringPID.SetSampleTime(10);
  steeringPID.SetMode(AUTOMATIC);

  // turnPID.Setup(&modifiedTargetHeading, &motorOffsetOutput, &headingTarget, 0.05, 0.001, 2.3, DIRECT);
  // turnPID.Setup(&modifiedTargetHeading, &motorOffsetOutput, &headingTarget, 1.4, 0.7, 0.01, DIRECT);
  // turnPID.Setup(&modifiedTargetHeading, &motorOffsetOutput, &headingTarget, 1.4, 0.7, 1.1, DIRECT);
  // turnPID.Setup(&modifiedTargetHeading, &motorOffsetOutput, &headingTarget, 1.05, 0.118, 0.005, DIRECT);

  // turnPID.Setup(&modifiedTargetHeading, &motorOffsetOutput, &headingTarget, 1.05, 0.53, 0.0005, DIRECT);
  // turnPID.Setup(&modifiedTargetHeading, &motorOffsetTurnOutput, &headingTarget, 0.66, 0, 0, DIRECT);

  // turnPID.SetOutputLimits(90, 110);
  // turnPID.SetSampleTime(10);
  // turnPID.SetMode(AUTOMATIC);

  setTargetHeading(0.0);

  waitGyroStabilize();
}

float getEncodersDistance() {
  float distanceL = wheelstepdistance * counter_motor_left;
  float distanceR = wheelstepdistance * counter_motor_right;
  
  return (distanceL + distanceR) / 2;
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

// #define n 25
// double turn_angle_average[n];

// double turnAngleAverage(double angle)
// {
//   for (int i = n - 1; i > 0; i--) turn_angle_average[i] = turn_angle_average[i - 1];

//   turn_angle_average[0] = angle;

//   double acc = 0;
//   for (int i = 0; i < n; i++) acc += turn_angle_average[i];
//   return acc / n;
// }

void GetHeading() {
  //calc heading from IMU
  // reset interrupt flag and get INT_STATUS byte
  mpuInterrupt = false;
  mpuIntStatus = mpu.getIntStatus();

  // get current FIFO count
  fifoCount = mpu.getFIFOCount();

  // check for overflow (this should never happen unless our code is too inefficient)
  if ((mpuIntStatus & 0x10) || fifoCount == 1024) {
    // reset so we can continue cleanly
    mpu.resetFIFO();
    Serial.println(F("FIFO overflow!"));

    // otherwise, check for DMP data ready interrupt (this should happen frequently)
  } 
  else if (mpuIntStatus & 0x02) 
  {
    // wait for correct available data length, should be a VERY short wait
    while (fifoCount < packetSize) fifoCount = mpu.getFIFOCount();

    // read a packet from FIFO
    mpu.getFIFOBytes(fifoBuffer, packetSize);
  
    // track FIFO count here in case there is > 1 packet available
    // (this lets us immediately read more without waiting for an interrupt)
    fifoCount -= packetSize;
    
    // display Euler angles in degrees
    mpu.dmpGetQuaternion(&q, fifoBuffer);
    mpu.dmpGetGravity(&gravity, &q);
    mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);
    //Serial.print("ypr\t");

    if (firstLoop) {
      initialPose = ((ypr[0] * 180) / M_PI);
      firstLoop = false;
    }

    heading = ((ypr[0] * 180) / M_PI);

    // Serial.print(" ypr\t");
    // Serial.print(heading);
    // Serial.print("\t");
    // Serial.print(ypr[1] * 180/M_PI);
    // Serial.print("\t");
    // Serial.println(ypr[2] * 180/M_PI);
  }//done
}

int GetQuadrant() {
  if (heading > 0) {
    if (heading > 90) {
        return Q4;
    }
    return Q2;
  } else {
    if (heading < -90) {
        return Q3;
    }
    return Q1;
  }
}

// void UpdateTurnPIDDirection() {
//   if (headingTarget > 0) {
//     turnPID.SetControllerDirection(DIRECT);
//   } else {
//     turnPID.SetControllerDirection(REVERSE);
//   }
// }

void TurnCommand()
{
  State = MOVING;

  // UpdateTurnPIDDirection();

  // if (turnPID.Compute())
  // {
    double diff = abs(headingTarget) - abs(heading);

    if (abs(diff) < 1.75)
    {
      turn_samples += 1;

      motor_left.setSpeed(0);
      motor_right.setSpeed(0);

      motor_left.run(RELEASE);
      motor_right.run(RELEASE);

      if (turn_samples == 100)
      {
        turn_samples = 0;

        State = PULL;
      }
    }
    else
    {
      int quadrant = GetQuadrant();

      // motor_left.setSpeed(motorOffsetTurnOutput);
      // motor_right.setSpeed(motorOffsetTurnOutput);
      motor_left.setSpeed(100);
      motor_right.setSpeed(100);

      if (headingTarget > 0)
      {
        // direita

        if (heading < headingTarget && quadrant != Q3)
        {
          motor_left.run(FORWARD);
          motor_right.run(BACKWARD);
        }
        else
        {
          motor_left.run(BACKWARD);
          motor_right.run(FORWARD);
        }
      }
      else
      {
        // esquerda

        if (heading > headingTarget && quadrant != Q4)
        {
          motor_left.run(BACKWARD);
          motor_right.run(FORWARD);
        }
        else
        {
          motor_left.run(FORWARD);
          motor_right.run(BACKWARD);
        }
      }
    }
  // }
}

void cleanMotorEncoders () {
  counter_motor_left = 0;
  counter_motor_right = 0;
}

void CrashDetect() {
  if (bumper_l == 0 || bumper_r == 0) {
    State = CRASH;
  }
}

void StopComand() {
  motor_left.setSpeed(0);
  motor_right.setSpeed(0);

  motor_left.run(RELEASE);
  motor_right.run(RELEASE);

  cleanMotorEncoders();

  State = PULL;
}

void GoComand() {
  State = MOVING;

  if (steeringPID.Compute()) {
    motor_left.setSpeed(int(turnSpeed + motorOffsetOutput));
    motor_right.setSpeed(int(turnSpeed - motorOffsetOutput));

    motor_left.run(FORWARD);
    motor_right.run(FORWARD);
  }
}

void BackComand() {
  State = MOVING;

  if (steeringPID.Compute()) {
    motor_left.setSpeed(int(turnSpeed - motorOffsetOutput));
    motor_right.setSpeed(int(turnSpeed + motorOffsetOutput));

    motor_left.run(BACKWARD);
    motor_right.run(BACKWARD);

    if (encoders_distance>= 15) {
      StopComand();
    }
  }
}

void PullNexCommand() {
  if (State == PULL) {
    if (Command == NONE || Command == TURN) {
      State = MOVING;
      Command = GO;
    } else if (Command == BACK) {
      State = MOVING;
      Command = TURN;

      setTargetHeading(180.0);
    }
  } else if (State == CRASH) {
    if (Command == GO) {
      State = MOVING;
      Command = BACK;
    }
  }
}

void ExecuteCommand() {
  switch(Command) {
    case STOP:
      StopComand();
      break;
    case GO:
      GoComand();
      break;
    case BACK:
      BackComand();
      break;
    case TURN:
      TurnCommand();
    case NONE:
      break;
  }
}

void UpdateReferencePID() {
  if (abs(heading - headingTarget) > 180.0) {
    if (heading >= 0) {
      modifiedTargetHeading = heading - 360.0;
    }
    else {
      modifiedTargetHeading = heading + 360.0;
    }
  } else {
    modifiedTargetHeading = heading;
  }
}

void UpdateSensors() {
  bumper_l = digitalRead(BUMPER_BTN_LEFT);
  bumper_r = digitalRead(BUMPER_BTN_RIGHT);

  if (Command != TURN) {
    left_side = getDistance(TRIGGER_PIN_LEFT, ECHO_PIN_LEFT);
    right_side = getDistance(TRIGGER_PIN_RIGHT, ECHO_PIN_RIGHT);

    encoders_distance = getEncodersDistance();
  }
}

void loop() {
  if (!dmpReady) return;

  while (!mpuInterrupt && fifoCount < packetSize) { }

  GetHeading();

  UpdateSensors();

  CrashDetect();

  PullNexCommand();

  UpdateReferencePID();

  ExecuteCommand();

  unsigned long currentMillis = millis();

  if (debug && currentMillis - previousMillis >= interval)
  {
    previousMillis = currentMillis;

    Serial.print("timestamp\t");
    Serial.print(millis());
    Serial.print("\tl_bumper\t");
    Serial.print(bumper_l);
    Serial.print("\tr_bumper\t");
    Serial.print(bumper_r);
    Serial.print("\tl_side\t");
    Serial.print(left_side);
    Serial.print("\tr_side\t");
    Serial.print(right_side);
    Serial.print("\tl_encoder\t");
    Serial.print(counter_motor_left);
    Serial.print("\tr_encoder\t");
    Serial.print(counter_motor_right);
    Serial.print("\tdistance\t");
    Serial.print(encoders_distance);
    Serial.print("\tinital_pose\t");
    Serial.print(initialPose);
    Serial.print("\theading\t");
    Serial.print(heading);
    Serial.print("\ttarget\t");
    Serial.print(headingTarget);
    Serial.print("\toffset\t");
    Serial.print(motorOffsetOutput);
    Serial.print("\tcommand\t");
    Serial.print(Command);
    Serial.print("\tstate\t");
    Serial.println(State);
  }

  blinkState = !blinkState;
  digitalWrite(LED_PIN, blinkState);
}
