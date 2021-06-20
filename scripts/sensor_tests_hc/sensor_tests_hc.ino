#define echoPin_L A0
#define triggerPin_L A1

#define echoPin_F A2
#define triggerPin_F A3

#define echoPin_R A4
#define triggerPin_R A5

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

  pinMode(echoPin_L, INPUT);
  pinMode(triggerPin_L, OUTPUT);

  pinMode(echoPin_F, INPUT);
  pinMode(triggerPin_F, OUTPUT);

  pinMode(echoPin_R, INPUT);
  pinMode(triggerPin_R, OUTPUT);
}

void loop() {
  int distance_f = getDistance(triggerPin_F, echoPin_F);
  int distance_r = getDistance(triggerPin_R, echoPin_R);
  int distance_l = getDistance(triggerPin_L, echoPin_L);

  Serial.print("HC-SR04 Left: ");
  Serial.print(distance_l);
  Serial.print(" HC-SR04 Right: ");
  Serial.print(distance_r);
  Serial.print(" HC-SR04 Front: ");
  Serial.println(distance_f);
}
