#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>

// Use ESP32’s second I²C port on SDA=17, SCL=16
#define PCA_SDA_PIN 17
#define PCA_SCL_PIN 16
TwoWire I2C_1 = TwoWire(1);
Adafruit_PWMServoDriver pca9685(0x40, I2C_1);

// HC-SR04 ultrasonic sensor pins
const int trigPin = 5;
const int echoPin = 18;

// define a small struct for each servo’s channel and pulse range
struct ServoProfile {
  uint8_t channel;
  int servoMin, servoMax;
};

// map 0–180° into 100–500 ticks on the PCA9685
ServoProfile baseRotate  = {  0, 100, 500 };
ServoProfile shoulder1   = {  2, 100, 500 };
ServoProfile shoulder2   = {  3, 100, 500 };
ServoProfile elbow1      = {  5, 100, 500 };
ServoProfile elbow2      = {  7, 100, 500 };
ServoProfile wrist1      = {  8, 100, 500 };
ServoProfile wrist2      = { 10, 100, 500 };
ServoProfile gripper     = { 11, 100, 500 };

const int speedDelay = 67;  // around 67 ms per degree for ~5 mm/s

// keep track of each joint’s current angle
int currentBase     = 90;
int currentShoulder = 90;
int currentElbow1   = 90;
int currentElbow2   = 90;
int currentWrist1   = 90;
int currentWrist2   = 90;
int currentGripper  = 90;

// helper to convert an angle into a PWM pulse
uint16_t angleToPulse(const ServoProfile &s, int angle) {
  return map(angle, 0, 180, s.servoMin, s.servoMax);
}

// instantly set one servo to a given angle
void setServoAngle(const ServoProfile &s, int angle) {
  pca9685.setPWM(s.channel, 0, angleToPulse(s, angle));
}

// smoothly move one servo from cur to tgt
void moveSmooth(const ServoProfile &s, int &cur, int tgt) {
  int step = (tgt > cur) ? 1 : -1;
  while (cur != tgt) {
    cur += step;
    setServoAngle(s, cur);
    delay(speedDelay);
  }
}

// move both shoulder servos together in opposite dir
void setShoulderAngle(int angle) {
  setServoAngle(shoulder1, angle);
  setServoAngle(shoulder2, 180 - angle);
}

// smoothly move both shoulder + both elbows at once
void moveArmGroup(int sT, int e1T, int e2T) {
  while (currentShoulder != sT ||
         currentElbow1  != e1T ||
         currentElbow2  != e2T) {
    if      (currentShoulder < sT)  currentShoulder++;
    else if (currentShoulder > sT)  currentShoulder--;
    if      (currentElbow1 < e1T)   currentElbow1++;
    else if (currentElbow1 > e1T)   currentElbow1--;
    if      (currentElbow2 < e2T)   currentElbow2++;
    else if (currentElbow2 > e2T)   currentElbow2--;
    setShoulderAngle(currentShoulder);
    setServoAngle(elbow1, currentElbow1);
    setServoAngle(elbow2, currentElbow2);
    delay(speedDelay);
  }
}

// move both wrists together
void moveWristGroup(int w1T, int w2T) {
  while (currentWrist1 != w1T ||
         currentWrist2 != w2T) {
    if      (currentWrist1 < w1T) currentWrist1++;
    else if (currentWrist1 > w1T) currentWrist1--;
    if      (currentWrist2 < w2T) currentWrist2++;
    else if (currentWrist2 > w2T) currentWrist2--;
    setServoAngle(wrist1, currentWrist1);
    setServoAngle(wrist2, currentWrist2);
    delay(speedDelay);
  }
}

// Read HC-SR04 (cm or –1 on timeout)
float readUltrasonic() {
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);
  long d = pulseIn(echoPin, HIGH, 30000);
  return d ? (d * 0.0343f) / 2.0f : -1;
}

// set of predefined poses for the arm
void homeMode() {
  moveSmooth(baseRotate,  currentBase,   0);
  moveArmGroup(           30, 150,      30);
  moveSmooth(wrist1,      currentWrist1,30);
  moveSmooth(wrist2,      currentWrist2,90);
  moveSmooth(gripper,     currentGripper,0);
}

void swanMode() {
  moveSmooth(baseRotate,  currentBase,   0);
  moveArmGroup(           45, 135,      45);
  moveSmooth(wrist1,      currentWrist1,45);
  moveSmooth(wrist2,      currentWrist2,20);
  moveSmooth(gripper,     currentGripper,0);
  delay(10000);
}

void collectMode() {
  moveSmooth(baseRotate,  currentBase,   0);
  moveArmGroup(           60, 135,      30);
  moveSmooth(wrist1,      currentWrist1,90);
  moveSmooth(wrist2,      currentWrist2,90);
  moveSmooth(gripper,     currentGripper,0);
}

void foodMode() {
  moveArmGroup(           70, currentElbow1, currentElbow2);
  moveSmooth(gripper,     currentGripper,90);
}

void gooseMode() {
  moveSmooth(baseRotate,  currentBase,   0);
  moveArmGroup(           45, 135,      45);
  moveSmooth(wrist1,      currentWrist1,45);
  moveSmooth(wrist2,      currentWrist2,20);
  moveSmooth(gripper,     currentGripper,90);
}

void feedingMode() {
  moveSmooth(baseRotate,  currentBase,   0);
  moveArmGroup(           75, 150,     110);
  moveSmooth(wrist1,      currentWrist1,45);
  moveSmooth(wrist2,      currentWrist2,20);
  moveSmooth(gripper,     currentGripper,90);
}

void scranMode() {
  moveWristGroup(         90, 90);
  moveSmooth(gripper,     currentGripper,0);
}

// run through all modes in a row, with pauses in between
void runSequence() {
  Serial.println("Starting in 5 s…");
  delay(5000);

  homeMode();    delay(500);
  swanMode();    delay(500);
  collectMode(); delay(500);
  foodMode();    delay(500);
  gooseMode();   delay(500);

  // wait for object ≤ 6 cm
  Serial.println("Waiting for distance ≤ 6 cm…");
  float d;
  do {
    d = readUltrasonic();
    Serial.print("  Distance = ");
    if (d < 0) Serial.println("Out of range");
    else       Serial.print(d, 1), Serial.println(" cm");
    delay(200);
  } while (d < 0 || d > 6);
  Serial.println("➡️ Distance OK");
  delay(500);

  feedingMode(); delay(500);
  scranMode();   delay(500);
  swanMode();    delay(500);
  homeMode();

  Serial.println("Sequence complete.");
}

bool sequenceStarted = false;

void setup() {
  Serial.begin(115200);
  Serial.println("\nType 'start' to begin.");

  pinMode(trigPin, OUTPUT);
  pinMode(echoPin, INPUT);

  // initialize I2C bus #1 on SDA=17, SCL=16 at 400kHz
  I2C_1.begin(PCA_SDA_PIN, PCA_SCL_PIN, 400000);
  pca9685.begin();
  pca9685.setPWMFreq(50);

  // center all servos at startup
  setServoAngle(baseRotate,   currentBase);
  setShoulderAngle(currentShoulder);
  setServoAngle(elbow1,       currentElbow1);
  setServoAngle(elbow2,       currentElbow2);
  setServoAngle(wrist1,       currentWrist1);
  setServoAngle(wrist2,       currentWrist2);
  setServoAngle(gripper,      currentGripper);
}

void loop() {
  if (!sequenceStarted && Serial.available()) {
    String cmd = Serial.readStringUntil('\n');
    cmd.trim();
    if (cmd.equalsIgnoreCase("start")) {
      sequenceStarted = true;
      runSequence();
    }
  }
}

