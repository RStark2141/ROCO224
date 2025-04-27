#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>

//I²C & PCA9685 setup 
// PCA9685 on SDA=17, SCL=16
TwoWire I2C_1 = TwoWire(1);
Adafruit_PWMServoDriver pca9685(0x40, I2C_1);


// ServoProfile & definitions

struct ServoProfile {
  uint8_t channel;
  int servoMin, servoMax;
};

// PCA9685 channels remain unchanged
ServoProfile baseRotate  = {  0, 100, 500 };
ServoProfile shoulder1   = {  2, 100, 500 };
ServoProfile shoulder2   = {  3, 100, 500 };
ServoProfile wrist1      = {  8, 100, 500 };
ServoProfile wrist2      = { 10, 100, 500 };
ServoProfile gripper     = { 11, 100, 500 };

const int speedDelay = 67;  // ~67 ms per degree for ~5 mm/s

// Mechanical limits
const float BASE_MIN     =   0.0;
const float BASE_MAX     = 180.0;
const float JOINT_MIN    =  20.0;
const float JOINT_MAX    = 160.0;
const float GRIPPER_OPEN   =   0.0;
const float GRIPPER_CLOSED = 180.0;

// Link lengths (mm)
const float d1 = 56.5;     // base to shoulder
const float a2 = 136.62;   // shoulder ti wrist1
const float a3 = 57.0;     // wrist1 to wrist2

// Current angles: [0]=base, [1]=sh1, [2]=sh2, [3]=w1, [4]=w2, [5]=gripper
float currentAngle[6] = {
  90,  // base
  90,  // shoulder1
  90,  // shoulder2
  90,  // wrist1
  90,  // wrist2
  GRIPPER_OPEN
};

// Map 0–180° to PCA9685 pulse
uint16_t angleToPulse(const ServoProfile &s, float ang) {
  return map((int)ang, 0, 180, s.servoMin, s.servoMax);
}

// instantly set one servo to a given angle
void setServoAngle(const ServoProfile &s, float ang) {
  ang = constrain(ang, 0, 180);
  pca9685.setPWM(s.channel, 0, angleToPulse(s, ang));
}

// smoothly move one servo from cur to tgt
void moveSmooth(const ServoProfile &s, float &cur, float tgt) {
  tgt = constrain(tgt, 0, 180);
  while (fabs(cur - tgt) > 0.5) {
    cur += (tgt > cur) ? 1 : -1;
    setServoAngle(s, cur);
    delay(speedDelay);
  }
}

// move both shoulder servos together in opposite dir
void moveShoulderSmooth(float tgt) {
  tgt = constrain(tgt, JOINT_MIN, JOINT_MAX);
  while (fabs(currentAngle[1] - tgt) > 0.5) {
    currentAngle[1] += (tgt > currentAngle[1]) ? 1 : -1;
    currentAngle[2] = 180 - currentAngle[1];
    setServoAngle(shoulder1, currentAngle[1]);
    setServoAngle(shoulder2, currentAngle[2]);
    delay(speedDelay);
  }
}

// move both wrists together
void moveWristGroup(float w1T, float w2T) {
  w1T = constrain(w1T, JOINT_MIN, JOINT_MAX);
  w2T = constrain(w2T, JOINT_MIN, JOINT_MAX);
  while (fabs(currentAngle[3] - w1T) > 0.5 ||
         fabs(currentAngle[4] - w2T) > 0.5) {
    if      (currentAngle[3] < w1T) currentAngle[3]++;
    else if (currentAngle[3] > w1T) currentAngle[3]--;
    if      (currentAngle[4] < w2T) currentAngle[4]++;
    else if (currentAngle[4] > w2T) currentAngle[4]--;
    setServoAngle(wrist1, currentAngle[3]);
    setServoAngle(wrist2, currentAngle[4]);
    delay(speedDelay);
  }
}

// helper to print current angles to Serial for feedback
void printAngles() {
  Serial.printf("Base:     %.1f°\n", currentAngle[0]);
  Serial.printf("Shoulder: %.1f° / %.1f°\n", currentAngle[1], currentAngle[2]);
  Serial.printf("Wrist1:   %.1f°\n",       currentAngle[3]);
  Serial.printf("Wrist2:   %.1f°\n",       currentAngle[4]);
  Serial.printf("Gripper:  %.1f°\n\n",     currentAngle[5]);
}

void setup() {
  Serial.begin(115200);
  while (!Serial) {}

  // Initialize I2C bus #1 on SDA=17, SCL=16 at 400kHz
  I2C_1.begin(17, 16, 400000);

  // Initialize PCA9685 on that bus
  pca9685.begin();
  pca9685.setPWMFreq(60);

  // center all servos at startup
  setServoAngle(baseRotate,   currentAngle[0]);
  setServoAngle(shoulder1,    currentAngle[1]);
  setServoAngle(shoulder2,    currentAngle[2]);
  setServoAngle(wrist1,       currentAngle[3]);
  setServoAngle(wrist2,       currentAngle[4]);
  setServoAngle(gripper,      currentAngle[5]);

  Serial.println("Arm ready.");
  Serial.println("Commands:");
  Serial.println("  0 0 0          → go to home posture (all 90°)");
  Serial.println("  X Y Z          → move to coordinates in mm");
  Serial.println("  gripper open   → open gripper");
  Serial.println("  gripper close  → close gripper");
}

// handle a manual XYZ command via inverse kinematics
void handleXYZ(float X, float Y, float Z) {
  // Special home command: 0 0 0
  if (fabs(X) < 1e-3 && fabs(Y) < 1e-3 && fabs(Z) < 1e-3) {
    moveSmooth(baseRotate, currentAngle[0], 90);
    moveShoulderSmooth(90);
    moveSmooth(wrist1,     currentAngle[3], 90);
    moveSmooth(wrist2,     currentAngle[4], 90);
    moveSmooth(gripper,    currentAngle[5], GRIPPER_OPEN);
    Serial.println("Homed to 90° posture");
    printAngles();
    return;
  }

  // Compute base angle
  float baseAng = atan2(Y, X) * 180.0 / PI;
  if (baseAng < 0) baseAng += 360;
  if (baseAng > 180) baseAng -= 360;
  bool ok = (baseAng >= BASE_MIN && baseAng <= BASE_MAX);

  // Planar 2-link reach (shoulder → wrist1)
  float R  = sqrt(X*X + Y*Y);
  float Zp = Z - d1;
  float D  = sqrt(R*R + Zp*Zp);
  if (D > (a2 + a3) || D < fabs(a2 - a3)) ok = false;

  float shAng = 0, w1Ang = 0;
  if (ok) {
    float c2 = (a2*a2 + a3*a3 - D*D) / (2*a2*a3);
    c2 = constrain(c2, -1.0, 1.0);
    w1Ang = acos(c2) * 180.0 / PI;
    float theta = atan2(Zp, R);
    float phi   = acos((a2*a2 + D*D - a3*a3) / (2*a2*D));
    shAng = (theta + phi) * 180.0 / PI;
    ok = ok
      && (shAng >= JOINT_MIN && shAng <= JOINT_MAX)
      && (w1Ang >= JOINT_MIN && w1Ang <= JOINT_MAX);
  }

  if (!ok) {
    Serial.println("Target out of range");
    return;
  }

  // Smoothly move to target
  moveSmooth(baseRotate, currentAngle[0], baseAng);
  moveShoulderSmooth(shAng);
  moveSmooth(wrist1,     currentAngle[3], w1Ang);

  Serial.printf("Moved to (%.1f, %.1f, %.1f)\n", X, Y, Z);
  printAngles();
}

void loop() {
  if (!Serial.available()) return;

  String cmd = Serial.readStringUntil('\n');
  cmd.trim();

  if (cmd.startsWith("gripper")) {
    if (cmd.indexOf("close") >= 0) currentAngle[5] = GRIPPER_CLOSED;
    else                           currentAngle[5] = GRIPPER_OPEN;
    setServoAngle(gripper, currentAngle[5]);
    Serial.println(currentAngle[5] > 0 ? "Gripper closed" : "Gripper open");
    printAngles();
    return;
  }

  int p1 = cmd.indexOf(' ');
  int p2 = cmd.indexOf(' ', p1 + 1);
  if (p1 < 0 || p2 < 0) {
    Serial.println("Use format: X Y Z");
    return;
  }
  float X = cmd.substring(0, p1).toFloat();
  float Y = cmd.substring(p1 + 1, p2).toFloat();
  float Z = cmd.substring(p2 + 1).toFloat();
  handleXYZ(X, Y, Z);
}

