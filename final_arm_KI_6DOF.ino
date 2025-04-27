#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>

// Use ESP32’s second I²C port on SDA=17, SCL=16
TwoWire I2C_1 = TwoWire(1);
Adafruit_PWMServoDriver pca9685(0x40, I2C_1);


// ServoProfile & definitions

struct ServoProfile {
  uint8_t channel;
  int servoMin, servoMax;
};

// map 0–180° into 100–500 ticks on the PCA9685
ServoProfile baseRotate   = {  0, 100, 500 };
ServoProfile shoulder1    = {  2, 100, 500 };
ServoProfile shoulder2    = {  3, 100, 500 };
ServoProfile elbow1       = {  5, 100, 500 };
ServoProfile elbow2       = {  7, 100, 500 };
ServoProfile wrist1       = {  8, 100, 500 };
ServoProfile wrist2       = { 10, 100, 500 };
ServoProfile gripper      = { 11, 100, 500 };

// Movement speed: ~67 ms per degree for ~5 mm/s
const int speedDelay = 67;

// Mechanical angle limits
const float BASE_MIN_ANGLE       =   0.0;   // base can sweep full 0–180°
const float BASE_MAX_ANGLE       = 180.0;
const float JOINT_MIN_ANGLE      =  20.0;   // other joints safe window
const float JOINT_MAX_ANGLE      = 160.0;
const float GRIPPER_OPEN_ANGLE   =   0.0;
const float GRIPPER_CLOSED_ANGLE = 180.0;

// Arm link lengths (mm)
const float d1 = 56.5;
const float a2 = 136.62;
const float a3 = 57.0;
const float a4 = 150.07;
const float a5 = 57.0;

// Track each joint’s current angle
float currentAngle[8] = {
  90, // base
  90, // shoulder1
  90, // shoulder2
  90, // elbow1
  90, // elbow2
  90, // wrist1
  90, // wrist2
  GRIPPER_OPEN_ANGLE // gripper
};

// helper to convert an angle into a PWM pulse
uint16_t angleToPulse(const ServoProfile &s, float angle) {
  return map((int)angle, 0, 180, s.servoMin, s.servoMax);
}

// instantly set one servo to a given angle
void setServoAngle(const ServoProfile &s, float angle) {
  angle = constrain(angle, 0, 180);
  pca9685.setPWM(s.channel, 0, angleToPulse(s, angle));
}

// smoothly move one servo from cur to tgt
void moveSmooth(const ServoProfile &s, float &cur, float tgt) {
  tgt = constrain(tgt, 0, 180);
  while (fabs(cur - tgt) > 0.5) {
    cur += (tgt > cur) ? 1.0 : -1.0;
    setServoAngle(s, cur);
    delay(speedDelay);
  }
}

// move both shoulder servos together in opposite dir
void moveShoulderSmooth(float tgt) {
  tgt = constrain(tgt, JOINT_MIN_ANGLE, JOINT_MAX_ANGLE);
  while (fabs(currentAngle[1] - tgt) > 0.5) {
    currentAngle[1] += (tgt > currentAngle[1]) ? 1.0 : -1.0;
    currentAngle[2] = 180 - currentAngle[1];
    setServoAngle(shoulder1, currentAngle[1]);
    setServoAngle(shoulder2, currentAngle[2]);
    delay(speedDelay);
  }
}

// move both wrists together
void moveWristGroup(float w1T, float w2T) {
  w1T = constrain(w1T, JOINT_MIN_ANGLE, JOINT_MAX_ANGLE);
  w2T = constrain(w2T, JOINT_MIN_ANGLE, JOINT_MAX_ANGLE);
  while (fabs(currentAngle[5] - w1T) > 0.5 ||
         fabs(currentAngle[6] - w2T) > 0.5) {
    if      (currentAngle[5] < w1T) currentAngle[5]++;
    else if (currentAngle[5] > w1T) currentAngle[5]--;
    if      (currentAngle[6] < w2T) currentAngle[6]++;
    else if (currentAngle[6] > w2T) currentAngle[6]--;
    setServoAngle(wrist1, currentAngle[5]);
    setServoAngle(wrist2, currentAngle[6]);
    delay(speedDelay);
  }
}

// helper to print current angles to Serial for feedback
void printAngles() {
  Serial.printf("Base:     %.1f°\n", currentAngle[0]);
  Serial.printf("Shoulder: %.1f°\n", currentAngle[1]);
  Serial.printf("Shoulder: %.1f°\n", currentAngle[2]);
  Serial.printf("Elbow1:   %.1f°\n", currentAngle[3]);
  Serial.printf("Elbow2:   %.1f°\n", currentAngle[4]);
  Serial.printf("Wrist1:   %.1f°\n", currentAngle[5]);
  Serial.printf("Wrist2:   %.1f°\n", currentAngle[6]);
  Serial.printf("Gripper:  %.1f°\n\n",currentAngle[7]);
}

void setup() {
  Serial.begin(115200);
  while (!Serial) {}
  Wire.begin();
  pca9685.begin();
  pca9685.setPWMFreq(60);

  // center all servos at startup
  setServoAngle(baseRotate,   currentAngle[0]);
  setServoAngle(shoulder1,    currentAngle[1]);
  setServoAngle(shoulder2,    currentAngle[2]);
  setServoAngle(elbow1,       currentAngle[3]);
  setServoAngle(elbow2,       currentAngle[4]);
  setServoAngle(wrist1,       currentAngle[5]);
  setServoAngle(wrist2,       currentAngle[6]);
  setServoAngle(gripper,      currentAngle[7]);

  Serial.println("Robotic Arm Ready.");
  Serial.println("Commands:");
  Serial.println("  X Y Z    → Move to coordinates (mm)");
  Serial.println("  gripper open|close");
}

// handle a manual XYZ command via inverse kinematics
void handleXYZ(float X, float Y, float Z) {
  // compute base angle
  float baseAng = atan2(Y, X) * 180.0 / PI;
  if (baseAng < 0) baseAng += 360.0;
  if (baseAng > 180.0) baseAng -= 360.0;
  bool ok = !(baseAng < BASE_MIN_ANGLE || baseAng > BASE_MAX_ANGLE);

  // planar distance and vertical offset
  float R  = sqrt(X*X + Y*Y);
  float Zp = Z - d1;
  // combined link lengths
  float L1 = a2 + a3, L2 = a4 + a5;
  float D  = sqrt(R*R + Zp*Zp);
  if (D > (L1 + L2) || D < fabs(L1 - L2)) ok = false;

  float shoulderAng = 0, elbowAng = 0;
  if (ok) {
    float c2 = (L1*L1 + L2*L2 - D*D) / (2*L1*L2);
    c2 = constrain(c2, -1.0, 1.0);
    elbowAng = acos(c2) * 180.0 / PI;
    float theta = atan2(Zp, R);
    float phi   = acos((L1*L1 + D*D - L2*L2)/(2*L1*D));
    shoulderAng = (theta + phi) * 180.0 / PI;
    ok = ok
      && (shoulderAng >= JOINT_MIN_ANGLE && shoulderAng <= JOINT_MAX_ANGLE)
      && (elbowAng    >= JOINT_MIN_ANGLE && elbowAng    <= JOINT_MAX_ANGLE);
  }

  if (!ok) {
    Serial.println("Target out of range");
    return;
  }

  float tgt[8];
  tgt[0] = baseAng;
  tgt[1] = shoulderAng;
  tgt[2] = 180 - shoulderAng;
  tgt[3] = elbowAng;
  tgt[4] = elbowAng;
  tgt[5] = currentAngle[5];
  tgt[6] = currentAngle[6];
  tgt[7] = currentAngle[7];

  bool moving;
  do {
    moving = false;
    // Base
    if (fabs(currentAngle[0] - tgt[0]) > 0.5) {
      currentAngle[0] += (tgt[0] > currentAngle[0]) ? 1 : -1;
      setServoAngle(baseRotate, currentAngle[0]);
      moving = true;
    }
    // Shoulder
    if (fabs(currentAngle[1] - tgt[1]) > 0.5) {
      currentAngle[1] += (tgt[1] > currentAngle[1]) ? 1 : -1;
      currentAngle[2] = 180 - currentAngle[1];
      setServoAngle(shoulder1, currentAngle[1]);
      setServoAngle(shoulder2, currentAngle[2]);
      moving = true;
    }
    // Elbows
    if (fabs(currentAngle[3] - tgt[3]) > 0.5) {
      currentAngle[3] += (tgt[3] > currentAngle[3]) ? 1 : -1;
      setServoAngle(elbow1, currentAngle[3]);
      moving = true;
    }
    if (fabs(currentAngle[4] - tgt[4]) > 0.5) {
      currentAngle[4] += (tgt[4] > currentAngle[4]) ? 1 : -1;
      setServoAngle(elbow2, currentAngle[4]);
      moving = true;
    }
    // Wrists
    if (fabs(currentAngle[5] - tgt[5]) > 0.5) {
      currentAngle[5] += (tgt[5] > currentAngle[5]) ? 1 : -1;
      setServoAngle(wrist1, currentAngle[5]);
      moving = true;
    }
    if (fabs(currentAngle[6] - tgt[6]) > 0.5) {
      currentAngle[6] += (tgt[6] > currentAngle[6]) ? 1 : -1;
      setServoAngle(wrist2, currentAngle[6]);
      moving = true;
    }
    if (moving) delay(speedDelay);
  } while (moving);

  Serial.print("Moved to (");
  Serial.print(X); Serial.print(", ");
  Serial.print(Y); Serial.print(", ");
  Serial.print(Z); Serial.println(")");
  printAngles();  // print updated angles
}

void loop() {
  if (!Serial.available()) return;
  String cmd = Serial.readStringUntil('\n');
  cmd.trim();

  if (cmd.startsWith("gripper")) {
    if (cmd.indexOf("close") >= 0) {
      currentAngle[7] = GRIPPER_CLOSED_ANGLE;
    } else {
      currentAngle[7] = GRIPPER_OPEN_ANGLE;
    }
    setServoAngle(gripper, currentAngle[7]);
    Serial.print("Gripper ");
    Serial.println((currentAngle[7] > 0) ? "closed" : "opened");
    printAngles();  // print updated angles
    return;
  }

  int p1 = cmd.indexOf(' ');
  int p2 = cmd.indexOf(' ', p1 + 1);
  if (p1 < 0 || p2 < 0) {
    Serial.println("Invalid format. Use: X Y Z");
    return;
  }
  float X = cmd.substring(0, p1).toFloat();
  float Y = cmd.substring(p1 + 1, p2).toFloat();
  float Z = cmd.substring(p2 + 1).toFloat();
  handleXYZ(X, Y, Z);
}

