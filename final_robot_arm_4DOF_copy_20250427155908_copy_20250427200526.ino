#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>

// Use ESP32’s second I²C port on SDA=17, SCL=16
TwoWire I2C_1 = TwoWire(1);
Adafruit_PWMServoDriver pca9685(0x40, I2C_1);

// define a small struct for each servo’s channel and pulse range
struct ServoProfile {
  uint8_t channel;
  int servoMin, servoMax;
};

// map 0–180° into 100–500 ticks on the PCA9685
ServoProfile baseRotate  = {  0, 100, 500 };
ServoProfile shoulder1   = {  2, 100, 500 };
ServoProfile shoulder2   = {  3, 100, 500 };
ServoProfile wrist1      = {  8, 100, 500 };
ServoProfile wrist2      = { 10, 100, 500 };
ServoProfile gripper     = { 11, 100, 500 };

const int speedDelay = 67;  // around 67 ms per degree for ~5 mm/s

// keep track of each joint’s current angle
int currentBase     = 90;
int currentShoulder = 90;
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
void moveShoulderSmooth(int tgt) {
  int step = (tgt > currentShoulder) ? 1 : -1;
  while (currentShoulder != tgt) {
    currentShoulder += step;
    setServoAngle(shoulder1, currentShoulder);
    setServoAngle(shoulder2, 180 - currentShoulder);
    delay(speedDelay);
  }
}

// move both wrists together
void moveWristGroup(int w1T, int w2T) {
  while (currentWrist1 != w1T || currentWrist2 != w2T) {
    if      (currentWrist1 < w1T) currentWrist1++;
    else if (currentWrist1 > w1T) currentWrist1--;
    if      (currentWrist2 < w2T) currentWrist2++;
    else if (currentWrist2 > w2T) currentWrist2--;
    setServoAngle(wrist1, currentWrist1);
    setServoAngle(wrist2, currentWrist2);
    delay(speedDelay);
  }
}

// set of predefined poses (modes) for the arm
void homeMode() {
  moveSmooth(baseRotate,  currentBase,   90);
  moveShoulderSmooth(160);
  moveSmooth(wrist1,      currentWrist1, 25);
  moveSmooth(wrist2,      currentWrist2,180);
  moveSmooth(gripper,     currentGripper,0);
}

void home2Mode() {
  moveSmooth(baseRotate,  currentBase,   90);
  moveShoulderSmooth(160);
  moveSmooth(wrist1,      currentWrist1, 25);
  moveSmooth(wrist2,      currentWrist2,180);
  moveSmooth(gripper,     currentGripper,180);
}

void swanMode() {
  moveSmooth(baseRotate,  currentBase,   45);
  moveShoulderSmooth(135);
  moveSmooth(wrist1,      currentWrist1,45);
  moveSmooth(wrist2,      currentWrist2,160);
  moveSmooth(gripper,     currentGripper,0);
}

void collectMode() {
  moveSmooth(baseRotate,  currentBase,   45);
  moveShoulderSmooth(90);
  moveSmooth(wrist1,      currentWrist1,25);
  moveSmooth(wrist2,      currentWrist2,120);
  moveSmooth(gripper,     currentGripper,180);
}

void foodMode() {
  moveShoulderSmooth(70);
}

void gooseMode() {
  moveSmooth(baseRotate,  currentBase,   45);
  moveShoulderSmooth(135);
  moveSmooth(wrist1,      currentWrist1,45);
  moveSmooth(wrist2,      currentWrist2,160);
  moveSmooth(gripper,     currentGripper,180);
}

void feedingMode() {
  moveSmooth(baseRotate,  currentBase,   45);
  moveShoulderSmooth(80);
  moveSmooth(wrist1,      currentWrist1,40);
  moveSmooth(wrist2,      currentWrist2,120);
  moveSmooth(gripper,     currentGripper,180);
}

void scranMode() {
  feedingMode();
  moveShoulderSmooth(85);
  moveWristGroup(45, 90);
  moveSmooth(gripper, currentGripper, 180);
}

// run through all modes in a row, with pauses in between
void runSequence() {
  homeMode();   delay(500);
  swanMode();   delay(500);
  collectMode();delay(500);
  foodMode();   delay(500);
  collectMode();delay(500);
  gooseMode();  delay(500);
  moveSmooth(wrist2,      currentWrist2,90);
  scranMode();  delay(500);
  delay(1000);
  gooseMode();  delay(500);
  home2Mode();
}

// helper to print current angles to Serial for fedback
void printAngles() {
  Serial.printf("Base: %d°\n",      currentBase);
  Serial.printf("Shoulder: %d°\n",  currentShoulder);
  Serial.printf("Wrist1: %d°\n",    currentWrist1);
  Serial.printf("Wrist2: %d°\n",    currentWrist2);
  Serial.printf("Gripper: %d°\n\n", currentGripper);
}

void setup() {
  Serial.begin(115200);
  delay(100);
  Serial.println("Ready. Commands:");
  Serial.println("  get angles");
  Serial.println("  set <motor> <angle>");
  Serial.println("  home | home2 | swan | collect | food | goose | feeding | scran | sequence");

  // initialize that second i2c bus on pins 17/16
  I2C_1.begin(17, 16, 400000);
  pca9685.begin();
  pca9685.setPWMFreq(50);

  // put everything at 90° to start
  moveSmooth(baseRotate,  currentBase,   90);
  moveShoulderSmooth(currentShoulder);
  moveSmooth(wrist1,      currentWrist1,90);
  moveSmooth(wrist2,      currentWrist2,90);
  moveSmooth(gripper,     currentGripper,90);
}

void loop() {
  if (!Serial.available()) return;

  String cmd = Serial.readStringUntil('\n');
  cmd.trim();

  if (cmd.equalsIgnoreCase("get angles")) {
    printAngles();
  }
  else if (cmd.startsWith("set ")) {
    int sp = cmd.indexOf(' ', 4);
    if (sp < 0) {
      Serial.println("Usage: set <motor> <angle>");
    } else {
      String m = cmd.substring(4, sp);
      int angle = cmd.substring(sp + 1).toInt();
      if      (m.equalsIgnoreCase("base"))     moveSmooth(baseRotate,  currentBase,   angle);
      else if (m.equalsIgnoreCase("shoulder")) moveShoulderSmooth(angle);
      else if (m.equalsIgnoreCase("wrist1"))   moveSmooth(wrist1,      currentWrist1, angle);
      else if (m.equalsIgnoreCase("wrist2"))   moveSmooth(wrist2,      currentWrist2, angle);
      else if (m.equalsIgnoreCase("gripper"))  moveSmooth(gripper,     currentGripper,angle);
      else Serial.println("Unknown motor.");
    }
  }
  else if (cmd.equalsIgnoreCase("home"))     homeMode();
  else if (cmd.equalsIgnoreCase("home2"))    home2Mode();
  else if (cmd.equalsIgnoreCase("swan"))     swanMode();
  else if (cmd.equalsIgnoreCase("collect"))  collectMode();
  else if (cmd.equalsIgnoreCase("food"))     foodMode();
  else if (cmd.equalsIgnoreCase("goose"))    gooseMode();
  else if (cmd.equalsIgnoreCase("feeding"))  feedingMode();
  else if (cmd.equalsIgnoreCase("scran"))    scranMode();
  else if (cmd.equalsIgnoreCase("sequence")) runSequence();
  else {
    Serial.println("Unknown command.");
  }
}

