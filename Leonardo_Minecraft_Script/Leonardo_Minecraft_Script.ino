#include "Wire.h"
#include "MPU6050.h"
#include "Keyboard.h"
#include "Mouse.h"

// Initialize the IMUs
MPU6050 imu1(0x69);
MPU6050 imu2(0x68);


// Pin Definitions
const int jrx = A0, jry = A1, jrsw = 5; // Right Joystick
const int jlx = A3, jly = A2, jlsw = 6; // Left Joystick
const int rBtn1 = 10;
const int rBtn = 11;
const int lBtn1 = 8;
const int lBtn2 = 9;
const int TRIG_PIN = 12;
const int ECHO_PIN = 13;

int centerX, centerY;
const int centerDeadZone = 20;
const float sensitivity = 0.05;
const float rotation_angle = 90.0;
float init_ultrasonic_distance;

// Walking
const float walkingThreshold = 1.0; // cm/s threshold for walking
const int sampleWindow = 5; // samples to average over for gait detection
const unsigned long walkingTimeout = 500; // ms to keep walking after last detection
unsigned long lastWalkingDetected = 0;
bool isWalking = false;

// Button thresholding 
const unsigned long buttonTime = 200;

// Mouse thresholding
unsigned long lastClickTriggered = 0;
const unsigned long clickTimeout = 1000;
bool isClicking = false;

unsigned long lastClickTriggeredR = 0;
const unsigned long clickTimeoutR = 1000;
bool isClickingR = false;

float distanceHistory[sampleWindow];
int historyIndex = 0;
bool historyFull = false;

float getAverageDistance() {
  int count = historyFull ? sampleWindow : historyIndex;
  if (count == 0){
    return -1;
  } 
  float sum = 0;
  for (int i = 0; i < count; i++){
    sum += distanceHistory[i];
    return sum/count;
  }
}

void updateGaitDetection(float newDistance){
  // Ignore readings that are out of range
  if (newDistance < 0) {
    return;
  }

  float prevAvg = getAverageDistance();

  distanceHistory[historyIndex] = newDistance;
  historyIndex = (historyIndex + 1) % sampleWindow;
  if (historyIndex == 0 ) {
    historyFull = true;
  }

  float newAvg = getAverageDistance();

  // How fast is the distance changing?
  if (historyFull) {
    float delta = abs(newAvg - prevAvg);
    // isWalking = (delta > walkingThreshold);

    if (delta > walkingThreshold) {
      lastWalkingDetected = millis();
      isWalking = true;
    } else if ( millis() - lastWalkingDetected > walkingTimeout) {
      isWalking = false;
    }
  }

}

int applyCenterDeadZone(int value, int center, int deadzone) {
  int offset = value - center;
  if (abs(offset) < deadzone ) {
    return 0;
  }
  if (offset > 0) {
    return offset - deadzone;
  } else {
    return offset + deadzone;
  }
}

void rotateJoystick(int x, int y, float angleDeg, int &outX, int &outY) {
  float angleRad = angleDeg * PI / 180.0;
  outX = (int)(x * cos(angleRad) - y*sin(angleRad));
  outY = (int)(x * sin(angleRad) + y*cos(angleRad));
}

float getDistanceCm() {
  digitalWrite(TRIG_PIN, LOW);
  delayMicroseconds(2);
  digitalWrite(TRIG_PIN, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIG_PIN, LOW);

  long duration = pulseIn(ECHO_PIN, HIGH, 30000); // Timeout after 30ms (~5m max range)
  if (duration == 0) return -1; // Out of range or no echo

  return duration * 0.0343 / 2.0;
}

void setup() {
  Serial.begin(9600);
  while(!Serial);

  Wire.begin();

  // Initialize the IMUs
  imu1.initialize();
  imu2.initialize();
  Serial.println(imu2.testConnection() ? "IMU2 OK" : "IMU2 FAILED");

  // Initialize mouse and keyboard
  Keyboard.begin();
  Mouse.begin();

  // Ultrasonic pins
  pinMode(TRIG_PIN, OUTPUT);
  pinMode(ECHO_PIN, INPUT);

  // Find the center of the right joystick
  long sumX = 0, sumY = 0;
  for (int i = 0; i < 100; i++) {
    sumX += analogRead(jrx);
    sumY += analogRead(jry);
    delay(5);
  }
  centerX = sumX/100;
  centerY = sumY/100;

  // Initialize the starting Ultrasonic distance reading
  init_ultrasonic_distance = getDistanceCm();


  // Initialize buttons
  pinMode(jlsw, INPUT_PULLUP);
  pinMode(jrsw, INPUT_PULLUP);
  pinMode(lBtn1, INPUT_PULLUP);
}

void loop() {
  // Read the joysticks
  int xL = analogRead(jlx);
  int yL = analogRead(jly);
  int xR_raw = analogRead(jrx);
  int yR_raw = analogRead(jry);

  // Read the buttons
  bool jsL = !digitalRead(jlsw);
  bool jsR = !digitalRead(jrsw);
  bool b1 = !digitalRead(rBtn1);
  // bool b2 = !digitalRead(rBtn2);
  bool b3 = !digitalRead(lBtn1);
  bool b4 = !digitalRead(lBtn2);


  // Read the IMUs
  int16_t axL, ayL, azL, gxL, gyL, gzL, axR, ayR, azR, gxR, gyR, gzR;
  imu1.getAcceleration(&axL, &ayL, &azL);
  imu1.getRotation(&gxL, &gyL, &gzL);
  imu2.getAcceleration(&axR, &ayR, &azR);
  imu2.getRotation(&gxR, &gyR, &gzR);

  float axR_g = axR / 16384.0;
  float ayR_g = ayR / 16384.0;
  float azR_g = azR / 16384.0;
  float gxRDeg = gxR / 131.0;
  float gyRDeg = gyR / 131.0;
  float gzRDeg = gzR / 131.0;

  float axL_g = axL / 16384.0;
  float ayL_g = ayL / 16384.0;
  float azL_g = azL / 16384.0;
  float gxLDeg = gxL / 131.0;
  float gyLDeg = gyL / 131.0;
  float gzLDeg = gzL / 131.0;

  // Control the IMUs
  if (abs(gzRDeg) > 200) {
    lastClickTriggered = millis();
    if (!isClicking) {
      Mouse.press(MOUSE_LEFT);
      isClicking = true;
    }
    // Serial.println("MINE");
    // Mouse.press(MOUSE_LEFT);
  } else if (millis() - lastClickTriggered > clickTimeout) {
    if (isClicking) {
      Mouse.release(MOUSE_LEFT);
      isClicking = false;
    }
  }

  // if (abs(gzLDeg) > 200) {
  //   lastClickTriggeredR = millis();
  //   if (!isClickingR) {
  //     Mouse.press(MOUSE_RIGHT);
  //     isClickingR = true;
  //   }
  //   // Serial.println("MINE");
  //   // Mouse.press(MOUSE_LEFT);
  // } else if (millis() - lastClickTriggeredR > clickTimeoutR) {
  //   if (isClickingR) {
  //     Mouse.release(MOUSE_RIGHT);
  //     isClickingR = false;
  //   }
  // }


  // Left Joystick Logic (WASD)
     if (xL < 400) {
      Keyboard.press(119);
     } else if (xL > 600) {
      Keyboard.press(115);
     } else {
      Keyboard.release(119);
      Keyboard.release(115);
     }
     if (yL < 400){
      Keyboard.press(97);
     } else if (yL > 600) {
      Keyboard.press(100);
     } else {
      Keyboard.release(97);
      Keyboard.release(100);
     }



  // if (yL < 400){
  //   Serial.println("LEFT");
  //   Keyboard.write(97);
  // } else if (yL > 600) {
  //   Serial.println("RIGHT");
  //   Keyboard.write(100);
  // }
  // if (xL < 400){
  //   Serial.println("UP");
  //   Keyboard.write(119);
  // } else if (xL > 600) {
  //   Serial.println("DOWN");
  //   Keyboard.write(115);
  // }

  if (jsL == HIGH) {
    Serial.println("Click!");
    Keyboard.press(KEY_LEFT_SHIFT);
  } else {
    Keyboard.release(KEY_LEFT_SHIFT);
  }

  if (b3) {
    Keyboard.press(101);
  } else {
    Keyboard.release(101);
  }

  if (b1) {
    // Keyboard.press(KEY_SPACE);
  } else {
    // Keyboard.release(KEY_SPACE);
  }

  // // Apply drift correction for the right joystick
  // int xR = applyCenterDeadZone(xR_raw, centerX, centerDeadZone);
  // int yR = applyCenterDeadZone(yR_raw, centerY, centerDeadZone);
  // int xR_rot, yR_rot;
  // rotateJoystick(xR, yR, rotation_angle, xR_rot, yR_rot);

  // Read the buttons
  // bool jsL = !digitalRead(jlsw);
  // bool jsR = !digitalRead(jrsw);
  // bool b3 = !digitalRead(btn3);

  // Read ultrasonic distance
  float distance = getDistanceCm();

  // // Handle the ultrasonic for walking
  // updateGaitDetection(distance);

  // if (distance < 0) {
  //   Serial.println("Out of range");
  // } else {
  //   Serial.print("Distance: "); Serial.print(distance); Serial.println(" cm");
  // }
  // Serial.print("Walking? "); Serial.println( isWalking ? "YES" : "NO");

  // // Right Joystick (Mouse)
  // int xMove = (int)(xR_rot*sensitivity);
  // int yMove = (int)(yR_rot*sensitivity);
  // if (xMove != 0 || yMove != 0) {
  //   Mouse.move(xMove,yMove,0);
  // }




  // Print Data Stream
  // Serial.print(" JR: "); Serial.print(xR); Serial.print(","); Serial.print(yR);
  Serial.print(" JL: "); Serial.print(xL); Serial.print(","); Serial.print(yL);
  Serial.print(" | BTN3: "); Serial.print(b3); Serial.print(" | BTN4: "); Serial.println(b4);
  Serial.print(" IMU R: "); Serial.print(axR_g); Serial.print("|"); Serial.print(ayR_g); Serial.print("|"); Serial.print(azR_g); Serial.print("|"); Serial.print(gxRDeg); Serial.print("|"); Serial.print(gyRDeg); Serial.print("|"); Serial.println(gzRDeg);
  Serial.print(" IMU L: "); Serial.print(axL_g); Serial.print("|"); Serial.print(ayL_g); Serial.print("|"); Serial.print(azL_g); Serial.print("|"); Serial.print(gxLDeg); Serial.print("|"); Serial.print(gyLDeg); Serial.print("|"); Serial.println(gzLDeg);


  delay(15);
}