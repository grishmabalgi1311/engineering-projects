#include <Servo.h>

Servo servoIndex;
Servo servoMiddle;

void setup() {
  Serial.begin(115200);

  servoIndex.attach(9);     // Index finger servo
  servoMiddle.attach(10);   // Middle finger servo

  // Start at zero
  servoIndex.write(0);
  servoMiddle.write(0);
}

void loop() {
  if (Serial.available()) {

    int indexAngle, middleAngle;
    String data = Serial.readStringUntil('\n');

    if (sscanf(data.c_str(), "%d %d", &indexAngle, &middleAngle) == 2) {

      indexAngle  = constrain(indexAngle, 0, 180);
      middleAngle = constrain(middleAngle, 0, 180);

      servoIndex.write(indexAngle);
      servoMiddle.write(middleAngle);
    }
  }
}
