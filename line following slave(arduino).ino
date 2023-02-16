#include <Wire.h>

#define SLAVE_ADDRESS 0x04

#define LEFT_MOTOR_PIN 3
#define RIGHT_MOTOR_PIN 5
#define SERVO_PIN 9

int leftMotorSpeed = 0;
int rightMotorSpeed = 0;
int servoAngle = 90;

void setup() {
  pinMode(LEFT_MOTOR_PIN, OUTPUT);
  pinMode(RIGHT_MOTOR_PIN, OUTPUT);
  pinMode(SERVO_PIN, OUTPUT);

  Wire.begin(SLAVE_ADDRESS);
  Wire.onReceive(receiveEvent);
}

void loop() {
  // do nothing
}

void receiveEvent(int bytesReceived) {
  while (Wire.available() < bytesReceived) {
    // wait for all the bytes to arrive
  }

  leftMotorSpeed = Wire.read();
  rightMotorSpeed = Wire.read();
  servoAngle = Wire.read();

  updateMotorSpeeds();
  updateServoAngle();
}

void updateMotorSpeeds() {
  if (leftMotorSpeed < 0) {
    digitalWrite(LEFT_MOTOR_PIN, LOW);
    analogWrite(LEFT_MOTOR_PIN, -leftMotorSpeed * 2);
  } else {
    digitalWrite(LEFT_MOTOR_PIN, HIGH);
    analogWrite(LEFT_MOTOR_PIN, leftMotorSpeed * 2);
  }

  if (rightMotorSpeed < 0) {
    digitalWrite(RIGHT_MOTOR_PIN, LOW);
    analogWrite(RIGHT_MOTOR_PIN, -rightMotorSpeed * 2);
  } else {
    digitalWrite(RIGHT_MOTOR_PIN, HIGH);
    analogWrite(RIGHT_MOTOR_PIN, rightMotorSpeed * 2);
  }
}

void updateServoAngle() {
  if (servoAngle < 0) {
    servoAngle = 0;
  } else if (servoAngle > 180) {
    servoAngle = 180;
  }
  analogWrite(SERVO_PIN, servoAngle);
}
