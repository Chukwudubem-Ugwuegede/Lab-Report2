#include <Wire.h>
#define SLAVE_ADDRESS  0x04

#define KP 0.5
#define KI 0.1
#define KD 0.2
#define NUM_SENSORS 6
int sensorPins[NUM_SENSORS] = {A0, A1, A2, A3, A4, A5};
int weights[NUM_SENSORS] = {1, 2, 3, 3, 2, 1}; // weights for weighted average
int lineSensorValues[NUM_SENSORS];


int irValues[6];
int prevError = 0;
int integral = 0;

void setup() {
  Wire.begin(I2C_SLAVE_ADDR);   // join i2c bus #4 - on the Arduino NANO the default I2C pins are A4 (SDA), A5 (SCL)
  Wire.onReceive(receiveEvent); // register event

  for (int i = 0; i < 6; i++) {
    pinMode(sensorPins[i], INPUT);
  }
}

void loop() {
  

  int error = getWeightedError(); // calculate error using weighted average;
  

  // Compute the PID output
  int derivative = error - prevError;
  integral += error;
  int pidOutput = KP * error + KI * integral + KD * derivative;

  // Update the motor speeds and servo angle
  int leftSpeed = 127 - pidOutput;
  int rightSpeed = 127 + pidOutput;
  int servoAngle = 90 - pidOutput / 2;

  Wire.beginTransmission(SLAVE_ADDRESS);
  Wire.write(leftSpeed);
  Wire.write(rightSpeed);
  Wire.write(servoAngle);
  Wire.endTransmission();


  // Remember the previous error
  prevError = error;
  
  delay(10);
  
// calculate the weighted average of sensor readings
int getWeightedError() {
  int sum = 0;
  int weightedSum = 0;
  for (int i = 0; i < NUM_SENSORS; i++) {
    lineSensorValues[i] = analogRead(sensorPins[i]);
    sum += lineSensorValues[i];
    weightedSum += lineSensorValues[i] * weights[i];
  }
  return (weightedSum * NUM_SENSORS / sum) - (NUM_SENSORS / 2);
}

// PID control loop

void loop() {
  int error = getWeightedError(); // calculate error using weighted average
  int control = computeControlOutput(error); // compute control output using PID algorithm
  setMotorSpeeds(control); // adjust motor speeds based on control output
}

}
