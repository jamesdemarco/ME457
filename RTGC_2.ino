#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps20.h"
#include <Kalman.h> // Kalman Filter Library
#include <Wire.h>
#include <SPI.h>
#include <nRF24L01.h>
#include <RF24.h>

MPU6050 mpu;
Kalman kalmanX; // Kalman instances for angle estimation
Kalman kalmanY;

RF24 radio(9, 10); // CE, CSN
const byte address[6] = "00001";

#define INTERRUPT_PIN 2
#define SENSOR_PIN A2
#define THRESHOLD_HIGH 300 // Adjust based on testing
#define THRESHOLD_LOW 50   // Adjust based on testing

bool dmpReady = false;
uint8_t mpuIntStatus;
uint16_t packetSize;
uint8_t fifoBuffer[64];

Quaternion q;
VectorFloat gravity;
float ypr[3];

double accX, accY, accZ;
double gyroX, gyroY, gyroZ;
double kalAngleX, kalAngleY;
uint32_t timer;

volatile bool mpuInterrupt = false;
void dmpDataReady() {
  mpuInterrupt = true;
}

struct AnglePackage {
  float angleX;
  float angleY;
  uint32_t timestamp; // Timestamp in milliseconds
  bool isLocked; 
};


// Pressure sensor variables
unsigned long debounceDelay = 200;
unsigned long timeWindow = 1000; // Adjustable based on user preference
unsigned long lastPressTime = 0;
unsigned long patternStartTime = 0;
unsigned long timeStamps[3] = {0, 0, 0};
int signalCount = 0;
bool waitingForPattern = false;

// State machine states
enum State { IDLE, TRANSMITTING, LOCKED };
State currentState = IDLE;

void setup() {
  Serial.begin(115200);
  Wire.begin();
  mpu.initialize();
  pinMode(INTERRUPT_PIN, INPUT);
  pinMode(SENSOR_PIN, INPUT);
  radio.begin();
  radio.openWritingPipe(address);
  radio.setPALevel(RF24_PA_MIN);
  radio.stopListening();

  if (!mpu.testConnection()) {
    Serial.println(F("MPU6050 connection failed"));
    while (1);
  }

  uint8_t devStatus = mpu.dmpInitialize(); // Declare devStatus as uint8_t
  if (devStatus == 0) {
    mpu.setXGyroOffset(0);
    mpu.setYGyroOffset(0);
    mpu.setZGyroOffset(0);
    mpu.setXAccelOffset(0);
    mpu.setYAccelOffset(0);
    mpu.setZAccelOffset(0);
    mpu.CalibrateAccel(6);
    mpu.CalibrateGyro(6);
    mpu.setDMPEnabled(true);
    attachInterrupt(digitalPinToInterrupt(INTERRUPT_PIN), dmpDataReady, RISING);
    mpuIntStatus = mpu.getIntStatus();
    dmpReady = true;
    packetSize = mpu.dmpGetFIFOPacketSize();
  } else {
    Serial.print(F("DMP Initialization failed (code "));
    Serial.print(devStatus);
    Serial.println(F(")"));
  }

  // Initialize Kalman filter angles
  while (!readRawData());
  timer = micros();
  kalmanX.setAngle(atan2(accY, accZ) * RAD_TO_DEG);
  kalmanY.setAngle(atan(-accX / sqrt(accY * accY + accZ * accZ)) * RAD_TO_DEG);
}

void loop() {
  // Always monitor the pressure sensor in all states
  monitorPressureSensor();

  // Handle current state
  switch (currentState) {
    case IDLE:
      // Remain idle, waiting for a valid pressure pattern
      Serial.println("In Idle State");
      break;
    case TRANSMITTING:
      // Continue transmitting IMU data and monitor pressure sensor
      transmitIMUData();
      monitorPressureSensor(); // Check for state transitions
      break;
    case LOCKED:
      // Send lock message and monitor pressure sensor
      sendOrientationLockMessage();
      monitorPressureSensor(); // Check for state transitions
      break;
  }
}


void monitorPressureSensor() {
  static bool lastState = LOW;
  int sensorValue = analogRead(SENSOR_PIN);
  bool currentStatePressure;

  // Determine HIGH/LOW based on thresholds
  if (sensorValue > THRESHOLD_HIGH) {
    currentStatePressure = HIGH;
  } else if (sensorValue < THRESHOLD_LOW) {
    currentStatePressure = LOW;
  } else {
    return; // Ignore values in the ambiguous range
  }

  // Debug: Print sensor state and value
  Serial.print("Pressure Sensor Value: ");
  Serial.print(sensorValue);
  Serial.print(" -> State: ");
  Serial.println(currentStatePressure ? "HIGH" : "LOW");

  if (currentStatePressure == HIGH && lastState == LOW && millis() - lastPressTime > debounceDelay) {
    // Detected a valid rising edge (HIGH signal)
    lastPressTime = millis();
    Serial.println("Valid HIGH detected");

    if (!waitingForPattern) {
      waitingForPattern = true;
      patternStartTime = millis();
      signalCount = 0; // Reset signal count for the new pattern
      Serial.println("Starting new pattern detection");
    }

    shiftAndStore(millis()); // Update timestamps
    signalCount++;
    Serial.print("Signal count: ");
    Serial.println(signalCount);
  }

  lastState = currentStatePressure;

  // Check if the timeWindow has elapsed and decide the pattern
  if (waitingForPattern && millis() - patternStartTime > timeWindow) {
    if (signalCount == 1) {
      Serial.println("Switching to TRANSMITTING state");
      currentState = TRANSMITTING;
    } else if (signalCount == 2) {
      Serial.println("Switching to IDLE state");
      currentState = IDLE; // Stop transmitting
    } else if (signalCount == 3) {
      Serial.println("Switching to LOCKED state");
      currentState = LOCKED; // Send Orientation Lock
    } else {
      Serial.println("No valid pattern detected, remaining in IDLE");
    }

    // Reset after decision
    resetPattern();
  }
}


void transmitIMUData() {
  Serial.println("Received HIGH-HIGH : In TRANSMITTING state");

  if (!dmpReady) return;

  if (mpu.dmpGetCurrentFIFOPacket(fifoBuffer)) {
    mpu.dmpGetQuaternion(&q, fifoBuffer);
    mpu.dmpGetGravity(&gravity, &q);
    mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);

    double dt = (double)(micros() - timer) / 1000000;
    timer = micros();

    if (readRawData()) {
      float angleY = (115 - (ypr[2] * 180 / M_PI));
      float angleX = ((ypr[1] * 180 / M_PI)+95);

      AnglePackage angles;
      angles.angleX = angleX;
      angles.angleY = angleY;
      angles.timestamp = millis(); // Add timestamp
      angles.isLocked = false; // Mark as regular angle package

      radio.write(&angles, sizeof(angles));
      Serial.print("X: ");
      Serial.print(angles.angleX);
      Serial.print(" Y: ");
      Serial.println(angles.angleY);
      Serial.println("    Data Successfully Transmitted");
    }
  }

  // Allow monitorPressureSensor to detect transitions
  if (currentState != TRANSMITTING) {
    Serial.println("Exiting TRANSMITTING state");
  }
}

void sendOrientationLockMessage() {
  Serial.println("In LOCKED state");

  // Gather a single set of angles
  if (mpu.dmpGetCurrentFIFOPacket(fifoBuffer)) {
    mpu.dmpGetQuaternion(&q, fifoBuffer);
    mpu.dmpGetGravity(&gravity, &q);
    mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);

    float angleX = (105 - (ypr[2] * 180 / M_PI));
    float angleY = (83 - (ypr[1] * 180 / M_PI));

    // Create a lock angle package
    AnglePackage lockAngles;
    lockAngles.angleX = angleX;
    lockAngles.angleY = angleY;
    lockAngles.isLocked = true; // Mark as a lock angle package

    // Send the lock angle package wirelessly
    radio.write(&lockAngles, sizeof(lockAngles));
    Serial.print("Sent Lock Angles -> X: ");
    Serial.print(lockAngles.angleX);
    Serial.print(" Y: ");
    Serial.println(lockAngles.angleY);
  }

  currentState = IDLE; // Return to idle state after sending
}

void shiftAndStore(unsigned long currentTime) {
  for (int i = 2; i > 0; i--) {
    timeStamps[i] = timeStamps[i - 1];
  }
  timeStamps[0] = currentTime;
}

void resetPattern() {
  signalCount = 0;
  waitingForPattern = false;
  for (int i = 0; i < 3; i++) {
    timeStamps[i] = 0;
  }
}

bool readRawData() {
  int16_t rawAccX, rawAccY, rawAccZ;
  int16_t rawGyroX, rawGyroY, rawGyroZ;
  mpu.getMotion6(&rawAccX, &rawAccY, &rawAccZ, &rawGyroX, &rawGyroY, &rawGyroZ);

  accX = rawAccX / 16384.0;
  accY = rawAccY / 16384.0;
  accZ = rawAccZ / 16384.0;
  gyroX = rawGyroX / 131.0;
  gyroY = rawGyroY / 131.0;
  gyroZ = rawGyroZ / 131.0;

  return true;
}
