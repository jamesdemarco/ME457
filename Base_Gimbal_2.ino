#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps20.h"
#include <Kalman.h> // Kalman Filter Library
#include <Wire.h>
#include <Servo.h>
#include <SPI.h>
#include <nRF24L01.h>
#include <RF24.h>

MPU6050 mpu;
Kalman kalmanX; // Kalman instances for angle estimation
Kalman kalmanY;

RF24 radio(9, 10); // CE, CSN
const byte address[6] = "00001";

struct AnglePackage {
  float angleX;
  float angleY;
  uint32_t timestamp; // Timestamp in milliseconds
  bool isLocked; 
};

AnglePackage angles;

#define INTERRUPT_PIN 2
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
unsigned long lastWirelessTime = 0;
unsigned long wirelessTimeout = 1000; // 1 second timeout

volatile bool mpuInterrupt = false;
void dmpDataReady() {
  mpuInterrupt = true;
}

Servo servo1; // X
Servo servo2; // Y

// State machine states
enum State { USING_IMU, USING_WIRELESS };
State currentState = USING_IMU;

void setup() {
  servo1.attach(3);
  servo2.attach(2);
  Serial.begin(115200);
  Wire.begin();
  mpu.initialize();
  pinMode(INTERRUPT_PIN, INPUT);
  radio.begin();
  radio.openReadingPipe(0, address);
  radio.setPALevel(RF24_PA_MIN);
  radio.startListening();

  if (!mpu.testConnection()) {
    Serial.println(F("MPU6050 connection failed"));
    while (1);
  }

  uint8_t devStatus = mpu.dmpInitialize(); // Declare devStatus as uint8_t
  if (devStatus == 0) {
    mpu.setXGyroOffset(-223);
    mpu.setYGyroOffset(17);
    mpu.setZGyroOffset(50);
    mpu.setXAccelOffset(2569);
    mpu.setYAccelOffset(-2470);
    mpu.setZAccelOffset(1063);
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
  if (!dmpReady) return;

  if (radio.available()) {
    radio.read(&angles, sizeof(angles));
    lastWirelessTime = millis();

    if (angles.isLocked) {
      lockServos();
      return; // Skip other updates if lock command received
    }

    currentState = USING_WIRELESS;
  }

  if (millis() - lastWirelessTime > wirelessTimeout) {
    currentState = USING_IMU;
  }

  if (currentState == USING_WIRELESS) {
    Serial.println("Wireless control");
    rtgc();
    
  } else {
    Serial.println("Regualar Gimbal");
    baseGimbal();
    
  }
}

void rtgc() {
  uint32_t sendTimestamp = millis(); // Record the timestamp when servo commands are sent

  servo1.write(angles.angleX); // X
  servo2.write(angles.angleY); // Y

}

void baseGimbal() {
  if (mpu.dmpGetCurrentFIFOPacket(fifoBuffer)) {
    mpu.dmpGetQuaternion(&q, fifoBuffer);
    mpu.dmpGetGravity(&gravity, &q);
    mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);

    float angle1 = (95 - ypr[2] * 180 / M_PI);
    float angle2 = (115 - ypr[1] * 180 / M_PI);

    if (angle1 < 20) angle1 = 20;
    if (angle2 < 5) angle2 = 5;
    if (angle2 > 143) angle2 = 143;

    servo1.write(angle1); // X
    servo2.write(angle2); // Y
  }
}

void lockServos() {
  servo1.write(angles.angleX); // Lock X
  servo2.write(angles.angleY); // Lock Y
  //Serial.println("Servos locked to specified angles.");
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
