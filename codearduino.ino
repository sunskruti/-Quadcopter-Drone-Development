#include <Wire.h>
#include <Servo.h>

Servo motorFL, motorFR, motorBL, motorBR;


const int MPU_addr = 0x68;
int16_t GyX, GyY, GyZ;


const int throttlePin = A0;
const int pitchPin = A1;
const int rollPin = A2;


float Kp = 1.5, Ki = 0.05, Kd = 0.1;
float pitchError, rollError, lastPitchError, lastRollError;
float pitchI = 0, rollI = 0;

unsigned long lastSignalTime = 0;
int failsafeTimeout = 1000;  // ms

void setup() {
  Wire.begin();
  Wire.beginTransmission(MPU_addr);
  Wire.write(0x6B);
  Wire.write(0);
  Wire.endTransmission(true);

  motorFL.attach(3);
  motorFR.attach(5);
  motorBL.attach(6);
  motorBR.attach(9);

  pinMode(throttlePin, INPUT);
  pinMode(pitchPin, INPUT);
  pinMode(rollPin, INPUT);

  armMotors();
}

void loop() {
  int throttle = pulseIn(throttlePin, HIGH, 25000);
  int pitchInput = pulseIn(pitchPin, HIGH, 25000);
  int rollInput = pulseIn(rollPin, HIGH, 25000);


  if (throttle < 1000) {
    stopMotors();
    return;
  }

  readMPU();

 
  pitchError = GyY / 131.0 - map(pitchInput, 1000, 2000, -10, 10);
  rollError = GyX / 131.0 - map(rollInput, 1000, 2000, -10, 10);


  pitchI += pitchError;
  rollI += rollError;

  float pitchD = pitchError - lastPitchError;
  float rollD = rollError - lastRollError;

  float pitchAdjust = Kp * pitchError + Ki * pitchI + Kd * pitchD;
  float rollAdjust = Kp * rollError + Ki * rollI + Kd * rollD;

  lastPitchError = pitchError;
  lastRollError = rollError;


  int baseSpeed = map(throttle, 1000, 2000, 1100, 1700);

  int fl = baseSpeed - pitchAdjust - rollAdjust;
  int fr = baseSpeed - pitchAdjust + rollAdjust;
  int bl = baseSpeed + pitchAdjust - rollAdjust;
  int br = baseSpeed + pitchAdjust + rollAdjust;

  fl = constrain(fl, 1000, 2000);
  fr = constrain(fr, 1000, 2000);
  bl = constrain(bl, 1000, 2000);
  br = constrain(br, 1000, 2000);

  motorFL.writeMicroseconds(fl);
  motorFR.writeMicroseconds(fr);
  motorBL.writeMicroseconds(bl);
  motorBR.writeMicroseconds(br);

  delay(10);
}


void readMPU() {
  Wire.beginTransmission(MPU_addr);
  Wire.write(0x43);  
  Wire.endTransmission(false);
  Wire.requestFrom(MPU_addr, 6, true);
  GyX = Wire.read() << 8 | Wire.read();
  GyY = Wire.read() << 8 | Wire.read();
  GyZ = Wire.read() << 8 | Wire.read();
}


void armMotors() {
  for (int i = 0; i < 3; i++) {
    motorFL.writeMicroseconds(1000);
    motorFR.writeMicroseconds(1000);
    motorBL.writeMicroseconds(1000);
    motorBR.writeMicroseconds(1000);
    delay(1000);
  }
}


void stopMotors() {
  motorFL.writeMicroseconds(1000);
  motorFR.writeMicroseconds(1000);
  motorBL.writeMicroseconds(1000);
  motorBR.writeMicroseconds(1000);
}
