#define PRINT_DEBUG_BUILD  // This is to print the MPU data on serial monitor to debug

#include <PID_v1.h>

#include <Servo.h>

 

// Required for MPU6050

#include "I2Cdev.h"

#include "MPU6050_6Axis_MotionApps20.h"

// Arduino Wire library is required if I2Cdev I2CDEV_ARDUINO_WIRE implementation

// is used in I2Cdev.h

#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE

  #include "Wire.h"

#endif

 

// MPU default I2C address is 0x68

MPU6050 mpu;

#define INTERRUPT_PIN 2  // Use pin 2 on Arduino Uno & most boards

 

// MPU control/status vars

bool dmpReady = false;  // Set true if DMP init was successful

uint8_t mpuIntStatus;   // Holds actual interrupt status byte from MPU

uint8_t devStatus;      // Return status after each device operation (0 = success, !0 = error)

uint16_t packetSize;    // Expected DMP packet size (default is 42 bytes)

uint16_t fifoCount;     // Count of all bytes currently in FIFO

uint8_t fifoBuffer[64]; // FIFO storage buffer

 

// Orientation/motion vars

Quaternion q;           // [w, x, y, z] quaternion container

VectorFloat gravity;    // [x, y, z] gravity vector

float ypr[3];           // [yaw, pitch, roll] yaw/pitch/roll container and gravity vector

VectorInt16 gy;         // [x, y, z] gyro sensor measurements

 

// PID variables for roll control

double setpointRollAngle = 0; // Zero degree setpoint for roll

double rollGyroAngle = 0;

double rollPIDOutput = 0;

 

// PID parameters

#define PID_ROLL_KP 20

#define PID_ROLL_KI 70

#define PID_ROLL_KD 1

 

PID rollPID(&rollGyroAngle, &rollPIDOutput, &setpointRollAngle, PID_ROLL_KP, PID_ROLL_KI, PID_ROLL_KD, DIRECT);

 

// Motor control pins

Servo servo1;

Servo servo2;

int motor1Pin1 = 10;

int motor2Pin2 = 11;

 

void setupPID()

{

    rollPID.SetOutputLimits(-255, 255);

    rollPID.SetMode(AUTOMATIC);

    rollPID.SetSampleTime(10); // 10 milliseconds sample time

}

 

void setupMotors()

{

    servo1.attach(motor1Pin1); // motor1Pin1 must be a PWM pin

    servo2.attach(motor2Pin2); // motor2Pin2 must be a PWM pin

    rotateMotor(0, 0); // Initialize servos to neutral position

}

 

void setupMPU()

{

    #if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE

        Wire.begin();

        Wire.setClock(400000); // 400kHz I2C clock

    #endif

 

    mpu.initialize();

    pinMode(INTERRUPT_PIN, INPUT);

    devStatus = mpu.dmpInitialize();

 

    // Set your own gyro offsets here, scaled for min sensitivity

    mpu.setXAccelOffset(-1630);

    mpu.setYAccelOffset(227);

    mpu.setZAccelOffset(3078);  

    mpu.setXGyroOffset(237);

    mpu.setYGyroOffset(-2);

    mpu.setZGyroOffset(154);

 

    if (devStatus == 0) {

        mpu.setDMPEnabled(true);

        mpuIntStatus = mpu.getIntStatus();

        dmpReady = true;

        packetSize = mpu.dmpGetFIFOPacketSize();

    } else {

        // ERROR handling code

    }

}

 

void setup()

{

    Serial.begin(115200); // Start serial communication at 115200 bauds

    setupMotors();

    setupMPU();

    setupPID();

}

 

void loop()

{

    if (!dmpReady) return;

 

    if (mpu.dmpGetCurrentFIFOPacket(fifoBuffer)) {

        mpu.dmpGetQuaternion(&q, fifoBuffer);

        mpu.dmpGetGravity(&gravity, &q);

        mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);

       

        rollGyroAngle = ypr[2] * 180/M_PI; // Convert roll angle to degrees

 

        rollPID.Compute();

 

        rotateMotor(rollPIDOutput, rollPIDOutput);

 

        //#ifdef PRINT_DEBUG_BUILD

        Serial.print("Roll Gyro Angle: ");

        Serial.println(rollGyroAngle);

        Serial.print("PID Output: ");

        Serial.println(rollPIDOutput);

        //#endiFf

    }

}

 

void rotateMotor(int speed1, int speed2) {

    int servoSpeed1 = map(speed1, -255, 255, 120, 60);

    int servoSpeed2 = map(speed2, -255, 255, 60, 120);

 

    servo1.write(servoSpeed1);

    servo2.write(servoSpeed2);

}
