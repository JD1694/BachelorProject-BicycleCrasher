#include <WiFi.h>
#include <I2Cdev.h>
#include "MPU6050_6Axis_MotionApps20.h"
#include <Wire.h>

// Switch between evaluation modes
#define EVAL_DISCRETE
#define EVAL_CONTINUOUS

// Parameters to tune
float GYRO_EXP_DECLINE_FACTOR = 0.1;
// Threshold  to tune
float THRESHOLD_SMOOTH_GYRO = 1;

// MPU control/status vars
bool dmpReady = false;  // set true if DMP init was successful
uint8_t mpuIntStatus;   // holds actual interrupt status byte from MPU
uint8_t devStatus;      // return status after each device operation (0 = success, !0 = error)
uint16_t packetSize;    // expected DMP packet size (default is 42 bytes)
uint16_t fifoCount;     // count of all bytes currently in FIFO
uint8_t fifoBuffer[64]; // FIFO storage buffer

// Data from sensor
Quaternion q;           // [w, x, y, z]         quaternion container
VectorInt16 aa;         // [x, y, z]            accel sensor measurements
VectorInt16 gy;         // [x, y, z]            gyro sensor measurements
VectorInt16 aaReal;     // [x, y, z]            gravity-free accel sensor measurements
VectorInt16 aaWorld;    // [x, y, z]            world-frame accel sensor measurements
VectorFloat gravity;    // [x, y, z]            gravity vector
float euler[3];         // [psi, theta, phi]    Euler angle container
float ypr[3];           // [yaw, pitch, roll]   yaw/pitch/roll container and gravity vector

// Further processed data
VectorInt16 accelIntegral;
VectorFloat gyroSmoothend;

// constants to connect to web
const char *ssid = "myGate";
const char *password = "192837465";

// constants to connect to server
const uint16_t port = 5555;
const char *host = "192.168.137.1"; //"192.168.0.207";

// return from evaluation
bool crashYesNo = false;
float crashPropability = 0.0;

MPU6050 mpu(0x68);


void setup() {
  Serial.begin(115200);
  while (!Serial);

  // initialize device
  Serial.println(F("Initializing I2C devices..."));
  mpu.initialize();

  // verify connection
  Serial.println(F("Testing device connections..."));
  Serial.println(mpu.testConnection() ? F("MPU6050 connection successful") : F("MPU6050 connection failed"));

  // load and configure the DMP
  Serial.println(F("Initializing DMP..."));
  devStatus = mpu.dmpInitialize();

  // gyro offsets here, scaled for min sensitivity
  mpu.setXGyroOffset(-56); //(220);
  mpu.setYGyroOffset(39); //(76);
  mpu.setZGyroOffset(-15); //(-85);
  mpu.setZAccelOffset(2925); //(1788);

  // make sure it worked (returns 0 if so)
  if (devStatus == 0) {
    // Calibration Time: generate offsets and calibrate our MPU6050
    mpu.CalibrateAccel(6);
    mpu.CalibrateGyro(6);
    mpu.PrintActiveOffsets();
    // turn on the DMP, now that it's ready
    Serial.println(F("Enabling DMP..."));
    mpu.setDMPEnabled(true);

    // set our DMP Ready flag so the main loop() function knows it's okay to use it
    Serial.println(F("DMP ready! Waiting for first interrupt..."));
    dmpReady = true;

    // get expected DMP packet size for later comparison
    packetSize = mpu.dmpGetFIFOPacketSize();
  } else {
    // ERROR!
    // 1 = initial memory load failed
    // 2 = DMP configuration updates failed
    // (if it's going to break, usually the code will be 1)
    Serial.print(F("DMP Initialization failed (code "));
    Serial.print(devStatus);
    Serial.println(F(")"));
  }

  // connect to WiFi
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.println("...");
  }

  Serial.print("WiFi connected with IP: ");
  Serial.println(WiFi.localIP());
}

void loop() {
  // Get sensor readings
  fetchCoordinates();
  
  // Prepare data, filter and convert to get more measurable input
  prepareData();
  
  // Evaluate inputs
  #ifdef EVAL_DISCRETE
    crashYesNo = evalDiscrete();
    if (crashYesNo)
      {crashPropability = 1.0;}
    else
      {crashPropability = 0.0;}
  #endif
  #ifdef EVAL_CONTINUOUS
    crashPropability = evalContinuous();
  #endif

  // Reaction
  if (crashPropability > 0.5) {
    output(crashPropability);
  }
    
}

void fetchCoordinates() {
  // Get sensor readings from IMU
  mpu.dmpGetQuaternion(&q, fifoBuffer);
  mpu.dmpGetGravity(&gravity, &q);
  mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);
  // display real acceleration, adjusted to remove gravity
  mpu.dmpGetAccel(&aa, fifoBuffer);
  mpu.dmpGetLinearAccel(&aaReal, &aa, &gravity);
}

void prepareData() {
  /* Prepare data, filter and convert to get more measurable input */

  // add new readings to collection (FIFO?)
  //// fifo_aaReal.add(aaReal); 
  //// last_aaReal = fifo_aaReal.pop();
  
  // integrate acceleration over the last fraction of a second: add newest, remove oldest value
  /*
  accelIntegral.x = accelIntegral.x + aaReal.x - last_aaReal.x;
  accelIntegral.y = accelIntegral.y + aaReal.y - last_aaReal.y;
  accelIntegral.z = accelIntegral.z + aaReal.z - last_aaReal.z;
  */
  
  // Filter gyro data (exponetial decline as time proceeds)
  gyroSmoothend.x = gyroSmoothend.x*(1-GYRO_EXP_DECLINE_FACTOR) + gy.x*GYRO_EXP_DECLINE_FACTOR;
  gyroSmoothend.y = gyroSmoothend.y*(1-GYRO_EXP_DECLINE_FACTOR) + gy.y*GYRO_EXP_DECLINE_FACTOR;
  gyroSmoothend.z = gyroSmoothend.z*(1-GYRO_EXP_DECLINE_FACTOR) + gy.z*GYRO_EXP_DECLINE_FACTOR;
}

bool evalDiscrete() {
  /* Evaluate inputs into discrete categories: Crash and Safe */
  if (abs(gyroSmoothend.x) > THRESHOLD_SMOOTH_GYRO){
    return true;
  }
  // ...

  // no Trigger found
  return false;
}

float evalContinuous() {
  /* Evaluate inputs into continuous categories: Propability of Crash */
  //THRESHOLD_SMOOTH_GYRO - abs(gyroSmoothend.x)
  return 0.0;
}

void output(float crashPropability) {
  /* Reaction if a crash is detected */
  Serial.print("Crash detected having propability: ");
  Serial.println(crashPropability*100);
}
