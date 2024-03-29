#include <CircularBuffer.h> //https://github.com/rlogiacco/CircularBuffer
#include <WiFi.h>
#include <I2Cdev.h>
#include "MPU6050_6Axis_MotionApps20.h"
#include <Wire.h>


// Switch between evaluation modes
#define EVAL_DISCRETE
//#define EVAL_CONTINUOUS

// Parameters to tune (initial guess)
const float GYRO_EXP_DECLINE_FACTOR = 0.1;// (0.1)
const int FIFO_INT_LENGTH_BIG = 20;      // (20)
const int FIFO_INT_LENGTH_SHORT = 8;     // (8)
// Thresholds  to tune
const float THRESHOLD_SMOOTH_GYRO_X = 1; // (1)
const float THRESHOLD_SMOOTH_GYRO_Y = 1; // (1)
const float THRESHOLD_SMOOTH_GYRO_Z = 1; // (1)
const float THRESHOLD_YAW = 360;         // (360)
const float THRESHOLD_PITCH = 50;        // (50)
const float THRESHOLD_ROLL = 50;         // (50)
const int   THRESHOLD_INT_ACCEL_X = 50;  // (50)
const int   THRESHOLD_INT_ACCEL_Y = 50;  // (50)
const int   THRESHOLD_INT_ACCEL_Z = 50;  // (50)
const int   THRESHOLD_INT_GRAVITY_X = 1; // (1)
const int   THRESHOLD_INT_GRAVITY_Y = 1; // (1)
const int   THRESHOLD_INT_GRAVITY_Z = 1; // (1)
const int   COMMON_GRAVITY_Z = 1000;     // (1000)
// continuous calc variables
float  ypr_eval;
float  gyro_eval;
int    accel_eval;
int    gravity_eval;
double delta_x;
double delta_y;
double delta_z;
double continuous_rating = 0.0;
double normalized_continuous_rating = 0.0;
// hold past data to filter
CircularBuffer<VectorInt16,FIFO_INT_LENGTH_BIG> fifo_aaReal;
CircularBuffer<VectorInt16,FIFO_INT_LENGTH_BIG> fifo_aa;
VectorInt16 last_aaReal;
VectorInt16 last_aa;

// Further processed data
VectorInt16 accelIntegral;
VectorInt16 accelGravityIntegral;
VectorFloat gyroSmoothend;

// return from evaluation
bool   crashYesNo = false;
double crashPropability = 0.0;

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
// rotated to new coord sys
float rotateZAngle = 1.0;   // Angle to rotate sensor coord sys into fixed coord sys
VectorFloat aa_rot;         // [x, y, z]            accel sensor measurements
VectorFloat gy_rot;         // [x, y, z]            gyro sensor measurements
VectorFloat aaReal_rot;     // [x, y, z]            gravity-free accel sensor measurements
float ypr_rot[3];           // [yaw, pitch, roll]   yaw/pitch/roll container and gravity vector

// constants to connect to web
const char *ssid = "myGate";
const char *password = "192837465";

// constants to connect to server
const uint16_t port = 5555;
const char *host = "192.168.137.1"; //"192.168.0.207";

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
    mpu.setXGyroOffset(-38); // (-56); //(220);
    mpu.setYGyroOffset(35); // (39); //(76);
    mpu.setZGyroOffset(4); // (-15); //(-85);
    mpu.setXAccelOffset(-2576);
    mpu.setYAccelOffset(-4021);
    mpu.setZAccelOffset(2101); // (2925); //(1788);

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
  getSensorReadings();

  // Prepare data, filter and convert to get more measurable input
  prepareData();

  // Evaluate inputs
#ifdef EVAL_DISCRETE
  crashYesNo = evalDiscrete();
  if (crashYesNo)
  {
    crashPropability = 1.0;
  }
  else
  {
    crashPropability = 0.0;
  }
#endif

#ifdef EVAL_CONTINUOUS
  crashPropability = evalContinuous();
#endif

  // Reaction
  if (crashPropability > 0.5) {
    output(crashPropability);
  }

}

void getSensorReadings() {
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
  // rotate used sensor data to new cood system
  rotateZ( rotateZAngle, aa, &aa_rot );
  rotateZ( rotateZAngle, aaReal, &aaReal_rot );
  rotateZ( rotateZAngle, gy, &gy_rot );
  rotateZ( rotateZAngle, ypr, &ypr_rot[0] );


  // add new readings to collection (FIFO of length FIFO_INT_LENGTH_BIG) (Exponential Moving Average possible?)
  if (fifo_aaReal.isFull()){
    last_aaReal = fifo_aaReal.shift();
    // integrate acceleration over the last fraction of a second: add newest, remove oldest value
    accelIntegral.x = accelIntegral.x + aaReal.x - last_aaReal.x;
    accelIntegral.y = accelIntegral.y + aaReal.y - last_aaReal.y;
    accelIntegral.z = accelIntegral.z + aaReal.z - last_aaReal.z;
  }
  if (fifo_aa.isFull()){
    last_aa = fifo_aa.shift();    
    // Accel with Gravity
    accelGravityIntegral.x = accelGravityIntegral.x + aa.x - last_aa.x;
    accelGravityIntegral.y = accelGravityIntegral.y + aa.y - last_aa.y;
    accelGravityIntegral.z = accelGravityIntegral.z + aa.z - last_aa.z;
  }
  fifo_aaReal.push(aaReal);
  fifo_aa.push(aa);
  
  // Filter gyro data (Exponential Moving Average)
  gyroSmoothend.x = gyroSmoothend.x * (1 - GYRO_EXP_DECLINE_FACTOR) + gy.x * GYRO_EXP_DECLINE_FACTOR;
  gyroSmoothend.y = gyroSmoothend.y * (1 - GYRO_EXP_DECLINE_FACTOR) + gy.y * GYRO_EXP_DECLINE_FACTOR;
  gyroSmoothend.z = gyroSmoothend.z * (1 - GYRO_EXP_DECLINE_FACTOR) + gy.z * GYRO_EXP_DECLINE_FACTOR;
}

bool evalDiscrete() {
  /* Evaluate inputs into discrete categories: Crash and Safe */

  // Absolute Angle
  if ( abs(ypr[0] * 180 / M_PI) > THRESHOLD_YAW ///// evtl y-p-r vertauscht
       || abs(ypr[1] * 180 / M_PI) > THRESHOLD_PITCH ///// change to rad for more efficienty
       || abs(ypr[2] * 180 / M_PI) > THRESHOLD_ROLL) {
    return true;
  }
  // Rate of rotation
  if ( abs(gyroSmoothend.x) > THRESHOLD_SMOOTH_GYRO_X
       || abs(gyroSmoothend.y) > THRESHOLD_SMOOTH_GYRO_Y
       || abs(gyroSmoothend.z) > THRESHOLD_SMOOTH_GYRO_Z) {
    return true;
  }
  // Acceleration
  if ( abs(accelIntegral.x) > THRESHOLD_INT_ACCEL_X
       || abs(accelIntegral.y) > THRESHOLD_INT_ACCEL_Y
       || abs(accelIntegral.z) > THRESHOLD_INT_ACCEL_Z) {
    return true;
  }
  // Direction of Gravity and Momentum
  if ( abs(accelGravityIntegral.x) > THRESHOLD_INT_GRAVITY_X
       || abs(accelGravityIntegral.y) > THRESHOLD_INT_GRAVITY_Y
       || abs(accelGravityIntegral.z - COMMON_GRAVITY_Z) > THRESHOLD_INT_GRAVITY_Z) {
    return true;
  }
  // ...

  // no Trigger found
  return false;
}

double evalContinuous() {
  /* Evaluate inputs into continuous categories: Propability of Crash */
  /* Notes: square may not have methods in arg*/
  //THRESHOLD_SMOOTH_GYRO - abs(gyroSmoothend.x)

  // Absolute Angle
  delta_x = abs(ypr[0] * 180 / M_PI) - THRESHOLD_YAW; ///// evtl y-p-r vertauscht///// change to rad for more efficienty
  delta_y = abs(ypr[0] * 180 / M_PI) - THRESHOLD_PITCH;
  delta_z = abs(ypr[0] * 180 / M_PI) - THRESHOLD_ROLL;
  ypr_eval = norm(delta_x, delta_y, delta_z);

  // Rate of rotation
  delta_x = abs(gyroSmoothend.x) - THRESHOLD_SMOOTH_GYRO_X;
  delta_y = abs(gyroSmoothend.y) - THRESHOLD_SMOOTH_GYRO_Y;
  delta_z = abs(gyroSmoothend.z) - THRESHOLD_SMOOTH_GYRO_Z;
  gyro_eval = norm(delta_x, delta_y, delta_z);

  // Acceleration
  delta_x = abs(accelIntegral.x) > THRESHOLD_INT_ACCEL_X;
  delta_y = abs(accelIntegral.y) > THRESHOLD_INT_ACCEL_Y;
  delta_z = abs(accelIntegral.z) > THRESHOLD_INT_ACCEL_Z;
  accel_eval = norm(delta_x, delta_y, delta_z);

  // Direction of Gravity and Momentum
  delta_x = abs(accelGravityIntegral.x) > THRESHOLD_INT_GRAVITY_X;
  delta_y = abs(accelGravityIntegral.y) > THRESHOLD_INT_GRAVITY_Y;
  delta_z = abs(accelGravityIntegral.z - COMMON_GRAVITY_Z) > THRESHOLD_INT_GRAVITY_Z;
  gravity_eval = norm(delta_x, delta_y, delta_z);

  // combine and normalize
  continuous_rating = ypr_eval + gyro_eval + accel_eval + gravity_eval;
  normalized_continuous_rating = normalized_continuous_rating*0.999 + continuous_rating*0.001; // assumed as safe ride

  // convert to useable and capped percentage result
  return sigmoid_function(continuous_rating/normalized_continuous_rating - 1);
}

void output(float crashPropability) {
  /* Reaction if a crash is detected */
  Serial.print("Crash detected having propability: ");
  Serial.println(crashPropability * 100);
  // Send WIFI Distress signal to extentions
}

double norm(double a, double b, double c){
  /* Calculates the euclidian norm of a vektor consisting of the 3 given values*/
  return sqrt( sq(delta_x) + sq(delta_y) + sq(delta_z) );
}

double sigmoid_function(double x){
  /* 
   *  A fast implementation (without exp(-x)) of the Sigmoid activation curve, modified to return return percentages.
   *  Returns results between 0 (for very negative x) and 1 (for very positive x) 
   *  centered around x=0 where 0.5 is returned*/
   return x / (2*(1 + abs(x))) + 0.5;
}

void rotateZ(float radAngle, VectorInt16 point, VectorFloat *rotatedPoint ){
  VectorFloat returnVal;
  returnVal.x = cos(radAngle)*point.x - sin(radAngle)*point.y;
  returnVal.y = sin(radAngle)*point.x + cos(radAngle)*point.y;
  *rotatedPoint = returnVal;
  delete &returnVal;
  return;
}

void rotateZ(float radAngle, float point[3], float *rotatedPoint ){
  /*Rotate a point around the Z-Axis by the angle. Writes the rotated point to the pointer given*/
  *rotatedPoint     = cos(radAngle)*point[0] - sin(radAngle)*point[1];
  *(rotatedPoint+1) = sin(radAngle)*point[0] + cos(radAngle)*point[1];
  *(rotatedPoint+2) = point[2];
  return;
}
