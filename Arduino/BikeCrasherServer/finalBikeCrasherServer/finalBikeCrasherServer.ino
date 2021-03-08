#include <WiFi.h>
#include <WiFiClient.h>
#include <WiFiAP.h>
#include <Wire.h>
#include <I2Cdev.h>
#include <MPU6050_6Axis_MotionApps20.h>
#include "Wire.h"


float threshholds[] = { 50.41649852, 
                        312.41631607, 
                        941.86966547, 
                        489.9528661, 
                        596.17546761, 
                        861.43288347, 
                        82.45992038, 
                        93.90663596, 
                        628.08448713, 
                        688.82112735, 
                        638.62796796, 
                        255.40699726, 
                        532.64472836 };
float THRESHOLD_SMOOTH_GYRO_X = threshholds[1];
float THRESHOLD_SMOOTH_GYRO_Y = threshholds[2];
float THRESHOLD_SMOOTH_GYRO_Z = threshholds[3];
float THRESHOLD_YAW = threshholds[4];
float THRESHOLD_PITCH = threshholds[5];
float THRESHOLD_ROLL = threshholds[6];
float THRESHOLD_INT_ACCEL_X = threshholds[7];
float THRESHOLD_INT_ACCEL_Y = threshholds[8];
float THRESHOLD_INT_ACCEL_Z = threshholds[9];
float THRESHOLD_INT_GRAVITY_X = threshholds[10];
float THRESHOLD_INT_GRAVITY_Y = threshholds[11];
float THRESHOLD_INT_GRAVITY_Z = threshholds[12];
float COMMON_GRAVITY_Z = threshholds[13];
float THRESHOLD_ACCEL_DELTA = threshholds[14];


// Discrete Step counter
float  stepCount;
// continuous calc variables
float  ypr_eval;
float  gyro_eval;
int    accel_eval;
int    gravity_eval;
double delta_x;
double delta_y;
double delta_z;
double former_x = 0;
double former_y = 0;
double former_z = 0;
double accel_delta;
double continuous_rating = 0.0;
double normalized_continuous_rating = 0.0;

// Further processed data
VectorInt16 accelIntegral;
VectorInt16 accelGravityIntegral;
VectorFloat gyroSmoothend;
const float MOVING_AVERAGE_DECLINE = 0.1;

// MPU control/status vars
bool dmpReady = false;  // set true if DMP init was successful
uint8_t mpuIntStatus;   // holds actual interrupt status byte from MPU
uint8_t devStatus;      // return status after each device operation (0 = success, !0 = error)
uint16_t packetSize;    // expected DMP packet size (default is 42 bytes)
uint16_t fifoCount;     // count of all bytes currently in FIFO
uint8_t fifoBuffer[64]; // FIFO storage buffer

VectorInt16 last_aaReal;
VectorInt16 last_aa;

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
float rotateZAngle = 0.471239;   // Angle to rotate sensor coord sys into fixed coord sys --> found CALIBRATION Value: 27 degrees
VectorFloat aa_rot;         // [x, y, z]            accel sensor measurements
VectorFloat gy_rot;         // [x, y, z]            gyro sensor measurements
VectorFloat aaReal_rot;     // [x, y, z]            gravity-free accel sensor measurements
float ypr_rot[3];           // [yaw, pitch, roll]   yaw/pitch/roll container and gravity vector
VectorFloat rotateZreturnVal;

float crashPropability;
String crashCause = "";

const char *ssid = "TestAP";
const char *pass = "passwort";

MPU6050 mpu(0x68);
WiFiServer server(80);


void setup() {

  Wire.begin();
  Wire.setClock(400000);

  //start serialmonitor
  Serial.begin(115200);
  while (!Serial);
  Serial.println();

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


  //start access point
  Serial.println("Accesspoint wird konfiguriert");
  WiFi.softAP(ssid, pass);

  Serial.print("IP: ");
  Serial.println(WiFi.softAPIP());
  server.begin();
  Serial.println("Server ready");
}


void loop() {
  // check if DPM is ready and get latest packet (also used as check)
  if (!dmpReady) return;
  if (!mpu.dmpGetCurrentFIFOPacket(fifoBuffer)) return;
  
  WiFiClient client = server.available();
  // Get sensor readings
  getSensorReadings();
  // Prepare data, filter and convert to get more measurable input
  prepareData();

  // Evaluate inputs
  crashPropability = evalDiscreteSteps();
  Serial.println(crashPropability);

  if (client) {                     //established connection
    Serial.println("Connected client");
    String currentLine = "";         //holds incoming data
    while (client.connected()) {
      if (client.available()) {     //check for data from client
        char c = client.read();
        Serial.write(c);
        if (c == '\n') {
          if (currentLine.length() == 0) {
            client.println("HTTP/1.1 200 OK"); //send HTTP header for text
            client.println("Content-type:text/html");
            client.println();

            client.print("Crash Propability:");
            client.println(crashPropability);
            client.print("Crash Cause:");
            client.println(crashCause);
            /*client.println("Sensorwerte:");
            client.print("\n Gyro X: \n");
            client.println(gyroSmoothend.x);
            client.print("\n Gyro Y: \n");
            client.println(gyroSmoothend.y);
            client.print("\n Gyro Z: \n");
            client.println(gyroSmoothend.z);
            client.print("\n Y: \n");
            client.println(ypr[0]);
            client.print("\n P: \n");
            client.println(ypr[1]);
            client.print("\n R \n");
            client.println(ypr[2]);*/
            client.println("<a href=\"/\">Refresh</a>");
            client.println(); //end http response
            break;
          }
          else {
            currentLine = "";
          }
        } else if (c != '\r') {
          currentLine += c;
        }

      }
    }
    client.stop();
    Serial.println("Client disconnected");
  }
}


void getSensorReadings() {
  /* Get sensor readings from IMU */
  mpu.dmpGetQuaternion(&q, fifoBuffer);
  mpu.dmpGetGravity(&gravity, &q);
  mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);
  // real acceleration, adjusted to remove gravity
  mpu.dmpGetAccel(&aa, fifoBuffer);
  mpu.dmpGetLinearAccel(&aaReal, &aa, &gravity);
  mpu.dmpGetGyro(&gy, fifoBuffer);
}


void prepareData() {
  /* Prepare data, filter and convert to get more measurable input */
  
  // rotate used sensor data to new cood system
  rotateZ( rotateZAngle, aa, &aa_rot );
  rotateZ( rotateZAngle, aaReal, &aaReal_rot );
  rotateZ( rotateZAngle, gy, &gy_rot );
  //rotateZ( rotateZAngle, ypr, &ypr_rot[0] );

  // Filter rotated aaReal data (Exponential Moving Average)
  accelIntegral.x = accelIntegral.x * (1 - MOVING_AVERAGE_DECLINE) + aaReal_rot.x * MOVING_AVERAGE_DECLINE;
  accelIntegral.y = accelIntegral.y * (1 - MOVING_AVERAGE_DECLINE) + aaReal_rot.y * MOVING_AVERAGE_DECLINE;
  accelIntegral.z = accelIntegral.z * (1 - MOVING_AVERAGE_DECLINE) + aaReal_rot.z * MOVING_AVERAGE_DECLINE;

  // Filter rotated aa data (Exponential Moving Average)
  accelGravityIntegral.x = accelGravityIntegral.x * (1 - MOVING_AVERAGE_DECLINE) + aa_rot.x * MOVING_AVERAGE_DECLINE;
  accelGravityIntegral.y = accelGravityIntegral.y * (1 - MOVING_AVERAGE_DECLINE) + aa_rot.y * MOVING_AVERAGE_DECLINE;
  accelGravityIntegral.z = accelGravityIntegral.z * (1 - MOVING_AVERAGE_DECLINE) + aa_rot.z * MOVING_AVERAGE_DECLINE;

  // Filter gyro data (Exponential Moving Average)
  gyroSmoothend.x = gyroSmoothend.x * (1 - MOVING_AVERAGE_DECLINE) + gy_rot.x * MOVING_AVERAGE_DECLINE;
  gyroSmoothend.y = gyroSmoothend.y * (1 - MOVING_AVERAGE_DECLINE) + gy_rot.y * MOVING_AVERAGE_DECLINE;
  gyroSmoothend.z = gyroSmoothend.z * (1 - MOVING_AVERAGE_DECLINE) + gy_rot.z * MOVING_AVERAGE_DECLINE;
  // Difference in acceleration compared to moving average
  delta_x = former_x - accelIntegral.x;
  delta_y = former_y - accelIntegral.y;
  delta_z = former_z - accelIntegral.z;

  accel_delta = norm(delta_x, delta_y, delta_z);

  former_x = accelIntegral.x * MOVING_AVERAGE_DECLINE + former_x * (1 - MOVING_AVERAGE_DECLINE);
  former_y = accelIntegral.y * MOVING_AVERAGE_DECLINE + former_y * (1 - MOVING_AVERAGE_DECLINE);
  former_z = accelIntegral.z * MOVING_AVERAGE_DECLINE + former_z * (1 - MOVING_AVERAGE_DECLINE);
}


double norm(double a, double b, double c) {
  /* Calculates the euclidian norm of a vektor consisting of the 3 given values*/
  return sqrt( sq(delta_x) + sq(delta_y) + sq(delta_z) );
}


void rotateZ(float radAngle, VectorInt16 point, VectorFloat *rotatedPoint ) {

  VectorFloat returnVal;

  returnVal.x = cos(radAngle) * point.x - sin(radAngle) * point.y;
  returnVal.y = sin(radAngle) * point.x + cos(radAngle) * point.y;
  *rotatedPoint = returnVal;

  //delete &returnVal;

  return;
}


void rotateZ(float radAngle, float point[3], float *rotatedPoint ) {
  /*Rotate a point around the Z-Axis by the angle. Writes the rotated point to the pointer given*/
  * rotatedPoint      = cos(radAngle) * point[0] - sin(radAngle) * point[1];
  *(rotatedPoint + 1) = sin(radAngle) * point[0] + cos(radAngle) * point[1];
  *(rotatedPoint + 2) = point[2];
  return;
}


double evalDiscreteSteps() {
  /* Evaluate inputs into discrete Steps according to the number of thresholds triggered. Result is converted to Percent */
  crashCause = "";
  stepCount = 0
              /*
              // Absolute Angle
              + 1 * (abs(ypr[1] * 180 / M_PI) > THRESHOLD_PITCH * 45 / 100) ///// change to rad for more efficienty
              + 1 * (abs(ypr[2] * 180 / M_PI) > THRESHOLD_ROLL * 50 / 100)

              // Rate of rotation
              + 1 * (abs(gyroSmoothend.x) > THRESHOLD_SMOOTH_GYRO_X * 200 / 100)
              + 1 * (abs(gyroSmoothend.y) > THRESHOLD_SMOOTH_GYRO_Y * 200 / 100)
              + 1 * (abs(gyroSmoothend.z) > THRESHOLD_SMOOTH_GYRO_Z * 200 / 100)

              // Acceleration
              + 1 * (abs(accelIntegral.x) > THRESHOLD_INT_ACCEL_X * 10000 / 100)
              + 1 * (abs(accelIntegral.y) > THRESHOLD_INT_ACCEL_Y * 10000 / 100)
              + 1 * (abs(accelIntegral.z) > THRESHOLD_INT_ACCEL_Z * 10000 / 100)

              // Direction of Gravity and Momentum
              + 1 * (abs(accelGravityIntegral.x) > THRESHOLD_INT_GRAVITY_X * 10000 / 100)
              + 1 * (abs(accelGravityIntegral.y) > THRESHOLD_INT_GRAVITY_Y * 10000 / 100)

              // Change in Acceleration
              + 1 * (abs(accel_delta) > THRESHOLD_ACCEL_DELTA * 10000 / 100)
              ;*/
              // Absolute Angle
              + 1 * checkThreshold(abs(ypr[1] * 180 / M_PI), THRESHOLD_PITCH * 45 / 100, "THRESHOLD_PITCH")///// change to rad for more efficienty
              + 1 * checkThreshold(abs(ypr[2] * 180 / M_PI), THRESHOLD_ROLL * 50 / 100, "THRESHOLD_ROLL")

              // Rate of rotation
              + 1 * checkThreshold(abs(gyroSmoothend.x), THRESHOLD_SMOOTH_GYRO_X * 200 / 100, "THRESHOLD_SMOOTH_GYRO_X")
              + 1 * checkThreshold(abs(gyroSmoothend.y), THRESHOLD_SMOOTH_GYRO_Y * 200 / 100, "THRESHOLD_SMOOTH_GYRO_Y")
              + 1 * checkThreshold(abs(gyroSmoothend.z), THRESHOLD_SMOOTH_GYRO_Z * 200 / 100, "THRESHOLD_SMOOTH_GYRO_Z")

              // Acceleration
              + 1 * checkThreshold(abs(accelIntegral.x), THRESHOLD_INT_ACCEL_X * 10000 / 100, "THRESHOLD_INT_ACCEL_X")
              + 1 * checkThreshold(abs(accelIntegral.y), THRESHOLD_INT_ACCEL_Y * 10000 / 100, "THRESHOLD_INT_ACCEL_Y")
              + 1 * checkThreshold(abs(accelIntegral.z), THRESHOLD_INT_ACCEL_Z * 10000 / 100, "THRESHOLD_INT_ACCEL_Z")

              // Direction of Gravity and Momentum
              + 1 * checkThreshold(abs(accelGravityIntegral.x), THRESHOLD_INT_GRAVITY_X * 10000 / 100, "THRESHOLD_INT_GRAVITY_X")
              + 1 * checkThreshold(abs(accelGravityIntegral.y), THRESHOLD_INT_GRAVITY_Y * 10000 / 100, "THRESHOLD_INT_GRAVITY_Y")

              // Change in Acceleration
              + 1 * checkThreshold(abs(accel_delta), THRESHOLD_ACCEL_DELTA * 10000 / 100, "THRESHOLD_ACCEL_DELTA")
              ;

  return stepCount / 11;
}

bool checkThreshold(double liveData, double thresholdVal, String thresholdName){
  if (liveData > thresholdVal){
    Serial.print("Triggered: ");
    Serial.println(thresholdName);
    crashCause += thresholdName;
    crashCause += ", \n";
    return true;
  }
  else{
    return false;
  }
}
