#include <WiFi.h>
#include "time.h"
#include <I2Cdev.h>
#include "MPU6050_6Axis_MotionApps20.h"
#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
    #include "Wire.h"
#endif

// MPU control/status vars
bool dmpReady = false;  // set true if DMP init was successful
uint8_t mpuIntStatus;   // holds actual interrupt status byte from MPU
uint8_t devStatus;      // return status after each device operation (0 = success, !0 = error)
uint16_t packetSize;    // expected DMP packet size (default is 42 bytes)
uint16_t fifoCount;     // count of all bytes currently in FIFO
uint8_t fifoBuffer[64]; // FIFO storage buffer

Quaternion q;           // [w, x, y, z]         quaternion container
VectorInt16 aa;         // [x, y, z]            accel sensor measurements
VectorInt16 aaReal;     // [x, y, z]            gravity-free accel sensor measurements
VectorInt16 aaWorld;    // [x, y, z]            world-frame accel sensor measurements
VectorFloat gravity;    // [x, y, z]            gravity vector
float euler[3];         // [psi, theta, phi]    Euler angle container
float ypr[3];           // [yaw, pitch, roll]   yaw/pitch/roll container and gravity vector

// constants to connect to web
const char *ssid="myGate";
const char *password="192837465";

// constants to connect to server
const uint16_t port= 5555;
const char *host ="192.168.137.1";//"192.168.0.207";

// time from web
const char* ntpServer = "pool.ntp.org";
const long  gmtOffset_sec = 3600;
const int   daylightOffset_sec = 3600;
float       callsPerSec  = 10;
int         callsThisSec  = 0;
int         lastSecondNum = 0;

MPU6050 mpu(0x68);

void printLocalTime(WiFiClient client)
{
  struct tm timeinfo;
  if(!getLocalTime(&timeinfo)){
    client.print("Failed to obtain time");
    Serial.print("Failed to obtain time");
    return;
  }

  // count fractions of second for more precision
  callsThisSec += 1;
  // reset fraction at next second
  if (lastSecondNum != timeinfo.tm_sec){
    lastSecondNum = timeinfo.tm_sec;
    callsPerSec = callsPerSec*0.6 + callsThisSec*0.4;
    callsThisSec = 0;
  }

  // send
  client.print( &timeinfo, "%Y-%m-%d_%H-%M-%S" );
  Serial.print( &timeinfo, "%Y-%m-%d_%H-%M-%S" );
  if (callsThisSec <= callsPerSec){
    client.print(String(callsThisSec/callsPerSec).substring(1));
    Serial.print(String(callsThisSec/callsPerSec).substring(1));
  }
  else{
    client.print(".9999");
    Serial.print(".9999");
  }  
}

void setup() {
  
    #if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
        Wire.begin();
        Wire.setClock(400000); // 400kHz I2C clock. Comment this line if having compilation difficulties
    #elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
        Fastwire::setup(400, true);
    #endif

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

  //init and get the current time online
  configTime(gmtOffset_sec, daylightOffset_sec, ntpServer);

}

void loop() {
    // connect to server
    WiFiClient client;

    if (!client.connect(host, port)) {
        Serial.println("Connection to host failed");
        delay(1000);
        return;
    }
    Serial.println("Connected to server successful!");

    // repeat getting and sending data
    while (client.connected()) {
  
      if (!dmpReady) return;
      // read a packet from FIFO
      if (mpu.dmpGetCurrentFIFOPacket(fifoBuffer)) { // Get the Latest packet 
          
          // display Euler angles in degrees
          mpu.dmpGetQuaternion(&q, fifoBuffer);
          mpu.dmpGetGravity(&gravity, &q);
          mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);
          // display real acceleration, adjusted to remove gravity
          mpu.dmpGetAccel(&aa, fifoBuffer);
          mpu.dmpGetLinearAccel(&aaReal, &aa, &gravity);
    
          // output to server
          printLocalTime(client);
          client.print(", ");
          client.print("ypr, ");
          client.print(ypr[0] * 180/M_PI);
          client.print(", ");
          client.print(ypr[1] * 180/M_PI);
          client.print(", ");
          client.print(ypr[2] * 180/M_PI);
          client.print(", ");
    
          // output to Serial Monitior
          Serial.print(", ");
          Serial.print("ypr, ");
          Serial.print(ypr[0] * 180/M_PI);
          Serial.print(", ");
          Serial.print(ypr[1] * 180/M_PI);
          Serial.print(", ");
          Serial.print(ypr[2] * 180/M_PI);
          Serial.print(", ");
    
          // output to server
          client.print("areal, ");
          client.print(aaReal.x);
          client.print(", ");
          client.print(aaReal.y);
          client.print(", ");
          client.println(aaReal.z);
          
          // output to Serial Monitior
          Serial.print("areal, ");
          Serial.print(aaReal.x);
          Serial.print(", ");
          Serial.print(aaReal.y);
          Serial.print(", ");
          Serial.println(aaReal.z);
     }
   }
}
