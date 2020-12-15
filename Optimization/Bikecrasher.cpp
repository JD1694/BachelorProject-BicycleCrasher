#include <stdlib.h>
#include <fstream>
#include <sstream>
#include <string>
#include<iostream>
#include<vector>
#include<list>
#include<algorithm>
#include<cctype>
//#include <I2Cdev.h>
#include "MPU6050_6Axis_MotionApps20.h"
#include "CircularBuffer.h"
using namespace std;



int main(int argc, char *argv[]){




const float GYRO_EXP_DECLINE_FACTOR= 0.1;
const int   FIFO_INT_LENGTH_BIG= 20;
const int FIFO_INT_LENGTH_SHORT = 8;
// Thresholds  to tune
extern float THRESHOLD_SMOOTH_GYRO_X;
THRESHOLD_SMOOTH_GYRO_X = stof(argv[2]);
extern float THRESHOLD_SMOOTH_GYRO_Y;
THRESHOLD_SMOOTH_GYRO_Y = stof(argv[3]);
extern float THRESHOLD_SMOOTH_GYRO_Z;
THRESHOLD_SMOOTH_GYRO_Z = stof(argv[4]);
extern float THRESHOLD_YAW;
THRESHOLD_YAW = stof(argv[5]);
extern float THRESHOLD_PITCH;
THRESHOLD_PITCH = stof(argv[6]);
extern float THRESHOLD_ROLL;
THRESHOLD_ROLL = stof(argv[7]);
extern int   THRESHOLD_INT_ACCEL_X;
THRESHOLD_INT_ACCEL_X = stoi(argv[8]);
extern int   THRESHOLD_INT_ACCEL_Y;
THRESHOLD_INT_ACCEL_Y = stoi(argv[9]);
extern int   THRESHOLD_INT_ACCEL_Z;
THRESHOLD_INT_ACCEL_Z = stoi(argv[10]);
extern int   THRESHOLD_INT_GRAVITY_X;
THRESHOLD_INT_GRAVITY_X = stoi(argv[11]);
extern int   THRESHOLD_INT_GRAVITY_Y;
THRESHOLD_INT_GRAVITY_Y = stoi(argv[12]);
extern int   THRESHOLD_INT_GRAVITY_Z;
THRESHOLD_INT_GRAVITY_Z = stoi(argv[13]);
extern int   COMMON_GRAVITY_Z;
COMMON_GRAVITY_Z = 1000;
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
extern VectorInt16 aa;         // [x, y, z]            accel sensor measurements
extern VectorInt16 gy;         // [x, y, z]            gyro sensor measurements
extern VectorInt16 aaReal;     // [x, y, z]            gravity-free accel sensor measurements
VectorInt16 aaWorld;    // [x, y, z]            world-frame accel sensor measurements
extern VectorFloat gravity;    // [x, y, z]            gravity vector
float euler[3];         // [psi, theta, phi]    Euler angle container
extern float ypr[3];           // [yaw, pitch, roll]   yaw/pitch/roll container and gravity vector

string filename(argv[1]);
list<vector<string>> sensorDataList=sensorDataGetter(filename);

for (auto &const sensorData:sensorDataList){
	// Get sensor readings
  getSensorReadings(sensorData, &aa, &gy, &aaReal,&gravity,ypr);

  // Prepare data, filter and convert to get more measurable input
  prepareData();

  // Evaluate inputs

  crashYesNo = evalDiscrete();
  if (crashYesNo)
  {
    crashPropability = 1.0;
  }
  else
  {
    crashPropability = 0.0;
  }

vector<int> t;

  crashPropability = evalContinuous();


  // Reaction
  if (crashPropability > 0.5) {
    output(sensorData[0]);
  }



}

  return 0;
}



list<vector<string>> sensorDataGetter(string argument){
	//Kommandozeile lesen, Filename aus 
string filename;

//cout << "Datei: ";
filename=argument;

ifstream input(filename);
 
  if (!input)
  {
    cerr << "Fehler beim Oeffnen der Datei " << filename << "\n";
    //return 1;
  }
 //Datei einlesen
  string line;
  list<vector<string>>dataTextList;
  
  while (getline(input, line)) //splits the file into lines and iterates through
  {
	  vector<string> dataText;

	  if(line.length()>50)
	  {
		  	
		    stringstream linestr;
  			string segment;
	  		linestr<<line;
	  		while(getline(linestr,segment,',')){		//splits lines into single strings with data
				  segment.erase(remove_if(segment.begin(),segment.end(),[](unsigned char x){return std::isspace(x);}),segment.end());
				  dataText.push_back(segment);
	  		}
		if(dataText.size()<17){
			dataTextList.push_back(dataText);
		}
	  }
  }
}



void getSensorReadings(vector<string> currentData, VectorInt16* aa, VectorInt16* gy, VectorInt16* aaReal,VectorFloat* gravity,float ypr[3]) {

aa->x=stoi(currentData[10]);
aa->y=stoi(currentData[11]);
aa->z=stoi(currentData[12]);         // [x, y, z]            accel sensor measurements
gy->x=stoi(currentData[14]);
gy->y=stoi(currentData[15]);
gy->z=stoi(currentData[16]);         // [x, y, z]            gyro sensor measurements
aaReal->x=stoi(currentData[6]);
aaReal->y=stoi(currentData[7]);
aaReal->z=stoi(currentData[8]);     // [x, y, z]            gravity-free accel sensor measurements
gravity;    // [x, y, z]            gravity vector
ypr[0]=stof(currentData[2]);
ypr[1]=stof(currentData[3]);
ypr[2]=stof(currentData[4]);
}

void output(string timestamp) {
  cout<<timestamp;
}

void prepareData() {
  /* Prepare data, filter and convert to get more measurable input */

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

double norm(double a, double b, double c){
  /* Calculates the euclidian norm of a vektor consisting of the 3 given values*/
  return sqrt( sqrt(delta_x) + sqrt(delta_y) + sqrt(delta_z) );
}

double sigmoid_function(double x){
  /* 
   *  A fast implementation (without exp(-x)) of the Sigmoid activation curve, modified to return return percentages.
   *  Returns results between 0 (for very negative x) and 1 (for very positive x) 
   *  centered around x=0 where 0.5 is returned*/
   return x / (2*(1 + abs(x))) + 0.5;
}