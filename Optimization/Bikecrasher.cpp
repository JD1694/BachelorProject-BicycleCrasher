//#include <math.h>
#define M_PI 3.141592654
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


float THRESHOLD_SMOOTH_GYRO_X;
float THRESHOLD_SMOOTH_GYRO_Y;
float THRESHOLD_SMOOTH_GYRO_Z;
float THRESHOLD_YAW;
float THRESHOLD_PITCH;
float THRESHOLD_ROLL;
float THRESHOLD_INT_ACCEL_X;
float THRESHOLD_INT_ACCEL_Y;
float THRESHOLD_INT_ACCEL_Z;
float THRESHOLD_INT_GRAVITY_X;
float THRESHOLD_INT_GRAVITY_Y;
float THRESHOLD_INT_GRAVITY_Z;
float COMMON_GRAVITY_Z;
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
const float GYRO_EXP_DECLINE_FACTOR = 0.1;
const int FIFO_INT_LENGTH_BIG = 20;
const int FIFO_INT_LENGTH_SHORT = 8;

// hold past data to filter
CircularBuffer<VectorInt16,FIFO_INT_LENGTH_BIG> fifo_aaReal;
CircularBuffer<VectorInt16,FIFO_INT_LENGTH_BIG> fifo_aa;
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

list<vector<string>> sensorDataGetter(string argument);
void getSensorReadings(vector<string> currentData);
void output(string timestamp);
void prepareData();
bool evalDiscrete();
double evalContinuous();
double norm(double a, double b, double c);
double sigmoid_function(double x);
void rotateZ(float radAngle, VectorInt16 point, VectorFloat *rotatedPoint );
void rotateZ(float radAngle, float point[3], float *rotatedPoint );

//set true for debugging output:
bool debug;



int main(int argc, char *argv[]){
// Thresholds  to tune

THRESHOLD_SMOOTH_GYRO_X = stof(argv[2]);

THRESHOLD_SMOOTH_GYRO_Y = stof(argv[3]);

THRESHOLD_SMOOTH_GYRO_Z = stof(argv[4]);

THRESHOLD_YAW = stof(argv[5]);

THRESHOLD_PITCH = stof(argv[6]);

THRESHOLD_ROLL = stof(argv[7]);

THRESHOLD_INT_ACCEL_X = stof(argv[8]);

THRESHOLD_INT_ACCEL_Y = stof(argv[9]);

THRESHOLD_INT_ACCEL_Z = stof(argv[10]);

THRESHOLD_INT_GRAVITY_X = stof(argv[11]);

THRESHOLD_INT_GRAVITY_Y = stof(argv[12]);

THRESHOLD_INT_GRAVITY_Z = stof(argv[13]);

COMMON_GRAVITY_Z = 1000;

//set true for debugging output:
//debug = argv[argc].find("debug")!=std::string::npos;
debug = std::string(argv[argc-1])=="debug";


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


string filename(argv[1]);
list<vector<string>> sensorDataList=sensorDataGetter(filename);
string sizeoflist=to_string(sensorDataList.size());
for (auto  sensorData:sensorDataList){
	// Get sensor readings
  getSensorReadings(sensorData);
  
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

// crashPropability = evalContinuous();

  // Reaction
  if (crashPropability > 0.5) {
    output(sensorData[0]);
    return 0;
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
      if(dataText.size()==17){  //checks if vector has 17 entries to prevent out of bounds errors
			dataTextList.push_back(dataText);
      }
	  }
  }
  return dataTextList;
}



void getSensorReadings(vector<string> currentData) {

aa.x=stoi(currentData[10]);
aa.y=stoi(currentData[11]);
aa.z=stoi(currentData[12]);         // [x, y, z]            accel sensor measurements
gy.x=stoi(currentData[14]);
gy.y=stoi(currentData[15]);
gy.z=stoi(currentData[16]);         // [x, y, z]            gyro sensor measurements
aaReal.x=stoi(currentData[6]);
aaReal.y=stoi(currentData[7]);
aaReal.z=stoi(currentData[8]);     // [x, y, z]            gravity-free accel sensor measurements
gravity;    // [x, y, z]            gravity vector
ypr[0]=stof(currentData[2])/180*M_PI;
ypr[1]=stof(currentData[3])/180*M_PI;
ypr[2]=stof(currentData[4])/180*M_PI;

}

void output(string timestamp) {
  cout<<timestamp;
}

void debug_output(bool debug, string timestamp) {
  if (debug){
	cout<<timestamp<<endl;
  }
}


void prepareData() {
  /* Prepare data, filter and convert to get more measurable input */
/*
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
  */
  // Filter aaReal data (Exponential Moving Average)
  accelIntegral.x = accelIntegral.x * (1 - GYRO_EXP_DECLINE_FACTOR) + aaReal.x * GYRO_EXP_DECLINE_FACTOR;
  accelIntegral.y = accelIntegral.y * (1 - GYRO_EXP_DECLINE_FACTOR) + aaReal.y * GYRO_EXP_DECLINE_FACTOR;
  accelIntegral.z = accelIntegral.z * (1 - GYRO_EXP_DECLINE_FACTOR) + aaReal.z * GYRO_EXP_DECLINE_FACTOR;
  
  // Filter aa data (Exponential Moving Average)
  accelGravityIntegral.x = accelGravityIntegral.x * (1 - GYRO_EXP_DECLINE_FACTOR) + aa.x * GYRO_EXP_DECLINE_FACTOR;
  accelGravityIntegral.y = accelGravityIntegral.y * (1 - GYRO_EXP_DECLINE_FACTOR) + aa.y * GYRO_EXP_DECLINE_FACTOR;
  accelGravityIntegral.z = accelGravityIntegral.z * (1 - GYRO_EXP_DECLINE_FACTOR) + aa.z * GYRO_EXP_DECLINE_FACTOR;
  
  // Filter gyro data (Exponential Moving Average)
  gyroSmoothend.x = gyroSmoothend.x * (1 - GYRO_EXP_DECLINE_FACTOR) + gy.x * GYRO_EXP_DECLINE_FACTOR;
  gyroSmoothend.y = gyroSmoothend.y * (1 - GYRO_EXP_DECLINE_FACTOR) + gy.y * GYRO_EXP_DECLINE_FACTOR;
  gyroSmoothend.z = gyroSmoothend.z * (1 - GYRO_EXP_DECLINE_FACTOR) + gy.z * GYRO_EXP_DECLINE_FACTOR;
}


bool evalDiscrete() {
  /* Evaluate inputs into discrete categories: Crash and Safe */

  // Absolute Angle
  if ( // abs(ypr[0] * 180 / M_PI) > THRESHOLD_YAW // YAW Abfage sinnlos ///// evtl y-p-r vertauscht
       abs(ypr[1] * 180 / M_PI) > THRESHOLD_PITCH*45 ///// change to rad for more efficienty
       || abs(ypr[2] * 180 / M_PI) > THRESHOLD_ROLL*50) {
	debug_output(debug, "Check: Absolute Angle");
    return true;
  }
  // Rate of rotation
  if ( abs(gyroSmoothend.x) > THRESHOLD_SMOOTH_GYRO_X*200
       || abs(gyroSmoothend.y) > THRESHOLD_SMOOTH_GYRO_Y*200
       || abs(gyroSmoothend.z) > THRESHOLD_SMOOTH_GYRO_Z*200) {
	debug_output(debug, "Check: Rate of rotation");
    return true;
  }
  // Acceleration
  if ( abs(accelIntegral.x) > THRESHOLD_INT_ACCEL_X*10000
       || abs(accelIntegral.y) > THRESHOLD_INT_ACCEL_Y*10000
       || abs(accelIntegral.z) > THRESHOLD_INT_ACCEL_Z*10000) {
	debug_output(debug, "Check: Acceleration");
    return true;
  }
  // Direction of Gravity and Momentum
  if ( abs(accelGravityIntegral.x) > THRESHOLD_INT_GRAVITY_X*10000
       || abs(accelGravityIntegral.y) > THRESHOLD_INT_GRAVITY_Y*10000
       //|| abs(accelGravityIntegral.z - COMMON_GRAVITY_Z) > THRESHOLD_INT_GRAVITY_Z*10000
	   ) {
	debug_output(debug, "Check: Gravity and Momentum");
    return true;
  }
  // ...

  // no Trigger found
  debug_output(debug, "No Trigger found");
  return false;
}


double evalContinuous() {
  /* Evaluate inputs into continuous categories: Propability of Crash */
  /* Notes: square may not have methods in arg*/
  //THRESHOLD_SMOOTH_GYRO - abs(gyroSmoothend.x)

  // Absolute Angle
  delta_x = abs(ypr[0] * 180 / M_PI) - THRESHOLD_YAW; ///// evtl y-p-r vertauscht///// change to rad for more efficienty
  delta_y = abs(ypr[1] * 180 / M_PI) - THRESHOLD_PITCH;
  delta_z = abs(ypr[2] * 180 / M_PI) - THRESHOLD_ROLL;
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
  return sqrt( (a*a) +(b*b) + (c*c) );
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