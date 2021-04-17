
// time update in milliseconds
unsigned long initTime = 1352746358;
int tm_hour_init = 1;
int tm_min_init = 1;
int tm_sec_init = 1;
int tm_hour = 0;
int tm_min = 0;
int tm_sec = 0;
unsigned long currentTime;      //set each send cycle
int           delta_hour = 0;
int           delta_min = 0;
int           delta_sec = 0;
int           millis_left = 0;

void printLocalTime()
{
	// add time since init (from millis(); accurate) to local time from web at init

	currentTime = millis();

/*
	// hour: 1millisec*1000*60*60 = 3600000
	delta_hour = currentTime / 3600000;
	tm_hour = tm_hour_init + delta_hour;
	// min: 1millisec*1000*60 = 60000
	delta_min = (currentTime-delta_hour*3600000) / 60000;
	tm_min = tm_min_init + delta_min;
	// sec: 1millisec*1000 = 1000
	delta_sec = (currentTime-delta_hour*3600000-delta_min*60000) / 1000;
	tm_sec = tm_sec_init + delta_sec;
	// millisec: everything left
	millis_left = (currentTime-delta_hour*3600000-delta_min*60000-delta_sec*1000);
	*/

	/*millis_left = currentTime % 3600000 % 60000 % 1000;
	tm_sec = currentTime % 3600000 % 60000;
	tm_min = currentTime % 3600000;
	tm_hour = currentTime / 3600000;*/

  // hour: 1millisec*1000*60*60 = 3600000
    tm_hour = (initTime + currentTime) / 3600000;
  // min: 1millisec*1000*60 = 60000
    tm_min = (initTime + currentTime-tm_hour*3600000) / 60000;
  // sec: 1millisec*1000 = 1000
    tm_sec = (initTime + currentTime-tm_hour*3600000-tm_min*60000) / 1000;
  // millisec: everything left
    millis_left = (initTime + currentTime-tm_hour*3600000-tm_min*60000-tm_sec*1000);
	// send

	Serial.print( tm_hour );
	Serial.print( ", " );
	Serial.print( tm_min );
	Serial.print( ", " );
	Serial.print( tm_sec );
	Serial.print( ", " );
	Serial.println(millis_left);
}

void setup()
{
	Serial.begin(9600);
}

void loop()
{
	printLocalTime();
	delay(1000);
}
