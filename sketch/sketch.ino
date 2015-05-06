#include <ArduinoHardware.h>
#include <ros.h>

//robot_localization
#include <sensor_msgs/Imu.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/NavSatFix.h>

//move_base (from navigation stack)
#include <geometry_msgs/Twist.h>

#include <tf/tf.h> //convert rpy to quaternion
#include <std_msgs/Bool.h>

#include <sensor_msgs/MagneticField.h>
#include <sensor_msgs/FluidPressure.h>
#include <sensor_msgs/Range.h>
#include <sensor_msgs/Temperature.h>
#include <sensor_msgs/RelativeHumidity.h>

// Enabler
#define MQ_Series		"MQ_Series"
#define PH_meter		"PH_meter"
#define DHT11 			"DHT11"
#define DS18B20 		"DS18B20"
#define FC28 			"FC28"
#define HC_SR04 		"HC_SR04"			// ROS not yet
#define HS_805BB		"HS_805BB"
#define H_BRIDGE		"H_BRIDGE"			//need testing
#define Selenoid 		"Selenoid"
#define IMU 			"IMU"
#define HMC58X3  		"HMC58X3"
#define BMP085 			"BMP085"
#define PMB_688 		"PMB_688"			// http://learn.parallax.com/KickStart/28500

// #define MQ_Calibration

#define NUM_SR04	3
#define NUM_SERVO	3
#define NUM_MOTOR	4

#ifdef IMU
#include <FreeSixIMU.h>
#endif
#ifdef BMP085
#include <Adafruit_BMP085.h>
#endif
#ifdef HMC58X3
#include <HMC5883L.h>
#endif
#ifdef DHT11
#include <idDHT11.h>
#endif
#ifdef HS_805BB
#include <Servo.h>
#endif
#ifdef DS18B20
#include <OneWire.h>
#include <DallasTemperature.h>
#endif
#ifdef HC_SR04
/* be mindful of PWM or timing conflicts messing with Timer2 may cause
namely PWM on pins 3 & 11 on Arduino, 
PWM on pins 9 and 10 on Mega, 
and Tone library */
#include <NewPing.h>
#endif
#ifdef PMB_688
#include <TinyGPS++.h>
#endif

//--------------------define input pin-----------------
//Pandora Box
#define PIN_MQ135		A0			//ADC
#define PIN_MQ136		A1			//ADC
#define PIN_Selenoid	43			//OUT
int PINs_MQHeater[6]	= {11, 12, 13, 14, 15, 16}; //OUT

//Sample gatherer
#define PIN_PHmeter					A2			//ADC
#define PIN_DS18B20					42			//OneWire
#define PIN_FC28					4			//IN
int PINs_servo[NUM_SERVO] 			= {6, 7, 12};	//PWM

//Body
#define PIN_DHT11					19			//INT
#define SerialGPS 					Serial1
int PINs_echoSR04[NUM_SR04] 		= {41, 38, 36};	//IN
int PINs_trigSR04[NUM_SR04] 		= {40, 39, 37};	//OUT
int PINs_motorSpeed[NUM_MOTOR] 		= {9, 11, 3, 4};	//PWM
int PINs_motorDirection[NUM_MOTOR] 	= {8, 10, 2, 5};	//OUT
//--------------------------------------------------------

//---sensor message (please replace it wiht ROS)
//publish
sensor_msgs::Imu imu_msg;
sensor_msgs::MagneticField compass_msg;
sensor_msgs::FluidPressure envPres_msg;
sensor_msgs::Range range_msg;
sensor_msgs::Temperature envTemp_msg, fluidTemp_msg;
sensor_msgs::RelativeHumidity envHum_msg;
sensor_msgs::NavSatFix gps_msg;

ros::Publisher imuPub("imu", &imu_msg);
ros::Publisher compassPub("compass", &compass_msg);
ros::Publisher envPresPub("pressure", &envPres_msg);
ros::Publisher rangePub("sonar", &range_msg);
ros::Publisher envTempPub("temperature", &envTemp_msg);
ros::Publisher fluidTempPub("arm/temperature", &fluidTemp_msg);
ros::Publisher envHumPub("humidity", &envHum_msg);
ros::Publisher gpsPub("gps", &gps_msg);

double CO2;
long distance[NUM_SR04];
float tempC; //DS18B20

//subscribe
int pos[NUM_SERVO];			//0~180
bool direction[NUM_MOTOR];
// -----------------

//Lain-lain
#define LEDBLINK_MS     1000  // Blink rate (in milliseconds).

#ifdef HC_SR04
#define MIN_DISTANCE 	2 	 // Maximum distance (in cm) to ping.
#define MAX_DISTANCE 	400 // Maximum distance (in cm) to ping.
#define PING_INTERVAL	33 // Milliseconds between sensor pings (29ms is about the min to avoid cross-sensor echo).

unsigned long pingTimer[NUM_SR04]; // Holds the times when the next ping should happen for each sensor.
unsigned int cm[NUM_SR04];         // Where the ping distances are stored.
uint8_t currentSensor = 0;          // Keeps track of which sensor is active.
#endif

#ifdef MQ_Calibration
#define         CALIBRATION_SAMPLE_TIMES     (50)    //define how many samples you are going to take in the calibration phase
#define         CALIBRATION_SAMPLE_INTERVAL  (500)   //define the time interal(in milisecond) between each samples in the cablibration phase

#define CO2ppm			500
#define COppm			10
#define NH4ppm			10
#define C2H50Hppm		10
#define CH3ppm			10
#define CH3_2COppm		10
#define SO2ppm			10
#define CH4ppm			10
#define COppm			10
#define H2Sppm			10
#define NH4ppm			10
#endif

#ifdef MQ_Series
#define         READ_SAMPLE_INTERVAL         (50)    //define how many samples you are going to take in normal operation
#define         READ_SAMPLE_TIMES            (5)     //define the time interal(in milisecond) between each samples in 

float CO2_135Curve[2] = {113.7105289, -3.019713765}; //MQ135
float CO_135Curve[2] = {726.7809737, -4.040111669}; //MQ135
float NH4_135Curve[2] = {84.07117895, -4.41107687}; //MQ135
float C2H50H_135Curve[2] = {74.77989144, 3.010328075}; //MQ135
float CH3_135Curve[2] = {47.01770503, -3.281901967}; //MQ135
float CH3_2CO_135Curve[2] = {7.010800878, -2.122018939}; //MQ135
float SO2_136Curve[2] = {40.44109566, -1.085728557}; //MQ136 http://china-total.com/product/meter/gas-sensor/MQ136.pdf
float CH4_136Curve[2] = {57.82777729, -1.187494933}; //MQ136 http://china-total.com/product/meter/gas-sensor/MQ136.pdf
float CO_136Curve[2] = {2142.297846, -2.751369226}; //MQ136 http://china-total.com/product/meter/gas-sensor/MQ136.pdf
float H2S_136Curve[2] = {0, 0}; //MQ136 http://www.sensorica.ru/pdf/MQ-136.pdf
float NH4_136Curve[2] = {0, 0}; //MQ136 http://www.sensorica.ru/pdf/MQ-136.pdf

float Ro135 = 2.511; //MQ135 2.51 this has to be tuned 10K Ohm
float Ro136; 		//MQ136 ... this has to be tuned 10K Ohm
float RL = 0.990; 	//FC-22
#endif

#ifdef PH_meter
#define Offset 0.00            //deviation compensate
#define samplingInterval 20
#define printInterval 800
#define ArrayLenth  40    //times of collection
int pHArray[ArrayLenth];   //Store the average value of the sensor feedback
int pHArrayIndex = 0;
#endif

#ifdef H_BRIDGE
#define RADIUS_WHEEL 		1 		// in meter
#define DISTANCE_WHEEL 		2 		// in meter
#define MIN_VELOCITY_WHEEL 	1 		// in rad/s
#define MAX_VELOCITY_WHEEL 	100 	// in rad/s
#define KP_WHEEL 			4		// in m/s
#define KD_WHEEL 			0.5		// in m/s
float w_right, w_left;
#endif

#ifdef HMC58X3
float heading, first_heading, SP_heading;
#endif

#ifdef PMB_688
#define GPS_BAUD 			4800
#endif
//------------------------------Lib instantiate--------------------------
ros::NodeHandle nh;

#ifdef HC_SR04
NewPing sonars[NUM_SR04];
#endif

#ifdef DHT11
void dht11_wrapper(); // must be declared before the lib initialization
idDHT11 dht11(PIN_DHT11, 0, dht11_wrapper);
#endif

#ifdef HS_805BB
Servo myservo[NUM_SERVO];  // create servo object to control a servo
#endif

#ifdef DS18B20
#define TEMPERATURE_PRECISION 9
OneWire oneWire(PIN_DS18B20);  // on pin 2 (a 4.7K resistor is necessary)
DallasTemperature sensors(&oneWire);
DeviceAddress tempDeviceAddress; // We'll use this variable to store a found device address
#endif

#ifdef IMU
FreeSixIMU sixDOF;
#endif

#ifdef HMC58X3
HMC5883L compass;
// Record any errors that may occur in the compass.
int error = 0;
#endif

#ifdef BMP085
Adafruit_BMP085 bmp;
#endif

#ifdef PMB_688
TinyGPSPlus gps;
#endif
//-------------------------------------------------------------------------------------------------

#define ROS_DEBUG_NAMED(name, string, ...)
#define ROS_INFO_NAMED(name, string, ...)
#define ROS_WARN_NAMED(name, string, ...)
#define ROS_FATAL_NAMED(name, string, ...)
#define ROS_ERROR_NAMED(name, string, ...)

// #define ROS_DEBUG_NAMED(name, string, ...) (log(0, name, string, ##__VA_ARGS__))
// #define ROS_INFO_NAMED(name, string, ...) (log(1, name, string, ##__VA_ARGS__))
// #define ROS_WARN_NAMED(name, string, ...) (log(2, name, string, ##__VA_ARGS__))
// #define ROS_FATAL_NAMED(name, string, ...) (log(3, name, string, ##__VA_ARGS__))
// #define ROS_ERROR_NAMED(name, string, ...) (log(4, name, string, ##__VA_ARGS__))

char *log_buf, *string;
void log(unsigned int verbosity, const char *name, const char *fmt, ...) {
	va_list args;
	va_start(args, fmt);
	vsprintf(string, fmt, args);
	va_end(args);
	sprintf(log_buf, "%s: %s", name, string);
	switch (verbosity) {
		case 0:
		nh.logdebug(log_buf);
		break;
		case 1:
		nh.loginfo(log_buf);
		break;
		case 2:
		nh.logwarn(log_buf);
		break;
		case 3:
		nh.logfatal(log_buf);
		break;
		default:
		nh.logerror(log_buf);
		break;
	}
}

#define toRad(degrees) ((degrees * 71) / 4068)

#ifdef H_BRIDGE
void TwistCb(const geometry_msgs::Twist& msg) // 2 Wheel drive
{	
	/////////////////////////
	// Mathematical model //
	/////////////////////////
	float V, W; 
	if((45<heading && heading<135) || (225<heading && heading<315)){
		V = -( 2*msg.linear.x / (RADIUS_WHEEL * sin(toRad(heading))) );
	} else {
		V = ( 2*msg.linear.y / (RADIUS_WHEEL * cos(toRad(heading))) );
	}	
	W = msg.angular.z*DISTANCE_WHEEL/RADIUS_WHEEL;
	w_right	= (V + W) / 2;
	w_left 	= (V - W) / 2;

	////////////////
	// PID model //
	////////////////
	float lastErr;
	SP_heading += msg.angular.z;
  	float err = SP_heading - heading;
  	float heading_balancer = KP_WHEEL * err + KD_WHEEL * (err - lastErr);
  	lastErr = err;

  	// w_right = msg.linear.x + heading_balancer;
  	// w_left = msg.linear.x - heading_balancer;

  	w_right += heading_balancer;
  	w_left -= heading_balancer;

	//////////////
  	// Output //
	/////////////
  	for(int n = 0; n < NUM_MOTOR/2; ++n)
	{
		digitalWrite(PINs_motorDirection[n], (w_left < 0));
		analogWrite(PINs_motorSpeed[n], map(fabs(w_left), MIN_VELOCITY_WHEEL, MAX_VELOCITY_WHEEL, 0, 255));
	}
	for(int n = NUM_MOTOR/2; n < NUM_MOTOR; n++){
		digitalWrite(PINs_motorDirection[n], (w_right < 0));
		analogWrite(PINs_motorSpeed[n], map(fabs(w_right), MIN_VELOCITY_WHEEL, MAX_VELOCITY_WHEEL, 0, 255));
	}
}
//TODO: connect /joy -> /cmd_vel by creating node for bridge
ros::Subscriber<geometry_msgs::Twist> subWheel("cmd_vel", &TwistCb);
#endif

#ifdef Selenoid
void LockCb(const std_msgs::Bool& msg)
{
	digitalWrite(PIN_Selenoid, msg.data);
}
ros::Subscriber<std_msgs::Bool> subLock("gasLock", &LockCb);
#endif

void setup()
{
	// Serial.begin(57600);
	// delay(5000);
	// Serial.println("starting........................");
	nh.initNode();

	ROS_INFO_NAMED("HELOO", "Starting");
	// wait until you are actually connected
	// while (!nh.connected() ){
	// 	nh.spinOnce();
	// }
	nh.advertise(rangePub);
	range_msg.radiation_type = sensor_msgs::Range::INFRARED;
  range_msg.header.frame_id =  "/ir_ranger";
  range_msg.field_of_view = 0.01;
  range_msg.min_range = 0.03;
  range_msg.max_range = 0.4;

#ifdef PMB_688
	SerialGPS.begin(GPS_BAUD);
	nh.advertise(gpsPub);
#endif

#ifdef IMU
	sixDOF.init(); //init the Acc and Gyro
	nh.advertise(imuPub);
#endif

#ifdef HMC58X3
	error = compass.SetScale(1.3); // Set the scale of the compass.
	error = compass.SetMeasurementMode(Measurement_Continuous); // Set the measurement mode to Continuous  
	if(error != 0) // If there is an error, print it out.
		ROS_ERROR_NAMED(HMC58X3, compass.GetErrorText(error));
#endif

#ifdef BMP085
  	if (bmp.begin() == false) {// init barometric pressure sensor
		ROS_ERROR_NAMED(BMP085, "BMP085 failed");
	}
	nh.advertise(envPresPub);
	nh.advertise(envTempPub);
#endif

#ifdef HC_SR04
	for(int i=0; i<NUM_SR04; i++){
		sonars[i] = NewPing(PINs_trigSR04[i], PINs_echoSR04[i], MAX_DISTANCE);
	}
	pingTimer[0] = millis() + 75;           // First ping starts at 75ms, gives time for the Arduino to chill before starting.
  	for (uint8_t i = 1; i < NUM_SR04; i++){ // Set the starting time for each sensor.
  		pingTimer[i] = pingTimer[i - 1] + PING_INTERVAL;
  	}
#endif

#ifdef DHT11
	nh.advertise(envHumPub);
#endif

#ifdef MQ_Calibration
  	Ro135 = MQCalibration(PIN_MQ135, COppm, RL, CO_135Curve);
  	Ro136 = MQCalibration(PIN_MQ136, COppm, RL, CO_136Curve);
  	ROS_INFO_NAMED(MQ_Series, "Ro for MQ135 is %f ohm", Ro135);
  	ROS_INFO_NAMED(MQ_Series, "Ro for MQ136 is %f ohm", Ro136);
#endif

#ifdef DS18B20
	Wire.begin();
  	sensors.begin();
	for(int i = 0; i < sensors.getDeviceCount(); i++)	// Loop through each device, print out address
	{
	// Search the wire for address
		if(sensors.getAddress(tempDeviceAddress, i))
		{
			ROS_INFO_NAMED(DS18B20, "Found device %d with address: %x", i, tempDeviceAddress[i]);
			ROS_INFO_NAMED(DS18B20, "Setting resolution to %f", TEMPERATURE_PRECISION);

			// set the resolution to TEMPERATURE_PRECISION bit (Each Dallas/Maxim device is capable of several different resolutions)
			sensors.setResolution(tempDeviceAddress, TEMPERATURE_PRECISION);
			ROS_INFO_NAMED(DS18B20, "Resolution actually set to: %f", sensors.getResolution(tempDeviceAddress));
		} else {
			ROS_WARN_NAMED(DS18B20, "Found ghost device at %d but could not detect address. Check power and cabling", i);
		}	
	}
	nh.advertise(fluidTempPub);
#endif

#ifdef HS_805BB
	for(int n = 0; n < NUM_SERVO; ++n)
	{
	if (myservo[n].attach(PINs_servo[n]) == 0)
	{
		ROS_ERROR_NAMED(HS_805BB, "Servo fail at %d", n);
		} else {
			ROS_INFO_NAMED(HS_805BB, "Servo %d is ready", n);
		}
	}
#endif

#ifdef H_BRIDGE
	for(int n = 0; n < NUM_MOTOR; ++n)
	{
		pinMode(PINs_motorDirection[n], OUTPUT);
	}
	for(int n = 0; n < NUM_MOTOR; ++n)
	{
		pinMode(PINs_motorSpeed[n], OUTPUT);
	}
	nh.subscribe(subWheel);
#endif

#ifdef Selenoid
	pinMode(PIN_Selenoid, OUTPUT);
	digitalWrite(PIN_Selenoid, LOW);
	nh.subscribe(subLock);
#endif

	pinMode(13, OUTPUT);
} // end void setup()

#ifdef DHT11
// This wrapper is in charge of calling
// mus be defined like this for the lib work
void dht11_wrapper()
{
	dht11.isrCallback();
}
#endif

void loop()
{
// ---------- Subscribe -------------------
#ifdef HS_805BB
	for(int n = 0; n < NUM_SERVO; ++n)
	{
		myservo[n].write(pos[n]);
	}
#endif
//--------------------------------------------

// ---------- Publish -------------------
#ifdef PMB_688
	if (SerialGPS.available())
	{
		gps.encode(SerialGPS.read());
		gps_msg.position_covariance_type = sensor_msgs::NavSatFix::COVARIANCE_TYPE_UNKNOWN;

		if (gps.location.isUpdated())
	  	{
			gps_msg.latitude = gps.location.lat();
			gps_msg.longitude = gps.location.lng();
		}
		if (gps.altitude.isUpdated())
	  	{
	  		gps_msg.altitude = gps.altitude.meters();
	  	}
		// if (gps.speed.isUpdated())
		// {
		// 	gps.speed.kmph();
		// }
		// if (gps.course.isUpdated())
		// {
		// 	gps.course.deg();
		// }	    
		
		gpsPub.publish(&gps_msg);
	}
#endif

#ifdef IMU
	float imu_raw[6];
	sixDOF.getValues(imu_raw);
	imu_msg.linear_acceleration.x = imu_raw[0];
	imu_msg.linear_acceleration.y = imu_raw[1];
	imu_msg.linear_acceleration.z = imu_raw[2];

	imu_msg.angular_velocity.x = imu_raw[3];
	imu_msg.angular_velocity.y = imu_raw[4];
	imu_msg.angular_velocity.z = imu_raw[5];

	float imu_quaternation[4];
	sixDOF.getQ(imu_quaternation);
	imu_msg.orientation.x = imu_quaternation[0];
	imu_msg.orientation.y = imu_quaternation[1];
	imu_msg.orientation.z = imu_quaternation[2];
	imu_msg.orientation.w = imu_quaternation[3];

	imuPub.publish(&imu_msg);
#endif

#ifdef HMC58X3
	// Retrived the scaled values from the compass (scaled to the configured scale).
	MagnetometerScaled scaled = compass.ReadScaledAxis();
	compass_msg.magnetic_field.x = scaled.XAxis;
	compass_msg.magnetic_field.y = scaled.YAxis;
	compass_msg.magnetic_field.z = scaled.ZAxis;

	// Calculate heading when the magnetometer is level, then correct for signs of axis.
	heading = atan2(scaled.YAxis, scaled.XAxis) + 0.0457;  // declinationAngle = 0.0457

	// Correct for when signs are reversed.
	if(heading < 0)
		heading += 2*PI;

	// Check for wrap due to addition of declination.
	if(heading > 2*PI)
		heading -= 2*PI;

	// Convert radians to degrees for readability.
	heading = heading * 180/M_PI;
	static float once_heading = heading;
	first_heading = once_heading; 
#endif

#ifdef BMP085
// Calculate altitude assuming 'standard' barometric
// pressure of 1013.25 millibar = 101325 Pascal
	envPres_msg.fluid_pressure = bmp.readPressure();
	envTemp_msg.temperature = bmp.readTemperature();
	envPresPub.publish(&envPres_msg);
	envTempPub.publish(&envTemp_msg);
	ROS_DEBUG_NAMED(BMP085, "Temperature = %f*C, Pressure = %fPa, Altitude = %fm", 
		envTemp_msg.temperature, envPres_msg.fluid_pressure, bmp.readAltitude());
#endif

#ifdef FC28
	float soilMoisture = map(analogRead(PIN_FC28), 0, 1023, 0, 100);
#endif

#ifdef DS18B20
	sensors.requestTemperatures(); // Send the command to get temperatures
	for(int i = 0; i < sensors.getDeviceCount(); i++)
	{
		if(sensors.getAddress(tempDeviceAddress, i))
		{
			fluidTemp_msg.temperature = sensors.getTempC(tempDeviceAddress);
			fluidTempPub.publish(&fluidTemp_msg);
		} else {
			ROS_ERROR_NAMED(DS18B20, "Device %d is disconnect", i);
		}
	}
#endif	

#ifdef PH_meter
	static unsigned long samplingTime = millis();
	static unsigned long printTime = millis();
	float pHValue, voltage;
	if(millis() - samplingTime > samplingInterval)
	{
		pHArray[pHArrayIndex++] = analogRead(PIN_PHmeter);
		if(pHArrayIndex == ArrayLenth)pHArrayIndex = 0;
		voltage = avergearray(pHArray, ArrayLenth) * 5.0 / 1024;
		pHValue = 3.5 * voltage + Offset;
		samplingTime = millis();
	}
	if(millis() - printTime > printInterval)   //Every 800 milliseconds, print a numerical, convert the state of the LED indicator
	{
		ROS_DEBUG_NAMED(PH_meter, "Voltage = %fV, PH = %f", voltage, pHValue);
		printTime = millis();
	}
#endif

#ifdef MQ_Series
	int ppm[2];
	ppm[0] = MQGetPercentage(MQRead(PIN_MQ135, RL), Ro135, CO2_135Curve);
	ppm[1] = MQGetPercentage(MQRead(PIN_MQ136, RL), Ro136, H2S_136Curve);
#endif

#ifdef HC_SR04
	for (uint8_t i = 0; i < NUM_SR04; i++) { // Loop through all the sensors.
		if (millis() >= pingTimer[i]) {         // Is it this sensor's time to ping?
	  		pingTimer[i] += PING_INTERVAL * NUM_SR04;  // Set next time this sensor will be pinged.
	  		if (i == 0 && currentSensor == NUM_SR04 - 1) oneSensorCycle(); // Sensor ping cycle complete, do something with the results.
	  		sonars[currentSensor].timer_stop();          // Make sure previous timer is canceled before starting a new ping (insurance).
	  		currentSensor = i;                          // Sensor being accessed.
	  		cm[currentSensor] = 0;                      // Make distance zero in case there's no ping echo for this sensor.
	  		sonars[currentSensor].ping_timer(echoCheck); // Do the ping (processing continues, interrupt will call echoCheck to look for echo).
		}
	}
#endif

#ifdef DHT11
	if (! dht11.acquiring())
	{
		switch (dht11.getStatus()) // debug
		{
			case IDDHTLIB_OK:
			break;
			case IDDHTLIB_ERROR_CHECKSUM:
				ROS_ERROR_NAMED(DHT11, "Checksum error");
			break;
			case IDDHTLIB_ERROR_ISR_TIMEOUT:
				ROS_ERROR_NAMED(DHT11, "ISR Time out error");
			break;
			case IDDHTLIB_ERROR_RESPONSE_TIMEOUT:
				ROS_ERROR_NAMED(DHT11, "Response time out error");
			break;
			case IDDHTLIB_ERROR_DATA_TIMEOUT:
				ROS_ERROR_NAMED(DHT11, "Data time out error");
			break;
			case IDDHTLIB_ERROR_ACQUIRING:
				ROS_DEBUG_NAMED(DHT11, "Acquiring");
			break;
			case IDDHTLIB_ERROR_DELTA:
				ROS_WARN_NAMED(DHT11, "Delta time to small");
			break;
			case IDDHTLIB_ERROR_NOTSTARTED:
				ROS_ERROR_NAMED(DHT11, "Not started");
			break;
			default:
				ROS_ERROR_NAMED(DHT11, "Unknown error");
			break;
		}
		envHum_msg.relative_humidity = dht11.getHumidity();
		envTemp_msg.temperature = dht11.getCelsius();
		double dew = dht11.getDewPoint();
		envHumPub.publish(&envHum_msg);
		envTempPub.publish(&envTemp_msg);
	}
	dht11.acquire();
#endif
//------------------------------------------------------------------------------------
ledBlink();
nh.spinOnce();
}

//
// LED Heartbeat routine by Allen C. Huffman (www.appleause.com)
//
void ledBlink()
{
	static unsigned int  ledStatus = LOW;  // Last set LED mode.
	static unsigned long ledBlinkTime = 0; // LED blink time.

	// LED blinking heartbeat. Yes, we are alive.
	// For explanation, see:
	// http://playground.arduino.cc/Code/TimingRollover
	if ( (long)(millis() - ledBlinkTime) >= 0 )
	{
		// Toggle LED.
		ledStatus = (ledStatus == HIGH ? LOW : HIGH);

		// Set LED pin status.
		digitalWrite(13, ledStatus);

		// Reset "next time to toggle" time.
		ledBlinkTime = millis() + LEDBLINK_MS;
	}
} // End of ledBlink()

double avergearray(int* arr, int number)
{
	int i;
	int max, min;
	double avg;
	long amount = 0;
	if(number <= 0)
	{
		#ifdef PH_meter
		ROS_ERROR_NAMED(PH_meter, "Error number for the array to avraging!/n");
		#endif
		return 0;
	}
	if(number < 5)  //less than 5, calculated directly statistics
	{
		for(i = 0; i < number; i++)
		{
			amount += arr[i];
		}
		avg = amount / number;
		return avg;
	}
	else
	{
		if(arr[0] < arr[1])
		{
			min = arr[0];
			max = arr[1];
		}
		else
		{
			min = arr[1];
			max = arr[0];
		}
		for(i = 2; i < number; i++)
		{
			if(arr[i] < min)
			{
				amount += min;      //arr<min
				min = arr[i];
			}
			else
			{
				if(arr[i] > max)
				{
					amount += max;  //arr>max
					max = arr[i];
				}
				else
				{
					amount += arr[i]; //min<=arr<=max
				}
			}//if
		}//for
		avg = (double)amount / (number - 2);
	}//if
	return avg;
}

#ifdef HC_SR04
/****************** sr04_getdistane ****************************************
Output:  the calculated sensor distance in cm
************************************************************************************/
unsigned long sr04_getdistane(int trigPin, int echoPin)
{
	/* The following trigPin/echoPin cycle is used to determine the
	distance of the nearest object by bouncing soundwaves off of it. */

	digitalWrite(trigPin, LOW);
	delayMicroseconds(2);
	digitalWrite(trigPin, HIGH);
	delayMicroseconds(10);

	digitalWrite(trigPin, LOW);

	return (pulseIn(echoPin, HIGH) / 58.2);
}

void echoCheck() { // If ping received, set the sensor distance to array.
	if (sonars[currentSensor].check_timer())
	cm[currentSensor] = sonars[currentSensor].ping_result / US_ROUNDTRIP_CM;
}

void oneSensorCycle() { // Sensor ping cycle complete, do something with the results.
  // The following code would be replaced with your code that does something with the ping results.
}
#endif

#ifdef MQ_Series
/****************** MQResistanceCalculation ****************************************
Input:   raw_adc - raw value read from adc, which represents the voltage
Output:  the calculated sensor resistance
Remarks: The sensor and the load resistor forms a voltage divider. Given the voltage
         across the load resistor and its resistance, the resistance of the sensor
         could be derived.
         ************************************************************************************/
float MQResistanceCalculation(int raw_adc, float rl_value)
{
//  return ( ((float)rl_value*(1023-raw_adc)/raw_adc));
	return  (long)((long)(1024 * 1000 * (long)rl_value) / raw_adc - (long)rl_value);
}

#ifdef MQ_Calibration
/***************************** MQCalibration ****************************************
Input:   mq_pin - analog channel
Output:  Ro of the sensor
Remarks: This function assumes that the sensor is in clean air. It use
         MQResistanceCalculation to calculates the sensor resistance in clean air.        .
************************************************************************************/
float MQCalibration(int mq_pin, double ppm, double rl_value, float *pcurve )
{
	int i;
	float val = 0;

	for (i = 0; i < CALIBRATION_SAMPLE_TIMES; i++)        //take multiple samples
	{
		val += MQResistanceCalculation(analogRead(mq_pin), rl_value);
		delay(CALIBRATION_SAMPLE_INTERVAL);
	}
	val = val / CALIBRATION_SAMPLE_TIMES;                 //calculate the average value
	//Ro = Rs * sqrt(a/ppm, b) = Rs * exp( ln(a/ppm) / b )

	return  (long)val * exp((log(pcurve[0] / ppm) / pcurve[1]));

}
#endif

/*****************************  MQRead *********************************************
Input:   mq_pin - analog channel
Output:  Rs of the sensor
Remarks: This function use MQResistanceCalculation to caculate the sensor resistenc (Rs).
         The Rs changes as the sensor is in the different consentration of the target
         gas. The sample times and the time interval between samples could be configured
         by changing the definition of the macros.
************************************************************************************/
float MQRead(int mq_pin, float rl_value)
{
	int i;
	float rs = 0;

	for (i = 0; i < READ_SAMPLE_TIMES; i++)
	{
		rs += MQResistanceCalculation(analogRead(mq_pin), rl_value);
		delay(READ_SAMPLE_INTERVAL);
	}

	rs = rs / READ_SAMPLE_TIMES;

	return rs;
}

/*****************************  MQGetPercentage **********************************
Input:   rs_ro_ratio - Rs divided by Ro
         pcurve      - pointer to the curve of the target gas
Output:  ppm of the target gas
Remarks: By using the slope and a point of the line. The x(logarithmic value of ppm)
         of the line could be derived if y(rs_ro_ratio) is provided. As it is a
         logarithmic coordinate, power of 10 is used to convert the result to non-logarithmic
         value.
************************************************************************************/
int  MQGetPercentage(float rs_ro_ratio, float ro, float *pcurve)
{
	return (double)(pcurve[0] * pow(((double)rs_ro_ratio / ro), pcurve[1]));
}
#endif