/* HINT: replace variable using sublime text editor
Serial.println with ROS_DEBUG or ROS_ERROR
*/

// Enabler
#define MQ_Series
#define PH_meter
#define DHT11
#define DS18B20
#define FC28
#define HC_SR04
#define HS_805BB
#define EMS_30A
#define Selenoid

#define MQ_Calibration

#define NUM_SR04	4
#define NUM_SERVO	4
#define NUM_MOTOR	4

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

//--------------------define input pin-----------------
//Pandora Box
#define PIN_MQ135		A0			//ADC
#define PIN_MQ136		A1			//ADC
#define PIN_Selenoid	5			//OUT
int PINs_MQHeater[6]	= {11, 12, 13, 14, 15, 16}; //OUT

//Sample gatherer
#define PIN_PHmeter					A2			//ADC
#define PIN_DS18B20					3			//OneWire
#define PIN_FC28					4			//OneWire
int PINs_servo[NUM_SERVO] 			= {2, 3, 4, 5};	//PWM

//Body
#define PIN_DHT11					2			//INT
int PINs_echoSR04[NUM_SR04] 		= {4, 5, 6, 7};	//IN
int PINs_trigSR04[NUM_SR04] 		= {4, 5, 6, 7};	//OUT
int PINs_motorSpeed[NUM_MOTOR] 		= {3, 4, 5, 6};	//PWM
int PINs_motorDirection[NUM_MOTOR] 	= {3, 4, 5, 6};	//OUT
//--------------------------------------------------------

//---sensor message (please replace it wiht ROS)
//publish
double CO2;
long distance[NUM_SR04];
float tempC; //DS18B20

//subscribe
int pos[NUM_SERVO];			//0~180
int speed[NUM_MOTOR];		//0~255
bool direction[NUM_MOTOR];
// -----------------

//Lain-lain
#define LEDBLINK_MS     1000  // Blink rate (in milliseconds).
 
#ifdef MQ_Calibration
#define         CALIBRATION_SAMPLE_TIMES     (50)    //define how many samples you are going to take in the calibration phase
#define         CALIBRATION_SAMPLE_INTERVAL  (500)   //define the time interal(in milisecond) between each samples in the cablibration phase
#define         READ_SAMPLE_INTERVAL         (50)    //define how many samples you are going to take in normal operation
#define         READ_SAMPLE_TIMES            (5)     //define the time interal(in milisecond) between each samples in 

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

// Lib instantiate
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

void setup()
{
#ifdef MQ_Calibration
	Ro135 = MQCalibration(PIN_MQ135, COppm, RL, CO_135Curve);
	Ro136 = MQCalibration(PIN_MQ136, COppm, RL, CO_136Curve);
#endif

#ifdef DS18B20
	sensors.begin();
	for(int i = 0; i < sensors.getDeviceCount(); i++)	// Loop through each device, print out address
	{
		// Search the wire for address
		if(sensors.getAddress(tempDeviceAddress, i))
		{
			Serial.print("Found device ");
			Serial.print(i, DEC);
			Serial.print(" with address: ");
			for (uint8_t i = 0; i < 8; i++)
			{
				// zero pad the address if necessary
				if (tempDeviceAddress[i] < 16) Serial.print("0");
				Serial.print(tempDeviceAddress[i], HEX);
			}
			Serial.println();

			Serial.print("Setting resolution to ");
			Serial.println(TEMPERATURE_PRECISION, DEC);

			// set the resolution to TEMPERATURE_PRECISION bit (Each Dallas/Maxim device is capable of several different resolutions)
			sensors.setResolution(tempDeviceAddress, TEMPERATURE_PRECISION);

			Serial.print("Resolution actually set to: ");
			Serial.print(sensors.getResolution(tempDeviceAddress), DEC);
			Serial.println();
		}
		else
		{
			Serial.print("Found ghost device at ");
			Serial.print(i, DEC);
			Serial.print(" but could not detect address. Check power and cabling");
		}
	}
#endif

#ifdef HS_805BB
	for(int n = 0; n < NUM_SERVO; ++n)
	{
		if (myservo[n].attach(PINs_servo[n]) == 0)
		{
			// fail
		}
	}
#endif

#ifdef EMS_30A
	for(int n = 0; n < NUM_MOTOR; ++n)
	{
		pinMode(PINs_motorDirection[n], OUTPUT);
	}
	for(int n = 0; n < NUM_MOTOR; ++n)
	{
		pinMode(PINs_motorSpeed[n], OUTPUT);
	}
#endif

#ifdef Selenoid
	pinMode(PIN_Selenoid, OUTPUT);
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
#ifdef EMS_30A
	for(int n = 0; n < NUM_MOTOR; ++n)
	{
		analogWrite(PINs_motorSpeed[n], speed[n]);
	}
	for(int n = 0; n < NUM_MOTOR; ++n)
	{
		digitalWrite(PINs_motorDirection[n], direction[n]);
	}
#endif

#ifdef HS_805BB
	for(int n = 0; n < NUM_SERVO; ++n)
	{
		myservo[n].write(pos[n]);
	}
#endif

#ifdef HC_SR04
	for(int n = 0; n < NUM_SR04; ++n)
	{
		distance[n] = sr04_getdistane(PINs_trigSR04[n], PINs_echoSR04[n]);
	}
#endif
//------------------------------------------------------------------------------------

// ---------- Publish -------------------
#ifdef FC28
	float soilMoisture = map(analogRead(PIN_FC28), 0, 1023, 0, 100);
#endif

#ifdef DS18B20
	sensors.requestTemperatures(); // Send the command to get temperatures
	for(int i = 0; i < sensors.getDeviceCount(); i++)
	{
		if(sensors.getAddress(tempDeviceAddress, i))
		{
			tempC = sensors.getTempC(tempDeviceAddress);
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
		Serial.print("Voltage:");
		Serial.print(voltage, 2);
		Serial.print("    pH value: ");
		Serial.println(pHValue, 2);

		printTime = millis();
	}
#endif

#ifdef Selenoid
	bool isLock;
	digitalWrite(PIN_Selenoid, isLock);
#endif
#ifdef MQ_Series
	int ppm[2];
	ppm[0] = MQGetPercentage(MQRead(PIN_MQ135, RL), Ro135, CO2_135Curve);
	ppm[1] = MQGetPercentage(MQRead(PIN_MQ136, RL), Ro136, H2S_136Curve);
#endif

#ifdef DHT11
	if (dht11.acquiring())
	{
		switch (dht11.getStatus()) // debug
		{
			case IDDHTLIB_OK:
				Serial.println("OK");
				break;
			case IDDHTLIB_ERROR_CHECKSUM:
				Serial.println("Error\n\r\tChecksum error");
				break;
			case IDDHTLIB_ERROR_ISR_TIMEOUT:
				Serial.println("Error\n\r\tISR Time out error");
				break;
			case IDDHTLIB_ERROR_RESPONSE_TIMEOUT:
				Serial.println("Error\n\r\tResponse time out error");
				break;
			case IDDHTLIB_ERROR_DATA_TIMEOUT:
				Serial.println("Error\n\r\tData time out error");
				break;
			case IDDHTLIB_ERROR_ACQUIRING:
				Serial.println("Error\n\r\tAcquiring");
				break;
			case IDDHTLIB_ERROR_DELTA:
				Serial.println("Error\n\r\tDelta time to small");
				break;
			case IDDHTLIB_ERROR_NOTSTARTED:
				Serial.println("Error\n\r\tNot started");
				break;
			default:
				Serial.println("Unknown error");
				break;
		}
		float humid = dht11.getHumidity();
		float heat = dht11.getCelsius();
		double dew = dht11.getDewPoint();
	}
#endif
//------------------------------------------------------------------------------------
	ledBlink();
	
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
		Serial.println("Error number for the array to avraging!/n");
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

long sr04_getdistane(int trigPin, int echoPin)
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
	;
}

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