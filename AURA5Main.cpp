/*
    
 HABJOE  Arduino Flight script
 HABJOE - Copyright Andrew Myatt and Joseph Myatt March/2013
 
 KEYWORDS: ARDUINO MEGA, MY_HIGH ALTITUDE BALLOON, BMP085, ADXL345, UBLOX, I2C, WIRING, SDCARD
 
 This code is in the public domain.
 
 This library is distributed in the hope that it will be useful,
 but WITHOUT ANY WARRANTY; without even the implied warranty of
 MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 Lesser General Public License for more details.

 You should have received a copy of the GNU Lesser General Public
 License along with this library; if not, write to the Free Software
 Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA  02110-1301  USA
 
 This software is free software; you can redistribute it and/or
 modify it under the terms of the GNU Lesser General Public
 License as published by the Free Software Foundation; either
 version 2.1 of the License, or (at your option) any later version.

 This software is distributed in the hope that it will be useful,
 but WITHOUT ANY WARRANTY; without even the implied warranty of
 MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 Lesser General Public License for more details.
 
 POLLS SENSORS GENRATES DATA STRUCTURE WHICH IS WRITTEN TO SD CARD AND SENT VIA 
 I2C INTERFACE TO MOBILE MODULE AND TRANSMITTER
 
 Xno,Yno,Date,Time,Lat,Long,altitude,angle,ascentRate,xAcc,yAcc,zAcc,GyXPayload,GyYPayload,GyZPayload,MgXPayload,MgYPayload,MgZPayload,tempin,pressure,tempex,compass,bSats,age,ihdop,rOnOff,sMobile,sCIPSTATUS,sRssi,balloonCutDoFire,balloonCutFired,ascent
*/

/*
 * Include Files
 */
#include <SPI.h>
#include <SdFat.h>
#include <EasyTransferI2C_NL.h>
#include <EasyTransfer.h>
#include <TinyGPS_HJOE.h>
#include <Wire.h>
#include <L3G.h>
#include <ADXL345.h>
#include <Comp6DOF_n0m1.h>
#include <HMC5883L.h> // Reference the HMC5883L Compass Library
#include <BMP085.h>
#include <OneWire.h>
#include <DallasTemperature.h>
#include <avr/wdt.h>

/*
 * Definitions
 */
// build modifiers
//#define DEBUG_ON
#define MY_HIGH HIGH
#define MY_LOW LOW

// LED Definitions
#define SET_LED_WHITE 4
#define SET_LED_RED 3
#define SET_LED_BLUE 2
#define SET_LED_GREEN 1
#define SET_LED_OFF 0

// Millisecond wait before Sending to Transmit device
#define WAIT_NTXB  					9000
#define WAIT_SIM900  				10000

// Define filenames
#define SD_LOG_FILE        			"AURA5.CSV"

// Arduino Pin assignment
#define PIN_GPS_RX 0        		//Fixed:Note: RX on Board is connected to RX on GPS Board
#define PIN_GPS_TX 1        		//Fixed:Note: TX on Board is connected to TX on GPS Board
#define PIN_LED_RED 6        		//Fixed: Red LED
#define PIN_LED_BLUE 5       		//Fixed: Blue LED
#define PIN_LED_GREEN 7      		//Fixed: Blue GREEN
#define PIN_IC2_SDA 20       		//Fixed: SDA
#define PIN_IC2_SLC 21       		//Fixed: SLC
#define PIN_SPI_CS 53        		//Fixed: Card Select for SD Card
#define ONE_WIRE_BUS 12				//Data wire is plugged into port 2 on the Arduino

#define I2C_SLV_SIM900_ADDRESS 9	//define slave i2c address
#define I2C_SLV_NTXB_ADDRESS 10		//define slave i2c address
#define ADXL345_ADDRESS 0x53		// Device address as specified in data sheet 
#define HMC5883_ADDRESS 0x1E 		//0011110b, I2C 7bit address of HMC5883
#define BMP085_ADDRESS 0x77  		// I2C address of BMP085
#define aref_voltage 3.3 

//#define SD_BUFF_SIZE 	512
#define SD_BUFF_SIZE 	1024
char SDBuffer[SD_BUFF_SIZE];


//Data structure used for sending data via the I2C and to the SD card
typedef union {
	typedef struct{
		uint16_t tCount;  				//2
		byte 	year;					//1 		  
		byte    month;					//1	  
		byte    day;					//1	  
		byte    hour;					//1
		byte    minute;					//1
		byte    second;					//1
		long    iLat;					//4
		long    iLong;					//4
		long    iAlt; 					//4	
		int16_t TmpPayload;				//2		Pressure Temp (*x0.1 DegC)
		int16_t TmpExternal;			//2		DS18B20 Temp (*x0.1 DegC)		
		int16_t TmpCamera;				//2		DS18B20 Temp (*x0.1 DegC)	
		byte 	bSats;					//1
		//
		// End of mapping with I2C data.
		//
		byte 	hundredths;				//1
		long    iAngle;				//4	
		long    iHspeed;				//4		Horizontal speed
		long    iVspeed;				//4		Vertical speed
		unsigned long age; 				//4
		unsigned long ihdop;			//4
		int16_t AcXPayload;					//2		accel x
		int16_t AcYPayload;					//2		accel y
		int16_t AcZPayload;					//2		accel z
		int16_t GyXPayload;					//2		gyro x
		int16_t GyYPayload;					//2		gyro y
		int16_t GyZPayload; 					//2		gyro z 
		int16_t MgXPayload;					//2		mag x
		int16_t MgYPayload;					//2		mag y
		int16_t MgZPayload; 					//2		mag z 
		long    CmpssPayload;					//4  	compass bearing using tilt compensation etc.
		int16_t AcXCamera;				//2		accel x
		int16_t AcYCamera;				//2		accel y
		int16_t AcZCamera;				//2		accel z
		int16_t GyXCamera;				//2		gyro x
		int16_t GyYCamera;				//2		gyro y
		int16_t GyZCamera; 				//2		gyro z 
		int16_t MgXCamera;				//2		mag x
		int16_t MgYCamera;				//2		mag y
		int16_t MgZCamera; 				//2		mag z 
		long    CmpssCamera;				//4  	compass bearing using tilt compensation etc.
		longPressurePayload;			//4 
		unsigned long uslCount;		//4	 
		} LOG_DATA_STRUCTURE;	
    LOG_DATA_STRUCTURE vals;

	typedef struct {
		uint16_t tCount;  				//2
		byte 	year;					//1 		  
		byte    month;					//1	  
		byte    day;					//1	  
		byte    hour;					//1
		byte    minute;					//1
		byte    second;					//1
		long    iLat;					//4
		long    iLong;					//4
		long    iAlt; 					//4	
		int16_t TmpPayload;				//2		Pressure Temp (*x0.1 DegC)
		int16_t TmpExternal;			//2		DS18B20 Temp (*x0.1 DegC)		
		int16_t TmpCamera;				//2		DS18B20 Temp (*x0.1 DegC)	
		byte 	bSats;					//1
	} I2C_STRUCTURE;
        I2C_STRUCTURE i2cOut;
} MYOUTPACKET;

typedef struct{
	unsigned long uslCountCamera;  		//4		record count ++
	int16_t	TmpCamera;					//2		DS18B20 Temp (*x0.1 DegC)
	int16_t AcXCamera;					//2		accel x
	int16_t AcYCamera;					//2		accel y
	int16_t AcZCamera;					//2		accel z
	int16_t GyXCamera;					//2		gyro x
	int16_t GyYCamera;					//2		gyro y
	int16_t GyZCamera; 					//2		gyro z 
	int16_t MgXCamera;					//2		mag x
	int16_t MgYCamera;					//2		mag y
	int16_t MgZCamera; 					//2		mag z 
	long    CmpssCamera;				//4  	compass bearing using tilt compensation etc
	int16_t TmpCamera;					//2		DS18B20 Temp (*x0.1 DegC)
	byte	CamStatus;					//1		Camera on or off
	byte	toneCode;					//1		toneNumber
	} LOG_CAMERA_DATA_STRUCTURE;	
LOG_CAMERA_DATA_STRUCTURE valsCamera;

// Declare datatype variables
EasyTransferI2C 	ETI2Cout;				//Easy Transfer I2C Out
EasyTransfer        ETSerialIn;				//Easy Transfer Serial In
MYOUTPACKET			mD;						//packet of telemetry for other slave arduino's
SdFat 				SD;						//SD
SdFile 				dataFile;				//SDFile
SdFile				flagFile;				//SDflag
L3G 				LGgyro;					//L3G Gyro
TinyGPS_HJOE 		gps;					//GPS
ADXL345 			accel;					//accelerometer
HMC5883L 			compass;				//Magnetometer/Compass
Comp6DOF_n0m1 		sixDOF;					//Tilt compensation from Compass
BMP085 				dps;					//Pressure and Temp
OneWire 			oneWire(ONE_WIRE_BUS);	// Set up which Arduino pin will be used for the 1-wire interface to the sensor
DallasTemperature 	sensors(&oneWire);
DeviceAddress 		outsideThermometer = { 0x28, 0x44, 0xD8, 0x7D, 0x5, 0x0, 0x0, 0xD7 };

// Declare utility global variables variables
int 			error = 0;
bool			SENDWIRE = false;
bool			NEWGPSDATA;
unsigned long 	elapseSIM900;
unsigned long 	elapseNTXB;
long			TmpPayloadFULL;

/*
 * functions
 */
 
 /***************************************************
 * writes the data values to SD card as time 
 * efficiently as possible. 
 **************************************************/

void writeSDData() {
    char SDString[200] = "";
   	int tmp_year = mD.vals.year + 2000;	
	sprintf(SDString, "%ld,%d,%04d-%02d-%02d,%02d:%02d:%02d.%02d,%ld,%ld,%ld,%ld,%ld,%d,%d,%d,%d,%d,%d,%d,%d,%d,%ld,%ld,%d,%ld,%d,%ld,%ld,%d,%d,%d,%d,%d,%d,%d\n", 
	mD.vals.uslCount,mD.vals.tCount,
	tmp_year, mD.vals.month, mD.vals.day,
	mD.vals.hour, mD.vals.minute, mD.vals.second, mD.vals.hundredths,
	mD.vals.iLat,mD.vals.iLong,mD.vals.iAlt,mD.vals.iAngle,mD.vals.iHspeed,
	mD.vals.AcXPayload,mD.vals.AcYPayload,mD.vals.AcZPayload,
	mD.vals.GyXPayload,mD.vals.GyYPayload,mD.vals.GyZPayload,
	mD.vals.MgXPayload,mD.vals.MgYPayload,mD.vals.MgZPayload,
	mD.vals.AcXCamera,mD.vals.AcYCamera,mD.vals.AcZCamera,
	mD.vals.GyXCamera,mD.vals.GyYCamera,mD.vals.GyZCamera,
	mD.vals.MgXCamera,mD.vals.MgYCamera,mD.vals.MgZCamera,
	mD.vals.PressurePayload,mD.vals.TmpExternal,mD.vals.TmpCamera,
	mD.vals.CmpssPayload,mD.vals.CmpssCamera,
	mD.vals.bSats,mD.vals.age,mD.vals.ihdop);
	
   #ifdef DEBUG_ON	
		Serial.println(SDString);
   #endif 
		
   if (sizeof(SDBuffer)-strlen(SDBuffer) < strlen(SDString)) {
		dataFile.write(SDBuffer,strlen(SDBuffer));
		dataFile.sync();		
		memset(SDBuffer, 0, sizeof(SDBuffer));
   }
   strcat(SDBuffer,SDString);
}

/***************************************************
 * LED STATUS DISPLAY FUNTIONS
 * LED is a Common Anode, therefore the pin is the cathode
 * and must be set MY_LOW to be on, and MY_HIGH to be turned off! 
 **************************************************/
void SET_LED_Status(int stat, int intDelay){
  
  digitalWrite(PIN_LED_RED, MY_HIGH);
  digitalWrite(PIN_LED_GREEN, MY_HIGH);
  digitalWrite(PIN_LED_BLUE, MY_HIGH); 
  if (stat == SET_LED_RED) {
		digitalWrite(PIN_LED_RED, MY_LOW);
  } else if (stat == SET_LED_BLUE) {
		digitalWrite(PIN_LED_BLUE, MY_LOW);
  } else if (stat == SET_LED_GREEN) {
		digitalWrite(PIN_LED_GREEN, MY_LOW);
  } else if (stat == SET_LED_WHITE){
		digitalWrite(PIN_LED_RED, MY_LOW);
		digitalWrite(PIN_LED_GREEN, MY_LOW);
		digitalWrite(PIN_LED_BLUE, MY_LOW); 
  }
  if (intDelay > 0 ) delay(intDelay);
}

void SET_LED_Status(int stat){
  SET_LED_Status(stat, 1000);
}

/*
 * feedgps - reads the GPS serial input and passes input to GPS.endcode function 
 */
 static bool feedgps() {
  while (Serial1.available()) {
    if (gps.encode(Serial1.read()))
      return true;
	}
  return false;
}
/*
 * Setup 
 */
void setup() {
	wdt_enable(WDTO_8S);
	wdt_reset();
	//Setup Ports
	Serial.begin(115200);				//Start Debug Serial 0
	Serial1.begin(9600); 				//Start GPS Serial 1
	Serial2.begin(9600);
 
	pinMode(PIN_LED_GREEN, OUTPUT);		//Blue GREEN
	pinMode(PIN_LED_RED, OUTPUT);		//Blue RED
	pinMode(PIN_LED_BLUE, OUTPUT);		//Blue LED
	pinMode(PIN_SPI_CS,OUTPUT);  		//Chip Select Pin for the SD Card
	pinMode(10, OUTPUT);				//SDcard library expect 10 to set set as output.
	
	// Initialise the GPS
	wdt_disable();
	gps.init();						
	gps.configureUbloxSettings();		// Configure Ublox for MY_HIGH altitude mode
	wdt_enable(WDTO_8S);
	// join I2C bus //start I2C transfer to the Module/Transmitter
	Wire.begin();
	//Set up the two EasyTransfer methods
	ETI2Cout.begin(details(mD.i2cOut), &Wire);	//setup the data structure to transfer out
	ETSerialIn.begin(details(vals), &Serial2);
	
	//Start up the LGgyro
    if (LGgyro.init()) {
		#ifdef DEBUG_ON	
			Serial.println("LGgyro OK");
		#endif
		LGgyro.enableDefault();
	} else {
		#ifdef DEBUG_ON	
			Serial.println("LGgyro not working");
		#endif
		SET_LED_Status(SET_LED_WHITE,500); 	//White LED
		SET_LED_Status(SET_LED_RED,1000); 	//Red LED 
	}

	//Start up the accelerometer
	accel = ADXL345(); 						// Create an instance of the accelerometer
	if(accel.EnsureConnected()) {			// Check that the accelerometer is connected.
		#ifdef DEBUG_ON	
			Serial.println("Connected to ADXL345.");
		#endif		
		accel.SetRange(2, true);				// Set the range of the accelerometer to a maximum of 2G.
		accel.EnableMeasurements();				// Tell the accelerometer to start taking measurements.		
	} else{
		#ifdef DEBUG_ON	
			Serial.println("Could not connect to ADXL345.");
		#endif
		SET_LED_Status(SET_LED_WHITE,500); 	//White LED
		SET_LED_Status(SET_LED_RED,2000); 	//Red LED 
	}

	//Start up the compass
	compass = HMC5883L(); 						// Construct a new HMC5883 compass.
	#ifdef DEBUG_ON	
		if(compass.EnsureConnected() == 1) {
			Serial.println("Connected to HMC5883L.");
		} else {
			Serial.println("Not Connected to HMC5883L.");
		}
	#endif
	error = compass.SetScale(1.3); 				// Set the scale of the compass.
	#ifdef DEBUG_ON	
		if(error != 0) {							// If there is an error, print it out.
			Serial.println("Compass Error 1");
			Serial.println(compass.GetErrorText(error));
		} else {
			Serial.println("Compass Ok 1");
		}
	#endif
	error = compass.SetMeasurementMode(Measurement_Continuous); // Set the measurement mode to Continuous
	#ifdef DEBUG_ON	
		if(error != 0) {							// If there is an error, print it out.
			Serial.println("Compass error 2");
			Serial.println(compass.GetErrorText(error));
		} else {
			Serial.println("Compass Ok 2");
		}
	#endif	
	
	//Start up the Pressure Sensor
	dps = BMP085();
	dps.init(); 
	#ifdef DEBUG_ON
		Serial.print("BMP Mode ");
		Serial.println(dps.getMode());
	#endif	
	wdt_reset();
	// Start up the OneWire Sensors library and turn off blocking takes too long!
	sensors.begin();
	sensors.setWaitForConversion(false);
  	sensors.requestTemperaturesByAddress(outsideThermometer); // Send the command to get temperature
	
	//Initialise all of the record values
	mD.vals.tCount = 0;
	mD.vals.uslCount = 0;
	mD.vals.year = 0;
	mD.vals.month = 0;
	mD.vals.day = 0;
	mD.vals.hour = 0;
	mD.vals.minute = 0;
	mD.vals.second = 0;
	mD.vals.hundredths = 0;
	mD.vals.iLat = 0;
	mD.vals.iLong = 0;
	mD.vals.iAlt = 0;
	mD.vals.bSats = 0;
	mD.vals.iAngle = 0;
	mD.vals.iHspeed = 0;
	mD.vals.iVspeed = 0;
	mD.vals.age = 0;
	mD.vals.ihdop = 0;
	mD.vals.AcXPayload = 0;
	mD.vals.AcYPayload = 0;
	mD.vals.AcZPayload = 0;
	mD.vals.GyXPayload = 0;
	mD.vals.GyYPayload = 0;
	mD.vals.GyZPayload = 0;
	mD.vals.MgXPayload = 0;
	mD.vals.MgYPayload = 0;
	mD.vals.MgZPayload = 0;
	mD.vals.TmpPayload = 0;
	
	//Connect to the SD Card	
	if(!SD.begin(PIN_SPI_CS, SPI_HALF_SPEED)) {
		#ifdef DEBUG_ON	
			Serial.println("SD not working!!");
		#endif 
		SET_LED_Status(SET_LED_WHITE,500); 	//White LED
		SET_LED_Status(SET_LED_RED,3000); 	//Red LED 
	} else {
		#ifdef DEBUG_ON	
			Serial.println("SD OK");
		#endif 	
		dataFile.open(SD_LOG_FILE, O_CREAT | O_WRITE | O_APPEND);	    //Open Logfile
		if (!dataFile.isOpen()) {
			#ifdef DEBUG_ON	
				Serial.println("SD Data File Not Opened");
			#endif 	
			SET_LED_Status(SET_LED_WHITE,500);
			SET_LED_Status(SET_LED_RED,3000);
		}
	}

	//Cycle lights
	SET_LED_Status(SET_LED_OFF,0);  
	SET_LED_Status(SET_LED_RED,500);
	SET_LED_Status(SET_LED_GREEN,500);
	SET_LED_Status(SET_LED_BLUE,500);
	SET_LED_Status(SET_LED_OFF,0);  
	
	elapseSIM900 = millis();				//Elapse counter for data to SIM900
	elapseNTXB = millis();					//Elapse counter for data to NTXB
	NEWGPSDATA = false;
	wdt_enable(WDTO_2S);
	wdt_reset();
}

/*
 * Main Loop 
 */
void loop() {
	wdt_reset();
	mD.vals.uslCount++;									//Increment main datarecord count
	AccelerometerScaled Ascaled = accel.ReadScaledAxis();	//Get Scaled Accelerometer
	AccelerometerRaw Araw = accel.ReadRawAxis();			//Get Raw Accelerometer
	MagnetometerScaled Mscaled = compass.ReadScaledAxis();	//Get Scaled Magnetometer
	MagnetometerRaw Mraw = compass.ReadRawAxis();			//Get Raw Magnetometer
	LGgyro.read();											//Get Gyro

	// offset compass by hard iron
	Mraw.XAxis += 40;
	Mraw.YAxis += 261;
	Mraw.ZAxis += 54;

	//write Acc, Mag, & Gyro values to record
	float AxisGs = Ascaled.XAxis;
	mD.vals.AcXPayload = AxisGs * 100;
	AxisGs = Ascaled.YAxis;
	mD.vals.AcYPayload = AxisGs * 100;
	AxisGs = Ascaled.ZAxis;
	mD.vals.AcZPayload = AxisGs * 100;
	mD.vals.MgXPayload = Mscaled.XAxis;
	mD.vals.MgYPayload = Mscaled.YAxis;
	mD.vals.MgZPayload = Mscaled.ZAxis;
	mD.vals.GyXPayload = LGgyro.g.x;
	mD.vals.GyYPayload = LGgyro.g.y;
	mD.vals.GyZPayload = LGgyro.g.z;

	//Perform tilt compensation calculation save to record
	sixDOF.compCompass(Mraw.XAxis, -Mraw.YAxis, -Mraw.ZAxis, Araw.XAxis, Araw.YAxis, Araw.ZAxis, true);
	float compHeading = sixDOF.atan2Int(sixDOF.xAxisComp(), sixDOF.yAxisComp());
	compHeading = compHeading /100;
	if (compHeading < 0 ) {
		compHeading = abs(compHeading);
	} else {
		compHeading = 180 - compHeading + 180;
	}
	mD.vals.CmpssPayload = compHeading;
	
	//get BMP085 values save to record
	dps.getTemperature(&TmpPayloadFULL); 
	dps.getPressure(&mD.vals.PressurePayload);

 	mD.vals.TmpPayload = (int16_t)(TmpPayloadFULL);	
	mD.vals.TmpExternal = (int16_t)(sensors.getTempC(outsideThermometer)* 10);
	sensors.requestTemperaturesByAddress(outsideThermometer); // Send the command to get temperatures
	
	//get GPS data
	byte lcount = 0;									//reset a loop counter
	while (!NEWGPSDATA && lcount++ < 255) {				//Exit the loop if we have new data or have been round it a number of times
		NEWGPSDATA = feedgps();							
	}
	if (NEWGPSDATA) {									//We have new GPS data, get all of the fields we need.
		int tmp_year = 0;
		gps.crack_datetime(&tmp_year, &mD.vals.month, &mD.vals.day,&mD.vals.hour, &mD.vals.minute, &mD.vals.second, &mD.vals.hundredths, &mD.vals.age);
		mD.vals.year = tmp_year - 2000;
		
        if (gps.altitude() != TinyGPS_HJOE::GPS_INVALID_ALTITUDE && gps.altitude() >= 0) {
			gps.get_position(&mD.vals.iLat, &mD.vals.iLong, &mD.vals.age);
			mD.vals.iAlt = gps.altitude(); 
			mD.vals.iAngle = gps.course();
			mD.vals.iHspeed = gps.speed(); 
			mD.vals.bSats = gps.satellites();
			mD.vals.ihdop = gps.hdop();
		}
		SET_LED_Status(SET_LED_BLUE,0);					//Flash blue to show we are getting GPS data
	} else {
		SET_LED_Status(SET_LED_GREEN,0);				//Flash Green to show that we are looping but not getting GPS data
	}

	if(ETSerialIn.receiveData()){

	}
  
	//flip flop between I2C's to avoid both on one loop
	if (SENDWIRE && (millis() - elapseSIM900) > WAIT_SIM900) {
		mD.vals.tCount++;
		ETI2Cout.sendData(I2C_SLV_SIM900_ADDRESS);		
		elapseSIM900 = millis();
	}

	if (!SENDWIRE && (millis() - elapseNTXB) > WAIT_NTXB) {
		mD.vals.tCount++;
		ETI2Cout.sendData(I2C_SLV_NTXB_ADDRESS);
		elapseNTXB = millis();
		//get I2C_SLV_SIM900_ADDRESS data
	}

	writeSDData();										//Write the data record to the SD card
	SET_LED_Status(SET_LED_OFF,0);						//turn off the LED
	NEWGPSDATA = false;									//Reset the New GPS Data flag
	SENDWIRE = !SENDWIRE;								//Flipflop this 
}
