/*
    
 HABJOE  Arduino Flight script
 HABJOE - Copyright Andrew Myatt and Joseph Myatt March/2013
 
 KEYWORDS: ARDUINO MINI PRO, MY_HIGH ALTITUDE BALLOON, BMP085, ADXL345, UBLOX, I2C, WIRING
 
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
 
 POLLS SENSORS GENRATES DATA STRUCTURE WHICH IS WRITTEN VIA SERIAL TO MAIN CONTROL BOARD

 */

/*
 * Include Files
 */
#include <Wire.h>
#include <L3G.h>
#include <ADXL345.h>
#include <Comp6DOF_n0m1.h>
#include <HMC5883L.h> // Reference the HMC5883L Compass Library
#include <BMP085.h>
#include <OneWire.h>
#include <DallasTemperature.h>
#include <EasyTransfer.h>
//#include <avr/wdt.h>

/*
 * Definitions
 */
// build modifiers
#define DEBUG_ON
#define MY_HIGH HIGH
#define MY_LOW LOW

// LED Definitions
#define SET_ON 1
#define SET_OFF 0

// Millisecond wait between Tones
#define INTERVAL_TONES  				300000	//5 minutes

// Camera on/off Temperatures

#define CAM_ON_TEMP  				550	//55degC *10 
#define CAM_OFF_TEMP  				800	//80degC *10 

// Arduino Pin assignment
#define PIN_LED 13		       		//Fixed:  LED
#define PIN_IC2_SDA 20       		//Fixed: SDA
#define PIN_IC2_SLC 21       		//Fixed: SLC
#define PIN_BUZZ 5					//Sound buzzer for syncronising Cameras
#define CAMERA_POWER 9				//Mosfet controlling Camera power
#define ONE_WIRE_BUS 12				//Camera Temperature input

#define ADXL345_ADDRESS 0x53		// Device address as specified in data sheet 
#define HMC5883_ADDRESS 0x1E 		//0011110b, I2C 7bit address of HMC5883
#define BMP085_ADDRESS 0x77  		// I2C address of BMP085
#define aref_voltage 3.3 

/*************************************************
 * Public Constants
 *************************************************/

#define NOTE_B0  31
#define NOTE_C1  33
#define NOTE_CS1 35
#define NOTE_D1  37
#define NOTE_DS1 39
#define NOTE_E1  41
#define NOTE_F1  44
#define NOTE_FS1 46
#define NOTE_G1  49
#define NOTE_GS1 52
#define NOTE_A1  55
#define NOTE_AS1 58
#define NOTE_B1  62
#define NOTE_C2  65
#define NOTE_CS2 69
#define NOTE_D2  73
#define NOTE_DS2 78
#define NOTE_E2  82
#define NOTE_F2  87
#define NOTE_FS2 93
#define NOTE_G2  98
#define NOTE_GS2 104
#define NOTE_A2  110
#define NOTE_AS2 117
#define NOTE_B2  123
#define NOTE_C3  131
#define NOTE_CS3 139
#define NOTE_D3  147
#define NOTE_DS3 156
#define NOTE_E3  165
#define NOTE_F3  175
#define NOTE_FS3 185
#define NOTE_G3  196
#define NOTE_GS3 208
#define NOTE_A3  220
#define NOTE_AS3 233
#define NOTE_B3  247
#define NOTE_C4  262
#define NOTE_CS4 277
#define NOTE_D4  294
#define NOTE_DS4 311
#define NOTE_E4  330
#define NOTE_F4  349
#define NOTE_FS4 370
#define NOTE_G4  392
#define NOTE_GS4 415
#define NOTE_A4  440
#define NOTE_AS4 466
#define NOTE_B4  494
#define NOTE_C5  523
#define NOTE_CS5 554
#define NOTE_D5  587
#define NOTE_DS5 622
#define NOTE_E5  659
#define NOTE_F5  698
#define NOTE_FS5 740
#define NOTE_G5  784
#define NOTE_GS5 831
#define NOTE_A5  880
#define NOTE_AS5 932
#define NOTE_B5  988
#define NOTE_C6  1047
#define NOTE_CS6 1109
#define NOTE_D6  1175
#define NOTE_DS6 1245
#define NOTE_E6  1319
#define NOTE_F6  1397
#define NOTE_FS6 1480
#define NOTE_G6  1568
#define NOTE_GS6 1661
#define NOTE_A6  1760
#define NOTE_AS6 1865
#define NOTE_B6  1976
#define NOTE_C7  2093
#define NOTE_CS7 2217
#define NOTE_D7  2349
#define NOTE_DS7 2489
#define NOTE_E7  2637
#define NOTE_F7  2794
#define NOTE_FS7 2960
#define NOTE_G7  3136
#define NOTE_GS7 3322
#define NOTE_A7  3520
#define NOTE_AS7 3729
#define NOTE_B7  3951
#define NOTE_C8  4186
#define NOTE_CS8 4435
#define NOTE_D8  4699
#define NOTE_DS8 4978

//Data structure used for sending data via the Serial

typedef struct{
	unsigned long usl_count;  		//4
	int16_t BMP085_T;				//2		Pressure Temp (*x0.1 DegC)
	int16_t	DS18B20_T_CM;			//2	DS18B20 Temp (*x0.1 DegC)
	int16_t Xax;					//2		accel x
	int16_t Xay;					//2		accel y
	int16_t Xaz;					//2		accel z
	int16_t Xgx;					//2		gyro x
	int16_t Xgy;					//2		gyro y
	int16_t Xgz; 					//2		gyro z 
	int16_t Xmx;					//2		mag x
	int16_t Xmy;					//2		mag y
	int16_t Xmz; 					//2		mag z 
	long    i_compass;				//4  	compass bearing using tilt compensation etc.
	long	BMP085_PFULL;			//4
	byte	CamStatus;				//1	
	} LOG_DATA_STRUCTURE;	
LOG_DATA_STRUCTURE vals;
		
// Declare datatype variables
L3G 				LGgyro;					//L3G Gyro
ADXL345 			accel;					//accelerometer
HMC5883L 			compass;				//Magnetometer/Compass
Comp6DOF_n0m1 		sixDOF;					//Tilt compensation from Compass
BMP085 				dps;					//Pressure and Temp
OneWire 			oneWire(ONE_WIRE_BUS);	// Set up which Arduino pin will be used for the 1-wire interface to the sensor
DallasTemperature 	sensors(&oneWire);
DeviceAddress 		cameraThermometer = { 0x28, 0x44, 0xD8, 0x7D, 0x5, 0x0, 0x0, 0xD7 };
EasyTransfer   		ETSerialOut;

// Declare utility global variables variables
int    	error = 0;
long   	BMP085_TFULL;
long	BMP085_ALT;
unsigned long 	elapseTones;


/*
 * functions
 */
 void setPwmFrequency(int pin, int divisor) {
 byte mode;
 if(pin == 5 || pin == 6 || pin == 9 || pin == 10) {
 switch(divisor) {
 case 1:
 mode = 0x01;
 break;
 case 8:
 mode = 0x02;
 break;
 case 64:
 mode = 0x03;
 break;
 case 256:
 mode = 0x04;
 break;
 case 1024:
 mode = 0x05;
 break;
 default:
 return;
 }
 if(pin == 5 || pin == 6) {
 TCCR0B = TCCR0B & 0b11111000 | mode;
 }
 else {
 TCCR1B = TCCR1B & 0b11111000 | mode;
 }
 }
 else if(pin == 3 || pin == 11) {
 switch(divisor) {
 case 1:
 mode = 0x01;
 break;
 case 8:
 mode = 0x02;
 break;
 case 32:
 mode = 0x03;
 break;
 case 64:
 mode = 0x04;
 break;
 case 128:
 mode = 0x05;
 break;
 case 256:
 mode = 0x06;
 break;
 case 1024:
 mode = 0x7;
 break;
 default:
 return;
 }
 TCCR2B = TCCR2B & 0b11111000 | mode;
 }
}
 
 /***************************************************
 * Melody to play when the camera is switched on
 * to synch the camera's via sound.
 **************************************************/
 
// notes in the startMelody:
int startMelody[] = {
	NOTE_D3,0,NOTE_C8,NOTE_D3,0,NOTE_E7
};
int intervalMelody[] = {
	NOTE_G4,0,NOTE_C8,NOTE_G4,0,NOTE_E7
};
// note durations: 4 = quarter note, 8 = eighth note, etc.:
int noteDurations[] = {
	8,8,4,8,8,8
};

void doStartTones() {
	for (int thisNote = 0; thisNote < 6; thisNote++) {
		int noteDuration = 1000 / noteDurations[thisNote];
		tone(PIN_BUZZ, startMelody[thisNote], noteDuration);
		int pauseBetweenNotes = noteDuration * 1.30;
		delay(pauseBetweenNotes);
		noTone(PIN_BUZZ);
	}
}

void doIntervalTones() {
	for (int thisNote = 0; thisNote < 6; thisNote++) {
		int noteDuration = 1000 / noteDurations[thisNote];
		tone(PIN_BUZZ, intervalMelody[thisNote], noteDuration);
		int pauseBetweenNotes = noteDuration * 1.30;
		delay(pauseBetweenNotes);
		noTone(PIN_BUZZ);
	}
}

/***************************************************
 * LED STATUS DISPLAY FUNTIONS
 * LED is a Common Anode, therefore the pin is the cathode
 * and must be set MY_LOW to be on, and MY_HIGH to be turned off! 
 **************************************************/
void SET_LED_Status(int stat, int intDelay){
 
  if (stat == SET_ON) {
		digitalWrite(PIN_LED, MY_HIGH);
  }
    if (stat == SET_OFF) {
		digitalWrite(PIN_LED, MY_LOW);
  }
  if (intDelay > 0 ) delay(intDelay);
}

void SET_LED_Status(int stat){
  SET_LED_Status(stat, 1000);
}

void doSendData(){
	vals.usl_count++;										//Increment main datarecord count
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
	vals.Xax = AxisGs * 100;
	AxisGs = Ascaled.YAxis;
	vals.Xay = AxisGs * 100;
	AxisGs = Ascaled.ZAxis;
	vals.Xaz = AxisGs * 100;
	vals.Xmx = Mscaled.XAxis;
	vals.Xmy = Mscaled.YAxis;
	vals.Xmz = Mscaled.ZAxis;
	vals.Xgx = LGgyro.g.x;
	vals.Xgy = LGgyro.g.y;
	vals.Xgz = LGgyro.g.z;

	//Perform tilt compensation calculation save to record
	sixDOF.compCompass(Mraw.XAxis, -Mraw.YAxis, -Mraw.ZAxis, Araw.XAxis, Araw.YAxis, Araw.ZAxis, true);
	float compHeading = sixDOF.atan2Int(sixDOF.xAxisComp(), sixDOF.yAxisComp());
	compHeading = compHeading /100;
	if (compHeading < 0 ) {
		compHeading = abs(compHeading);
	} else {
		compHeading = 180 - compHeading + 180;
	}
	vals.i_compass = compHeading;
	
	//get BMP085 values save to record
	dps.getTemperature(&BMP085_TFULL); 
	dps.getPressure(&vals.BMP085_PFULL);

 	vals.BMP085_T = (int16_t)(BMP085_TFULL);
	vals.DS18B20_T_CM = (int16_t)(sensors.getTempC(cameraThermometer)* 10);
	sensors.requestTemperaturesByAddress(cameraThermometer); // Send the command to get temperatures
	ETSerialOut.sendData();
}

/*
 * Setup 
 */
void setup() {
	//wdt_enable(WDTO_8S);
	//wdt_reset();
	//Setup Ports
	Serial.begin(9600);			//Start Debug Serial 0
    ETSerialOut.begin(details(vals), &Serial);
	pinMode(PIN_LED, OUTPUT);
	digitalWrite(PIN_LED, MY_LOW);	//LED off
	pinMode(CAMERA_POWER, OUTPUT);
	digitalWrite(CAMERA_POWER, MY_LOW);	// Camera off
	
	//wdt_disable();
	// join I2C bus //start I2C transfer to the Module/Transmitter
	Wire.begin();

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
		SET_LED_Status(SET_ON,500); 	//White LED
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
		SET_LED_Status(SET_ON,500); 	//White LED
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
    dps.init(MODE_STANDARD, 7500, true); 		//Set Starting Altitude at 75 Meters
	#ifdef DEBUG_ON
		Serial.print("BMP Mode ");
		Serial.println(dps.getMode());
	#endif	
	//wdt_reset();
	
	sensors.begin();
	sensors.setWaitForConversion(false);
  	sensors.requestTemperaturesByAddress(cameraThermometer); // Send the command to get temperature	

	//Initialise values
	vals.usl_count = 0;
	vals.CamStatus = 0;			// Camera off
	elapseTones = millis();		//Elapse counter for data to SIM900
	//Cycle lights
	SET_LED_Status(SET_OFF,0);  
	SET_LED_Status(SET_ON,500);
	SET_LED_Status(SET_OFF,0);  
	
	//wdt_enable(WDTO_2S);
	//wdt_reset();
}

/*
 * Main Loop 
 */
void loop() {
	if ((millis() - elapseTones) > INTERVAL_TONES) {
		SET_LED_Status(SET_ON,0);
		doIntervalTones();
		elapseTones = millis();
	}
	//wdt_reset();
	SET_LED_Status(SET_OFF,0);						//turn off the LED

	//Camera Logic
	
	if (vals.DS18B20_T_CM < CAM_ON_TEMP) {
		//Switch the Camera on
		digitalWrite(CAMERA_POWER, MY_HIGH);	// Camera on
		delay(10000);	// wait 10 seconds
		digitalWrite(CAMERA_POWER, MY_LOW);		// Camera off
		delay(10000);	// wait 10 seconds
		digitalWrite(CAMERA_POWER, MY_HIGH);	// Camera on
		delay(5000);	// wait 5 seconds
		vals.CamStatus = SET_ON;
		doSendData();
		doStartTones();
		}

	if (vals.DS18B20_T_CM > CAM_OFF_TEMP) {
		//Switch the Camera off its overheating
		vals.CamStatus == SET_OFF;
		doSendData();
		digitalWrite(CAMERA_POWER, MY_LOW);		// Camera off
	}
	
	doSendData();
}
