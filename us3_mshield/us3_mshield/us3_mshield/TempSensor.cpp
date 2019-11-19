/*

Copyright (c) Silver Spring Networks, Inc.
All rights reserved.

Permission is hereby granted, free of charge, to any person obtaining a copy of
this software and associated documentation files (the ""Software""), to deal in
the Software without restriction, including without limitation the rights to
use, copy, modify, merge, publish, distribute, sublicense, and/or sell copies of
the Software, and to permit persons to whom the Software is furnished to do so,
subject to the following conditions:

The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED AS IS, WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS
FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR
COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER
IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN
CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.

Except as contained in this notice, the name of Silver Spring Networks, Inc.
shall not be used in advertising or otherwise to promote the sale, use or other
dealings in this Software without prior written authorization from Silver Spring
Networks, Inc.

*/
#include "TempSensor.h"
#include <Filters.h> 


// DHT11 Sensor Object
#define DHT_TYPE           DHT11
DHT_Unified dht(A1, DHT_TYPE);

// Sensor Context. Contains the unit of measure and alert state.
static temp_ctx_t context;



//////////////////////////////////////////////////////////////////////////
//
// Initialization functions. Callback functions. Payload building functions.
//
//////////////////////////////////////////////////////////////////////////

//////////////////////////////////////////////////////////////////////////
//
// Initialize DHT11 temp sensor. Callback called by sapi_init_sensor function.
//
//////////////////////////////////////////////////////////////////////////
sapi_error_t temp_init_sensor()
{
	// Set context defaults and enable the sensor
	context.scalecfg = FAHRENHEIT_SCALE;
	context.alertstate = tsat_disabled;
	temp_sensor_enable();
	
	// Initialize temperature/humidity sensor
	dht.begin();
	
	// Log a banner for the sensor with sensor details
	println("DHT11 Sensor Initialized!");
	
	sensor_t sensor;
	dht.temperature().getSensor(&sensor);
	println("");
	println("------------------------------------");
	print  ("Sensor:       "); println(sensor.name);
	print  ("Driver Ver:   "); printnum(sensor.version);    println("");
	print  ("Unique ID:    "); printnum(sensor.sensor_id);  println("");
	print  ("Max Value:    "); printnum(sensor.max_value);  println(" C");
	print  ("Min Value:    "); printnum(sensor.min_value);  println(" C");
	print  ("Resolution:   "); printnum(sensor.resolution); println(" C");
	println("------------------------------------");        println("");

	return SAPI_ERR_OK;
}


//////////////////////////////////////////////////////////////////////////
//
// Reads a DHT11 sensor. Builds and returns the payload. Callback called on
//  CoAP Get sensor value
//  CoAP Observe notification of sensor value
//
//////////////////////////////////////////////////////////////////////////
sapi_error_t temp_read_sensor(char *payload, uint8_t *len)
{
	float reading = 0.0;
    sapi_error_t rc;
	char buffer[64];

	// Read temp sensor, already in network order
    rc = read_dht11(&reading);
	if (rc != SAPI_ERR_OK)
	{
		return rc;
	}
	
	// Assemble the Payload
	rc = temp_build_payload(buffer, &reading);
	strcpy(payload, buffer);
	*len = strlen(buffer);
    return rc;
}


//////////////////////////////////////////////////////////////////////////
//
// Reads a DHT11 sensor. Read sensor configuration. Builds and returns the payload. Callback called on
//   CoAP Get configuration value
//
//////////////////////////////////////////////////////////////////////////
sapi_error_t temp_read_cfg(char *payload, uint8_t *len)
{
	// Assemble the Payload
	// Trick - if sensor value is NULL than the payload builder just returns the UOM.
	sapi_error_t rc = temp_build_payload(payload, NULL);
	strcpy(payload, payload);
	*len = strlen(payload);
	
    return rc;
}


//////////////////////////////////////////////////////////////////////////
//
// Write sensor configuration. Processes payload sent from client. Callback called on
//  CoAP Put configuration value
//
//////////////////////////////////////////////////////////////////////////
sapi_error_t temp_write_cfg(char *payload, uint8_t *len)
{
	if (!strcmp(payload, "cfg=C"))
	{
		context.scalecfg = CELSIUS_SCALE;
	}
	else if (!strcmp(payload, "cfg=F"))
	{
		context.scalecfg = FAHRENHEIT_SCALE;
	}
	// Config not supported
	else
	{
		return SAPI_ERR_NOT_IMPLEMENTED;
	}

	return SAPI_ERR_OK;
}

//////////////////////////////////////////////////////////////////////////
//
// Measure AC Voltage ZMPT101B And AC Current ACS712
// Created 04/10/2019
// By Ryan Trisnojoyo
//////////////////////////////////////////////////////////////////////////
// Measuring Voltage Using ZMPT101B
float testFrequency = 50;                     // test signal frequency (Hz)
float windowLength = 40.0/testFrequency;     // how long to average the signal, for statistist

int rawValue = 0;
float voltageReading = 0;
float slope = 0.0405;
int sensorValue = 0;
float intercept = -0.1129; // to be adjusted based on calibration testing
int temp = 0;
int temp1 = 0;

float current_amps; // estimated actual current in amps
/*
Measuring AC Current Using ACS712
*/


float Vref = 0;
const int ACPin = A5;         //set arduino signal read pin
#define ACTectionRange 20;    //set Non-invasive AC Current Sensor tection range (20A,30A,50A,100A)

/*
 * relay have to be powered with 3.3V
 * LOW to turn on
 * HIGH to turn off
 */
//digitalWrite(pinRelay, LOW); // turn on tehe light
float calculateVoltage(){
  //digitalWrite(pinRelay, LOW); //turn on the light
  //analogWrite(pinRelay, 255);
  // Measuring Voltage
  RunningStatistics inputStats;
  inputStats.setWindowSecs( windowLength );

  //take 10000 samples to get more accurate reading
  while(temp != 10000){
    rawValue = analogRead(A4);
    inputStats.input(rawValue);

    //Serial.print("Raw Value : ");
    //Serial.println(inputStats.sigma());

    voltageReading = slope * inputStats.sigma();
    //voltageReading = voltageReading * 40.3231;
    voltageReading = voltageReading * 47.1931;

    temp = temp + 1;
  }
	temp = 0;
  //Serial.print("Voltage = ");
  //Serial.println(voltageReading);

  return voltageReading;
 }

 float readACCurrentValue()
{
  float ACCurrtntValue = 0;
  unsigned int peakVoltage = 0;
  unsigned int voltageVirtualValue = 0;  //Vrms
  for (int i = 0; i < 3000; i++ )
  {
    peakVoltage  += analogRead(ACPin);   //read peak voltage
    delay(1);
  }
  peakVoltage = peakVoltage / 5;
  voltageVirtualValue = peakVoltage * 0.707;    //change the peak voltage to the Virtual Value of voltage

  /*The circuit is amplified by 2 times, so it is divided by 2.*/
  voltageVirtualValue = (voltageVirtualValue * Vref / 1024) / 2;

  ACCurrtntValue = voltageVirtualValue * ACTectionRange;

  return ACCurrtntValue/1000;
}

/*Read reference voltage*/
 long readVref()
 {
   long result;
 #if defined(__AVR_ATmega168__) || defined(__AVR_ATmega328__) || defined (__AVR_ATmega328P__)
   ADMUX = _BV(REFS0) | _BV(MUX3) | _BV(MUX2) | _BV(MUX1);
 #elif defined(__AVR_ATmega32U4__) || defined(__AVR_ATmega1280__) || defined(__AVR_ATmega2560__) || defined(__AVR_AT90USB1286__)
   ADMUX = _BV(REFS0) | _BV(MUX4) | _BV(MUX3) | _BV(MUX2) | _BV(MUX1);
   ADCSRB &= ~_BV(MUX5);   // Without this the function always returns -1 on the ATmega2560 http://openenergymonitor.org/emon/node/2253#comment-11432
 #elif defined (__AVR_ATtiny24__) || defined(__AVR_ATtiny44__) || defined(__AVR_ATtiny84__)
   ADMUX = _BV(MUX5) | _BV(MUX0);
 #elif defined (__AVR_ATtiny25__) || defined(__AVR_ATtiny45__) || defined(__AVR_ATtiny85__)
   ADMUX = _BV(MUX3) | _BV(MUX2);
 #endif
 #if defined(__AVR__)
   delay(2);                                        // Wait for Vref to settle
   ADCSRA |= _BV(ADSC);                             // Convert
   while (bit_is_set(ADCSRA, ADSC));
   result = ADCL;
   result |= ADCH << 8;
   result = 1126400L / result;  //1100mV*1024 ADC steps http://openenergymonitor.org/emon/node/1186
   return result;
 #elif defined(__arm__)
   return (3300);                                  //Arduino Due
 #else
   return (3300);                                  //Guess that other un-supported architectures will be running a 3.3V!
 #endif
 }
//////////////////////////////////////////////////////////////////////////
//
// Code to build the sensor payload. Temp payload is text with this format:
//   <epoch>,<temp>,<uom>
//     <epoch> is a 4 byte (8 hex char) timestamp
//     <temp> is a decimal number
//     <uom> is a char: C or F
//
//  Note that the payload is text. Payloads can also be a byte array of binary data.
//
//////////////////////////////////////////////////////////////////////////
sapi_error_t temp_build_payload(char *buf, float *reading)
{
	char 		payload[64];
	char		reading_buf[32];
	char        datatype30[] = "30,";
	char		datatype31[] = "31,"; //current datatype is 31
	char		unitVoltage[] = "V";
	char		unitCurrent[] = "A";
	char		unit_buf[4];
	time_t     	epoch;
	uint32_t	indx;
	char    rvoltage[] = "120.00,";
	char    rcurrent[] = "0.23,";
	char    temp_epoch[20];
	
	
	
	Vref = readVref();
	sprintf(rvoltage, "%.2f,", calculateVoltage());
	sprintf(rcurrent, "%.2f,", readACCurrentValue());
	
	// Create string containing the UNIX epoch
	epoch = get_rtc_epoch();
	sprintf(temp_epoch, "%d,", epoch);
	sprintf(payload, "%d,", epoch);
	strcat(payload, datatype30);
	strcat(payload, rvoltage);
	strcpy(unit_buf, unitVoltage);
	strcat(payload, unit_buf);
	strcat(payload, ";");
	
	strcat(payload,temp_epoch);
	strcat(payload, datatype31);
	strcat(payload, rcurrent);
	strcpy(unit_buf, unitCurrent);
	strcat(payload, unit_buf);
	
	/*
	// Check if we have a sensor reading
	if (reading != NULL)
	{
		// Create string containing reading and add it to the buffer
		sprintf(reading_buf, ",%.2f", *reading);
		strcat(payload, reading_buf);
	}
	
	// Add scale
	strcpy( unit_buf, ",F" );
	if ( context.scalecfg == CELSIUS_SCALE )
	{
		strcpy( unit_buf, ",C" );
	}
	
	// Add scale to payload
	strcat(payload, unit_buf);
	*/
	
	/*
	int valueA0 = analogRead(A0);
	int valueA2 = analogRead(A2);
	int valueA4 = analogRead(A4);
	char data0[] = "1";
	char data2[] = "2";
	char data4[] = "3";

	epoch = get_rtc_epoch();
	sprintf(payload, "%d,", epoch);
	
	strcat(payload,"A0_5V,");
	sprintf(data0, "%d,", valueA0);
	strcat(payload, data0);
	strcat(payload, ";A2_12V,");
	sprintf(data2, "%d,", valueA2);
	strcat(payload, data2);
	strcat(payload, ";A4,");
	sprintf(data4, "%d", valueA4);
	strcat(payload, data4);
*/
	strcpy(buf, payload);
	
	dlog(LOG_DEBUG, "Temp Payload: %s", payload);
	return SAPI_ERR_OK;
}



//////////////////////////////////////////////////////////////////////////
//
// Sensor (hardware) access functions.
//
//////////////////////////////////////////////////////////////////////////

//////////////////////////////////////////////////////////////////////////
//
// Read the temp value from the DHT11 sensor.
//
//////////////////////////////////////////////////////////////////////////
sapi_error_t read_dht11(float *reading)
{
	sapi_error_t rc = SAPI_ERR_OK;
	float re = INVALID_TEMP;

	// Get temperature event
	sensors_event_t event;
	dht.temperature().getEvent(&event);

	// Check for NaN
	if (isnan(event.temperature))
	{
		re = NO_SENSOR_TEMP;
		rc = SAPI_ERR_OK;
	}
	else
	{
		re = event.temperature;
		rc = SAPI_ERR_OK;
	}
	
	// Reading is in C. Convert to F if needed.
	if (context.scalecfg == FAHRENHEIT_SCALE)
	{
		// Convert from Celsius to Fahrenheit
		re *= 1.8;
		re += 32;
	}
	
	// Assign output
	*reading = re;
	return rc;
}


//////////////////////////////////////////////////////////////////////////
//
// Context and Alert functions. Support for sensor enable and disable. Support for alerts.
//
//////////////////////////////////////////////////////////////////////////

sapi_error_t temp_sensor_enable(void)
{
	context.enable = 1;
	context.alertstate = tsat_cleared;
	return SAPI_ERR_OK;
}


sapi_error_t temp_sensor_disable(void)
{
	context.enable = 0;
	context.alertstate = tsat_disabled;
	return SAPI_ERR_OK;
}