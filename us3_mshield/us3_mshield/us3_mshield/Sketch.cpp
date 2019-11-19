/*Begining of Auto generated code by Atmel studio */
#include <Arduino.h>

/*End of auto generated code by Atmel studio */

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


/*! \mainpage

   \section intro_sec Introduction

   The mshield Arduino application and associated library provides a reference CoAP Server
   for use with the Itron Starfish network and the milli shield hardware.
   Adding the milli shield to the appropriate Arduino board provides the basis for building out
   IoT applications.
   The mshield application was written for the
   Arduino M0 Pro board and the Adafruit Metro M0 Express board.<br>

   The code release consists of the ssni_coap_server library and an Arduino sketch.<br>

   \subsection coap_server The CoAP Server library

   The CoAP Server library is packaged as ssni_coap_server. It contains a basic CoAP server that supports
   the Itron milli chip and Arduino milli shield product and a variety of demonstration sensors.
   
   \subsection The Arduino Sketch

   The Arduino setup() and loop() functions are located in mshield.ino and mshield.h.
   Review the mshield.h as it contains valuable information.<br>

   \section install_sec Installation
   \subsection step1  Step 1:  Download and install Arduino IDE 1.8.7 from https://www.arduino.cc/en/Main/Software
   \subsection step2  Step 2:  In the preferences, locate the field "Additionial Board Manager URLs". Add this URL to the list: https://adafruit.github.io/arduino-board-index/package_adafruit_index.json
   \subsection step3  Step 3:  Using the board manager, add the Adafruit SAMD boards. Select the Adafruit Metro M0 Express board.
   \subsection step4  Step 4:  From the mshield distribution (zip file), copy the sub-folder mshield to C:\Users\<user>\Documents\Arduino.
   \note {Replace <user> with your Windows user name.}
   \subsection step5  Step 5:  Copy the folder ssni_coap_server to C:\Users\<user>\Documents\Arduino\libraries.
   \subsection step6  Step 6:  In the folder Arduino\mshield, click on mshield.ino and your Arduino IDE will launch
   \subsection step7  Step 7:  In the IDE, go to the Sketch menu and choose Include Library, and select ssni_coap_server from the Contributed Libraries section
   \subsection step8  Step 8a: In the IDE, go to Sketch/Include Library/Manage Libraries
   \subsection step9  Step 8b:   In the search bar, type RTCZero
   \subsection step10  Step 8c:  Click on "More info"
   \subsection step11  Step 8d:  In the drop-down menu, select v1.5.2 and click Install
   \subsection step12 Step 9a: The following libraries are needed to support the DHT-11 temp sensor (these are needed even if you do not hook a DHT-11 up):
   \subsection step14 Step 9b:   Adafruit Unified Sensor v1.0.2
   \subsection step15 Step 9c:   DHT Sensor Library v1.3.0
   \subsection step16 Step 10: Build and upload the application to your Adafruit Metro M0 Express board. Explore and have fun!
*/

#include <Arduino.h>
#include "us3_mshield.h"
#include "sapi_error.h"
#include "sapi.h"
#include "TempSensor.h"
#include "EchoSensor.h"
//Beginning of Auto generated function prototypes by Atmel Studio
//End of Auto generated function prototypes by Atmel Studio




static uint8_t temp_sensor_id;
static uint8_t echo_sensor_id;

//
//  Arduino setup function.
//
void setup()
{
	sapi_error_t rcode;
	
	// Initialize Sensor API
	sapi_initialize(NULL);
	
	// Register temp sensor
	temp_sensor_id = sapi_register_sensor(TEMP_SENSOR_TYPE, temp_init_sensor, temp_read_sensor, temp_read_cfg, temp_write_cfg, 1, 60);
	
	// Initialize temp sensor
	rcode = sapi_init_sensor(temp_sensor_id);
	
	
	// Register echo sensor
	//echo_sensor_id = sapi_register_sensor(ECHO_SENSOR_TYPE, echo_init_sensor, echo_read_sensor, NULL, echo_write_cfg, 1, 60);
	
	// Initialize echo sensor
	//rcode = sapi_init_sensor(echo_sensor_id);
	
	
}


//
// Arduino main loop function.
//
void loop()
{
	// Call SAPI run to do the heavy lifting
	sapi_run();

}