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

#ifndef RTC_ZERO_H
#define RTC_ZERO_H

#include "Arduino.h"


// Is syncbusy forward declaration
static inline bool is_rtc_syncbusy();


// RTC specific clock register helper structure. Supports 24 hour clock.
struct rtc_clockreg_value
{
	uint8_t  second;
	uint8_t  minute;
	uint8_t  hour;
	uint8_t  day;
	uint8_t  month;
	uint16_t year;
};


class RTCZero {
	
public:
	RTCZero();
	void begin();
  
	// Time/Date get functions
	uint8_t getSeconds();
	uint8_t getMinutes();
	uint8_t getHours();
	uint8_t getDay();
	uint8_t getMonth();
	uint8_t getYear();

	// Time/Date set functions
	void setSeconds(uint8_t seconds);
	void setMinutes(uint8_t minutes);
	void setHours(uint8_t hours);
	void setTime(uint8_t hours, uint8_t minutes, uint8_t seconds);
	void setDay(uint8_t day);
	void setMonth(uint8_t month);
	void setYear(uint8_t year);
	void setDate(uint8_t day, uint8_t month, uint8_t year);

	// Epoch functions
	uint32_t getEpoch();
	void setEpoch(uint32_t epoch);
	
	// Utility functions
	void rtc_set_time(rtc_clockreg_value *time);

private:
	bool _configured;
	
	uint32_t rtc_get_time();
	uint32_t rtc_time_to_register_value(rtc_clockreg_value *time);
	void rtc_register_value_to_time(uint32_t register_value, rtc_clockreg_value *time);

	void RTCdisable();
	void RTCenable();
	void RTCreset();
};

#endif // RTC_ZERO_H
