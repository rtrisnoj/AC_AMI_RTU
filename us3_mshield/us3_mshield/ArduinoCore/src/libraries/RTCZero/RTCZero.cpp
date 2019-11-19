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

#include <time.h>
#include "RTCZero.h"

#define EPOCH_TIME_OFF      946684800  // This is 1st January 2000, 00:00:00 in epoch time


RTCZero::RTCZero()
{
	_configured = false;
}

void RTCZero::begin()
{
	// MCLK Configuration
	MCLK->APBAMASK.reg |= MCLK_APBAMASK_RTC;				// turn on digital interface clock
  
	// 32K Oscillator configuration - select the 1K ISC32KCTRL output so we can count seconds.
	OSC32KCTRL->RTCCTRL.reg = OSC32KCTRL_RTCCTRL_RTCSEL_XOSC1K;
	
	// Reset the RTC (includes disable)
	RTCreset();

	// RTC specific configuration
	uint16_t tmp_reg = 0;
	tmp_reg = RTC_MODE2_CTRLA_MODE(2) | RTC_MODE2_CTRLA_PRESCALER_DIV1024;	// set prescaler to 1024 for MODE2
	tmp_reg |= RTC_MODE2_SYNCBUSY_CLOCKSYNC;				// set CLOCKSYNC
	tmp_reg &= ~RTC_MODE2_CTRLA_CLKREP;					// 24h time representation
	tmp_reg &= ~RTC_MODE2_CTRLA_MATCHCLR;				// disable clear on match


	RTC->MODE2.CTRLA.reg = tmp_reg;


	// Enable RTC and Sync Busy
	RTCenable();

	_configured = true;
}


/*
 * Get Functions
 */

uint8_t RTCZero::getSeconds()
{
	uint32_t register_value = rtc_get_time();
	
	int8_t value = ((register_value & RTC_MODE2_CLOCK_SECOND_Msk) >> RTC_MODE2_CLOCK_SECOND_Pos);
	return value;
}

uint8_t RTCZero::getMinutes()
{
	uint32_t register_value = rtc_get_time();
	
	int8_t value = ((register_value & RTC_MODE2_CLOCK_MINUTE_Msk) >> RTC_MODE2_CLOCK_MINUTE_Pos);
	return value;
}

uint8_t RTCZero::getHours()
{
	uint32_t register_value = rtc_get_time();
	
	int8_t value = ((register_value & RTC_MODE2_CLOCK_HOUR_Msk) >> RTC_MODE2_CLOCK_HOUR_Pos);
	return value;
}

uint8_t RTCZero::getDay()
{
	uint32_t register_value = rtc_get_time();
	
	int8_t value = ((register_value & RTC_MODE2_CLOCK_DAY_Msk) >> RTC_MODE2_CLOCK_DAY_Pos);
	return value;
}

uint8_t RTCZero::getMonth()
{
	uint32_t register_value = rtc_get_time();
	
	int8_t value = ((register_value & RTC_MODE2_CLOCK_MONTH_Msk) >> RTC_MODE2_CLOCK_MONTH_Pos);
	return value;
}

uint8_t RTCZero::getYear()
{
	uint32_t register_value = rtc_get_time();
	
	int8_t value = ((register_value & RTC_MODE2_CLOCK_YEAR_Msk) >> RTC_MODE2_CLOCK_YEAR_Pos);
	return value;
}

/*
 * Set Functions
 */

void RTCZero::setSeconds(uint8_t seconds)
{
	if (_configured) {
		RTC_MODE2_CLOCK_Type rtcClock;
		rtcClock.reg = RTC->MODE2.CLOCK.reg;
		while (is_rtc_syncbusy())
		{
			// Wait for synchronization
		}
		rtcClock.bit.SECOND = seconds;
		RTC->MODE2.CLOCK.reg = rtcClock.reg;
		while (is_rtc_syncbusy())
		{
			// Wait for synchronization
		}
	}
}

void RTCZero::setMinutes(uint8_t minutes)
{
	if (_configured) {
		RTC_MODE2_CLOCK_Type rtcClock;
		rtcClock.reg = RTC->MODE2.CLOCK.reg;
		while (is_rtc_syncbusy())
		{
			// Wait for synchronization
		}
		rtcClock.bit.MINUTE = minutes;
		RTC->MODE2.CLOCK.reg = rtcClock.reg;
		while (is_rtc_syncbusy())
		{
			// Wait for synchronization
		}
	}
}

void RTCZero::setHours(uint8_t hours)
{
	if (_configured) {
		RTC_MODE2_CLOCK_Type rtcClock;
		rtcClock.reg = RTC->MODE2.CLOCK.reg;
		while (is_rtc_syncbusy())
		{
			// Wait for synchronization
		}
		rtcClock.bit.HOUR = hours;
		RTC->MODE2.CLOCK.reg = rtcClock.reg;
		while (is_rtc_syncbusy())
		{
			// Wait for synchronization
		}
	}
}

void RTCZero::setTime(uint8_t hours, uint8_t minutes, uint8_t seconds)
{
  if (_configured) {
	setSeconds(seconds);
	setMinutes(minutes);
	setHours(hours);
  }
}

void RTCZero::setDay(uint8_t day)
{
	if (_configured) {
		RTC_MODE2_CLOCK_Type rtcClock;
		rtcClock.reg = RTC->MODE2.CLOCK.reg;
		while (is_rtc_syncbusy())
		{
			// Wait for synchronization
		}
		rtcClock.bit.DAY = day;
		RTC->MODE2.CLOCK.reg = rtcClock.reg;
		while (is_rtc_syncbusy())
		{
			// Wait for synchronization
		}
	}
}

void RTCZero::setMonth(uint8_t month)
{
	if (_configured) {
		RTC_MODE2_CLOCK_Type rtcClock;
		rtcClock.reg = RTC->MODE2.CLOCK.reg;
		while (is_rtc_syncbusy())
		{
			// Wait for synchronization
		}
		
		rtcClock.bit.MONTH = month;
		RTC->MODE2.CLOCK.reg = rtcClock.reg;
		while (is_rtc_syncbusy())
		{
			// Wait for synchronization
		}
	}
}

void RTCZero::setYear(uint8_t year)
{
	if (_configured) {
		RTC_MODE2_CLOCK_Type rtcClock;
		rtcClock.reg = RTC->MODE2.CLOCK.reg;
		while (is_rtc_syncbusy())
		{
			// Wait for synchronization
		}
		
		rtcClock.bit.YEAR = year;
		RTC->MODE2.CLOCK.reg = rtcClock.reg;
		while (is_rtc_syncbusy())
		{
			// Wait for synchronization
		}
	}
}

void RTCZero::setDate(uint8_t day, uint8_t month, uint8_t year)
{
  if (_configured) {
	setDay(day);
	setMonth(month);
	setYear(year);
  }
}


uint32_t RTCZero::getEpoch()
{
	//begin();
	uint32_t register_value = rtc_get_time();
	struct rtc_clockreg_value time;
	rtc_register_value_to_time(register_value, &time);

	struct tm tm;
	tm.tm_year = time.year - 1900;		// Years since 1900
	tm.tm_mon  = time.month - 1;
	tm.tm_mday = time.day;
	tm.tm_hour = time.hour;
	tm.tm_min  = time.minute;
	tm.tm_sec  = time.second;
	uint32_t epoch = mktime(&tm);

	return epoch;
}


void RTCZero::setEpoch(uint32_t epoch)
{
	if (epoch < EPOCH_TIME_OFF)
	{
		epoch = EPOCH_TIME_OFF;
	}

	time_t etime = epoch;
	struct tm* tmp = gmtime(&etime);
		
	struct rtc_clockreg_value time;
	time.day    = tmp->tm_mday;
	time.month  = tmp->tm_mon + 1;
	time.year   = tmp->tm_year + 1900;		// Years since 2000
	time.hour   = tmp->tm_hour;
	time.minute = tmp->tm_min;
	time.second = tmp->tm_sec;
		
	rtc_set_time(&time);
}


/*
 * Private Utility Functions
 */

uint32_t RTCZero::rtc_get_time()
{
	while (is_rtc_syncbusy())
	{
		// Wait for synchronization
	}
	 
	// Read clock register
	uint32_t register_value = RTC->MODE2.CLOCK.reg;
	return register_value;
}


void RTCZero::rtc_set_time(rtc_clockreg_value *time)
{
	uint32_t register_value = rtc_time_to_register_value(time);

	while (is_rtc_syncbusy())
	{
		// Wait for synchronization
	}
	
	RTCdisable();

	// Write value to clock register
	RTC->MODE2.CLOCK.reg = register_value;

	while (is_rtc_syncbusy())
	{
		// Wait for synchronization
	}
	
	RTC->MODE2.CTRLA.reg |= RTC_MODE2_SYNCBUSY_CLOCKSYNC;
	while (is_rtc_syncbusy())
	{
		// Wait for synchronization
	}
	
	RTCenable();
}


uint32_t RTCZero::rtc_time_to_register_value(rtc_clockreg_value *time)
{
	uint32_t clock_reg_value;

	// Set year value into register_value
	clock_reg_value = ((time->year - 2000) << RTC_MODE2_CLOCK_YEAR_Pos);
	// Set month value into register_value
	clock_reg_value |= (time->month << RTC_MODE2_CLOCK_MONTH_Pos);
	// Set day value into register_value
	clock_reg_value |= (time->day << RTC_MODE2_CLOCK_DAY_Pos);

	// Set 24 hour value into register_value
	clock_reg_value |= (time->hour << RTC_MODE2_CLOCK_HOUR_Pos);
	// Set minute value into register_value
	clock_reg_value |= (time->minute << RTC_MODE2_CLOCK_MINUTE_Pos);
	// Set second value into register_value
	clock_reg_value |= (time->second << RTC_MODE2_CLOCK_SECOND_Pos);

	return clock_reg_value;
}


void RTCZero::rtc_register_value_to_time(uint32_t clock_reg_value, rtc_clockreg_value *time)
{
	// Set year. Add initial year offset.
	time->year = ((clock_reg_value & RTC_MODE2_CLOCK_YEAR_Msk) >> RTC_MODE2_CLOCK_YEAR_Pos) + 2000;
	// Set month
	time->month = ((clock_reg_value & RTC_MODE2_CLOCK_MONTH_Msk) >> RTC_MODE2_CLOCK_MONTH_Pos);
	// Set day
	time->day = ((clock_reg_value & RTC_MODE2_CLOCK_DAY_Msk) >> RTC_MODE2_CLOCK_DAY_Pos);
	
	// Set hour
	time->hour = ((clock_reg_value & RTC_MODE2_CLOCK_HOUR_Msk) >> RTC_MODE2_CLOCK_HOUR_Pos);
	// Set minute
	time->minute = ((clock_reg_value & RTC_MODE2_CLOCK_MINUTE_Msk) >> RTC_MODE2_CLOCK_MINUTE_Pos);
	// Set second
	time->second = ((clock_reg_value & RTC_MODE2_CLOCK_SECOND_Msk) >> RTC_MODE2_CLOCK_SECOND_Pos);
}


static inline bool is_rtc_syncbusy()
{
	if (RTC->MODE2.SYNCBUSY.reg)
	{
		return true;
	}
	
	return false;
}


void RTCZero::RTCdisable()
{
	while (is_rtc_syncbusy())
	{
		// Wait for synchronization
	}
	
	RTC->MODE2.INTENCLR.reg = RTC_MODE2_INTENCLR_MASK;		// disable interrupt
	RTC->MODE2.INTFLAG.reg = RTC_MODE2_INTFLAG_MASK;			// clear interrupt flag
	RTC->MODE2.CTRLA.reg &= ~RTC_MODE2_CTRLA_ENABLE;			// disable RTC
	
	while (is_rtc_syncbusy())
	{
		// Wait for synchronization
	}
}

void RTCZero::RTCenable()
{
	while (is_rtc_syncbusy())
	{
		// Wait for synchronization
	}
	
	RTC->MODE2.CTRLA.reg |= RTC_MODE2_CTRLA_ENABLE;			// enable RTC
	
	while (is_rtc_syncbusy())
	{
		// Wait for synchronization
	}
}

void RTCZero::RTCreset()
{
	RTCdisable();
	
	while (is_rtc_syncbusy())
	{
		// Wait for synchronization
	}
	
	RTC->MODE2.CTRLA.reg |= RTC_MODE2_CTRLA_SWRST;			// software reset
	
	while (is_rtc_syncbusy())
	{
		// Wait for synchronization
	}
}
