/*
 * EchoSensor.cpp
 *
 * Created: 6/27/2019 10:57:31 AM
 *  Author: bob
 */ 

#include "EchoSensor.h"

static char		 echostring[32];
static uint32_t	 echocount;


sapi_error_t echo_read_sensor(char *payload, uint8_t *len)
{
	char	count[16];
	// Assemble the Payload
	
	strcpy(payload, echostring);
	sprintf(count, " %d", echocount++);
	strcat(payload, count);
	*len = strlen(payload);
	
	dlog(LOG_DEBUG, "Echo Payload: %s", payload);
    return SAPI_ERR_OK;
}


sapi_error_t echo_write_cfg(char *payload, uint8_t *len)
{
	if (strncmp(payload, "cfg=",4) == 0)
	{
		strcpy(echostring, &payload[4]);
		echocount = 1;
		
		return SAPI_ERR_OK;
	}
	else
	{
		return SAPI_ERR_NOT_IMPLEMENTED;
	}
}


sapi_error_t echo_init_sensor()
{
	strcpy(echostring, "Echo Echo Echo");
	
	echocount = 1;
	
	dlog(LOG_DEBUG, "Initialized Echo Sensor");
	return SAPI_ERR_OK;
}