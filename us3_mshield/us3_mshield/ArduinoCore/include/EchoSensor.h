/*
 * EchoSensor.h
 *
 * Created: 6/27/2019 10:49:58 AM
 *  Author: bob
 */ 


#ifndef ECHOSENSOR_H_
#define ECHOSENSOR_H_

#include <Arduino.h>
#include <DHT_U.h>
#include "sapi_error.h"
#include "arduino_time.h"
#include "log.h"


#define ECHO_SENSOR_TYPE		"echo"


sapi_error_t echo_init_sensor();

sapi_error_t echo_read_sensor(char *payload, uint8_t *len);

sapi_error_t echo_write_cfg(char *payload, uint8_t *len);



#endif /* ECHOSENSOR_H_ */