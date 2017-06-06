/*
 * sensorhub_com.h
 *
 *  Created on: Apr 28, 2017
 *      Author: Kip
 */

#ifndef SENSORHUB_COM_H_
#define SENSORHUB_COM_H_

#include "project_types.h"

void init_sensor_com(void);
void sensor_hub_read(void);
void sensor_callback_read();

char sensor_hub_bytes[7];   //SENSR READ DATA


#endif /* SENSORHUB_COM_H_ */
