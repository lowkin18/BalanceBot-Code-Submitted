/*
 * camera_control.h
 *
 *  Created on: Mar 16, 2017
 *      Author: c_ker
 */

#ifndef CAMERA_CONTROL_H_
#define CAMERA_CONTROL_H_

#include "project_types.h"


#define LED0_ON 0x06
#define LED0_OFF 0x08

#define PWM_FREQ 0x19	// freq = 25mhz/(4096*freq_wanted) = 0x19h or 25dec

#define DISTANCE_LEVEL_MAX_BACK 1185
#define DISTANCE_LEVEL_MAX_FORWARD 1815
#define CAMERA_TILT_MAX_BACK 575
#define CAMERA_TILT_MAX_FORWARD 2050
#define CAMERA_TURN_MAX_RIGHT 575
#define CAMERA_TURN_MAX_LEFT 2275

#define CAMERA_TILT_HOME 1350
#define CAMERA_TURN_HOME 1400
#define LEVEL_HOME	1510


#define TILT_SCA 9.5588
#define TILT_CAM_SCA 8.9394
#define TURN_CAM_SCA 9.4444

I2C_Handle i2c_com;
I2C_Transaction i2cTransaction_cam;

typedef struct camera_string
{
	char txBuffer[10];
	char * ptr_txBuffer;
}camera_string;


typedef struct servo_data
{
	float camera_tilt_offset;

	float camera_tilt_p;
	float camera_turn_p;
	float level_tilt_p;

	short camera_tilt_duty;
	short camera_turn_duty;
	short level_tilt_duty;

}servo_data;

servo_data servo_d;
camera_string cam_string;

void level_mounts(float angle,float turn_angle);
short camera_tilt_calc(float angle);
short camera_turn_calc(float angle);
short level_tilt_calc(float angle);
char i2c_write_pwm(char *values,char length);


void PWM_Servo_write(char led_num, short PWM_Val);
void init_i2c_camera(I2C_Handle i2c);



#endif /* CAMERA_CONTROL_H_ */
