/*
 * main_header.h
 *
 *  Created on: Feb 12, 2017
 *      Author: c_ker
 */

#ifndef PROJECT_TYPES_H_
#define PROJECT_TYPES_H_


#include "balance_control.h"
#include "button_control.h"
#include "lcd_gui.h"
#include "Board.h"
#include "bno055.h"
#include "camera_control.h"
#include "motion_control.h"
#include "position_data.h"
#include "sensorhub_com.h"

/* XDCtools Header files */
#include <xdc/std.h>
#include <xdc/runtime/System.h>
#include <xdc/runtime/Log.h>
/* BIOS Header files */
#include <ti/sysbios/BIOS.h>
#include <ti/sysbios/knl/Task.h>
#include <ti/sysbios/knl/Semaphore.h>
#include <xdc/cfg/global.h> 				//header file for statically defined objects/handles
#include <ti/sysbios/knl/Clock.h>
/* TI-RTOS Header files */
#include <ti/drivers/GPIO.h>
#include <ti/drivers/I2C.h>
#include <ti/drivers/PWM.h>
#include <ti/drivers/UART.h>
/* Example/Board Header files */
#include "Board.h"
#include "bno055.h"

/*Hardware Drivers*/
#include "inc/hw_gpio.h"
#include "inc/hw_types.h"
#include "inc/hw_ints.h"
#include "inc/hw_memmap.h"

/* Driver Libs */
#include "driverlib/sysctl.h"
#include "driverlib/pin_map.h"
#include "driverlib/gpio.h"
#include "driverlib/qei.h"
#include "driverlib/timer.h"


#include <stdint.h> // Variable definitions for the C99 standard.
#include <stdio.h> // Input and output facilities for the C99 standard.
#include <stdbool.h> // Boolean definitions for the C99 standard.
#include <math.h>


#include "inc/hw_ints.h"
#include "inc/hw_types.h"
#include "driverlib/debug.h"
#include "driverlib/rom.h"
#include "driverlib/sysctl.h"
#include "driverlib/pwm.h"
#include "driverlib/sysctl.h" // Definitions and macros for System Control API of DriverLib.
#include "driverlib/interrupt.h" // Defines and macros for NVIC Controller API of DriverLib.
#include "driverlib/gpio.h" // Definitions and macros for GPIO API of DriverLib.
#include "driverlib/timer.h" // Defines and macros for Timer API of DriverLib.
#include "driverlib/pin_map.h" //Mapping of peripherals to pins for all parts.
#include "driverlib/adc.h" // Definitions for ADC API of DriverLib.
#include "driverlib/fpu.h" // Prototypes for the FPU manipulation routines.
#include "inc/hw_memmap.h" // Memory map definitions of the Tiva C Series device.
#include "inc/hw_types.h" // Definitions of common types and macros.



#define WHEEL_CIRC .946
#define WHEEL_WIDTH .5474
#define WHEEL_CENTER .2737
#define BOT_WEIGTH 15
#define HEIGTH 0.68
#define PI 3.14159265
#define PI_2 6.2831853
#define MAX_ENCODER 39400


#define _PIDA PID_CONTROL_DATA._pid_angle			//MACRO for shorter typedef call
#define _PIDA_P PID_CONTROL_DATA->_pid_angle		//MACRO for shorter typedef call
#define _PIDS PID_CONTROL_DATA._pid_speed			// speed macro
#define _PIDS_P PID_CONTROL_DATA->_pid_speed		// speed macro


//IMU DATA data
typedef struct IMU_DATA
{
struct bno055_euler_float_t  euler_values;	//euler angles struct read from IMU bno055
float x_gyro;								//Gyro values
// Add in gravity and quaternion and accel for LED
}IMU_DATA;
IMU_DATA imu_data[20];


typedef struct pos_auton
{
	float straight_setpoint;
	float X_square;
	float Y_square;
}pos_auton;

pos_auton auto_setpoints;

typedef struct sensor_hub_data
{
char frontR_dist;
char frontL_dist;
char backR_dist;
char backL_dist;
signed char turn;
signed char angle;
char flags;
char cRemote_state;
char fail_safe; //0x01;
char manual_mode; //0x03;
char autonomous_mode; //0x05;

}sensor_hub_data;

sensor_hub_data sensor_hub;

typedef struct wheel_Data
{
	uint16_t encoder_cnt[20];	//encoder count -- 20 array to read previous values debugging
	float vel[20];				//velocity -- 20 array for averaging debugging
	float accel[20];			//accel -- same thing
	int8_t  wheel_dir;			// current wheel direction used for ABS and direction of ESC
	int32_t var_pos[20];		// variable position will be used for short turns
	int32_t set_point;			// setPoint of the position - used for pivot turn and where we want to go
	int32_t abs_pos;			// keeps track of the absolute position of the robot from starting reference
	int32_t dynamic_pos;		// dynamic position that gets changed after each turn so robot can maintain orientation
	int32_t pwm_out;			// pwm output of the wheels to the motors
}wheel_Data;
//Quadrature encoder struct data to be used to determine current positions of the robot
typedef struct QEI_DATA
{
wheel_Data left_wheel;	//left wheel data QEI
wheel_Data right_wheel;	//right wheel data QEI
}QEI_DATA;
QEI_DATA qei_data;

//IMU data for the current position as well as the velocity and pointer to last position
typedef struct IMU_POSITION
{
	volatile float output;			//current angle
	volatile float current_pos;		//current angle position
	volatile float current_vel;		//current velocity - gyro
	volatile float *last_pos;		//pointer to last_pos -- refactor this out
	volatile unsigned char accel_cal;
	volatile unsigned char gyro_cal;
	volatile unsigned char mag_cal;
}IMU_POSITION;
IMU_POSITION pid_imu_position[20];

//cycle data to be able to go back 10 values in the array;
typedef struct CYCLE_DATA{		//struct to hold loop cycle data;
	int now;
	int back10;
	int back9;
	int back8;
	int back7;
	int back6;
	int back5;
	int back4;
	int back3;
	int back2;
	int back1;
}CYCLE_DATA;
CYCLE_DATA cycle_data[20];


//PID CONTROL DATA FOR SPEED
typedef struct SPEED_PID
{
	float Pgain;			//proportional gain
	float Igain;			// Integral gain
	float Dgain; 			// Derivative gain
	float Ierror;			// Speed Ierror Value - Integral
	float setPoint;			// the set-point calculated use 20 because we average
}SPEED_PID;

//PID CONTROL DATA FOR ANGLE
typedef struct ANGLE_PID
{
	float Pgain;
	float Igain;
	float Dgain;
	float Ierror;
	float setPoint;
}ANGLE_PID;

typedef struct POSITION_PID
{
	float Pgain;			//proportional gain
	float Igain;			//Integral gain
	float Dgain;			//Derivative gain
	float Ierror;			// Ierror value  -- += error value every calc
	float setPoint;
}POSITION_PID;

//structure used to keep track of all the PIDS
//Currently we used 3 PIDS 2 at a time dependant on state and what we want
typedef struct PID_CONTROLS
{
ANGLE_PID _pid_angle;			//ANGLE PID
SPEED_PID _pid_speed;			//SPEED PID
POSITION_PID _pid_position;		//POSITION PID
}PID_CONTROLS;


PID_CONTROLS PID_CONTROL_DATA;

//enum for the states for the control algorithms
enum control_state
{
	straight_line,
	s_turn,
	p_turn_right,
	p_turn_left,
	c_turn_right,
	c_turn_left
};


//Control data to help determine the correct motion planning of the robot
typedef struct CONTROLS_INFORMATION
{
float c_X_pos[20];					//distance in meters X-Y
float c_Y_Pos[20];					//absololute position in meters X-Y
float x_abs_pos;					//x_abs_pos
float y_abs_pos;					//y_abs_pos
float x_set_point;
float y_set_point;
long wheel_L_cpos[20];				//current position of left wheel
long wheel_R_cpos[20];				//current position of right wheel
long wheel_L_fpos;					//final position of left wheel
long wheel_R_fpos;					//final position right wheel
long wheel_L_spos; 					//starting postion before move
long wheel_R_spos; 					//starting position before long move
volatile float current_orientation;	//value is in RADIANS -- CONVERT TO ANGLE
float final_orientation;			//direction of robot orientation of robot and in RADIANS
float IMU_orientation;
float current_speed;				//speed requested of the robot
char turn_state;					//Control state of robot turning - forward
char control_state;					//major state IE remote control - automated - search
int timer_tick;						//use to control time transition
int direction_flag;
int pause_tick;
int revolutions;
char autonomous_mode;				//sets the modes either SQUARE CIRCLE or STRAIGHT FOORWARD
char autonomous_state; 				//0 for starting move 1 for finished turn to orientation 2 for moving forward 3 for final orientation
char turn_flag[2];
float turn_percent;  				//use values from -100% to 100% left to right
int flag_square;
int counter;

}CONTROL_INFORMATION;

CONTROL_INFORMATION ctrl_robot;

//MENU STRUCT TO DETERMINING THE CURRENT MENU STATE
typedef struct menu_state
{
	int main_state;				//MAIN state of menu - top level
	int middle_state;			//middle state of menu - inner tree
	int line_state;				//current line value
	int change_state;			//bool for changing the value of something
	int menu_state_change;
}MENU_STATE;

MENU_STATE menu_state;


//int debug_swap_left[100];
//int debug_swap_right[100];
//int debug_pwm_left[100];
//int debug_pwm_right[100];
//int debug_count;

volatile float setAngle[20];		//REFACTOR THIS INTO A STRUCTURE FOR REMOTE CONTROL
volatile float turnAngle[20];		//REFACTOR THIS INTO REMOTE CONTROL STRUCTURE

int speed_loop;						//REFACTOR INTO STRUCTURE USED FOR LOOP REFERENCE
int angle_loop;						//Used to reference the LOOP array data for position

#endif /* PROJECT_TYPES_H_ */
