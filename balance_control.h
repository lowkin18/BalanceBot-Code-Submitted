/*
 * balance_control.h
 *
 *  Created on: Feb 12, 2017
 *      Author: c_ker
 */

#include "project_types.h"
#include <math.h>

#ifndef BALANCE_CONTROL_H_
#define BALANCE_CONTROL_H_

//init the timecycle data for looping data and previous values
void INIT_TIMECYCLE();


//Given an input set-Point angle it will perform this action
void PID_angle(float pos);
void PID_speed(float pos);
float PID_pos();

//INITIALIZE ALL THE STARTING VALUES OF ROBOT to 0 for initial position

void init_pid_values();
char init_main_loop();

//TURNING ALGORITHMS
void center_turn_left();
void center_turn_right();
void forward_wheel_control();
void soft_turn_control();
void wheel_control();
void right_wheel_pivot();
void left_wheel_pivot();


int i2c_check_data();


#endif /* BALANCE_CONTROL_H_ */
