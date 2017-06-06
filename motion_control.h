/*
 * motion_control.h
 *
 *  Created on: Mar 21, 2017
 *      Author: c_ker
 */

#ifndef MOTION_CONTROL_H_
#define MOTION_CONTROL_H_


#include "project_types.h"

void init_balance_control();
void balance_control_algo();

void calc_wheel_movement_turn(float theta);
//CALCULATES THETA DISPLACEMENT TO THE DESIRED ANGLE
float calc_theta_displacement(float current,float final);
void calculate_turn_state(float theta);


void autonomous_movement_control(void);

//THIS FUNCTION IS FOR YOUR TURNS FROM REMOTE CONTROLLER
//USE YOUR GLOBAL VARIABLES YOU MADE - MAKE A STRUCT IN PROJECT TYPE
//FOR REMOTE CONTROL DATA OR THE SPI DATA


int forward_straight_movement(float distance);
int backward_straight_movement(float distance);

void calc_forward_wheel_movement(float distance);
float angular_to_path(float X, float Y);

//AUTONOMOUS FUNCTIONS
void turn_to_straight_transition(float theta_displacement);
void move_to_XY(float displacement);
void move_to_THETA(float displacement_theta);
void center_to_theta(float displacement_theta);

int turn_right_movement(float distance);
int turn_left_movement(float distance);

void next_position_call(int position_call);
void forward_and_back();

void figure_8_movement_wheels();

float angle_SCA(signed char angle);


#endif /* MOTION_CONTROL_H_ */
