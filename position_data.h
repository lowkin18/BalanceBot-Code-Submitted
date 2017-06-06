/*
 * position_data.h
 *
 *  Created on: Mar 21, 2017
 *      Author: c_ker
 */

#ifndef POSITION_DATA_H_
#define POSITION_DATA_H_


#include "project_types.h"


void init_QEI();
void getWheel_Data();



//POSITION X-Y calcs
float position_XY_calc(float arc1,float arc2,float *x_movement,float *y_movement);
void rotation_matrix(float orientation,float *x_val,float *y_val);
float center_axis_rotation(int encoder_count,float * x_val, float*y_val);

void position_calculations();

void calculate_wheel_f_pos(float x, float y, float orientation);


#endif /* POSITION_DATA_H_ */
