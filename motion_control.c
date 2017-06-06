/*
 * motion_control.c
 *
 *  Created on: Mar 21, 2017
 *      Author: c_ker
 *
 *
 *      THIS FILE IS MEANT FOR THE MOTION CONTROL ALGORITHMS OF THE ROBOT
 *      IT WILL CONTRAIN ALL THE FUNCTIONS PERTAINING TO THE MOTION CONTROL
 *      ALGORITHMS THE BOT WILL USE.
 *
 *      BASICALLY PATH PLANNING AND STATE DECISIONS THAT WILL BE MADE
 */

#include "motion_control.h"




/***************************************************************************************
 * init_balance_control()
 *
 * This function initializes all the starting values for the balance_control
 * Sets the initial position as the reference and all X - Y values are from starting orientation
 * of the robot Y Axis is forward and X axis is parallel to wheels
 *
 *
 **************************************************************************************/

void init_balance_control() {
	ctrl_robot.current_orientation = 0;
	ctrl_robot.x_abs_pos = 0.000;				//x_abs_pos
	ctrl_robot.y_abs_pos = 0.000;				//y_abs_pos
	ctrl_robot.wheel_L_fpos = 0;	 			//final position of left wheel
	ctrl_robot.wheel_R_fpos = 0; 				//final position right wheel
	ctrl_robot.current_orientation = 0;	//value is in RADIANS -- CONVERT TO ANGLE
	ctrl_robot.final_orientation = 0;//direction of robot orientation of robot and in RADIANS
	ctrl_robot.IMU_orientation = 0;
	ctrl_robot.current_speed = 0;				//speed requested of the robot
	ctrl_robot.turn_state = 0;
	ctrl_robot.turn_percent = 0;  //use values from -100% to 100% left to right
	ctrl_robot.control_state = 0;
	ctrl_robot.timer_tick = 0;
	qei_data.left_wheel.set_point = 0;
	qei_data.right_wheel.set_point = 0;
	ctrl_robot.pause_tick = 0;
	ctrl_robot.revolutions =0;
	ctrl_robot.flag_square=0;
	ctrl_robot.counter =0;

	int counter = 0;
	for (counter = 0; counter < 20; counter++) {
		ctrl_robot.c_X_pos[counter] = 0;		//distance in meters X-Y
		ctrl_robot.c_Y_Pos[counter] = 0;	//absololute position in meters X-Y
		ctrl_robot.wheel_L_cpos[counter] = 0;	//current position of left wheel
		ctrl_robot.wheel_R_cpos[counter] = 0;//current position of right wheel
	}
}


void forward_and_back()
{
	int flag = 0;
	float distance = ctrl_robot.x_set_point - ctrl_robot.x_abs_pos;
	float movement_val = 0;
	float wheel_distance = ctrl_robot.wheel_L_fpos-ctrl_robot.wheel_L_cpos[cycle_data[speed_loop].now];
	float wheel_movement = wheel_distance/39400 *WHEEL_CIRC;
	float set_point_left = distance - wheel_movement;

	flag = (distance > 0) ? 1:0;
	if(flag == 1)
	{
	if(flag == 1 && set_point_left > 0.1)
		{
		movement_val = 0.1;
		}
	forward_straight_movement(movement_val);
	}
	else
	{
	if(flag == 0 && set_point_left < -0.1)
		{
			movement_val = 0.1;
		}
	backward_straight_movement(movement_val);

	}
}

/*************************************************************************************
 * balance_control_algo()
 *
 * No Params yet
 *
 * this function will be in charge of determining the correct motion plan of the robot
 * and what state it needs to be in to achieve this motion
 *
 * the states will be forward motion - which is the same as reverse motion
 * soft turning which will be forward motion with a slight turn
 * pivot turning left/right which will lock one wheel in place and turn around it
 * center turning left/right which will let the robot turn in place moving both wheels around
 * its center of axis.
 *
 *
 *
 *
 ******************************************************************************************/
void balance_control_algo()
{

	getWheel_Data();
	position_calculations();
	switch (ctrl_robot.control_state) {
	case 0:
		PID_speed(PID_pos()); //HOLD POSITION STATE
		break;
	case 1:		// AUTONOMOUS MOTION STATE

		PID_speed(PID_pos());
		break;
	case 2:
		break;
	case 3:
		ctrl_robot.turn_state = 1;
		PID_angle(angle_SCA(sensor_hub.angle));//PUT YOUR REMOTE CONTROL SPEED VALUE INTO THE SPEED PID
		break;
	case 4:
		break;
	default:
		PID_speed(PID_pos()); //HOLD POSITION STATE
		break;
	}
}

float angle_SCA(signed char angle)
{
	float angle_calc;
	angle_calc = (float)angle/5;
	if(angle_calc > 20)
	{
		angle_calc = 20;
	}
	if(angle_calc < -20)
	{
		angle_calc = -20;
	}
	if(angle_calc > 0.5 || angle_calc < -0.5)
	{
	return angle_calc;
	}
	else{
	return 0;}
}

/*************************************************************************************
 * float calc_theta_displacement(float current,float final)
 *
 * Params Current Theta and Final THeta
 *
 *returns theta displacement angle
 *
 *This function will take two angles and then return the total angular displacement
 *It will determine the shortest angular displacment as well.
 *
 ******************************************************************************************/
float calc_theta_displacement(float current, float final) {
	float theta_displacement = final - current;
	if (theta_displacement >= PI) {
		theta_displacement -= PI_2;
	} else if (theta_displacement <= -PI) {
		theta_displacement += PI_2;
	}

	return theta_displacement;

}

/*************************************************************************************
* void autonomous_movement_control(void)
 *
 * nothing
 *
 *CALCULATES THE BEST WAY TO GET FROM THE CURRENT POSITION TO THE SETPOINT POSITION
 *
 *
 ******************************************************************************************/
void autonomous_movement_control(void)
{
	//NO ORIENTATION CHANGE
	float displacement,displacement_x,displacement_y, theta_displacement,theta_movement;
	displacement_x = ctrl_robot.x_set_point - ctrl_robot.x_abs_pos;
	displacement_y = ctrl_robot.y_set_point - ctrl_robot.y_abs_pos;
	theta_displacement = calc_theta_displacement(ctrl_robot.current_orientation,ctrl_robot.final_orientation);
	theta_movement = calc_theta_displacement(ctrl_robot.current_orientation,atan2(displacement_y,displacement_x));
	displacement = sqrt(displacement_x * displacement_x + displacement_y*displacement_y);



	if(ctrl_robot.timer_tick == 0)
	{
		if(displacement > 0)
		{
			ctrl_robot.direction_flag = 1;
		}
		else
		{
			ctrl_robot.direction_flag = 0;
		}
	}
	if(theta_movement > 0.05 && ctrl_robot.autonomous_state <2 && (displacement > .5 || displacement <-.5))
	{
	ctrl_robot.autonomous_state = 1;
	ctrl_robot.turn_state = 3;//left wheel pivot;
	move_to_THETA(theta_movement);
	}
	else if(theta_movement <-0.05 && ctrl_robot.autonomous_state <2 &&(displacement > .5 || displacement <-.5))
	{
	ctrl_robot.autonomous_state = 1;
	ctrl_robot.turn_state = 2;	//right wheel pivot;
	move_to_THETA(theta_movement);
	}
	else if((displacement>0.25 || displacement <-0.25) && ctrl_robot.autonomous_state <3)
	{
		ctrl_robot.autonomous_state = 2;
		move_to_XY(displacement);
	}
	else if(theta_displacement>0.1)
	{
		ctrl_robot.autonomous_state = 3;
		move_to_THETA(theta_displacement);
	}
	else if(theta_displacement<-0.1)
	{
		ctrl_robot.autonomous_state = 3;
		move_to_THETA(theta_displacement);
	}
	else
	{
		if(ctrl_robot.turn_state== 2 || ctrl_robot.turn_state == 3)
		{
			//turn_to_straight_transition(theta_displacement);
			ctrl_robot.turn_state = 0;
		}
		ctrl_robot.autonomous_state = 4;
		ctrl_robot.turn_state = 0;
	}
}

void move_to_XY(float displacement)
{
	ctrl_robot.turn_state = 0;
	float movement_val = 0;
	float wheel_distance = ctrl_robot.wheel_L_fpos-ctrl_robot.wheel_L_cpos[cycle_data[speed_loop].now];
	float wheel_movement = wheel_distance/39400 *WHEEL_CIRC;
	float set_point_left = displacement - wheel_movement;

	if(ctrl_robot.direction_flag == 1)
	{
		if(set_point_left > 0.01)
		{
			if(ctrl_robot.timer_tick < 25)
			{
			movement_val = 0.1;
			}
			else
			{
			movement_val = set_point_left*.05;
			}
		}
		forward_straight_movement(movement_val);
	}
	else
	{
		if(set_point_left <-0.01)
		{
			if(ctrl_robot.timer_tick < 25)
			{
			movement_val = 0.1;
			}
			else
			{
			movement_val = set_point_left*.05;
			}
		}
		backward_straight_movement(movement_val);
	}
}

void turn_to_straight_transition(float theta_displacement)
{
	int wheel_offset = ((theta_displacement*WHEEL_CENTER)/WHEEL_CIRC)*39400;

	if(wheel_offset > 0)
	{
		qei_data.right_wheel.dynamic_pos += wheel_offset;
	}
	else
	{
		qei_data.left_wheel.dynamic_pos += wheel_offset;
	}
}

void move_to_THETA(float theta_displacement)
{
	float movement_val;
	float displacement_wheel;
	float wheel_distance;
	float wheel_movement;
	float set_point_left;

	if(theta_displacement > 0)
	{
		movement_val = 0;
		displacement_wheel = (theta_displacement*WHEEL_CENTER);
		wheel_distance = ctrl_robot.wheel_R_fpos-ctrl_robot.wheel_R_cpos[cycle_data[speed_loop].now];
		wheel_movement = wheel_distance/39400 *WHEEL_CIRC;
		set_point_left = displacement_wheel - wheel_movement;

		if(set_point_left > 0.01)
		{
			if(ctrl_robot.timer_tick < 10)
			{
				if(ctrl_robot.timer_tick == 0)
				{
					qei_data.left_wheel.set_point = qei_data.left_wheel.abs_pos;
					ctrl_robot.turn_state = 3;
				}
				movement_val = 0.05;
			}
			else
			{
				if(set_point_left < 0.05)
				{
				movement_val = set_point_left;
				}
				else
				{
				movement_val = 0.05;
				}
			}
			turn_left_movement(movement_val);
		}
		else
		{
			if(wheel_distance < 1000)
			{
			ctrl_robot.turn_state = 0;
			qei_data.left_wheel.dynamic_pos = 0;
			qei_data.right_wheel.dynamic_pos = wheel_distance;
			}
			ctrl_robot.timer_tick = 0;
		}
	}
	else
	{
		movement_val = 0;
		displacement_wheel = (-1*theta_displacement*WHEEL_CENTER);
		wheel_distance = ctrl_robot.wheel_L_fpos-ctrl_robot.wheel_L_cpos[cycle_data[speed_loop].now];
		wheel_movement = wheel_distance/39400 *WHEEL_CIRC;
		set_point_left = displacement_wheel - wheel_movement;
		if(set_point_left > 0.01)
		{
			if(ctrl_robot.timer_tick < 10)
			{
				if(ctrl_robot.timer_tick == 0)
				{
				qei_data.right_wheel.set_point = qei_data.right_wheel.abs_pos;
				ctrl_robot.turn_state = 2;
				}
				movement_val = 0.05;
			}
			else
			{
				if(set_point_left < 0.05)
							{
							movement_val = set_point_left;
							}
							else
							{
								movement_val = 0.05;
							}
			}
			turn_right_movement(movement_val);
		}
		else
		{
			if(wheel_distance < 1000)
			{
			ctrl_robot.turn_state = 0;
			qei_data.right_wheel.dynamic_pos =0;
			qei_data.left_wheel.dynamic_pos = wheel_distance;
			}
			ctrl_robot.timer_tick = 0;
		}
	}
}

/*************************************************************************************
 * figure_8_movement_wheels()
 *
 * Params Nothings
 *
 * Performs a continous Figure 8 movement
 *
 *
 ******************************************************************************************/
void figure_8_movement_wheels()
{
	static int flag_first_time;
	if(flag_first_time == 0)
	{
	flag_first_time = 1;
	ctrl_robot.final_orientation = 6.25;
	}
	float accepted_difference = 0.01;
	if(ctrl_robot.revolutions == 0 && fabs(ctrl_robot.final_orientation+6.25) < accepted_difference)
	{
		ctrl_robot.pause_tick++;
		if(ctrl_robot.pause_tick == 20)
		{
		ctrl_robot.final_orientation = 6.25;
		ctrl_robot.pause_tick = 0;
		ctrl_robot.timer_tick = 0;
		}
	}
	else if(ctrl_robot.revolutions == 1 && fabs(ctrl_robot.final_orientation-6.25)<accepted_difference)
	{
		ctrl_robot.pause_tick++;
		if(ctrl_robot.pause_tick == 20)
		{
		ctrl_robot.final_orientation = -6.25;
		ctrl_robot.pause_tick = 0;
		ctrl_robot.timer_tick = 0;
	    }
	}
	else
	{
	float theta_displacement = ctrl_robot.final_orientation - ctrl_robot.current_orientation;
	move_to_THETA(theta_displacement);
	ctrl_robot.timer_tick++;
	}
}

/*************************************************************************************
 *int forward_movement(float distance)
 *
 * Params Distance
 *
 * Calculates the wheel final positions based on the distance needed to travel
 *
 *
 ******************************************************************************************/
int forward_straight_movement(float distance)
{
	int wheel_pos_final = (int) ((distance/WHEEL_CIRC)*39400);
	ctrl_robot.wheel_L_fpos = ctrl_robot.wheel_L_fpos  + wheel_pos_final;
	ctrl_robot.wheel_R_fpos = ctrl_robot.wheel_R_fpos  + wheel_pos_final;
	return 0;
}

/*************************************************************************************
 *int forward_movement(float distance)
 *
 * Params Distance
 *
 * Calculates the wheel final positions based on the distance needed to travel
 *
 *
 ******************************************************************************************/
int backward_straight_movement(float distance)
{
	int wheel_pos_final = (int) ((distance/WHEEL_CIRC)*39400);
	ctrl_robot.wheel_L_fpos = ctrl_robot.wheel_L_fpos  - wheel_pos_final;
	ctrl_robot.wheel_R_fpos = ctrl_robot.wheel_R_fpos  - wheel_pos_final;
	return 0;
}

int turn_left_movement(float distance)
{
	int wheel_pos_final = (int) ((distance/WHEEL_CIRC)*39400);
	ctrl_robot.wheel_R_fpos = ctrl_robot.wheel_R_fpos + wheel_pos_final;
	return 0;
}

int turn_right_movement(float distance)
{
	int wheel_pos_final = (int) ((distance/WHEEL_CIRC)*39400);
	ctrl_robot.wheel_L_fpos = ctrl_robot.wheel_L_fpos  + wheel_pos_final;
	return 0;
}

/*************************************************************************************
 *void calculate_turn_state(float theta)
 *
 * Params THeta
 *
 * Calculates teh state needed to perform the Turn
 *
 * Need to add in Pivot turn control algorithm to decide when it's best to pivot turn
 *
 ******************************************************************************************/
void calculate_turn_state(float theta) {
	if (theta > 0) {
		ctrl_robot.turn_state = 5;
	}
	if (theta < 0) {
		ctrl_robot.turn_state = 4;
	}
	calc_wheel_movement_turn(theta);
}

/*************************************************************************************
 * void calc_wheel_movement_turn(float theta);
 *
 * Params Theta
 *
 *This function will see what the theta value is and add it to the wheel_L_fpos value
 *
 ******************************************************************************************/
void calc_wheel_movement_turn(float theta)
{
	if(theta < 0)
	{
		ctrl_robot.wheel_L_fpos = (theta * WHEEL_CENTER) * (39400/WHEEL_CIRC) + ctrl_robot.wheel_L_cpos[cycle_data[speed_loop].now];//final position of left wheel
		ctrl_robot.wheel_R_fpos = (theta * WHEEL_CENTER) * (39400/WHEEL_CIRC) + ctrl_robot.wheel_R_cpos[cycle_data[speed_loop].now];//final position right wheel
	}
	else
	{
		ctrl_robot.wheel_L_fpos = (theta * WHEEL_CENTER) * (39400/WHEEL_CIRC) + ctrl_robot.wheel_L_cpos[cycle_data[speed_loop].now];//final position of left wheel
		ctrl_robot.wheel_R_fpos = (theta * WHEEL_CENTER) * (39400/WHEEL_CIRC) + ctrl_robot.wheel_R_cpos[cycle_data[speed_loop].now];//final position right wheel
	}
}


/*************************************************************************************
 * void angular_to_path()
 *
 * X and Y arguments
 *
 * this function will calculate the desired angle to get to the X and Y plane
 *
 ******************************************************************************************/
float angular_to_path(float X, float Y)
{
	if((X> 0.30 || X <-0.30)|| (Y > 0.30 || Y <-0.30) )
	{
	float theta;
	if(!(X > 0.30 || X <-0.30))
	{
		if(!(Y > 0.30 || Y <-0.30))
		{
		return 0.00;
		}
		else if(Y < -0.30)
		{
		return -PI/2;
		}
		else if(Y > 0.30);
		return PI/2;
	}
	else if(!(Y > 0.30 || Y <-0.30))
	{
		if(X > 0.30)
		{
			return 0;
		}
		else if(X < -0.30)
		{
			return PI;
		}
		else
		{
			return 0;
		}
	}
	else if(X != 0 && Y != 0)
	{
		theta = (float)atan2(Y,X);
		return theta;
	}
	}
	else
		{return 0;}
}



void calc_forward_wheel_movement(float distance)
{
	float encoder_pulses = (distance/WHEEL_CIRC) * 39400;
	ctrl_robot.wheel_L_fpos = encoder_pulses+ctrl_robot.wheel_L_cpos[cycle_data[speed_loop].now];
	ctrl_robot.wheel_R_fpos = encoder_pulses+ctrl_robot.wheel_R_cpos[cycle_data[speed_loop].now];
}


void next_position_call(int position_call)
{
	ctrl_robot.autonomous_state = 0;
	switch(position_call)
		{
			case 0:
				ctrl_robot.x_set_point = auto_setpoints.X_square;
				ctrl_robot.y_set_point = 0;
				ctrl_robot.final_orientation = 0;
			break;
			case 1:
				ctrl_robot.x_set_point = auto_setpoints.X_square;
				ctrl_robot.y_set_point = auto_setpoints.Y_square;
				ctrl_robot.final_orientation = PI/2;
			break;
			case 2:
				ctrl_robot.x_set_point = 0;
				ctrl_robot.y_set_point = auto_setpoints.Y_square;
				ctrl_robot.final_orientation = PI_2 - PI/2;
			break;
			case 3:
				ctrl_robot.x_set_point = 0;
				ctrl_robot.y_set_point = 0;
				ctrl_robot.final_orientation = 0;
			break;
		}
}
