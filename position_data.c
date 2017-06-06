/*
 * position_data.c
 *
 *  Created on: Mar 21, 2017
 *      Author: c_ker
 *
 *
 *      THIS FILE CONTAINS ALL THE FUNCTIONS RELATED TO POSITION OF THE ROBOT
 *      THE POSITION IS CALCULATED BASED ON THE QUADRATURE ENCODER DATA
 *      WE THEN USE SOME POSITIONAL KINEMATICS AND ROTATION MATRICES TO DETERMINE
 *      ABSOLUTE POSITION AND ORIENTATION OF THE ROBOT. THIS DOES NOT ACCOUNT
 *      FOR SLIPPAGE OF THE ROBOT WHEELS.
 *
 */



#include "position_data.h"


/*****************************************************************************************
 * position_calculation()
 *
 * Given the wheel displacements read from the QEI this function will determine
 * the X and Y displacement of the robot from it's last position. All X -Y positions
 * are from the starting point of the robot
 *
 *
 ****************************************************************************************/
void position_calculations()
{

	float arcLengthL = qei_data.left_wheel.var_pos[cycle_data[speed_loop].now] - qei_data.left_wheel.var_pos[cycle_data[speed_loop].back1];
	float arcLengthR = qei_data.right_wheel.var_pos[cycle_data[speed_loop].now] - qei_data.right_wheel.var_pos[cycle_data[speed_loop].back1];
	float X_val = 0;
	float Y_val = 0;
	float previous_orientation = ctrl_robot.current_orientation;

			if (arcLengthL>0 && arcLengthR > 0 && arcLengthL != arcLengthR)
			{
				if (arcLengthL > arcLengthR)
				{
					//call function to convert arclengths to X and Y position relative current position
					ctrl_robot.current_orientation -= position_XY_calc(arcLengthR, arcLengthL, &X_val, &Y_val);
					//perform rotation matrix on X-Y to get into starting X - Y coordinate system
					rotation_matrix(previous_orientation, &X_val, &Y_val);
				}
				if (arcLengthL < arcLengthR)
				{
					ctrl_robot.current_orientation += position_XY_calc(arcLengthL, arcLengthR, &X_val, &Y_val);
					Y_val *= -1;
					rotation_matrix(previous_orientation, &X_val, &Y_val);
				}
			}

			else if (arcLengthL < 0 && arcLengthR < 0 && arcLengthL != arcLengthR)
			{
				if (arcLengthL > arcLengthR)
				{
					ctrl_robot.current_orientation += position_XY_calc(arcLengthL, arcLengthR, &X_val, &Y_val);
					Y_val *= -1;
					rotation_matrix(previous_orientation, &X_val, &Y_val);
				}
				if (arcLengthL < arcLengthR)
				{
					ctrl_robot.current_orientation -= position_XY_calc(arcLengthR, arcLengthL, &X_val, &Y_val);
					rotation_matrix(previous_orientation, &X_val, &Y_val);
				}

			}

			else if (arcLengthL == arcLengthR)	//if arclengths are equal robot just moved forward in Y direction
			{
				X_val = (arcLengthL / 39400)*WHEEL_CIRC; //find tangential displacement of the wheel
				Y_val = 0;
				rotation_matrix(previous_orientation, &X_val, &Y_val);	//convert movement starting X-Y coordinates

			}


			else if (arcLengthL == 0 || arcLengthR == 0)
			{
				if (arcLengthL == 0 && arcLengthR > 0)
				{
					ctrl_robot.current_orientation += center_axis_rotation(arcLengthR, &X_val, &Y_val);
					rotation_matrix(previous_orientation, &X_val, &Y_val);
				}
				else if (arcLengthL == 0 && arcLengthR < 0)
				{
					ctrl_robot.current_orientation += center_axis_rotation(arcLengthR, &X_val, &Y_val);
					rotation_matrix(previous_orientation, &X_val, &Y_val);
				}
				else if (arcLengthR == 0 && arcLengthL > 0)
				{
					ctrl_robot.current_orientation -= center_axis_rotation(arcLengthL, &X_val, &Y_val);
					Y_val *= -1;
					rotation_matrix(previous_orientation, &X_val, &Y_val);
				}
				else if (arcLengthR == 0 && arcLengthL < 0)
				{
					ctrl_robot.current_orientation -= center_axis_rotation(arcLengthL, &X_val, &Y_val);
					Y_val *= -1;
					rotation_matrix(previous_orientation, &X_val, &Y_val);

				}
			}


			else
			{
				float temp_X_val = 0;
				float temp_Y_val = 0;

				if (arcLengthL < 0 && arcLengthR > 0)		//THINK OF ALGORITHM FOR CENTER POINT TURNING THAT IS UNEQUAL
				{
					ctrl_robot.current_orientation -= center_axis_rotation(arcLengthL, &X_val, &Y_val);
					Y_val *= -1;
					rotation_matrix(previous_orientation, &X_val, &Y_val);


					ctrl_robot.current_orientation += center_axis_rotation(arcLengthR, &temp_X_val, &temp_Y_val);
					rotation_matrix(previous_orientation, &temp_X_val, &temp_Y_val);

					X_val += temp_X_val;
					Y_val += temp_Y_val;
				}
				else if (arcLengthL >0 && arcLengthR < 0)		//THINK OF ALGORITHM FOR CENTER POINT TURNING, just choose 1 as pivot?
				{
					ctrl_robot.current_orientation -= center_axis_rotation(arcLengthL, &X_val, &Y_val);
					Y_val *= -1;
					rotation_matrix(previous_orientation, &X_val, &Y_val);

					ctrl_robot.current_orientation += center_axis_rotation(arcLengthR, &temp_X_val, &temp_Y_val);
					rotation_matrix(previous_orientation, &temp_X_val, &temp_Y_val);

					X_val += temp_X_val;
					Y_val += temp_Y_val;
				}
			}
			/*
			System_printf("X = %f\t",ctrl_robot.x_abs_pos);
			System_printf("Y = %f\t",ctrl_robot.y_abs_pos);
			System_flush();
			*/
	if(ctrl_robot.current_orientation > PI_2)
	{
		ctrl_robot.current_orientation -= PI_2;
		ctrl_robot.revolutions++;
	}
	if(ctrl_robot.current_orientation < -PI_2)
	{
		ctrl_robot.current_orientation += PI_2;
		ctrl_robot.revolutions--;
	}

	ctrl_robot.x_abs_pos += X_val;
	ctrl_robot.y_abs_pos += Y_val;
}


/**********************************************************************
 * rotation_matrix(orientation , x_val, y_val
 *
 * function calculates the X and Y values in euclidean space back into
 * starting orientation of the robot to find absolute position with reference
 * to starting position of the robot.
 *
 *********************************************************************/
void rotation_matrix(float orientation,float *x_val,float *y_val)
{
	float x_temp = *x_val;
	*x_val = cos(orientation)**x_val-sin(orientation)**y_val;	//2by2 matrix for 2D rotation in X and Y space
	*y_val= sin(orientation)*x_temp+cos(orientation)**y_val;	//2by2 matrix for 2D rotation in X and Y space
}

/*************************************************************************************
 * float position_XY_calc(float arc1, float arc2,float * x,float * y)
 *
 * This function will find the X_position of of the robot
 * returns the theta value to find new orientation of the robot in degrees
 *
 *the X and Y positions are not relative to starting position - need to convert them
 *with a rotation matrix back into the starting orientation to get ABS  X and Y positions
 *with relative to starting location
 *
 ************************************************************************************/
float position_XY_calc(float arc1,float arc2,float *x_movement,float *y_movement)
{
	float theta;
	float ratio = arc1/arc2;
	float arc1_meters = (arc1/39400)*WHEEL_CIRC; 	//convert arc1 pulse to meters
	float radius = (ratio*WHEEL_WIDTH)/(1-ratio);	//convert Arc1 arc1 to the radius to Wheel of arc1

	theta = arc1_meters/radius;						//find the common theta to use in X-Y calcs

	*y_movement = (radius+WHEEL_CENTER)-(radius+WHEEL_CENTER)*cos(theta); //store the DELTA X value
	*x_movement = (radius+WHEEL_CENTER)*sin(theta);						 //store the DELTA y value
	return theta;
}

/*************************************************************************************
 *  center_axis_rotation();
 *
 * Will take encoder count and the X and Y pointers and will return the X y movement
 *
 *
 *
 *
 *
 ******************************************************************************************/
float center_axis_rotation(int encoder_count,float * x_val, float*y_val)
{
	float encoder_pulse = (float)encoder_count;
	float theta = ((float)(encoder_pulse/MAX_ENCODER)*WHEEL_CIRC)/WHEEL_WIDTH;
	*y_val = cos(theta)*WHEEL_CENTER-WHEEL_CENTER;
	*x_val = sin(theta)*WHEEL_CENTER;
	return theta;
}


/***************************************************************************************************************
 * THIS FUNCTION GETS THE WHEEL DATA FROM THE QEI ENCODER API
 *
 * THIS FUNCTION WILL DECIPHER THE DATA FROM THE VELOCITY API AND POSITION API
 *
 *
 * WILL STORE ABSOLUTE POSITION AND VAR POSITION AND CURRENT DIRECTION OF MOTORS
 * INTO THE QEI_DATA STRUCTURE FILE FOR EACH WHEEL
 *
 */
void getWheel_Data()
{

	//THIS NEEDS TO BE REFACTORED THIS PART IS NOT WORKING BECAUSE OF CHANGES IN DIRECTION CAUSING PROBLEMS
	//THE QEI DATA IS GETTING LOGGED INCORRECTLY IN ABSOLUTE POOSITION - NEED TO THINK OF A NEW ALGORITHM TO DEAL WITH THIS
	// SHOULD BE A SIMPLE FIX OF JUST CHECKING PULSE VALUE and NEGATING DIRECTION FOR CALCULATIONS.
	qei_data.left_wheel.encoder_cnt[cycle_data[cycle_data[speed_loop].now].now] = QEIPositionGet(QEI0_BASE);
	qei_data.left_wheel.wheel_dir = QEIDirectionGet(QEI0_BASE);
	qei_data.right_wheel.encoder_cnt[cycle_data[speed_loop].now] = QEIPositionGet(QEI1_BASE);
	qei_data.right_wheel.wheel_dir = QEIDirectionGet(QEI1_BASE);
	qei_data.left_wheel.vel[cycle_data[speed_loop].now] = QEIVelocityGet(QEI0_BASE);
	qei_data.right_wheel.vel[cycle_data[speed_loop].now] = QEIVelocityGet(QEI1_BASE);

	ctrl_robot.current_speed = (qei_data.left_wheel.vel[cycle_data[speed_loop].now] + qei_data.right_wheel.vel[cycle_data[speed_loop].now])/2;


	int displacement_left;
	int displacement_right;
	//day four - captains log, they still think I know how to code
	if(qei_data.left_wheel.wheel_dir == 1) //check current direction of encoder count
	{
		if(qei_data.left_wheel.encoder_cnt[cycle_data[speed_loop].back1]>19700 && qei_data.left_wheel.encoder_cnt[cycle_data[speed_loop].now]<19700)
		{
			 displacement_left = (39400+qei_data.left_wheel.encoder_cnt[cycle_data[speed_loop].now]) -qei_data.left_wheel.encoder_cnt[cycle_data[speed_loop].back1];
			if(displacement_left > 20000 ||  displacement_left < -20000)
			{
				 displacement_left = qei_data.left_wheel.encoder_cnt[cycle_data[speed_loop].now]-qei_data.left_wheel.encoder_cnt[cycle_data[speed_loop].back1];
			}
		}
		else
		{
			displacement_left = qei_data.left_wheel.encoder_cnt[cycle_data[speed_loop].now]-qei_data.left_wheel.encoder_cnt[cycle_data[speed_loop].back1];
			if(displacement_left > 20000 ||  displacement_left < -20000)
			{
			displacement_left = (qei_data.left_wheel.encoder_cnt[cycle_data[speed_loop].now]-39400)-qei_data.left_wheel.encoder_cnt[cycle_data[speed_loop].back1];
			}
		}
		qei_data.left_wheel.abs_pos +=  displacement_left;
		qei_data.left_wheel.dynamic_pos +=  displacement_left;
	}
	if(qei_data.left_wheel.wheel_dir == -1)
	{
			qei_data.left_wheel.vel[cycle_data[speed_loop].now] *= -1;
			if(qei_data.left_wheel.encoder_cnt[cycle_data[speed_loop].back1]<19700 && qei_data.left_wheel.encoder_cnt[cycle_data[speed_loop].now]>19700)
			{
				 displacement_left = (qei_data.left_wheel.encoder_cnt[cycle_data[speed_loop].now]-39400)-qei_data.left_wheel.encoder_cnt[cycle_data[speed_loop].back1];
				if( displacement_left > 20000 ||  displacement_left < -20000)
				{
					 displacement_left = qei_data.left_wheel.encoder_cnt[cycle_data[speed_loop].now]-qei_data.left_wheel.encoder_cnt[cycle_data[speed_loop].back1];
				}
			}
			else
			{
				displacement_left = qei_data.left_wheel.encoder_cnt[cycle_data[speed_loop].now]-qei_data.left_wheel.encoder_cnt[cycle_data[speed_loop].back1];
				if( displacement_left > 20000|| displacement_left < -20000)
				{
				displacement_left = (39400+qei_data.left_wheel.encoder_cnt[cycle_data[speed_loop].now]) -qei_data.left_wheel.encoder_cnt[cycle_data[speed_loop].back1];
				}
			}
			qei_data.left_wheel.abs_pos +=  displacement_left;
			qei_data.left_wheel.dynamic_pos +=  displacement_left;

	}
	if(qei_data.right_wheel.wheel_dir == 1)
	{

			if(qei_data.right_wheel.encoder_cnt[cycle_data[speed_loop].back1]>19700 && qei_data.right_wheel.encoder_cnt[cycle_data[speed_loop].now]<19700)
			{
				displacement_right = (39400+qei_data.right_wheel.encoder_cnt[cycle_data[speed_loop].now]) -qei_data.right_wheel.encoder_cnt[cycle_data[speed_loop].back1];
				if(displacement_right> 20000 || displacement_right < -20000)
				{
					displacement_right = qei_data.right_wheel.encoder_cnt[cycle_data[speed_loop].now]-qei_data.right_wheel.encoder_cnt[cycle_data[speed_loop].back1];
				}

			}
			else
			{
				displacement_right = qei_data.right_wheel.encoder_cnt[cycle_data[speed_loop].now]-qei_data.right_wheel.encoder_cnt[cycle_data[speed_loop].back1];
				if(displacement_right> 20000 || displacement_right < -20000)
				{
				displacement_right= (qei_data.right_wheel.encoder_cnt[cycle_data[speed_loop].now]-39400)-qei_data.right_wheel.encoder_cnt[cycle_data[speed_loop].back1];
				}
			}
			qei_data.right_wheel.abs_pos += displacement_right;
			qei_data.right_wheel.dynamic_pos += displacement_right;
	}
	if(qei_data.right_wheel.wheel_dir == -1)
	{
		qei_data.right_wheel.vel[cycle_data[speed_loop].now] *= -1;
			if(qei_data.right_wheel.encoder_cnt[cycle_data[speed_loop].back1]<19700 && qei_data.right_wheel.encoder_cnt[cycle_data[speed_loop].now]>19700)
			{
				displacement_right= (qei_data.right_wheel.encoder_cnt[cycle_data[speed_loop].now]-39400)-qei_data.right_wheel.encoder_cnt[cycle_data[speed_loop].back1];
				if(displacement_right > 20000 || displacement_right< -20000)
				{
					displacement_right = qei_data.right_wheel.encoder_cnt[cycle_data[speed_loop].now]-qei_data.right_wheel.encoder_cnt[cycle_data[speed_loop].back1];
				}
			}
			else
			{
				displacement_right = qei_data.right_wheel.encoder_cnt[cycle_data[speed_loop].now]-qei_data.right_wheel.encoder_cnt[cycle_data[speed_loop].back1];
				if(displacement_right > 20000|| displacement_right< -20000)
				{
					displacement_right = (39400+qei_data.right_wheel.encoder_cnt[cycle_data[speed_loop].now]) -qei_data.right_wheel.encoder_cnt[cycle_data[speed_loop].back1];
				}
			}

			qei_data.right_wheel.abs_pos += displacement_right;
			qei_data.right_wheel.dynamic_pos += displacement_right;
	}

	qei_data.left_wheel.accel[cycle_data[speed_loop].now] =
			((float)(qei_data.left_wheel.vel[cycle_data[speed_loop].now]-qei_data.left_wheel.vel[cycle_data[speed_loop].back1]));

	qei_data.right_wheel.accel[cycle_data[speed_loop].now] =
			(float)((qei_data.right_wheel.vel[cycle_data[speed_loop].now]-qei_data.right_wheel.vel[cycle_data[speed_loop].back1]));


	qei_data.left_wheel.var_pos[cycle_data[speed_loop].now] = qei_data.left_wheel.abs_pos;
	qei_data.right_wheel.var_pos[cycle_data[speed_loop].now] = qei_data.right_wheel.abs_pos;
	ctrl_robot.wheel_L_cpos[cycle_data[speed_loop].now] = qei_data.left_wheel.var_pos[cycle_data[speed_loop].now];
	ctrl_robot.wheel_R_cpos[cycle_data[speed_loop].now] = qei_data.right_wheel.var_pos[cycle_data[speed_loop].now];

	//ABSOLUTE POSITION MIGHT BE GETTING JUNKED, MAYBE WRITE A FAILSAFE TO DISCARD RIDICULOUS POSITION DATA
}


/***************************************************************************************************************
 * void calculate_wheel_f_pos(float x, float y, float orientation);
 *
 * parameters float X final pos and Y final pos and final orientation
 *
 *This will calculate the Final pos of the wheels based on the movement required from the wheels
 *
 */
void calculate_wheel_f_pos(float x, float y, float orientation)
{
	float X_displacement = x - ctrl_robot.x_abs_pos;
	float Y_displacement = y - ctrl_robot.y_abs_pos;

	float right_wheel_pos = 0;
	float left_wheel_pos =0;

	float theta_displacement = orientation - ctrl_robot.current_orientation;
	if(theta_displacement > PI)
	{
		theta_displacement -=2*PI;
	}
	if(theta_displacement < PI)
	{
		theta_displacement +=2*PI;
	}
	if(theta_displacement < 0)
	{
		right_wheel_pos -=(theta_displacement*WHEEL_CENTER)*(39400/WHEEL_CIRC);
		left_wheel_pos  +=(theta_displacement*WHEEL_CENTER)*(39400/WHEEL_CIRC);
	}
	if(theta_displacement > 0)
	{
		right_wheel_pos +=(theta_displacement*WHEEL_CENTER)*(39400/WHEEL_CIRC);
		left_wheel_pos  -= (theta_displacement*WHEEL_CENTER)*(39400/WHEEL_CIRC);
	}

	right_wheel_pos += sqrt(X_displacement*X_displacement + Y_displacement*Y_displacement)*(39400/WHEEL_CIRC);
	left_wheel_pos  += sqrt(X_displacement*X_displacement + Y_displacement*Y_displacement)*(39400/WHEEL_CIRC);
}

/**********************************************************************************
 * init_QEI()
 *
 * THIS FUNCTION INITIALIZES THE QEI PERIPHERALS ON THE BOARD
 * WE SET UP THE CORRECT PINS OF PD6 PD7 and PC5 PC6
 *
 * THIS FUNCTION ALSO SETS UP THE DIRECTION BITS THAT CONTROL MOTOR DIRECTION
 *
 * THIS FUNCTION ALSO INITIALIZES THE STARTING VALUES TO ZERO SO WE DON'T HAVE INITIAL CONDITION ERRORS
 *
 *	ALSO INITS THE DIRECTION -- DIRECTION PINS FOR THE MOTORS
 *
 */
void init_QEI()
{
		// Enable QEI Peripherals
		SysCtlPeripheralEnable(SYSCTL_PERIPH_QEI0);
		SysCtlPeripheralEnable(SYSCTL_PERIPH_QEI1);

		//Unlock GPIOD7 - Like PF0 its used for NMI - Without this step it doesn't work
		HWREG(GPIO_PORTD_BASE + GPIO_O_LOCK) = GPIO_LOCK_KEY; //In Tiva include this is the same as "_DD" in older versions (0x4C4F434B)
		HWREG(GPIO_PORTD_BASE + GPIO_O_CR) |= 0x80;
		HWREG(GPIO_PORTD_BASE + GPIO_O_LOCK) = 0;

		GPIOPinTypeGPIOOutput(GPIO_PORTE_BASE, GPIO_PIN_0);

		//Set Pins to be PHA0 and PHB0
		GPIOPinConfigure(GPIO_PD6_PHA0);
		GPIOPinConfigure(GPIO_PD7_PHB0);


		GPIOPinConfigure(GPIO_PD6_PHA0);


		GPIOPinConfigure(GPIO_PC5_PHA1);
		GPIOPinConfigure(GPIO_PC6_PHB1);
		//Set GPIO pins for QEI. PhA0 -> PD6, PhB0 ->PD7. I believe this sets the pull up and makes them inputs
		GPIOPinTypeQEI(GPIO_PORTD_BASE, GPIO_PIN_6 |  GPIO_PIN_7);
		GPIOPinTypeQEI(GPIO_PORTC_BASE, GPIO_PIN_5 |  GPIO_PIN_6);
		//DISable peripheral and int before configuration
		QEIDisable(QEI0_BASE);

		QEIIntDisable(QEI0_BASE,QEI_INTERROR | QEI_INTDIR | QEI_INTTIMER | QEI_INTINDEX);

		QEIDisable(QEI1_BASE);

		QEIIntDisable(QEI1_BASE,QEI_INTERROR | QEI_INTDIR | QEI_INTTIMER | QEI_INTINDEX);

		uint32_t MAXPOS = 39400;
		uint32_t MIDPOS = 19700;

		QEIConfigure(QEI0_BASE, (QEI_CONFIG_CAPTURE_A_B  | QEI_CONFIG_NO_RESET 	| QEI_CONFIG_QUADRATURE | QEI_CONFIG_NO_SWAP), MAXPOS);
		QEIVelocityConfigure(QEI0_BASE, QEI_VELDIV_1,1000000);

		QEIConfigure(QEI1_BASE, (QEI_CONFIG_CAPTURE_A_B  | QEI_CONFIG_NO_RESET 	| QEI_CONFIG_QUADRATURE | QEI_CONFIG_NO_SWAP), MAXPOS);
		QEIVelocityConfigure(QEI1_BASE, QEI_VELDIV_1,1000000);
		// Enable the quadrature encoder.
		QEIEnable(QEI0_BASE);
		QEIEnable(QEI1_BASE);
		//Set position to a middle value so we can see if things are working
		QEIPositionSet(QEI0_BASE, MIDPOS);
		QEIPositionSet(QEI1_BASE, MIDPOS);
		//Configure velocity setup

		QEIVelocityEnable(QEI0_BASE);
		QEIVelocityEnable(QEI1_BASE);

		GPIOPinWrite(GPIO_PORTE_BASE, GPIO_PIN_0, 0);


		qei_data.right_wheel.encoder_cnt[cycle_data[speed_loop].back1] = 0;
		qei_data.right_wheel.vel[cycle_data[speed_loop].back1] = 0;
		qei_data.left_wheel.encoder_cnt[cycle_data[speed_loop].back1] = 0;
		qei_data.left_wheel.vel[cycle_data[speed_loop].back1] = 0;

		int k = 0;
		for(k = 0; k < 20;k++)
		{
			qei_data.right_wheel.accel[k] = 0;
			qei_data.right_wheel.encoder_cnt[k] = 0;
			qei_data.right_wheel.vel[k] = 0;
			qei_data.right_wheel.encoder_cnt[k] = 19700;

			qei_data.left_wheel.accel[k] = 0;
			qei_data.left_wheel.encoder_cnt[k] = 0;
			qei_data.left_wheel.vel[k] = 0;
			qei_data.left_wheel.encoder_cnt[k] = 19700;
		}
		//MOTOR DIRECTION PINS FOR BALANCE BOT
		//THEY CAN BE CHANGED IF NEEDED
        HWREG(GPIO_PORTA_BASE + GPIO_O_LOCK) = GPIO_LOCK_KEY;
           HWREG(GPIO_PORTA_BASE + GPIO_O_CR) |= GPIO_PIN_5;
           GPIOPinTypeGPIOOutput(GPIO_PORTA_BASE, GPIO_PIN_5);

	    GPIOPinTypeGPIOOutput(GPIO_PORTE_BASE, GPIO_PIN_0);
	    //GPIOPinTypeGPIOOutput(GPIO_PORTD_BASE, GPIO_PIN_2);
	    GPIOPinWrite(GPIO_PORTE_BASE, GPIO_PIN_0, 0);
	    //GPIOPinWrite(GPIO_PORTD_BASE, GPIO_PIN_2, 0);
	    GPIOPinWrite(GPIO_PORTA_BASE, GPIO_PIN_5, 0);

	    //GPIOPinTypeGPIOOutput(GPIO_PORTB_BASE, GPIO_PIN_3); /// TESTING PIN FOR SPEED PERFORMANCE
	   // GPIOPinWrite(GPIO_PORTB_BASE, GPIO_PIN_3, GPIO_PIN_3);

}
