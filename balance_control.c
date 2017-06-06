/*
 * balance_control.c
 *
 *This file is meant for control of a Balancing robot with 2 wheels using QEI sensors and
 *IMU to determine position and angle
 *
 *
 *Meant for working with the TM4c123GH6PM board from TI
 *
 *
 *  Created on: Feb 12, 2017
 *      Author: c_ker
 */


#include "balance_control.h"


/***************************************************************************************
 * init_PID_values()
 *
 * This function initializes all the starting values for the balance_control
 * This will set the base PID values taht I have tuned to be somewhat workable - working on dynamic
 * changing of the PID with machine learning or just GUI control
 *
 *
 **************************************************************************************/
void init_pid_values()
{

	//REMEMBER THESE VALUES ARE NOT DEALING WITH THE SAME UNITS AND NUMBERS
	//SO PROPORTIONAL TO INTEGRAL TO DERIVATIVE ARE NOT REALLY GOOD INDICATORS
	//OF THERE RELEVANT WEIGHT ALONE


    _PIDA.Pgain = 3700;			// good base value 3800;
    //TAKES IMU ANGLE DATA EULER PITCH
    _PIDA.Igain = 40;			// good base values 220;
    //TAKES IMU ANGLE DATA EULER PITCH += EULER PITCH NOW
    _PIDA.Dgain = 380;			// good base value 315;
    //TAKES IMU GYROSCOPE DATA


	_PIDS.Pgain = 0.01;	//0.01	//good base value 0.00365;
	//TAKES ENCODER VELOCITY
	//_PIDS.Igain = 0.00005;
    _PIDS.Igain = 0.00025;	//0.00025	//good base value 0.000170;
    //TAKES ENCODER VELOCITY += ENCODER_VELOCITY NOW
    _PIDS.Dgain = 0.01;	//0.01	//good base value 0.045
    //TAKES ENCODER VELOCITY_FINAL - VELOCITY_PREVIOUS

    //PID_CONTROL_DATA._pid_position.Pgain = 0.0000; //debug values
    PID_CONTROL_DATA._pid_position.Pgain = 0.014; // good base value 0.00850
    //TAKES SET POINT ENCODER POSITION - VARIABLE ENCODER POSITION
    //PID_CONTROL_DATA._pid_position.Igain = 0.00000; //debug values
	PID_CONTROL_DATA._pid_position.Igain = 0.00025;  //good base value 0.000230
	//TAKES THE SET_POINT ERROR CUMULATIVE
	//PID_CONTROL_DATA._pid_position.Dgain = 0.00; //debug values
	PID_CONTROL_DATA._pid_position.Dgain = 1.1;	//good base value 1.05
	//THIS IS THE ENCODER VELOCITY AGAIN
}

/*************************************************************************************
 * PID_pos()
 *
 * Params this function has no params
 *
 * it returns the SPEED requested to aqcuire set-point position
 *
 *This function will read the current offset of the position and combine
 *proportional and integral and Derivative to set the desired speed of the robot
 *to attain this desired position set_point
 *
 */
float PID_pos()
{
	float speed = 0;
	//Perror is the distance you are from your set-Point location
	//We need to refactor this to work with X - Y and Orientation setpoints
	float Perror_position;
	float Derror;
	switch(ctrl_robot.turn_state)
	{
	case 0:
	 Perror_position =  ((ctrl_robot.wheel_L_fpos  - (ctrl_robot.wheel_L_cpos[cycle_data[speed_loop].now] +
			                  ctrl_robot.wheel_L_cpos[cycle_data[speed_loop].back1] + ctrl_robot.wheel_L_cpos[cycle_data[speed_loop].back2])/3)
							  + (ctrl_robot.wheel_R_fpos - (ctrl_robot.wheel_R_cpos[cycle_data[speed_loop].now] +
							  ctrl_robot.wheel_R_cpos[cycle_data[speed_loop].back1] +ctrl_robot.wheel_R_cpos[cycle_data[speed_loop].back2])/3))/2;
	//Derror is the velocity error you are at I use both left and right and average them
	 Derror = (qei_data.left_wheel.vel[cycle_data[speed_loop].now] + qei_data.left_wheel.vel[cycle_data[speed_loop].back1] +
					qei_data.right_wheel.vel[cycle_data[speed_loop].now] +qei_data.right_wheel.vel[cycle_data[speed_loop].back1]+
					qei_data.left_wheel.vel[cycle_data[speed_loop].back2] + qei_data.left_wheel.vel[cycle_data[speed_loop].back3] +
					qei_data.right_wheel.vel[cycle_data[speed_loop].back2] +qei_data.right_wheel.vel[cycle_data[speed_loop].back3]+
					qei_data.left_wheel.vel[cycle_data[speed_loop].back4] + qei_data.left_wheel.vel[cycle_data[speed_loop].back5] +
					qei_data.right_wheel.vel[cycle_data[speed_loop].back4] +qei_data.right_wheel.vel[cycle_data[speed_loop].back5]+
					qei_data.left_wheel.vel[cycle_data[speed_loop].back6] + qei_data.left_wheel.vel[cycle_data[speed_loop].back7] +
					qei_data.right_wheel.vel[cycle_data[speed_loop].back6] +qei_data.right_wheel.vel[cycle_data[speed_loop].back7])/16;
	break;
	case 2:
		Perror_position =  ((ctrl_robot.wheel_L_fpos  - (ctrl_robot.wheel_L_cpos[cycle_data[speed_loop].now] +
				                  ctrl_robot.wheel_L_cpos[cycle_data[speed_loop].back1] + ctrl_robot.wheel_L_cpos[cycle_data[speed_loop].back2])/3));
		//Derror is the velocity error you are at I use both left and right and average them
		Derror = (qei_data.left_wheel.vel[cycle_data[speed_loop].now] + qei_data.left_wheel.vel[cycle_data[speed_loop].back1] +
						qei_data.left_wheel.vel[cycle_data[speed_loop].back2] + qei_data.left_wheel.vel[cycle_data[speed_loop].back3] +
						qei_data.left_wheel.vel[cycle_data[speed_loop].back4] + qei_data.left_wheel.vel[cycle_data[speed_loop].back5] +
						qei_data.left_wheel.vel[cycle_data[speed_loop].back6] + qei_data.left_wheel.vel[cycle_data[speed_loop].back7] )/8;
		break;
	case 3:
		 Perror_position =		(ctrl_robot.wheel_R_fpos - (ctrl_robot.wheel_R_cpos[cycle_data[speed_loop].now] +
								  ctrl_robot.wheel_R_cpos[cycle_data[speed_loop].back1] +ctrl_robot.wheel_R_cpos[cycle_data[speed_loop].back2])/3)/2;
		//Derror is the velocity error you are at I use both left and right and average them
		Derror = (qei_data.right_wheel.vel[cycle_data[speed_loop].now] +qei_data.right_wheel.vel[cycle_data[speed_loop].back1]+
						qei_data.right_wheel.vel[cycle_data[speed_loop].back2] +qei_data.right_wheel.vel[cycle_data[speed_loop].back3]+
						qei_data.right_wheel.vel[cycle_data[speed_loop].back4] +qei_data.right_wheel.vel[cycle_data[speed_loop].back5]+
						qei_data.right_wheel.vel[cycle_data[speed_loop].back6] +qei_data.right_wheel.vel[cycle_data[speed_loop].back7])/8;
		break;
	default:
	Perror_position =  ((ctrl_robot.wheel_L_fpos  - (ctrl_robot.wheel_L_cpos[cycle_data[speed_loop].now] +
			                  ctrl_robot.wheel_L_cpos[cycle_data[speed_loop].back1] + ctrl_robot.wheel_L_cpos[cycle_data[speed_loop].back2])/3)
							  + (ctrl_robot.wheel_R_fpos - (ctrl_robot.wheel_R_cpos[cycle_data[speed_loop].now] +
							  ctrl_robot.wheel_R_cpos[cycle_data[speed_loop].back1] +ctrl_robot.wheel_R_cpos[cycle_data[speed_loop].back2])/3))/2;
	//Derror is the velocity error you are at I use both left and right and average them
	Derror = (qei_data.left_wheel.vel[cycle_data[speed_loop].now] + qei_data.left_wheel.vel[cycle_data[speed_loop].back1] +
					qei_data.right_wheel.vel[cycle_data[speed_loop].now] +qei_data.right_wheel.vel[cycle_data[speed_loop].back1]+
					qei_data.left_wheel.vel[cycle_data[speed_loop].back2] + qei_data.left_wheel.vel[cycle_data[speed_loop].back3] +
					qei_data.right_wheel.vel[cycle_data[speed_loop].back2] +qei_data.right_wheel.vel[cycle_data[speed_loop].back3]+
					qei_data.left_wheel.vel[cycle_data[speed_loop].back4] + qei_data.left_wheel.vel[cycle_data[speed_loop].back5] +
					qei_data.right_wheel.vel[cycle_data[speed_loop].back4] +qei_data.right_wheel.vel[cycle_data[speed_loop].back5]+
					qei_data.left_wheel.vel[cycle_data[speed_loop].back6] + qei_data.left_wheel.vel[cycle_data[speed_loop].back7] +
					qei_data.right_wheel.vel[cycle_data[speed_loop].back6] +qei_data.right_wheel.vel[cycle_data[speed_loop].back7])/16;
		break;
	}
	//Ierror is the accumulated Perror Values - this is capped at 400000 need to refactor this as well
	//** REFACTOR **//
	if(PID_CONTROL_DATA._pid_position.Ierror >300000)
	{
		PID_CONTROL_DATA._pid_position.Ierror = 300000;
	}
	if(PID_CONTROL_DATA._pid_position.Ierror< -300000)
	{
		PID_CONTROL_DATA._pid_position.Ierror = -300000;
	}
	PID_CONTROL_DATA._pid_position.Ierror +=Perror_position/100;

	float KPerror = Perror_position * PID_CONTROL_DATA._pid_position.Pgain;
	float KDerror = Derror *  PID_CONTROL_DATA._pid_position.Dgain;
	float KIerror = PID_CONTROL_DATA._pid_position.Igain * PID_CONTROL_DATA._pid_position.Ierror;
	switch(ctrl_robot.turn_state)
	{
		case 0:
			KPerror = Perror_position * PID_CONTROL_DATA._pid_position.Pgain;
			KDerror = Derror *  PID_CONTROL_DATA._pid_position.Dgain;
			KIerror = PID_CONTROL_DATA._pid_position.Igain * PID_CONTROL_DATA._pid_position.Ierror;

			break;
		case 2:
			KPerror = Perror_position * PID_CONTROL_DATA._pid_position.Pgain*.75;
			KDerror = Derror *  PID_CONTROL_DATA._pid_position.Dgain*.75;
			KIerror = PID_CONTROL_DATA._pid_position.Igain * PID_CONTROL_DATA._pid_position.Ierror*.75;

			break;
		case 3:
			KPerror = Perror_position * PID_CONTROL_DATA._pid_position.Pgain*.75;
			KDerror = Derror *  PID_CONTROL_DATA._pid_position.Dgain*.75;
			KIerror = PID_CONTROL_DATA._pid_position.Igain * PID_CONTROL_DATA._pid_position.Ierror*.75;

			break;
		default:
			break;
	}


	if(KPerror > 6000) KPerror = 6000;
	if(KPerror <-6000)	KPerror = -6000;
	if(KDerror > 6000)	KDerror = 6000;
	if(KDerror <-6000)	KDerror = -6000;
	if(KIerror >1000) KIerror = 1000;
	if(KIerror<-1000) KIerror= -1000;




	speed = (KPerror - KDerror + KIerror);
	if(speed>5000)
	{
		speed = 5000;
	}
	if(speed < -5000)
	{
		speed = -5000;
	}
	PID_CONTROL_DATA._pid_position.setPoint = speed;

	return speed;
}




/*************************************************************************************
 * PID_speed()
 *
 * Params Float Speed - the speed setpoint the robot will try to achieve
 *
 *THIS FUNCTION WILL TAKE THE CURRENT AUTOMATED SPEED CALCULATION AND DECIDE WHAT ANGLE
 *THE ROBOT NEEDS TO BE AT TO ACHIEVE THIS SPEED SETTING- CURRENTLY WE HAVE THE PID
 *IN A CASCADING MANNER - THE SPEED PID CHANGES THE SETPOINT OF THE ANGLE PID
 *WE HAVE THE CURRENT SPEED PID CALLED 1 time for EVERY 10 TIMES OF THE ANGLE PID
 *WE DO THIS TO REDUCE THE INTERFERENCE OF THE TWO SIGNALS AND ALLOW THE ROBOT
 *TO ACHEIVE THE CURRENT SETPOINT BEFORE HAVING TO CHANGE VALUES AGAIN
 *
 * THE ROBOT AUTOMATION SOFTWARE WILL DECIDE THE CURRENT SPEED IT WISHES TO ACHIEVE TO GET TO
 * THE CORRECT POSITION OR TO MAINTAIN THE CORRECT POSITION
 *
 *
 */
void PID_speed(float speed)
{

	float set_point = 0;
	float Perror;
	float Derror;
	Perror = speed - ((float)(qei_data.left_wheel.vel[cycle_data[speed_loop].now] + qei_data.left_wheel.vel[cycle_data[speed_loop].back1]+
			qei_data.left_wheel.vel[cycle_data[speed_loop].back2] + qei_data.left_wheel.vel[cycle_data[speed_loop].back3]+
			qei_data.left_wheel.vel[cycle_data[speed_loop].back3] + qei_data.left_wheel.vel[cycle_data[speed_loop].back4]+
			qei_data.left_wheel.vel[cycle_data[speed_loop].back5] + qei_data.left_wheel.vel[cycle_data[speed_loop].back6]+
			qei_data.right_wheel.vel[cycle_data[speed_loop].now] + qei_data.right_wheel.vel[cycle_data[speed_loop].back1]+
			qei_data.right_wheel.vel[cycle_data[speed_loop].back2] + qei_data.right_wheel.vel[cycle_data[speed_loop].back3]+
			qei_data.right_wheel.vel[cycle_data[speed_loop].back3] + qei_data.right_wheel.vel[cycle_data[speed_loop].back4]+
			qei_data.right_wheel.vel[cycle_data[speed_loop].back4] + qei_data.right_wheel.vel[cycle_data[speed_loop].back5])/16);


	Derror = ((qei_data.left_wheel.accel[cycle_data[speed_loop].now] + qei_data.left_wheel.accel[cycle_data[speed_loop].back1]+
			qei_data.left_wheel.accel[cycle_data[speed_loop].back2] + qei_data.left_wheel.accel[cycle_data[speed_loop].back3]+
			qei_data.left_wheel.accel[cycle_data[speed_loop].back4] + qei_data.left_wheel.accel[cycle_data[speed_loop].back5]+
			qei_data.left_wheel.accel[cycle_data[speed_loop].back6] + qei_data.left_wheel.accel[cycle_data[speed_loop].back7]+
			qei_data.right_wheel.accel[cycle_data[speed_loop].now]+ qei_data.right_wheel.accel[cycle_data[speed_loop].back1]+
			qei_data.right_wheel.accel[cycle_data[speed_loop].back2]+ qei_data.right_wheel.accel[cycle_data[speed_loop].back3]+
			qei_data.right_wheel.accel[cycle_data[speed_loop].back4]+ qei_data.right_wheel.accel[cycle_data[speed_loop].back5]+
			qei_data.right_wheel.accel[cycle_data[speed_loop].back6]+ qei_data.right_wheel.accel[cycle_data[speed_loop].back7])/16);

	//CAPS FOR Derror and Perror



	_PIDS.Ierror += Perror/100;
	if(_PIDS.Ierror > 10000) _PIDS.Ierror = 10000; // THESE NUMBERS PLAY A LARGE ROLE IN THE INTEGRAL PART
	if(_PIDS.Ierror < -10000)_PIDS.Ierror =-10000; // THESE NUMBERS NEED TO BE TUNED FOR DESIRED MAX INTEGRAL
	//THESE NUMBERS WILL HELP DEALING WITH INTEGRAL WINDUP

	//SET POINT CROSS IS ANOTHER WAY I THOUGHT WE COULD DEAL WITH INTEGRAL WINDUP SO THAT WHEN WE HIT DESIRED SET POINT
	//WE EQUALIZE -- HOWEVER THIS ADDED MORE INSTABILITY TO THE PROBLEM
/*
	float set_point_cross = (qei_data.left_wheel.vel[cycle_data[speed_loop].now]+qei_data.right_wheel.vel[cycle_data[speed_loop].now])/2;
	float set_point_cross_back1 = (qei_data.left_wheel.vel[cycle_data[speed_loop].back1] +qei_data.right_wheel.vel[cycle_data[speed_loop].back1])/2;

	if((speed < set_point_cross && set_point_cross_back1 > speed)||speed > set_point_cross && set_point_cross_back1 < speed)
	{
		_PIDS.Ierror /= 2;
	}
*/
	float KPerror = _PIDS.Pgain * Perror;
	float KDerror = _PIDS.Dgain * Derror;
	float KIerror = _PIDS.Igain * _PIDS.Ierror;


	if(KPerror>10)KPerror  =  10;		//CAPPING THE PROPORTIONAL ERROR
	if(KPerror<-10)KPerror = -10;		//CAPPING THE PROP
	if(KDerror>10)KDerror =  10;		//CAPPING DERIVATIVE ERROR
	if(KDerror<-10)KDerror = -10;		//CAPPING DERIVATIVE ERROR
	if(KIerror>5)KIerror  =  5;		//CAPPING INTEGRAL ERROR
	if(KIerror<-5)KIerror= -5;		//CAPPING INTEGRAL ERROR


	//debug_swap_left[debug_count] =(float) KDerror;		//DEBUG ARRAY to check values
	//debug_swap_right[debug_count] =(float) KPerror;		//DEBUG ARRAY
	//debug_pwm_left[debug_count] =Derror;

	_PIDS.setPoint = (KPerror - KDerror + KIerror); //ADDING PIDS to SETPOINT



	if(_PIDS.setPoint > 15.0)_PIDS.setPoint = 15.0; //CAPPING SETPOINT TO 20 Degrees
	if(_PIDS.setPoint <-15.0)_PIDS.setPoint =-15.0;

	//debug_pwm_right[debug_count]=_PIDS.setPoint;

	set_point = _PIDS.setPoint;

	//THIS DECREASES OSCILLATION AROUND CENTERPOINT SO THE ROBOT DOESNT CONSTANTLY SWITCH DIRECTION

	PID_angle(set_point);
	//PID_angle(0);
	//INCREMENT SPEED LOOP FOR ARRAY FUNCTIONALITY


}




/*****************************************************************************************************
 *
 * PID_ANGLE
 *
 * THIS FUNCTION WILL TAKE THE GIVEN FLOAT POS AND THEN SET THE PID VALUES BASED ON IMU DATA
 * THE PIDS WILL THEN DETERMINE THE OUTPUT OF THE PWM SIGNAL TO THE MOTORS TO ACHIEVE THE SET POINT
 *
 * THE INTEGRAL VALUE IS CAPPED AND UNWOUND WHENEVER CROSSING THE SETPOINT
 * D Value IS THE GYRO DATA
 * P Value IS THE ERROR FROM OFFSET
 * I Value is the Culmination of the error
 *
 *
 *
 */
void PID_angle(float pos)
{
	float errorP;
	float errorD;
	_PIDA.setPoint = -pos;

	//FIRST TIME THE LOOP RUNS WE WANT NON GARBAGE VALUES SO WE INITALIZE THEM TO ZERO
	if(angle_loop== -1)
	{
	init_main_loop();
	pid_imu_position[angle_loop].current_pos =  imu_data[angle_loop].euler_values.p;
	errorP = _PIDA.setPoint-imu_data[angle_loop].euler_values.p;
	errorD = 0;
	_PIDA.Ierror = errorP;
	}
	else //After first run values are initialized as non-garbage We should clean this up to reduce timings
	{
	pid_imu_position[cycle_data[angle_loop].now].current_pos = imu_data[angle_loop].euler_values.p-1.35;
	pid_imu_position[cycle_data[angle_loop].now].current_vel = imu_data[angle_loop].x_gyro;
			errorP = (float) (_PIDA.setPoint - (pid_imu_position[cycle_data[angle_loop].now].current_pos));
			errorD = (float)((pid_imu_position[cycle_data[angle_loop].now].current_vel));

			/*
		if(pid_imu_position[cycle_data[angle_loop].now].current_pos<pos && pid_imu_position[cycle_data[angle_loop].back1].current_pos > pos)
		{
			_PIDA.Ierror /=2;
		}
		if(pid_imu_position[cycle_data[angle_loop].now].current_pos > pos && pid_imu_position[cycle_data[angle_loop].back1].current_pos <pos)
		{
			_PIDA.Ierror /=2;
		}
		*/
		_PIDA.Ierror += errorP;
		if(_PIDA.Ierror > 150.0)_PIDA.Ierror = 150;	//capping the IError
		if(_PIDA.Ierror <-150.0)_PIDA.Ierror = -150; //capping the IError
	}


	float errorKP;
	float errorKD;
	float errorKI;
	//THE PID VALUES ARE DYNAMICALLY CHANGED BASED ON THE ANGULAR POSITION- THE FURTHER AWAY FROM 0
	// THE HIGHER THE Perror_angle_factor is, increasing the output at higher angles
	//float Perror_angle_factor = sin(imu_data[angle_loop].euler_values.p*(3.14159/180.0));
	//Perror_angle_factor = sqrt(sqrt(sqrt(sqrt(sqrt(fabs(Perror_angle_factor))))));


	//the ErrorKP and KD and KI values to determine the output of the motors
	errorKP = errorP * _PIDA.Pgain;//* Perror_angle_factor;
	errorKD = errorD * _PIDA.Dgain;//* Perror_angle_factor;
	errorKI = _PIDA.Ierror *_PIDA.Igain;//*Perror_angle_factor;

	//cap the outputs from the errors so we avoid lopsided issues
	if(errorKP > 65535.0)
	{
		errorKP = 65535.0;
	}
	if(errorKP < -65535.0)
	{
	errorKP = -65535.0;
	}
	if(errorKD > 65535.0)
	{
		errorKD = 65535.0;
	}
	if(errorKD < -65535.0)
	{
	errorKD = -65535.0;
	}

	//Take the error values and set it to the output
	pid_imu_position[cycle_data[angle_loop].now].output = errorKP + (errorKI) + (errorKD);
	//Set the PWM out as the output value for the wheels
	qei_data.left_wheel.pwm_out = pid_imu_position[cycle_data[angle_loop].now].output;
	//pid_imu_position[cycle_data[angle_loop].back2].output)/3;
	qei_data.right_wheel.pwm_out = qei_data.left_wheel.pwm_out;

	wheel_control();

	//We loop the values of the error 20 times so we can look back at the data or average the signals if data is bad
	angle_loop++;
	if(angle_loop == 20)
	{
		angle_loop = 0;
	}
	speed_loop++;

	if(speed_loop == 20)//CAP LOOP AT 20
	{
	speed_loop=0;
	}
}



/*********************************************************************
 * wheel_control()
 *
 * THIS FUNCTION WILL DETERMINE THE CORRECT STATE OPERATION OF THE WHEELS BASED ON THE ROBOT OBJECTIVE
 *
 * THIS WILL TUNE THE PWM OUTPUT TO ACHIEVE SPECIFIC ACTIONS BASED ON THE CORRECT MOTION PLANNING
 *
 * WE WILL NEED TO DECIDE IF WE WANT TO GO STRAIGHT FORWARD - SOFT TURN - PIVOT TURN - OR CENTER TURN
 *
 *
 * ALGORITHM WILL NEED TO BE DEVISED BEFORE HAND SO THIS FUNCTION CAN CALL THE NESCESSARY LOGIC TO ACHIEVE
 * THE CORRECT MOTION PROFILE
 *
 *
 *
 *
 ****************************************************************************************/
void wheel_control()
{

	switch(ctrl_robot.turn_state)
	{
	case 0:
	forward_wheel_control();		//FORWARD MOTION KEEP WHEELS ALIGNED
		break;
	case 1:
	soft_turn_control();			//FORWARD MOTION ONE WHEEL FASTER - BASED ON TURNING ANGLE
		break;
	case 2:
	right_wheel_pivot();			//RIGHT WHEEL PIVOT TURN - RIGHT WHEEL STAYS AT IT'S POSITION
		break;
	case 3:
	left_wheel_pivot();				//LEFT WHEEL PIVOT TURN - LEFT WHEEL STAYS AT IT'S POSITION
		break;
	case 4:
	center_turn_right();			//CENTER AXIS TURN TO THE RIGHT NEG RADIANS
		break;
	case 5:
	center_turn_left();				//CENTER AXIS TURN TO THE LEFT POS RADIANS
		break;
	case 6:
		break;
	case 7:
		break;
	default:
		forward_wheel_control();	//DEFAULT STAY IN LINE FORWARD OR REVERSE
		break;
	}
}

/**************************************************************************************
 * soft_turn_control()
 *
 *
 *PARAMS are left percentage and Right percentage shift
 * THIS FUNCTION WILL PERFORM AN ALGORITHM TO PERFORM A GRADUAL TURN WHILE MOVING FORWARD
 * WE WOULD LIKE TO IMPLEMENT A INPUT THAT DECIDES HOW GRADUAL AND WHAT RATE THE ROBOT NEEDS TO TURN AT
 * THIS VALUE WILL NEED TO BE CALCULATED BY THE MOTION PLANNING OF THE ROBOT TO ACHIEVE ITS OBJECTIVE
 * GRADUAL TURN WILL BE USED WHEN THE ROBOT NEEDS TO GO FORWARD AND TURN AS WELL
 *
 *
 */

void soft_turn_control()
{
	float left = (float)sensor_hub.turn;
	float right = (float)sensor_hub.turn;
	//SHIFT THE OUTPUT PWM FROM ONE WHEEL TO THE OTHER WHICH WILL INCREASE THE TURNING
	if(qei_data.right_wheel.pwm_out < 0)
	{
		qei_data.left_wheel.pwm_out *=-1;
		qei_data.right_wheel.pwm_out*=-1;
		if(sensor_hub.turn < -5)
		{
		left = -left/100 + 1;
		right = 1+right/100;
		qei_data.left_wheel.pwm_out *= left;
		qei_data.right_wheel.pwm_out *= right;
		}
		if(sensor_hub.turn > 5)
		{
		left = -left/100 + 1;
		right = 1+right/100;
		qei_data.left_wheel.pwm_out *= left;
		qei_data.right_wheel.pwm_out *= right;
		}
		qei_data.right_wheel.wheel_dir = -1;
		qei_data.left_wheel.wheel_dir = -1;
	}
	else if(qei_data.right_wheel.pwm_out > 0)
	{
		if(sensor_hub.turn > 5)
		{
		left = left/100 + 1;
		right = 1-right/100;
		qei_data.left_wheel.pwm_out *= left;
		qei_data.right_wheel.pwm_out *= right;
		}
		if(sensor_hub.turn < -5)
		{
		left = left/100 + 1;
		right = 1-right/100;
		qei_data.left_wheel.pwm_out *= left;
		qei_data.right_wheel.pwm_out *= right;
		}
		qei_data.right_wheel.wheel_dir = 1;
		qei_data.left_wheel.wheel_dir = 1;
	}
	if(qei_data.left_wheel.pwm_out <0)
		{qei_data.left_wheel.pwm_out = 0;}
	if(	qei_data.left_wheel.pwm_out < 0)
		{qei_data.left_wheel.pwm_out = 0;}
	if(qei_data.left_wheel.pwm_out > 65535)qei_data.left_wheel.pwm_out = 65535;
	if(qei_data.right_wheel.pwm_out > 65535)qei_data.right_wheel.pwm_out = 65535;
}


/***************************************************************************
 * If moving straight forward or backward this function will
 * Pick the correct direction of the motors and will make sure
 * that the wheels stay in perfect alignment
 *
 *
 *
 */
void forward_wheel_control()
{
	if(qei_data.right_wheel.pwm_out > 65535.0 || qei_data.right_wheel.pwm_out < -65535.0)
		{
			if(qei_data.right_wheel.pwm_out<0)
			{
				qei_data.right_wheel.wheel_dir = -1;
			}
			else
			{
				qei_data.right_wheel.wheel_dir = 1;
			}
			qei_data.right_wheel.pwm_out  = 65535.0;
		}
	else if(qei_data.right_wheel.pwm_out < 0)
		{
			qei_data.right_wheel.pwm_out = fabs(qei_data.right_wheel.pwm_out);
			qei_data.right_wheel.wheel_dir = -1;
		}
		else
		{
			qei_data.right_wheel.wheel_dir = 1;
		}
	qei_data.left_wheel.wheel_dir = qei_data.right_wheel.wheel_dir;
	qei_data.left_wheel.pwm_out =qei_data.right_wheel.pwm_out;


	// DYNAMICALLY CHANGE THE WHEELS PWM TO FIX CURVING DIRECTIONS

	float val = 0;
	float right_percent = 1;
	float left_percent = 1;
	float pwm_value_right = qei_data.right_wheel.pwm_out;
	float pwm_value_left = qei_data.left_wheel.pwm_out;
	if(qei_data.right_wheel.wheel_dir==-1)
	{
		if(qei_data.right_wheel.dynamic_pos > qei_data.left_wheel.dynamic_pos)
		{
			val = qei_data.right_wheel.dynamic_pos - qei_data.left_wheel.dynamic_pos;
			//This will slightly offset the PWM values of the outputs if one wheel lags behind
			right_percent = 1+(val/1000);
			left_percent = 1-(val/1000);
			//CAP THE PERCENTAGE FOR MORE GRADUAL TURNS back to origin
			if(right_percent > 1.4)
			{
				right_percent = 1.4;
			}
			if(left_percent < 0.6)
			{
				left_percent = 0.6;
			}
			pwm_value_right = qei_data.right_wheel.pwm_out*right_percent;
			pwm_value_left = qei_data.left_wheel.pwm_out *left_percent;
			qei_data.right_wheel.pwm_out = pwm_value_right;
			qei_data.left_wheel.pwm_out = pwm_value_left;

		}
		if(qei_data.right_wheel.dynamic_pos < qei_data.left_wheel.dynamic_pos)
		{
			val = qei_data.left_wheel.dynamic_pos - qei_data.right_wheel.dynamic_pos;
						//This will slightly offset the PWM values of the outputs if one wheel lags behind
						right_percent = 1-(val/1000);
						left_percent = 1+(val/1000);
						//CAP THE PERCENTAGE FOR MORE GRADUAL TURNS back to origin
						if(right_percent < 0.6)
						{
							right_percent = 0.6;
						}
						if(left_percent > 1.4)
						{
							left_percent = 1.4;
						}
						pwm_value_right = qei_data.right_wheel.pwm_out*right_percent;
						pwm_value_left = qei_data.left_wheel.pwm_out *left_percent;
			qei_data.right_wheel.pwm_out =  pwm_value_right;
			qei_data.left_wheel.pwm_out = pwm_value_left;
		}
	}
	//DYNAMICALLY CHANGE WHEELS PWM TO FIX OFFSETS
	if(qei_data.right_wheel.wheel_dir==1)
	{
		if(qei_data.right_wheel.dynamic_pos > qei_data.left_wheel.dynamic_pos)
		{
			val = qei_data.right_wheel.dynamic_pos - qei_data.left_wheel.dynamic_pos;
					//This will slightly offset the PWM values of the outputs if one wheel lags behind
					right_percent = 1-(val/1000);
					left_percent = 1+(val/1000);
					//CAP THE PERCENTAGE FOR MORE GRADUAL TURNS back to origin
					if(right_percent < 0.6)
					{
						right_percent = 0.6;
					}
					if(left_percent > 1.4)
					{
						left_percent = 1.4;
					}
					pwm_value_right = qei_data.right_wheel.pwm_out*right_percent;
					pwm_value_left = qei_data.left_wheel.pwm_out *left_percent;
			qei_data.right_wheel.pwm_out =  pwm_value_right;
			qei_data.left_wheel.pwm_out = pwm_value_left;
		}
		if(qei_data.right_wheel.dynamic_pos < qei_data.left_wheel.dynamic_pos)
		{
			val = qei_data.left_wheel.dynamic_pos - qei_data.right_wheel.dynamic_pos;
			//This will slightly offset the PWM values of the outputs if one wheel lags behind
			right_percent = 1+(val/1000);
			left_percent = 1-(val/1000);
			//CAP THE PERCENTAGE FOR MORE GRADUAL TURNS back to origin
			if(right_percent > 1.4)
			{
				right_percent = 1.4;
			}
			if(left_percent < 0.6)
			{
				left_percent = 0.6;
			}
			pwm_value_right = qei_data.right_wheel.pwm_out*right_percent;
			pwm_value_left = qei_data.left_wheel.pwm_out *left_percent;
			qei_data.right_wheel.pwm_out =  pwm_value_right;
			qei_data.left_wheel.pwm_out = pwm_value_left;
		}
	}
	if(qei_data.right_wheel.pwm_out > 65535.0)
	{
		qei_data.right_wheel.pwm_out = 65535.0;
	}
	if(qei_data.left_wheel.pwm_out > 65535.0)
	{
		qei_data.left_wheel.pwm_out = 65535.0;
	}
}

/***************************************************************************************
 * center_turn_left()
 *
 * This function will turn the robot in place to the left it will just keep turning while the function is called
 * control of when to call this function will happen elsewhere.
 *
 *
 *
 */
void center_turn_left()
{

		if(qei_data.right_wheel.pwm_out > 65535.0 || qei_data.right_wheel.pwm_out < -65535.0)
		{
			if(qei_data.right_wheel.pwm_out<0)
			{
				qei_data.right_wheel.wheel_dir = -1;
			}
			else
			{
				qei_data.right_wheel.wheel_dir = 1;
			}
			qei_data.right_wheel.pwm_out = 65535.0;
		}
		else
		{
			if(qei_data.right_wheel.pwm_out<0)
			{
			qei_data.right_wheel.wheel_dir = -1;
			qei_data.right_wheel.pwm_out = fabs(qei_data.right_wheel.pwm_out);
			}
			else
			{
			qei_data.right_wheel.wheel_dir = 1;
			}
		}
		float percentage = (float)(fabs(qei_data.right_wheel.pwm_out/65535.0));
		float new_pwm = (float)qei_data.right_wheel.pwm_out/2;

		if(qei_data.right_wheel.wheel_dir == 1)
		{
			qei_data.left_wheel.pwm_out = (int)(3000+new_pwm);
			qei_data.right_wheel.pwm_out = 3000;
		}
		if(qei_data.right_wheel.wheel_dir == -1)
		{
			qei_data.left_wheel.pwm_out = 3000;
			qei_data.right_wheel.pwm_out = (int)(3000+new_pwm);
		}
	qei_data.right_wheel.wheel_dir = -1;
	qei_data.left_wheel.wheel_dir = 1;
}


/***************************************************************************************
 * center_turn_right()
 *
 * This function will turn the robot in place to the left it will just keep turning while the function is called
 * control of when to call this function will happen elsewhere.
 *
 *
 *
 */
void center_turn_right()
{

	if(qei_data.right_wheel.pwm_out > 65535.0 || qei_data.right_wheel.pwm_out < -65535.0)
	{
		if(qei_data.right_wheel.pwm_out<0)
		{
			qei_data.right_wheel.wheel_dir = -1;
			qei_data.right_wheel.pwm_out = fabs(qei_data.right_wheel.pwm_out);
		}
		else
		{
			qei_data.right_wheel.wheel_dir = 1;
		}
		qei_data.right_wheel.pwm_out = 65535.0;
	}
	else
	{
		if(qei_data.right_wheel.pwm_out<0)
		{
		qei_data.right_wheel.wheel_dir = -1;
		qei_data.right_wheel.pwm_out = fabs(qei_data.right_wheel.pwm_out);
		}
		else
		{
		qei_data.right_wheel.wheel_dir = 1;
		}
	}
	float new_pwm = (float)qei_data.right_wheel.pwm_out/2;

	if(qei_data.right_wheel.wheel_dir == 1)
	{
		qei_data.left_wheel.pwm_out = 3000;
		qei_data.right_wheel.pwm_out = (int)(3000+new_pwm);
	}
	if(qei_data.right_wheel.wheel_dir == -1)
	{
		qei_data.left_wheel.pwm_out = (int)(3000+new_pwm);
		qei_data.right_wheel.pwm_out = 3000;
	}
qei_data.right_wheel.wheel_dir = 1;
qei_data.left_wheel.wheel_dir = -1;
}



/***************************************************************************************
 * left_wheel_pivot()
 *
 *THIS FUNCTION WILL TIE ONE OF THE WHEELS TO IT"S CURRENT LOCATION AND HAVE THE OTHER WHEEL
 *DRIVE IT SLIGTHLY FORWARD TURNING THE ROBOT ON A PIVOT POINT OF IT'S LEFT WHEEL
 *THIS FUNCTION WILL BE CALLED WHEN YOU ROBOT DETERMINES IT NEEDS TO PIVOT TURN
 *
 *
 *
 */
void left_wheel_pivot()
{
	float Derror = qei_data.left_wheel.vel[cycle_data[speed_loop].now];
	int Perror = qei_data.left_wheel.set_point - qei_data.left_wheel.abs_pos;


	if(!(Perror<5000 && Perror<-5000))
	{
	PID_CONTROL_DATA._pid_position.Ierror += Perror;
	}

	//check if the value has passed setpoint then reduce the Ivalue
	if(qei_data.left_wheel.var_pos[cycle_data[speed_loop].now] <qei_data.left_wheel.set_point&& qei_data.left_wheel.var_pos[cycle_data[speed_loop].back1]>qei_data.left_wheel.set_point)
	{
		PID_CONTROL_DATA._pid_position.Ierror /=5;
	}
	//check if the value has passed setpoint then reduce the Ivalue
	if(qei_data.left_wheel.var_pos[cycle_data[speed_loop].now]>qei_data.left_wheel.set_point && qei_data.left_wheel.var_pos[cycle_data[speed_loop].back1]<qei_data.left_wheel.set_point)
	{
		PID_CONTROL_DATA._pid_position.Ierror /=5;
	}

	float KDerror = 0; //Derror * PID_CONTROL_DATA._pid_position.Dgain;
	float KPerror = .025 * Perror ; //Perror * PID_CONTROL_DATA._pid_position.Pgain;
	float KIerror = 0; //PID_CONTROL_DATA._pid_position.Ierror * PID_CONTROL_DATA._pid_position.Igain;

	qei_data.left_wheel.pwm_out = (int)(KDerror + KPerror + KIerror);

	if(qei_data.left_wheel.pwm_out < 0)
	{
		qei_data.left_wheel.pwm_out= fabs(qei_data.left_wheel.pwm_out);
		qei_data.left_wheel.wheel_dir = -1;
	}
	else
	{
		qei_data.left_wheel.wheel_dir = 1;
	}
	if(qei_data.left_wheel.pwm_out > 65535)
	{
		qei_data.left_wheel.pwm_out = 65535;
	}

	qei_data.right_wheel.pwm_out *= 2;
	if(qei_data.right_wheel.pwm_out<0)
	{
	qei_data.right_wheel.wheel_dir = -1;
	qei_data.right_wheel.pwm_out = fabs(qei_data.right_wheel.pwm_out);
	}
	else
	{
		qei_data.right_wheel.wheel_dir = 1;
	}
	if(qei_data.right_wheel.pwm_out>65535)
	{
		qei_data.right_wheel.pwm_out = 65535;

	}

	qei_data.right_wheel.dynamic_pos = 0;
	qei_data.left_wheel.dynamic_pos = 0;
}

/***************************************************************************************
 * right_wheel_pivot()
 *
 *THIS FUNCTION WILL TIE ONE OF THE WHEELS TO IT"S CURRENT LOCATION AND HAVE THE OTHER WHEEL
 *DRIVE IT SLIGTHLY FORWARD TURNING THE ROBOT ON A PIVOT POINT OF IT'S RIGHT WHEEL
 *THIS FUNCTION WILL BE CALLED WHEN YOU ROBOT DETERMINES IT NEEDS TO PIVOT TURN
 *
 *
 *
 */
void right_wheel_pivot()
{
		float Derror = qei_data.right_wheel.vel[cycle_data[speed_loop].now];
		int Perror = qei_data.right_wheel.set_point - qei_data.right_wheel.abs_pos;


		if(!(Perror<5000 && Perror<-5000))
		{
		PID_CONTROL_DATA._pid_position.Ierror += Perror;
		}

		//check if the value has passed setpoint then reduce the Ivalue
		if(qei_data.right_wheel.var_pos[cycle_data[speed_loop].now] <qei_data.right_wheel.set_point&& qei_data.right_wheel.var_pos[cycle_data[speed_loop].back1]>qei_data.right_wheel.set_point)
		{
			PID_CONTROL_DATA._pid_position.Ierror /=5;
		}
		//check if the value has passed setpoint then reduce the Ivalue
		if(qei_data.right_wheel.var_pos[cycle_data[speed_loop].now]>qei_data.right_wheel.set_point && qei_data.right_wheel.var_pos[cycle_data[speed_loop].back1]<qei_data.right_wheel.set_point)
		{
			PID_CONTROL_DATA._pid_position.Ierror /=5;
		}

		float KDerror = 0; //Derror * PID_CONTROL_DATA._pid_position.Dgain;
		float KPerror = .025*Perror ; //Perror * PID_CONTROL_DATA._pid_position.Pgain;
		float KIerror = 0; //PID_CONTROL_DATA._pid_position.Ierror * PID_CONTROL_DATA._pid_position.Igain;

		qei_data.right_wheel.pwm_out = (int)(KDerror + KPerror + KIerror);

		if(qei_data.right_wheel.pwm_out < 0)
		{
			qei_data.right_wheel.pwm_out= fabs(qei_data.right_wheel.pwm_out);
			qei_data.right_wheel.wheel_dir = -1;
		}
		else
		{
			qei_data.right_wheel.wheel_dir = 1;
		}

		if(qei_data.right_wheel.pwm_out > 65535)
		{
			qei_data.right_wheel.pwm_out = 65535;
		}
		qei_data.left_wheel.pwm_out *= 2;
		if(qei_data.left_wheel.pwm_out<0)
		{
		qei_data.left_wheel.wheel_dir = -1;
		qei_data.left_wheel.pwm_out = fabs(qei_data.left_wheel.pwm_out);
		}
		else
		{
			qei_data.left_wheel.wheel_dir = 1;
		}
		if(qei_data.left_wheel.pwm_out>65535)
		{
			qei_data.left_wheel.pwm_out = 65535;
		}
		qei_data.right_wheel.dynamic_pos = 0;
		qei_data.left_wheel.dynamic_pos = 0;
}


/************************************************************************************
 * i2c_check_data()
 *
 * function checks the data right after reading from I2C, if the data is deemed
 * not plausible the function returns 1 to tell the system to restart i2c transaction
 *
 *
 * Will write more in here that it will fault if multiple wrong data reads continuously
 *
 ************************************************************************************/
int i2c_check_data()
{
	int check = 0;

	float difference = imu_data[cycle_data[angle_loop].now].euler_values.p - imu_data[cycle_data[angle_loop].back1].euler_values.p;

	if(difference > 4.5) //check that the angle doesn't jump more than 5 degrees, shouldn't be plausible in timeframe of 10ms intervals
	{
		check = 1;
	}
	if(difference < -4.5)
	{
		check = 1;
	}

	//will add in gyro values to make prediction on what range it could be in.

	return check;
}

/*************************************************************************************
 * init_main_loop()
 *
 * This function initializes the main loop values to 0 so that we don't have initial
 * condition fail points or undefined behavior from the PID values
 *
 *
 *
 *
 */
char init_main_loop()
{
			angle_loop = 0;
			//IMU data init so we don't run into errors
				pid_imu_position[19].current_vel = 0;
				pid_imu_position[18].current_vel = 0;
				pid_imu_position[17].current_vel = 0;
				pid_imu_position[16].current_vel = 0;
				pid_imu_position[15].current_vel = 0;
				pid_imu_position[14].current_vel = 0;
				pid_imu_position[13].current_vel = 0;
				pid_imu_position[12].current_vel = 0;
				pid_imu_position[11].current_vel = 0;
				pid_imu_position[10].current_vel = 0;
				//IMU data init so we don't run into errors
					pid_imu_position[cycle_data[angle_loop].now].output = 0;
					pid_imu_position[cycle_data[angle_loop].back1].output =0;
					pid_imu_position[cycle_data[angle_loop].back2].output =0;
					pid_imu_position[cycle_data[angle_loop].back3].output =0;

					//IMU data init so we don't run into errors
					pid_imu_position[cycle_data[angle_loop].back1].current_pos=0;
					pid_imu_position[cycle_data[angle_loop].back2].current_pos =0;
					pid_imu_position[cycle_data[angle_loop].back3].current_pos =0;
					pid_imu_position[cycle_data[angle_loop].back4].current_pos =0;
					pid_imu_position[cycle_data[angle_loop].back5].current_pos =0;
					int i = 0;
				for(i  = 0 ; i<20;i++)
				{
					pid_imu_position[i].accel_cal = 0;
					pid_imu_position[i].gyro_cal = 0;
					pid_imu_position[i].mag_cal = 0;
				}
	return 0x1;
}


/*************************************************************************************
 * init_main_loop()
 *
 * THIS FUNCTION CREATES A LOOPING ARRAY SET OF VALUES SO THAT YOU CAN CALL THE CORRECT
 * PREVIOUS 20 VALUES BASED ON THE CURRENT LOOP CONDITION THROUGH A PLACEMENT ARRAY
 *
 * THIS LETS YOU CALL BACK ALL 20 SPOTS OF THE DATA ALLOWING YOU TO SEE EFFECTIVELY 200mS
 * IN THE PAST OF THE ROBOTS ACTIONS
 *
 * WE MAY IMPLEMENT POSITION PREDICTION IN THE FUTURE WHICH WE WILL USE THIS VALUES TO SLOPE THE NEXT
 * SPOT TO HELP INCREASE THE ROBOTS RESPONSIVENESS
 *
 *
 *
 */


void INIT_TIMECYCLE()
{
	cycle_data[0].now = 0;				//Initizalize reference numbers for 10 locations back 20 array loop cycle
	cycle_data[0].back1 = 19;
	cycle_data[0].back2 = 18;
	cycle_data[0].back3 = 17;
	cycle_data[0].back4 = 16;
	cycle_data[0].back5= 15;
	cycle_data[0].back6 = 14;
	cycle_data[0].back7 = 13;
	cycle_data[0].back8 = 12;
	cycle_data[0].back9 = 11;
	cycle_data[0].back10 = 10;

	cycle_data[1].now = 1;
	cycle_data[1].back1 = 0;
	cycle_data[1].back2 = 19;
	cycle_data[1].back3 = 18;
	cycle_data[1].back4 = 17;
	cycle_data[1].back5= 16;
	cycle_data[1].back6 = 15;
	cycle_data[1].back7 = 14;
	cycle_data[1].back8 = 13;
	cycle_data[1].back9 = 12;
	cycle_data[1].back10 = 11;

	cycle_data[2].now = 2;
	cycle_data[2].back1 = 1;
	cycle_data[2].back2 = 0;
	cycle_data[2].back3 = 19;
	cycle_data[2].back4 = 18;
	cycle_data[2].back5= 17;
	cycle_data[2].back6 = 16;
	cycle_data[2].back7 = 15;
	cycle_data[2].back8 = 14;
	cycle_data[2].back9 = 13;
	cycle_data[2].back10 = 12;

	cycle_data[3].now = 3;
	cycle_data[3].back1 = 2;
	cycle_data[3].back2 = 1;
	cycle_data[3].back3 = 0;
	cycle_data[3].back4 = 19;
	cycle_data[3].back5= 18;
	cycle_data[3].back6 = 17;
	cycle_data[3].back7 = 16;
	cycle_data[3].back8 = 15;
	cycle_data[3].back9 = 14;
	cycle_data[3].back10 = 13;

	cycle_data[4].now = 4;
	cycle_data[4].back1 = 3;
	cycle_data[4].back2 = 2;
	cycle_data[4].back3 = 1;
	cycle_data[4].back4 = 0;
	cycle_data[4].back5= 19;
	cycle_data[4].back6 = 18;
	cycle_data[4].back7 = 17;
	cycle_data[4].back8 = 16;
	cycle_data[4].back9 = 15;
	cycle_data[4].back10 = 14;

	cycle_data[5].now = 5;
	cycle_data[5].back1 = 4;
	cycle_data[5].back2 = 3;
	cycle_data[5].back3 = 2;
	cycle_data[5].back4 = 1;
	cycle_data[5].back5= 0;
	cycle_data[5].back6 = 19;
	cycle_data[5].back7 = 18;
	cycle_data[5].back8 = 17;
	cycle_data[5].back9 = 16;
	cycle_data[5].back10 = 15;

	cycle_data[6].now = 6;
	cycle_data[6].back1 = 5;
	cycle_data[6].back2 = 4;
	cycle_data[6].back3 = 3;
	cycle_data[6].back4 = 2;
	cycle_data[6].back5= 1;
	cycle_data[6].back6 = 0;
	cycle_data[6].back7 = 19;
	cycle_data[6].back8 = 18;
	cycle_data[6].back9 = 17;
	cycle_data[6].back10 = 16;

	cycle_data[7].now = 7;
	cycle_data[7].back1 = 6;
	cycle_data[7].back2 = 5;
	cycle_data[7].back3 = 4;
	cycle_data[7].back4 = 3;
	cycle_data[7].back5= 2;
	cycle_data[7].back6 = 1;
	cycle_data[7].back7 = 0;
	cycle_data[7].back8 = 19;
	cycle_data[7].back9 = 18;
	cycle_data[7].back10 = 17;

	cycle_data[8].now = 8;
	cycle_data[8].back1 = 7;
	cycle_data[8].back2 = 6;
	cycle_data[8].back3 = 5;
	cycle_data[8].back4 = 4;
	cycle_data[8].back5= 3;
	cycle_data[8].back6 = 2;
	cycle_data[8].back7 = 1;
	cycle_data[8].back8 = 0;
	cycle_data[8].back9 = 19;
	cycle_data[8].back10 = 18;

	cycle_data[9].now = 9;
	cycle_data[9].back1 = 8;
	cycle_data[9].back2 = 7;
	cycle_data[9].back3 = 6;
	cycle_data[9].back4 = 5;
	cycle_data[9].back5 = 4;
	cycle_data[9].back6 = 3;
	cycle_data[9].back7 = 2;
	cycle_data[9].back8 = 1;
	cycle_data[9].back9 = 0;
	cycle_data[9].back10 = 19;


	int i;
	for( i = 10;i<20;i++)
	{
	cycle_data[i].now = i;
	cycle_data[i].back1 = i-1;
	cycle_data[i].back2 = i-2;
	cycle_data[i].back3 = i-3;
	cycle_data[i].back4 = i-4;
	cycle_data[i].back5= i-5;
	cycle_data[i].back6 = i-6;
	cycle_data[i].back7 = i-7;
	cycle_data[i].back8 = i-8;
	cycle_data[i].back9 = i-9;
	cycle_data[i].back10 = i-10;
	}
}
