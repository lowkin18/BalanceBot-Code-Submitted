/*
 * Copyright (c) 2015, Texas Instruments Incorporated
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * *  Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 *
 * *  Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 *
 * *  Neither the name of Texas Instruments Incorporated nor the names of
 *    its contributors may be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
 * PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR
 * CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
 * EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
 * PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS;
 * OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
 * WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR
 * OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
 * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

/*
 *    ======== i2ctmp006.c ========
 */

#include "project_types.h"
#include "balance_control.h"
#include "button_control.h"
#include "lcd_gui.h"

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




int lcd_refresh;

PWM_Handle pwm1;
PWM_Handle pwm2;

I2C_Handle i2c;
I2C_Handle i2c1;
I2C_Transaction i2cTransaction;


/*
 *  ======== taskFxn ========
 *  Task for this function is created statically. See the project's .cfg file.
 */



Void task_i2c_fxn(UArg arg0, UArg arg1)
{
	I2C_Params      i2cParams;
	/* Create I2C for usage */
	I2C_Params_init(&i2cParams);
	i2cParams.bitRate = I2C_400kHz;				//CLOCK SPEED FAST
	i2cTransaction.slaveAddress = 0X28;
	i2c = I2C_open(1, &i2cParams);	//initialize the params
	if (i2c == NULL) {

	    }
	int check = 0;

	i2c_init_bno055_tm4c123(&i2c);
	lcd_refresh = 0;

	for(;;)
	{
		if(menu_state.main_state != 10)
		{
		PWM_setDuty(pwm1, 0);									//LEFT WHEEL
		PWM_setDuty(pwm2, 0);									//RIGHT WHEEL
		}
		read_control();
		check = i2c_check_data(); //FUNCTION TO CHECK THE RETURNING DATA
		//IF THE DATA IS WRONG RETURNS 1 and WILL RE-READ THE DATA
		if(check)
		{
		bno055_convert_float_euler_hpr_deg(&imu_data[angle_loop].euler_values);
		bno055_convert_float_gyro_x_dps(&imu_data[angle_loop].x_gyro);
		}
		//LCD_DATA LOOP POST WILL ENABLE WHEN WIRED UP

		//THIS IS POSTS THE PWM SEMAPHORE THAT DOES THE CONTROL LOGIC
		if(menu_state.main_state == 10)
		{
		//CONTROL LOGIC SEMAPHORE - ONLY RUNS WHEN ROBOT IN OPERATION STATE
		Semaphore_post(Semaphore_i2c_data);
		}
		if(menu_state.main_state != 10)
		{
		//SEMAPHORE POST FOR LCD GUI REFRESH at 5HZ
			if(lcd_refresh%10 == 1)
			{
			Semaphore_post(Semaphore_gui);
			lcd_refresh = 0;
			}
		}

		//IF ROBOT IN RUNNING MODE REFRESH SCREEN AT .25HZ
		if(menu_state.main_state == 10)
		{
			if(lcd_refresh > 100)
			{
			Semaphore_post(Semaphore_gui);//SEMAPHORE POST FOR LCD GUI
			lcd_refresh = 0;
			}
		}

		lcd_refresh++;
	Task_sleep(10); //SLEEP FOR 9 miliseconds cause you can only read at 100hz
	}
}


Void task_pwm_fxn(UArg arg0, UArg arg1)
{

	    PWM_Params params;
	    uint32_t   pwmPeriod = 80;      // Period and duty in microseconds
	    uint32_t   duty = 0;
	    PWM_Params_init(&params);
	    params.period = pwmPeriod;
	    params.dutyMode = PWM_DUTY_SCALAR;
	    pwm1 = PWM_open(Board_PWM0, &params);
	    if (pwm1 == NULL) {
	        System_abort("Board_PWM0 did not open");
	    }
	    if (Board_PWM1 != Board_PWM0) {
	        pwm2 = PWM_open(Board_PWM1, &params);
	        if (pwm2 == NULL) {
	            System_abort("Board_PWM1 did not open");
	        }
	    }
	    //FOREVER TASK LOOP
	    for(;;)
		{
	    //WILL WAIT ON THIS SEMAPHORE TO RUN THE CONTROL LOGIC
		Semaphore_pend(Semaphore_i2c_data, BIOS_WAIT_FOREVER);

		//THIS IS THE CONTROL ALGORITHMS OF THE ROBOT - RUNS AT 100HZ
		balance_control_algo();	  //PID LOOP DATA;

		/*
		if(qei_data.left_wheel.pwm_out > 5000)
		{
		qei_data.left_wheel.wheel_dir == 1 ? GPIOPinWrite(GPIO_PORTE_BASE, GPIO_PIN_0, GPIO_PIN_0): GPIOPinWrite(GPIO_PORTE_BASE, GPIO_PIN_0, 0);
		}
		if(qei_data.right_wheel.pwm_out > 5000)
		{
		qei_data.right_wheel.wheel_dir == 1 ? GPIOPinWrite(GPIO_PORTD_BASE, GPIO_PIN_2, GPIO_PIN_2):GPIOPinWrite(GPIO_PORTD_BASE, GPIO_PIN_2, 0);
		}
		*/
		if(qei_data.left_wheel.pwm_out < 0 &&  qei_data.right_wheel.pwm_out < 0)
		{
			qei_data.left_wheel.pwm_out = 0;
			qei_data.right_wheel.pwm_out = 0;
		}
		else if (qei_data.left_wheel.pwm_out >65535 &&  qei_data.right_wheel.pwm_out >65535)
		{
			qei_data.left_wheel.pwm_out = 65535;
			qei_data.right_wheel.pwm_out = 65535;
		}
		else
		{
		qei_data.left_wheel.wheel_dir == 1 ? GPIOPinWrite(GPIO_PORTE_BASE, GPIO_PIN_0, 0): GPIOPinWrite(GPIO_PORTE_BASE, GPIO_PIN_0, GPIO_PIN_0);
		qei_data.right_wheel.wheel_dir == 1 ? GPIOPinWrite(GPIO_PORTA_BASE, GPIO_PIN_5, 0):GPIOPinWrite(GPIO_PORTA_BASE, GPIO_PIN_5,  GPIO_PIN_5);
		//SET THE PWM VALUES FOR THE MOTORS

		//DEBUG THE WEIRD OSCILLATION
		//debug_swap_left[debug_count] = qei_data.left_wheel.wheel_dir;
		//debug_swap_right[debug_count] = qei_data.right_wheel.wheel_dir;
		//debug_pwm_left[debug_count] = qei_data.left_wheel.pwm_out;
		//debug_pwm_right[debug_count]= qei_data.right_wheel.pwm_out;

		PWM_setDuty(pwm1, qei_data.left_wheel.pwm_out);										//LEFT WHEEL
		PWM_setDuty(pwm2, qei_data.right_wheel.pwm_out);									//RIGHT WHEEL
		}
		Semaphore_post(semaphore_pwm_level);
		}
}


Void task_pwm_balance(UArg arg0, UArg arg1)
{
	Semaphore_pend(semaphore_pwm_level,BIOS_WAIT_FOREVER);

	I2C_Params      i2cParams;
	I2C_Params_init(&i2cParams);
	i2cParams.bitRate = I2C_400kHz;				//CLOCK SPEED FAST
	I2C_Handle i2cPwm =I2C_open(2,&i2cParams);
			if (i2c1 == NULL)
			{

			}
			init_i2c_camera(i2cPwm);
	for(;;)
	{
		Semaphore_pend(semaphore_pwm_level,BIOS_WAIT_FOREVER);
		level_mounts(imu_data[angle_loop].euler_values.p-1,0);
	}
}

Void task_gui_fxn(UArg arg0, UArg arg1)
{
		//WAIT FOR SEMAPHORE POST TO WRITE TO THE LCD GUI
		Semaphore_pend(Semaphore_gui,BIOS_WAIT_FOREVER);

		//THIS SET'S UP THE I2C communication for LCD On different pins
		I2C_Params      i2cParams;
		I2C_Params_init(&i2cParams);
		i2cParams.bitRate = I2C_400kHz;				//CLOCK SPEED FAST
		i2cTransaction.slaveAddress = 0X27;
		i2c1 = I2C_open(0, &i2cParams);	//initialize the params
		if (i2c1 == NULL) {
			   }
		init_LCD_i2c(i2c1);	//INIT THE I2C LCD SETUP
		start_up_sequence(); //SENS THE START UP SEQUENCE FOR 4 BIT MODE
		for(;;)
		{
		Semaphore_pend(Semaphore_gui,BIOS_WAIT_FOREVER); //WAIT FOREVER FOR SEMAPHORE POST
		//robot_abs_position();
		menu_print_control();							//MENU PRINT BASED ON MENU STATE
		Task_sleep(100);								//TASK _SLEEP FOR 100mS
		}
}

Void task_predictions_fxn(UArg arg0, UArg arg1)
{
	float orientation_left;
	ctrl_robot.autonomous_mode = 0;
	ctrl_robot.timer_tick = 0;
	for(;;)
	{
		if(menu_state.main_state == 10)
		{
			switch(ctrl_robot.autonomous_mode)
			{
			case 0:


			break;
			case 1:
			if(ctrl_robot.x_abs_pos<0.15 && ctrl_robot.x_set_point == 0)
			{
				ctrl_robot.timer_tick++;
				if(ctrl_robot.timer_tick == 50)
				{
				ctrl_robot.x_set_point = auto_setpoints.straight_setpoint;
				ctrl_robot.timer_tick = 0;
				}
			}
			if(ctrl_robot.x_abs_pos >auto_setpoints.straight_setpoint-0.15 && ctrl_robot.x_set_point==auto_setpoints.straight_setpoint)
			{
				ctrl_robot.timer_tick++;
				if(ctrl_robot.timer_tick == 50)
				{
				ctrl_robot.x_set_point = 0;
				ctrl_robot.timer_tick = 0;
			    }
			}
			forward_and_back();
			break;
			case 2:
			if(ctrl_robot.autonomous_state >= 3)
			{
				orientation_left = ctrl_robot.final_orientation-ctrl_robot.current_orientation;
				if(!(orientation_left > 0.05 || orientation_left <-0.05))
				{
					ctrl_robot.timer_tick=0;
					ctrl_robot.flag_square = 0;
				}
			}

			if(ctrl_robot.flag_square == 0)
			{

				next_position_call(ctrl_robot.counter);
				ctrl_robot.counter++;
				if(ctrl_robot.counter == 4)
				{
				ctrl_robot.counter =0;
				}
			}
			autonomous_movement_control();
			ctrl_robot.timer_tick++; 			//timer tick is 100mS
			ctrl_robot.flag_square = 1;
			break;

			case 3:
			figure_8_movement_wheels();
			break;
			case 4:
				//AUTONOMOUS ROAM MODE, NEED TO WRITE THIS
			break;
			}
		}
		Task_sleep(100);
	}
}

Void UART_com(UArg arg0, UArg arg1)
{
	Task_sleep(10000);
	for(;;)
	{
		if(menu_state.main_state == 10)
		{
		//sensor_hub_read();
		}
	/*
	 *
	 * READ THE UART SINGLES THEN DECIDE TO GO SAFE MODE OR NOT
	 *
	 */
	Task_sleep(50);
	}
}


/*
 *  ======== main ========
 */
int main(void)
{
	//TI_RTOS BOARD INIT PARAMS
    Board_initGeneral();
    Board_initGPIO();
    Board_initI2C();
    Board_initPWM();

    ////////////////////////////////////////////////////////////



    INIT_TIMECYCLE();		//init time cycle foor looping arrays
    init_QEI(); 			//init QEI hardware for quadrature
    init_balance_control(); //init balance starting points set values two 0
    init_pid_values();
    init_sensor_com();
    //TIMER_Initialize();
    init_buttons();			//init buttons for the LED gui;
    BIOS_start();			//start the BIOS for the TI_RTOS - TASKS START HERE
    return (0);
}







