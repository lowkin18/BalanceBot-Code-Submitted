/*
 * sensorhub_com.c
 *
 *  Created on: Apr 28, 2017
 *      Author: Kip
 */

#include "sensorhub_com.h"

 UART_Handle uart;
 UART_Params uartParams;



void init_sensor_com(void)
{
    EK_TM4C123GXL_initUART();
    GPIOPinTypeGPIOOutput(GPIO_PORTE_BASE, GPIO_PIN_4);	//ACTIVE HIGH UART TRANSMISSION READY
    //GPIOPinTypeGPIOOutput(GPIO_PORTD_BASE, GPIO_PIN_2);
    GPIOPinWrite(GPIO_PORTE_BASE, GPIO_PIN_4, 0);


        /* Create a UART with data processing off. */
        UART_Params_init(&uartParams);
        uartParams.writeDataMode = UART_DATA_BINARY;
        uartParams.readDataMode = UART_DATA_BINARY;
        uartParams.readReturnMode = UART_RETURN_FULL;
        uartParams.readEcho = UART_ECHO_OFF;
        uartParams.baudRate = 3686400;
        uart = UART_open(Board_UART0, &uartParams);

        if (uart == NULL) {
            System_abort("Error opening the UART");
        }



}


void sensor_hub_read(void)
{
	 GPIOPinWrite(GPIO_PORTE_BASE, GPIO_PIN_4, GPIO_PIN_4);
	 	 UART_read(uart, sensor_hub_bytes, 7);
	 	 sensor_hub.angle = sensor_hub_bytes[0];
		 sensor_hub.turn = sensor_hub_bytes[1];
		 sensor_hub.flags = sensor_hub_bytes [2];
		 sensor_hub.backR_dist = sensor_hub_bytes[3];
		 sensor_hub.backL_dist = sensor_hub_bytes[4];
		 sensor_hub.frontL_dist = sensor_hub_bytes[5];
		 sensor_hub.frontR_dist = sensor_hub_bytes[6];
		 char distance_flag_front = 0;
		 char distance_flag_rear = 0;
		 /*
		 if(((sensor_hub.frontR_dist < 20 || sensor_hub.frontL_dist < 20) && (sensor_hub.frontR_dist > 6 || sensor_hub.frontL_dist > 6)) && sensor_hub.cRemote_state != 0x03)
		 {
			 distance_flag_front = 1;
		 }
		 if(((sensor_hub.backR_dist < 20 || sensor_hub.backL_dist < 20) && (sensor_hub.backR_dist > 6 || sensor_hub.backL_dist > 6))&&sensor_hub.cRemote_state != 0x03)
		 {
			 distance_flag_rear = 1;
		 }
		 */
		 if((sensor_hub.flags  == 0x01 && sensor_hub.cRemote_state != 0x01) || distance_flag_front == 1 || distance_flag_rear == 1)
		 {
			 ctrl_robot.control_state = 0;
			 ctrl_robot.autonomous_mode = 0;
			 ctrl_robot.turn_state = 0;
			 ctrl_robot.x_set_point = ctrl_robot.x_abs_pos;
			 ctrl_robot.final_orientation = ctrl_robot.current_orientation;
			 ctrl_robot.wheel_L_fpos = ctrl_robot.wheel_L_cpos[cycle_data[speed_loop].back1];
			 ctrl_robot.wheel_R_fpos = ctrl_robot.wheel_R_cpos[cycle_data[speed_loop].back1];
			 qei_data.left_wheel.dynamic_pos = 0;
			 qei_data.right_wheel.dynamic_pos = 0;
			 sensor_hub.cRemote_state = 0x01;
		 }
		 if(sensor_hub.flags  == 0x03 && sensor_hub.cRemote_state != 0x03)
			 {

				 ctrl_robot.autonomous_mode = 4;
				 ctrl_robot.control_state = 3;
				 ctrl_robot.x_set_point = ctrl_robot.x_abs_pos;
				 ctrl_robot.final_orientation = ctrl_robot.current_orientation;
				 ctrl_robot.wheel_L_fpos = ctrl_robot.wheel_L_cpos[cycle_data[speed_loop].back1];
				 ctrl_robot.wheel_R_fpos = ctrl_robot.wheel_R_cpos[cycle_data[speed_loop].back1];
				 qei_data.left_wheel.dynamic_pos = 0;
				 qei_data.right_wheel.dynamic_pos = 0;
				 sensor_hub.cRemote_state = 0x03;
			 }
		 if(sensor_hub.flags == 0x05 &&  sensor_hub.cRemote_state != 0x05)
		 {
			 ctrl_robot.control_state = 0;
			 ctrl_robot.autonomous_mode = 0;
			 ctrl_robot.turn_state = 0;
			 ctrl_robot.x_set_point = ctrl_robot.x_abs_pos;
			 ctrl_robot.final_orientation = ctrl_robot.current_orientation;
			 ctrl_robot.wheel_L_fpos = ctrl_robot.wheel_L_cpos[cycle_data[speed_loop].back1];
			 ctrl_robot.wheel_R_fpos = ctrl_robot.wheel_R_cpos[cycle_data[speed_loop].back1];
			 qei_data.left_wheel.dynamic_pos = 0;
			 qei_data.right_wheel.dynamic_pos = 0;
			 sensor_hub.cRemote_state = 0x05;
		 }
		 GPIOPinWrite(GPIO_PORTE_BASE, GPIO_PIN_4, 0);
}

