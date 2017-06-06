/*
 * camera_control.c
 *
 *  Created on: Mar 16, 2017
 *      Author: c_ker
 */

#include "camera_control.h"








//Function to init the I2C communication to the PWM driver
void init_i2c_camera(I2C_Handle i2c)
{
	i2c_com = i2c;
    uint8_t         txBuffer[20];
    /* Create I2C for usage */

    	txBuffer[0] = 0x00;
    	txBuffer[1] = 0x10;
    	i2cTransaction_cam.slaveAddress = 0x40;
    	i2cTransaction_cam.writeBuf = txBuffer;
    	i2cTransaction_cam.writeCount = 2;
    	i2cTransaction_cam.readCount = 0;
    	if (I2C_transfer(i2c_com, &i2cTransaction_cam)) {
    	 }
    	Task_sleep(1);
    	txBuffer[0] = 0xFE;
    	txBuffer[1] = PWM_FREQ;
    	i2cTransaction_cam.slaveAddress = 0x40;
    	i2cTransaction_cam.writeBuf = txBuffer;
    	i2cTransaction_cam.writeCount = 2;
    	if (I2C_transfer(i2c_com, &i2cTransaction_cam)) {

    	 }
    	Task_sleep(10);
    	txBuffer[0] = 0x00;
    	txBuffer[1] = 0xA2;
    	i2cTransaction_cam.slaveAddress = 0x40;
    	i2cTransaction_cam.writeBuf = txBuffer;
    	i2cTransaction_cam.writeCount = 2;
    	if (I2C_transfer(i2c_com, &i2cTransaction_cam)) {

    	  }
    		Task_sleep(1);

    	txBuffer[0] = 0x01;
    	txBuffer[1] = 0x04;
    	i2cTransaction_cam.slaveAddress = 0x40;
    	i2cTransaction_cam.writeBuf = txBuffer;
    	i2cTransaction_cam.writeCount = 2;
    	if (I2C_transfer(i2c_com, &i2cTransaction_cam)) {
    		}
    	Task_sleep(10);
    	txBuffer[0] = LED0_ON; //LED1
    	txBuffer[1] = 0x00;
    	txBuffer[2] = 0x00;

    	i2cTransaction_cam.slaveAddress = 0x40;
    	i2cTransaction_cam.writeBuf = txBuffer;
    	i2cTransaction_cam.writeCount = 3;
    	if (I2C_transfer(i2c_com, &i2cTransaction_cam)) {
    		}
    	Task_sleep(1);
    	txBuffer[0] = LED0_ON+4;	//LED 1
    	txBuffer[1] = 0x00;
    	txBuffer[2] = 0x00;

    	i2cTransaction_cam.slaveAddress = 0x40;
    	i2cTransaction_cam.writeBuf = txBuffer;
    	i2cTransaction_cam.writeCount = 3;
    	if (I2C_transfer(i2c_com, &i2cTransaction_cam)) {
    		}
    	Task_sleep(1);
    	txBuffer[0] = LED0_ON+8;	//LED2
    	txBuffer[1] = 0x00;
    	txBuffer[2] = 0x00;

    	i2cTransaction_cam.slaveAddress = 0x40;
    	i2cTransaction_cam.writeBuf = txBuffer;
    	i2cTransaction_cam.writeCount = 3;
    	if (I2C_transfer(i2c_com, &i2cTransaction_cam)) {
    		}
    	Task_sleep(1);
    	level_mounts(0,0);
}


void level_mounts(float angle,float turn_angle)
{
	servo_d.camera_tilt_duty = camera_tilt_calc(angle);
	servo_d.camera_turn_duty = camera_turn_calc(turn_angle);
	servo_d.level_tilt_duty = level_tilt_calc(-angle);

	PWM_Servo_write(0,servo_d.camera_tilt_duty);
	PWM_Servo_write(1,servo_d.camera_turn_duty);
	PWM_Servo_write(2,servo_d.level_tilt_duty);
}


short camera_tilt_calc(float angle)
{
	//WRITE SCA EQUATION FOR THE PWM OUTPUTS
	short PWM_value = (short)(angle*TILT_CAM_SCA + 1350);
	if(PWM_value > 2050)
	{
		PWM_value = 2050;
	}
	if(PWM_value < 575)
	{
		PWM_value = 575;
	}
	return PWM_value;
}

short camera_turn_calc(float angle)
{
	short PWM_value = (short)(angle*TURN_CAM_SCA + 1425);
	return PWM_value;
}

short level_tilt_calc(float angle)
{
	short PWM_value = (short)(angle*TILT_SCA + 1510);
	return PWM_value;
}

void PWM_Servo_write(char led_num, short PWM_Val)
{
	char position_num = 0x00;
	char *ptr_temp = (char*)&PWM_Val;
	cam_string.txBuffer[position_num]  = LED0_OFF + 0x04*led_num;
	position_num++;
	cam_string.txBuffer[position_num] = *ptr_temp;
	ptr_temp++;
	position_num++;
	cam_string.txBuffer[position_num]  = *ptr_temp;
	position_num++;
	i2c_write_pwm(cam_string.txBuffer,position_num);
}

char i2c_write_pwm(char *values,char length) {

	i2cTransaction_cam.slaveAddress = 0X40;
	i2cTransaction_cam.readCount = 0;
	i2cTransaction_cam.writeBuf = values;//setting up i2c transaction params
	i2cTransaction_cam.writeCount = length;	//writecount added to params

	if (I2C_transfer(i2c_com, &i2cTransaction_cam))	//perform the I2C transaction TIVA API Call
			{
		return 0;								//0 = SUCCESS
	}
	return 1;								//1 = FAILURE
}



