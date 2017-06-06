/*
 * lcd_gui.c
 *
 *  Created on: Feb 7, 2017
 *      Author: c_ker
 */

#include "lcd_gui.h"
#include "project_types.h"

/*************************************************/
/**\START_UP_CONFIG*/
/*
 INITIALIZE THE LCD FOR GUI OPERATION WITHIN THE MODULE
 SETS UP THE I2C transaction variable for it's slave address


 */
/*************************************************/

/*************************************************************************************
 * init_LCD_i2c()
 *
 * This function does nothing other than pass the I2C handle value to this function so it
 * can use the I2C bus when needed
 *
 *
 */
char init_LCD_i2c(I2C_Handle value) {
	i2c_lcd = value;
	return 0;
}

/*************************************************************************************
 * PID_print()
 *
 * This function will print the current PID Values for the SPEED OR ANGLE
 *
 * params are the proportional gain, integral gain, derivative gain
 *
 */
void PID_print(float proportional, float integral, float derivative) {

	lcd_strings pid_string;
	if (menu_state.middle_state == 1) {
		pid_string.string = "PID S Values";
	} else if (menu_state.middle_state == 2) {
		pid_string.string = "PID A Values";
	} else {
		pid_string.string = "PID P Values";
	}
	lcd_clear_screen();
	pid_string.length = 12;
	lcd_title_write(&pid_string, 1);
	pid_string.string = "P Val";
	pid_string.length = 5;
	lcd_write(&pid_string, proportional, 2);
	pid_string.string = "I Val";
	pid_string.length = 5;
	lcd_write(&pid_string, integral, 3);
	pid_string.string = "D Val";
	pid_string.length = 5;
	lcd_write(&pid_string, derivative, 4);
}

/*************************************************************************************
 * euler_angle_print()
 *
 * This function will print the current euler angles to the LED Screen
 *
 * params are the pitch value, roll value and the heading value
 *
 */
void euler_angle_print(float pitch, float roll, float heading) {
	lcd_clear_screen();
	lcd_strings euler_string;
	euler_string.string = "Euler Angles";
	euler_string.length = 12;
	lcd_title_write(&euler_string, 1);
	euler_string.string = "PITCH";
	euler_string.length = 5;
	lcd_write(&euler_string, pitch, 2);
	euler_string.string = "ROLL";
	euler_string.length = 4;
	lcd_write(&euler_string, roll, 3);
	euler_string.string = "HEADING";
	euler_string.length = 7;
	lcd_write(&euler_string, heading, 4);
}

/*************************************************************************************
 * void pid_setpoints(float positon, float speed, float angle)
 *
 * This function shows the setpoints from the PID loops
 *
 * params are the positon setpoint the speed setpoint and angle setpoint
 *
 */
void pid_setpoints(float position, float speed, float angle) {
	lcd_clear_screen();
	lcd_strings setpoints;
	setpoints.string = "PID Setpoints";
	setpoints.length = 13;
	lcd_title_write(&setpoints, 1);
	setpoints.string = "POS";
	setpoints.length = 3;
	lcd_write(&setpoints,position, 2);
	setpoints.string = "SPEED";
	setpoints.length = 5;
	lcd_write(&setpoints,speed, 3);
	setpoints.string = "ANGLE";
	setpoints.length = 5;
	lcd_write(&setpoints,angle, 4);
}

/*************************************************************************************
 * gravity_print
 *
 * This function will print the current gravity values to the screen
 *
 * params are the X axis the Y axis and the Z axis
 *
 */
void gravity_print(float x, float y, float z) {
	lcd_clear_screen();
	lcd_strings gravity_string;
	lcd_title_write(&gravity_string, 1);
	gravity_string.string = "X Axis";
	gravity_string.length = 6;
	lcd_write(&gravity_string, x, 2);
	gravity_string.string = "Y Axis";
	gravity_string.length = 6;
	lcd_write(&gravity_string, y, 3);
	gravity_string.string = "Z Axis";
	gravity_string.length = 6;
	lcd_write(&gravity_string, z, 4);
}

/*************************************************************************************
 * void Ierror_print(float angles, float speed, float position)
 *
 * This function will print the Ierror Values to see what they are
 *
 * params are the Ierror Angles - Ierror Speed - Ierror Position
 *
 */
void Ierror_print(float angles, float speed, float position) {
	lcd_clear_screen();
	lcd_strings Ierror;
	Ierror.string = "Ierror Values";
	Ierror.length = 13;
	lcd_title_write(&Ierror, 1);
	Ierror.string = "*AngI";
	Ierror.length = 5;
	lcd_write(&Ierror, angles, 2);
	Ierror.string = "*SpeedI";
	Ierror.length = 7;
	lcd_write(&Ierror, speed, 3);
	Ierror.string = "*PosI";
	Ierror.length = 5;
	lcd_write(&Ierror, position/1000, 4);
}


/*************************************************************************************
 * Gryo_print()
 *
 * This function will print the current gyro values to the LED Screen
 *
 * params are the X axis the Y axis the Z axis
 *
 */
void gyro_print(float x, float y, float z) {
	lcd_clear_screen();
	lcd_strings gyro_string;
	gyro_string.string = "GYRO VALUES";
	gyro_string.length = 11;
	lcd_title_write(&gyro_string, 1);
	gyro_string.string = "X Axis";
	gyro_string.length = 6;
	lcd_write(&gyro_string, x, 2);
	gyro_string.string = "Y Axis";
	gyro_string.length = 6;
	lcd_write(&gyro_string, y, 3);
	gyro_string.string = "Z Axis";
	gyro_string.length = 6;
	lcd_write(&gyro_string, z, 4);
}
/*************************************************************************************
 * wheel_print()
 *
 * this function will print the wheels positions so you can llook at it
 *
 * Params Void
 */

void wheel_print(int l_wheel, int r_wheel) {

	float left_wheel = (float) l_wheel;
	float right_wheel = (float) r_wheel;

	left_wheel = (left_wheel / 1000);
	right_wheel = (right_wheel / 1000);

	lcd_clear_screen();
	lcd_strings wheel_string;
	wheel_string.string = "WHEEL VALUES-(m)";
	wheel_string.length = 16;
	lcd_title_write(&wheel_string, 1);
	wheel_string.string = "Left";
	wheel_string.length = 4;
	lcd_write(&wheel_string, left_wheel, 2);
	wheel_string.string = "Right";
	wheel_string.length = 5;
	lcd_write(&wheel_string, right_wheel, 3);
	wheel_string.string = "  *************** ";
	wheel_string.length = 18;
	lcd_title_write(&wheel_string, 4);
}

/*************************************************************************************
 * angle_menu_print()
 *
 * this function will print the angle values you cna look at menu- subMenu of the main
 *
 * Params Void
 */
angle_menu_print() {
	lcd_clear_screen();
	lcd_strings main_menu;
	main_menu.string = "IMU VALUES";
	main_menu.length = 10;
	lcd_title_write(&main_menu, 1);

	main_menu.string = "** EULER ANGLES **";
	main_menu.length = 18;
	lcd_title_write(&main_menu, 2);

	main_menu.string = "** GYRO VALUES  **";
	main_menu.length = 18;
	lcd_title_write(&main_menu, 3);

	main_menu.string = "**WHEEL POSITION**";
	main_menu.length = 17;
	lcd_title_write(&main_menu, 4);
}

/*************************************************************************************
 * robot_running_menu()
 *
 * This function will print the robot-running screen to show the robot is in automation mode
 *
 * params Void
 *
 */
void robot_running_menu() {
	lcd_clear_screen();
	lcd_strings main_menu;
	main_menu.string = "**ROBOT RUNNING**";
	main_menu.length = 17;
	lcd_title_write(&main_menu, 1);
	main_menu.string = "**X-POS";
	main_menu.length = 7;
	lcd_write(&main_menu, ctrl_robot.x_abs_pos, 2);
	main_menu.string = "**Y-POS";
	main_menu.length = 7;
	lcd_write(&main_menu, ctrl_robot.y_abs_pos, 3);
	main_menu.string = "**ANG";
	main_menu.length = 5;
	lcd_write(&main_menu, ctrl_robot.current_orientation, 4);
}

/*************************************************************************************
 * robot_running_menu()
 *
 * This function will print the robot-running screen to show the robot is in automation mode
 *
 * params Void
 *
 */
void move_straight_menu() {
	lcd_clear_screen();
	lcd_strings main_menu;
	main_menu.string = "**MOVE STRAIGHT**";
	main_menu.length = 17;
	lcd_title_write(&main_menu, 1);
	main_menu.string = "*Distance";
	main_menu.length = 9;
	lcd_write(&main_menu, auto_setpoints.straight_setpoint, 2);
	main_menu.string = "*Speed";
	main_menu.length = 6;
	lcd_title_write(&main_menu,3);
	main_menu.string = "**";
	main_menu.length = 2;
	lcd_title_write(&main_menu, 4);
}

/*************************************************************************************
 * robot_running_menu()
 *
 * This function will print the robot-running screen to show the robot is in automation mode
 *
 * params Void
 *
 */
void square_menu() {
	lcd_clear_screen();
	lcd_strings main_menu;
	main_menu.string = "**SQUARE MOVE**";
	main_menu.length = 15;
	lcd_title_write(&main_menu, 1);
	main_menu.string = "**X-POS";
	main_menu.length = 7;
	lcd_write(&main_menu, auto_setpoints.X_square, 2);
	main_menu.string = "**Y-POS";
	main_menu.length = 7;
	lcd_write(&main_menu, auto_setpoints.Y_square, 3);
	main_menu.string = "**";
	main_menu.length = 2;
	lcd_title_write(&main_menu, 4);
}


/*************************************************************************************
 * robot_running_menu()
 *
 * This function will print the robot-running screen to show the robot is in automation mode
 *
 * params Void
 *
 */
void figure_8() {
	lcd_clear_screen();
	lcd_strings main_menu;
	main_menu.string = "**FIGURE EIGHT**";
	main_menu.length = 17;
	lcd_title_write(&main_menu, 1);
	main_menu.string = "**";
	main_menu.length = 7;
	lcd_title_write(&main_menu, 2);
	main_menu.string = "**";
	main_menu.length = 7;
	lcd_title_write(&main_menu, 3);
	main_menu.string = "**";
	main_menu.length = 5;
	lcd_title_write(&main_menu, 4);
}


/*************************************************/
/*robot_abs_position()
 * RETURNS NOTHING
 * ARGUMENTS NON
 * Prints the absolute position of the robot in euclidian space
 * with reference to the starting position and orientation.
 */
/*************************************************/
void robot_wheel_position() {
	lcd_clear_screen();
	lcd_strings abs_pos;
	abs_pos.string = "Absolute Position";
	abs_pos.length = 17;
	lcd_title_write(&abs_pos, 1);
	abs_pos.string = "Xpos(m)";
	abs_pos.length = 7;
	lcd_write(&abs_pos, ctrl_robot.x_abs_pos, 2);

	abs_pos.string = "Ypos(m)";
	abs_pos.length = 7;
	lcd_write(&abs_pos, ctrl_robot.y_abs_pos, 3);

	abs_pos.string = "Theta(r)";
	abs_pos.length = 8;
	lcd_write(&abs_pos, ctrl_robot.current_orientation, 4);
}

/*************************************************/
/*calib_menu()
 * RETURNS NOTHING
 * ARGUMENTS NON
 PRINTS THE MAIN MENU for the LCD screen on startup
 */
/*************************************************/
void calib_menu() {
	/*
	 if(!(pid_imu_position[angle_loop].accel_cal == pid_imu_position[cycle_data[angle_loop].back10].accel_cal)||
	 !(pid_imu_position[angle_loop].gyro_cal == pid_imu_position[cycle_data[angle_loop].back10].gyro_cal)||
	 !(pid_imu_position[angle_loop].mag_cal == pid_imu_position[cycle_data[angle_loop].back10].mag_cal))
	 {
	 */
	lcd_clear_screen();
	lcd_strings calibration;
	calibration.string = "**Calib Data**";
	calibration.length = 14;
	lcd_title_write(&calibration, 1);
	calibration.string = "  Accel";
	calibration.length = 7;
	lcd_write(&calibration, (float) pid_imu_position[angle_loop].accel_cal, 2);

	calibration.string = "  Gyro";
	calibration.length = 6;
	lcd_write(&calibration, (float) pid_imu_position[angle_loop].gyro_cal, 3);

	calibration.string = "  Mag";
	calibration.length = 5;
	lcd_write(&calibration, (float) pid_imu_position[angle_loop].mag_cal, 4);
	//}
}

/*************************************************/
/**\MAIN MENU PRINT FUNCTION*/
/*
 * RETURNS NOTHING
 * ARGUMENTS NON
 PRINTS THE MAIN MENU for the LCD screen on startup
 */
/*************************************************/
void main_menu_print() {
	lcd_clear_screen();
	lcd_strings main_menu;
	main_menu.string = "**START ROBOT**";
	main_menu.length = 15;
	lcd_title_write(&main_menu, 1);

	main_menu.string = "**IMU VALUES**";
	main_menu.length = 14;
	lcd_title_write(&main_menu, 2);

	main_menu.string = "**PID VALUES**";
	main_menu.length = 14;
	lcd_title_write(&main_menu, 3);

	main_menu.string = "**ROBOT CONTROL**";
	main_menu.length = 17;
	lcd_title_write(&main_menu, 4);
}

void pid_menu_print() {
	lcd_clear_screen();
	lcd_strings main_menu;
	main_menu.string = "**PID MENU**";
	main_menu.length = 12;
	lcd_title_write(&main_menu, 1);

	main_menu.string = "**PID SPEED**";
	main_menu.length = 13;
	lcd_title_write(&main_menu, 2);

	main_menu.string = "**PID ANGLES**";
	main_menu.length = 14;
	lcd_title_write(&main_menu, 3);

	main_menu.string = "**PID POSITION**";
	main_menu.length = 16;
	lcd_title_write(&main_menu, 4);
}
/*************************************************************************************
 * control_menu()
 *
 * this menu will be used to issue a choreographed routine of the robot to display functionality
 *
 * Currently the menu items are just joke values but we plan to implement movements from here
 * the first planned movement is forward 8 meters and back 8 meters
 *
 * then we would like figure 8 pattern either pivot turning or soft turning dependant on space
 *
 * then we would like to implement center turning at high velocity followed by in place oscillation
 * showcasing the speed and agility of the robot and it's ability to balance while also balancing objects
 *
 */
void control_menu() {
	lcd_clear_screen();
	lcd_strings main_menu;
	main_menu.string = "**CONTROL MENU**";
	main_menu.length = 16;
	lcd_title_write(&main_menu, 1);

	main_menu.string = "**STRAIGHT LINE**";
	main_menu.length = 17;
	lcd_title_write(&main_menu, 2);

	main_menu.string = "**SQUARE**";
	main_menu.length = 10;
	lcd_title_write(&main_menu, 3);

	main_menu.string = "**FIGURE EIGHT**";
	main_menu.length = 16;
	lcd_title_write(&main_menu, 4);
}

void lcd_clear_screen() {
	i2c_lcd_command(0);
	Task_sleep(3);
}

/*************************************************************************************
 * menu_print_control()
 *
 * This is the main control of the menu and it decides what values to print onto the screen based on the
 * current menu state
 *
 *
 */
void menu_print_control() {
	int loop_val;
	loop_val = angle_loop - 1;
	if (loop_val < 0) {
		loop_val = 0;
	}
	switch (menu_state.main_state) {
	case 0:
		main_menu_print();
		break;
	case 1:
		switch (menu_state.middle_state) {
		case 0:
			angle_menu_print();
			break;
		case 1:
			euler_angle_print(imu_data[angle_loop].euler_values.p,
					imu_data[angle_loop].euler_values.r,
					imu_data[angle_loop].euler_values.h);
			break;
		case 2:
			gyro_print(imu_data[angle_loop].x_gyro, imu_data[angle_loop].x_gyro,
					imu_data[angle_loop].x_gyro);
			break;
		case 3:
			wheel_print(QEIPositionGet(QEI0_BASE),
					QEIPositionGet(QEI1_BASE)); //REFACTOR THIS
			break;
		}
		break;
	case 2:
		switch (menu_state.middle_state) {
		case 0:
			pid_menu_print();
			break;
		case 1:
			PID_print(_PIDS.Pgain, _PIDS.Igain, _PIDS.Dgain);
			break;
		case 2:
			PID_print( _PIDA.Pgain, _PIDA.Igain, _PIDA.Dgain);
			break;
		case 3:
			PID_print(PID_CONTROL_DATA._pid_position.Pgain,
					PID_CONTROL_DATA._pid_position.Igain,
					PID_CONTROL_DATA._pid_position.Dgain);
		}
		break;
	case 3:
		switch (menu_state.middle_state) {
		case 0:
			control_menu();
			break;
		case 1:
			move_straight_menu();
			break;
		case 2:
			square_menu();
			break;
		case 3:
			break;
		}
		break;
	case 10:
		switch(menu_state.middle_state){
		case 0:
			robot_running_menu();
			break;
		case 1:
			euler_angle_print(imu_data[angle_loop].euler_values.p,
								imu_data[angle_loop].euler_values.r,
								imu_data[angle_loop].euler_values.h);						//robot angles
			break;
		case 2:
			Ierror_print(_PIDA.Ierror, _PIDS.Ierror, PID_CONTROL_DATA._pid_position.Ierror); 						//encoder counts
			break;
		case 3:
			pid_setpoints(PID_CONTROL_DATA._pid_position.setPoint, _PIDS.setPoint, _PIDA.setPoint);
			break;

		}
		break;
	case 15:
		calib_menu();
		break;
	}

}

/*************************************************/
/**\START_UP_CONFIG*/
/*
 * Start up sequence of the LCD to clear the LCD and enable
 * 4 bit mode for writing to and from the LCD with i2c bus


 */
/*************************************************/

void start_up_sequence() {
	//initialize the startup bite sequence
	lcd_strings string;
	unsigned char init_value[2] = { 0x3C, 0x38 };//Write to start-up the module
	string.string = init_value;
	string.length = 2;
	i2c_write(&string);
	delay_write_ms(15);
	i2c_write(&string);
	delay_write_ms(5);
	i2c_write(&string);
	delay_write_us(150);
	init_value[0] = 0x24;
	init_value[1] = 0x20;
	i2c_write(&string);
	delay_write_ms(5);
	init_value[0] = 0x04;
	init_value[1] = 0x00;
	i2c_write(&string);
	delay_write_us(20);
	init_value[0] = 0xD4;
	init_value[1] = 0xD0;
	i2c_write(&string);
	delay_write_us(20);
	init_value[0] = 0x04;
	init_value[1] = 0x00;
	i2c_write(&string);
	delay_write_us(20);
	init_value[0] = 0x14;
	init_value[1] = 0x10;
	i2c_write(&string);
	delay_write_ms(2);
	init_value[0] = 0x04;
	init_value[1] = 0x00;
	i2c_write(&string);
	delay_write_us(20);
	init_value[0] = 0x64;
	init_value[1] = 0x60;
	i2c_write(&string);
	delay_write_us(20);
	init_value[0] = 0x84;
	init_value[1] = 0x80;
	i2c_write(&string);
	delay_write_us(20);
	init_value[0] = 0x0C;
	init_value[1] = 0x08;
	i2c_write(&string);
	delay_write_us(20);

}

/*************************************************/
/**\START_UP_CONFIG*/
/*
 * Converts a Char into a 4bit write mode of a nibble and a bit
 * adds in the enable bit as well for writing to the port
 */
/*************************************************/
char lcd_4bit_convert_write(lcd_strings *string, unsigned char * value) {
	int count = 0;
	int string_count = 0;
	int total = string->length * 4;
	for (count; count < total; count += 4) {
		string->string_buffer[count] = ((0xF0) & *(value + string_count)) | 0x0C
				| 0x01;    //Segmenting the string into 4 bit chunk write mode
		string->string_buffer[count + 1] = ((0xF0) & *(value + string_count))
				| 0x08 | 0x01;	//Toggling Enable bit to read
		string->string_buffer[count + 2] = ((0xF0)
				& *(value + string_count) << 4) | 0x0C | 0x01; //Lower Bit Nibble to send
		string->string_buffer[count + 3] = ((0xF0)
				& *(value + string_count) << 4) | 0x08 | 0x01; //Toggle enable bit - latches data on E falling
		string_count++;
	}
	string->string = string->string_buffer;
	string->length *= 4;
	if (i2c_write(string))
		return 0x01;
	return 0x00;
}

char i2c_lcd_command(int command) {

	lcd_strings lcd_command;
	lcd_strings *ptr_lcd_command = &lcd_command;
	switch (command) {
	case 0:
		lcd_command.line_value[0] = 0x01;
		lcd_command.length = 1;
		lcd_4bit_convert_command(ptr_lcd_command, lcd_command.line_value);
		return 0x00;
	case 1:
		lcd_command.line_value[0] = 0x02;
		lcd_command.length = 1;
		lcd_4bit_convert_command(ptr_lcd_command, lcd_command.line_value);
		return 0x00;
	case 2:
		lcd_command.line_value[0] = 0xC0;
		lcd_command.length = 1;
		lcd_4bit_convert_command(ptr_lcd_command, lcd_command.line_value);
		return 0x00;
	case 3:
		lcd_command.line_value[0] = 0x94;
		lcd_command.length = 1;
		lcd_4bit_convert_command(ptr_lcd_command, lcd_command.line_value);
		return 0x00;
	case 4:
		lcd_command.line_value[0] = 0xD4;
		lcd_command.length = 1;
		lcd_4bit_convert_command(ptr_lcd_command, lcd_command.line_value);
		return 0x00;
	case 5:

		break;
	case 6:

		break;
	case 7:

		break;
	case 8:

		break;
	case 9:
		return 0x00;
	default:
		break;
	}
	return 0x00;
}

/*************************************************/
/**\START_UP_CONFIG*/
/*
 * Converts a Char into a 4bit write mode of a nibble and a bit
 * adds in the enable bit as well for writing to the port
 */
/*************************************************/
char lcd_4bit_convert_command(lcd_strings *string, unsigned char* value) {
	int count = 0;
	int string_count = 0;
	int total = string->length * 4;
	for (count; count < total; count += 4) {
		string->string_buffer[count] = ((0xF0) & *(value + string_count))
				| 0x0C;    //Segmenting the string into 4 bit chunk write mode
		string->string_buffer[count + 1] = ((0xF0) & *(value + string_count))
				| 0x08;	//Toggling Enable bit to read
		string->string_buffer[count + 2] = ((0xF0)
				& *(value + string_count) << 4) | 0x0C; //Lower Bit Nibble to send
		string->string_buffer[count + 3] = ((0xF0)
				& *(value + string_count) << 4) | 0x08; //Toggle enable bit - latches data on E falling
		string_count++;
	}
	string->string = string->string_buffer;
	string->length *= 4;
	if (i2c_write(string))
		return 1;
	return 0;
}

char i2c_write(lcd_strings *string) {

	I2C_Transaction i2cTransaction;
	i2cTransaction.slaveAddress = 0X27;
	i2cTransaction.writeBuf = string->string;//setting up i2c transaction params
	i2cTransaction.writeCount = string->length;	//writecount added to params
	i2cTransaction.readBuf = 0;					//not used but will wire up
	i2cTransaction.readCount = 0;				//read count 0
	if (I2C_transfer(i2c_lcd, &i2cTransaction))	//perform the I2C transaction TIVA API Call
			{
		return 0;								//0 = SUCCESS
	}
	return 1;								//1 = FAILURE
}

/*************************************************/
/**\LCD_title_write*/
/*
 * PARAMS LCD STRINGS
 * STRING LITERAL NO NULL
 *  LINE VALUE
 *  WRITE THE GUI TITLE
 *
 RETURNS 0 on SUCCESS OR 1 ON FAILURE
 */
/*************************************************/
char lcd_title_write(lcd_strings * final_val, int line) {

	int k = 1;
	if (line == 1) {
		final_val->line_value[0] = 0x7F;
	} else {
		switch (line) {
		case 1: //LINE 1
			i2c_lcd_command(cursor_home);
			Task_sleep(4);
			break;
		case 2: //LINE 2
			i2c_lcd_command(line2);
			break;
		case 3: //LINE 3
			i2c_lcd_command(line3);
			break;
		case 4: //LINE 4
			i2c_lcd_command(line4);
			break;
		default:
			//CLEAR SCREEN
			break;
		}
		final_val->line_value[0] = ' ';
	}
	int total = final_val->length + 1;
	for (k; k <= total; k++) {
	final_val->line_value[k] = *(final_val->string + k - 1);
	}
	final_val->string = final_val->line_value;
	final_val->length = total;
	if (lcd_4bit_convert_write(final_val, final_val->line_value)) {
		return 1;
	}
	if (line == 4) {
		switch (menu_state.line_state) {
		case 1: //LINE 1
			i2c_lcd_command(cursor_home);
			break;
		case 2: //LINE 2
			i2c_lcd_command(line2);
			break;
		case 3: //LINE 3
			i2c_lcd_command(line3);
			break;
		case 4: //LINE 4
			i2c_lcd_command(line4);
			break;
		default:
			//CLEAR SCREEN
			break;
		}
	}

	return 0;

}
/*************************************************/
/**\WRITES A VALUE TO LCD SCREEN*/
/*
 * PARAMS
 * STRING LITERAL NO NULL
 *  LENGTH
 *  float VALUE TO CONCATENATE
 *  LINE NUMBER
 RETURNS 0 on SUCCESS OR 1 ON FAILURE
 */
/*************************************************/

char lcd_write(lcd_strings *final_val, float data, int line) {
	switch (line) {
	case 1: //LINE 1
		i2c_lcd_command(cursor_home);
		//Task_sleep(2);
		break;
	case 2: //LINE 2
		i2c_lcd_command(line2);
		break;
	case 3: //LINE 3
		i2c_lcd_command(line3);
		break;
	case 4: //LINE 4
		i2c_lcd_command(line4);
		break;
	default:
		//CLEAR SCREEN
		break;
	}
	lcd_strings data_val;
	data_val.line_value[0] = ' ';
	data_val.line_value[1] = '=';
	data_val.line_value[2] = ' ';
	int counter = 3;
	for(counter = 3;counter < 13;counter++)
	{
		data_val.line_value[counter] = '0';
	}
	float_to_char(data, &data_val);
	int k = 1;
	final_val->line_value[0] = ' ';
	int total = data_val.length + final_val->length + 1;

	for (k ; k <=total; k++) {
		if (k < final_val->length + 1) {
			final_val->line_value[k] = *(final_val->string + k - 1);
		}
		if (k >= final_val->length + 1) {
			final_val->line_value[k] = *(data_val.string
					+ (k - final_val->length - 1));
		}
		//string_val->line_value[k] = (k < count) ? *(s+k) : *(data_val->string+(k-count));
	}

	final_val->string = final_val->line_value;
	final_val->length = total;
	if (final_val->length > 20) {
		final_val->length = 20;
	}
	if (lcd_4bit_convert_write(final_val, final_val->line_value)) {
		return 1;
	}

	if (line == 4) {
		switch (menu_state.line_state) {
		case 1:
			i2c_lcd_command(cursor_home);
			break;
		case 2:
			i2c_lcd_command(line2);
			break;
		case 3:
			i2c_lcd_command(line3);
			break;
		case 4:
			i2c_lcd_command(line4);
			break;
		default:
			break;
		}
	}
	return 0;
}

/*************************************************/
/**\float TO CHAR CONVERSION*/
/*
 * float TO CHAR CONVERSION OF VALUE
 * WILL CONVERT A float TO A CHAR OUTPUT


 */
/*************************************************/
// IN PROCESS OF REFACTORING THIS FUNCTION
void float_to_char(float value,lcd_strings * string_val) {
	int object_count = 0;
	int temp = 9;
	string_val->line_value[object_count] = ' ';
	object_count++;
	string_val->line_value[object_count] = '=';
	object_count++;
	string_val->line_value[object_count] = ' ';
	object_count++;
	float temp_float = value;
	temp = value;
	if (value < 0) {
		value = -1 * (value);
		string_val->line_value[object_count] = '-';
		object_count++;
	}
	float temp2 = value;
	if (value >= 1000) {
		temp = 0;
	} else if (value >= 100) {
		temp = 1;
	} else if (value >= 10) {
		temp = 2;
	} else {
		temp = 3;
	}

	switch (temp) {
	case 0:
		temp = value / 1000;
		string_val->line_value[object_count] = integer_to_char(temp);
		temp2 = (int) value % 1000;
		object_count++;
	case 1:
		temp = temp2 / 100;
		string_val->line_value[object_count] = integer_to_char(temp);
		temp2 = (int) value % 100;
		object_count++;
	case 2:
		temp = temp2 / 10;
		string_val->line_value[object_count] = integer_to_char(temp);
		temp2 = (int) value % 10;
		object_count++;
	case 3:
		temp_float = value / 1;
		temp = temp2 / 1;
		string_val->line_value[object_count] = integer_to_char(temp);
		object_count++;
		temp_float *= 1000000;
		temp = (int) temp_float;
		temp = temp % 1000000;
		temp2 = temp;
		temp = temp / 100000;
		string_val->line_value[object_count] = '.';
		object_count++;
		string_val->line_value[object_count] = integer_to_char(temp);
		object_count++;


		temp = temp2;
		temp = temp % 100000;
		temp2 = temp;
		temp = temp / 10000;
		string_val->line_value[object_count] = integer_to_char(temp);
		object_count++;


		temp = temp2;
		temp = temp % 10000;
		temp = temp / 1000;
		string_val->line_value[object_count] = integer_to_char(temp);
		object_count++;


		temp = temp2;
		temp = temp % 1000;
		temp = temp / 100;
		string_val->line_value[object_count] = integer_to_char(temp);
		object_count++;


		temp = temp2;
		temp = temp % 100;
		temp = temp / 10;
		string_val->line_value[object_count] = integer_to_char(temp);
		object_count++;


		temp = temp2;
		temp = temp % 10;
		temp = temp / 1;
		string_val->line_value[object_count] = integer_to_char(temp);
		object_count++;


		break;
	default:
		//LOG ERROR HERE
		break;

	}
	string_val->string = string_val->line_value;
	string_val->length = object_count;
}

/*************************************************/
/**\INTEGER TO CHAR CONVERSION*/
/*
 * INTEGER TO CHAR CONVERSION OF VALUE
 * WILL CONVERT A float TO A CHAR OUTPUT


 */
/*************************************************/

char integer_to_char(int value) {
	switch (value) {
	case 0:
		return '0';
	case 1:
		return '1';
	case 2:
		return '2';
	case 3:
		return '3';
	case 4:
		return '4';
	case 5:
		return '5';
	case 6:
		return '6';
	case 7:
		return '7';
	case 8:
		return '8';
	case 9:
		return '9';
	default:
		return 'A';
	}
}

/**********************************************************************
 * _unit_test_string()
 *
 *
 * we are having problems with some float strings being junk symbols
 * so I am writing this function to test the output while debugging
 * so that I can pinpoint the issue.
 *
 **********************************************************************/

char _unit_test_string(char * string, int length) {
	char flag = 1;
	int n = 3;
	for (n = 3; n < length; n++) {

		if ((57 < *(string + n) || 48 > *(string + n))
				&& (*(string + n) != 46 && *(string + n) != 45)) {
			flag = 0;
		}
	}
	return flag;

}

/*************************************************/
/**\DELAY IN MILISECONDS*/
/*
 * DELAY FUNCTION USING SYSTICK
 * WILL DELAY THE FUNCTION FOR mS on input argument


 */
/*************************************************/
void delay_write_ms(int delay) {
	SysCtlDelay(delay * 1000);
}

void delay_write_us(int delay) {
	SysCtlDelay(delay);
}
