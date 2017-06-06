/*
 * lcd_gui.h
 *
 *  Created on: Feb 7, 2017
 *      Author: c_ker
 */

#ifndef LCD_GUI_H_
#define LCD_GUI_H_

#include <stdint.h>
#include <stdbool.h>

#include "Board.h"
#include "bno055.h"
#include <ti/drivers/GPIO.h>
#include <ti/drivers/I2C.h>
#include <ti/sysbios/knl/task.h>
#include <driverlib/sysctl.h>
#include <xdc/runtime/Log.h>


//lcd_string type to hold the string data to write
typedef struct lcd_string
{
	unsigned char string_buffer[90];
	unsigned char line_value[26];
	unsigned char * string;
	int length;
}lcd_strings;


enum menu
{
	MAIN,
	PID,
	EULER,
	GRAVITY,
	GYROMETER
}menu;

typedef enum commands
{
	clear_screen,
	cursor_home,
	line2,
	line3,
	line4,
}lcd_commands;

typedef struct GUI_menu
{
	lcd_strings data1;
	lcd_strings data2;
	lcd_strings data3;
	int int_data1;
	int int_data2;
	int int_data3;
	struct GUI_menu *next;
	struct GUI_menu *prev;
}GUI_menu;



//LCD functions and print screen algorithms
void lcd_clear_screen();
char init_LCD_i2c(I2C_Handle);
void start_up_sequence();
char i2c_write(lcd_strings*);
char lcd_4bit_convert(lcd_strings *,unsigned char *);
char lcd_4bit_convert_command(lcd_strings *,unsigned char *);
char i2c_lcd_command(int command);
char lcd_title_write(lcd_strings * s_val,int line);
void float_to_char(float value,lcd_strings * string_val);
void gui_navigate(unsigned char);
char integer_to_char(int value);
char lcd_write(lcd_strings*,float,int);



//LCD screen write menus
void robot_running_menu();
void menu_print_control();
void PID_print(float proportional,float integral, float derivative);
void euler_angle_print(float pitch,float roll, float heading);
void gravity_print(float x,float y, float z);
void main_menu_print();
void gyro_print(float x,float y, float z);
void wheel_print(int l_wheel,int r_wheel);
void calib_menu();
void move_straight_menu();
void figure_8();
void square_menu();

void Ierror_print(float angles, float speed, float position);

//delay functions
void delay_write_ms(int delay);
void delay_write_us(int delay);


void control_menu();
void pid_menu_print();


char _unit_test_string(char * string,int length);

I2C_Handle i2c_lcd;					//I2C main handle structure for i2c communication


#endif /* LCD_GUI_H_ */
