/*
 * button_control.c
 *
 *  Created on: Feb 12, 2017
 *      Author: c_ker
 */

/****************************************************************************************
 *
 * This file is for the button controls of the GUI menu and the state changes from presses
 *
 *
 * Written by chris Kerluke for use in final project Mechatronics and Robotics Balancing bot
 *
 *
 * Edited and refactored by Tyler Lemery Project Partner
 *
 */

#include "button_control.h"

int flag = 0;

/*//////////////////////////////////////////////////////
 * init the buttons for the gui control of the LED
 *
 * no input arguments
 * no return parameters
 *
 * initilize the callback functions and the GPIO interrupts
 */
void init_buttons() {
	init_main_state();
	GPIO_setCallback(3, up_button_deb);
	//GPIO call back up button check board init for PIN declaration
	GPIO_enableInt(3);
	GPIO_setCallback(4, down_button_deb);
	//GPIO call back up button check board init for PIN declaration
	GPIO_enableInt(4);
	GPIO_setCallback(5, main_button_deb);
	//GPIO call back up button check board init for PIN declaration
	GPIO_enableInt(5);
}

/*////////////////////////////////////////////////////////////////
 * Function to refresh the int's if you disable them
 *
 * no input arguments no return types
 *
 * initialize the inteerrupts for the GUI buttons
 */
void refresh_debounce() {
	GPIO_enableInt(3);
	GPIO_enableInt(4);
	GPIO_enableInt(5);
}

void init_main_state() {
	menu_state.main_state = 15;
	menu_state.middle_state = 0;
	menu_state.line_state = 1;
}

/*****************************************************************************
 * Main Button Fxn press
 *
 * No arguments
 *
 *
 * Function will change teh state of the menu based on the main button pressed
 * Called from ClockHandle Debounce Function
 *
 *
 *
 */
Void main_button_fxn() {
	menu_state.menu_state_change = 1;
	if (!(GPIO_read(5))) {
		switch (menu_state.main_state) {
		case 0:
			switch (menu_state.line_state) {
			case 1:
				menu_state.main_state = 10;
				init_QEI();
				init_balance_control();
				break;
			case 2:
				menu_state.main_state = 1;
				break;
			case 3:
				menu_state.main_state = 2;
				break;
			case 4:
				menu_state.main_state = 3;
				break;
			}
			break;
		case 1:
			switch (menu_state.middle_state) {
			case 0:
				switch (menu_state.line_state) {
				case 1:
					menu_state.main_state = 0;
					break;
				case 2:
					menu_state.middle_state = 1;
					break;
				case 3:
					menu_state.middle_state = 2;
					break;
				case 4:
					menu_state.middle_state = 3;
				}
				break;
			case 1:
				menu_state.middle_state = 0;
				break;
			case 2:
				menu_state.middle_state = 0;
				break;
			case 3:
				menu_state.middle_state = 0;
				break;
			}
			break;
		case 2:
			switch (menu_state.middle_state) {
			case 0:
				switch (menu_state.line_state) {
				case 1:
					menu_state.main_state = 0;
					break;
				case 2:
					menu_state.middle_state = 1;
					break;
				case 3:
					menu_state.middle_state = 2;
					break;
				case 4:
					menu_state.middle_state = 3;
				}
				break;
			case 1:
				switch (menu_state.line_state) {
				case 1:
					menu_state.middle_state = 0;
					break;
				case 2:
					menu_state.change_state = 1 - menu_state.change_state;
					break;
				case 3:
					menu_state.change_state = 1 - menu_state.change_state;
					break;
				case 4:
					menu_state.change_state = 1 - menu_state.change_state;
					break;
				}
				break;
			case 2:
				switch (menu_state.line_state) {
				case 1:
					menu_state.middle_state = 0;
					break;
				case 2:
					menu_state.change_state = 1 - menu_state.change_state;
					break;
				case 3:
					menu_state.change_state = 1 - menu_state.change_state;
					break;
				case 4:
					menu_state.change_state = 1 - menu_state.change_state;
					break;
				}
				break;
			case 3:
				switch (menu_state.line_state) {
				case 1:
					menu_state.middle_state = 0;
					break;
				case 2:
					menu_state.change_state = 1 - menu_state.change_state;
					break;
				case 3:
					menu_state.change_state = 1 - menu_state.change_state;
					break;
				case 4:
					menu_state.change_state = 1 - menu_state.change_state;
					break;
				}
				break;

			}
			break;
		case 3:
			switch (menu_state.middle_state) {
			case 0:
				switch (menu_state.line_state) {
				case 1:
					menu_state.main_state = 0;
					break;
				case 2:
					menu_state.middle_state = 1;
					//CHANGE TO STRAIGHT FORWARD
					ctrl_robot.autonomous_mode = 1;
					break;
				case 3:
					menu_state.middle_state = 2;
					//CHANGE TO SQUARE
					ctrl_robot.autonomous_state = 0;
					ctrl_robot.autonomous_mode = 2;
					ctrl_robot.flag_square = 0;
					ctrl_robot.counter = 0;
					break;
				case 4:
					//CHANGE TO FIGURE 8;
					ctrl_robot.autonomous_mode = 3;
					menu_state.main_state = 0;
					menu_state.middle_state =0;
					break;
				}
				break;
			case 1:
				switch (menu_state.line_state) {
				case 1:
				menu_state.middle_state = 0;
				break;
				case 2:
					menu_state.change_state = 1 - menu_state.change_state;
					break;
					case 3:
					break;
					case 4:
					break;
					}
				break;
				case 2:
					switch (menu_state.line_state) {
					case 1:
						menu_state.middle_state = 0;
						break;
					case 2:
						menu_state.change_state = 1 - menu_state.change_state;
						break;
					case 3:
						menu_state.change_state = 1 - menu_state.change_state;
						break;
					case 4:
						break;
					}
					break;
			}
			break;
		case 10:
			menu_state.main_state = 0;
			break;
		case 15:
			menu_state.main_state = 0;
		default:
			break;
		}
	}

}

/*****************************************************************************
 *Up button Pressed
 *
 * No arguments
 *
 *
 * Function will change teh state of the menu based on the up button pressed
 * Called from ClockHandle Debounce Function
 *
 *
 *
 ****************************************************************************/

Void up_button_fxn() {
	if (!(GPIO_read(3))) {
		if(menu_state.main_state == 10)
		{
			menu_state.middle_state ++;
			if(menu_state.middle_state ==4)
			{
				menu_state.middle_state = 0;
			}
		}
		else if (menu_state.change_state == 0) {
			if (menu_state.line_state == 1) {
				menu_state.line_state = 4;
			} else {
				menu_state.line_state--;
			}
		} else if (menu_state.change_state == 1) {
			add_value_item();
		}
	}

}

/*****************************************************************************
 *Down Button Pressed
 *
 * No arguments
 *
 *
 * Function will change teh state of the menu based on the up button pressed
 * Called from ClockHandle Debounce Function
 *
 *
 *
 ****************************************************************************/
Void down_button_fxn() {
	if (!(GPIO_read(4))) {
		if(menu_state.main_state == 10)
		{
			menu_state.middle_state --;
			if(menu_state.middle_state < 0)
			{
				menu_state.middle_state = 3;
			}
		}
		else if(menu_state.change_state == 0) {
			if (menu_state.line_state == 4) {
				menu_state.line_state = 1;
			} else {
				menu_state.line_state++;
			}
		} else if (menu_state.change_state == 1) {
			neg_value_item();
		}
	}
}

/*****************************************************************************
 * Add value to the current item when up is pressed
 *
 * No arguments
 *
 *
 * Function will change the valueu based on the up button pressed
 * Called from ClockHandle Debounce Function
 *
 *
 *
 ****************************************************************************/
void add_value_item() {
	switch (menu_state.main_state) {
	case 2:
	switch (menu_state.middle_state) {
	case 1:
		switch (menu_state.line_state) {
		case 2:
			_PIDS.Pgain += 0.001;
			break;
		case 3:
			_PIDS.Igain += 0.00001;
			break;
		case 4:
			_PIDS.Dgain += 0.001;
			break;
		}
		break;
	case 2:
		//statement to check which line the menu is at
		switch (menu_state.line_state) {
		case 2:
			_PIDA.Pgain += 10.0;
			break;
		case 3:
			_PIDA.Igain += 2.5;
			break;
		case 4:
			_PIDA.Dgain += 5;
			break;
		}
		break;
	case 3:
		switch (menu_state.line_state) {
		case 2:
			PID_CONTROL_DATA._pid_position.Pgain += 0.0001;
			break;
		case 3:
			PID_CONTROL_DATA._pid_position.Igain += 0.00001;
			break;
		case 4:
			PID_CONTROL_DATA._pid_position.Dgain += 0.01;
			break;
		}
		break;
		}
	break;
	case 3:
		switch(menu_state.middle_state)
		{
		case 1:
			switch (menu_state.line_state) {
					case 2:
						//X Value
						auto_setpoints.straight_setpoint +=0.25;
						break;
					case 3:
						break;
					case 4:
						break;
					}
		case 2:
			switch (menu_state.line_state) {
					case 2:
						//X Value
						auto_setpoints.X_square +=0.25;
						break;
					case 3:
						//Y Value
						auto_setpoints.Y_square+=0.25;
						break;
					case 4:
						break;
					}
		case 3:
			switch (menu_state.line_state) {
					case 2:
						break;
					case 3:
						break;
					case 4:
						break;
					}


		}
	}

}

/*****************************************************************************
 * Add value to the current item when down is pressed
 *
 * No arguments
 *
 *
 * Function will change the value based on the down button is pressed
 * Called from ClockHandle Debounce Function
 *
 *
 *
 ****************************************************************************/
void neg_value_item() {
	//check the middle state value
	switch (menu_state.main_state)
	{

	case 2:
	switch (menu_state.middle_state) {
	case 1:
		//check the current line state
		switch (menu_state.line_state) {
		case 2:
			_PIDS.Pgain -= 0.001;
			break;
		case 3:
			_PIDS.Igain -= 0.00001;
			break;
		case 4:
			_PIDS.Dgain -= 0.001;
			break;
		}
		break;
	case 2:
		//check line state to make sure that the right value is raised
		switch (menu_state.line_state) {
		case 2:
			_PIDA.Pgain -= 10;
			break;
		case 3:
			_PIDA.Igain -= 2.5;
			break;
		case 4:
			_PIDA.Dgain -= 5;
			break;
		}
		break;
	case 3:
		switch (menu_state.line_state) {
		case 2:
			PID_CONTROL_DATA._pid_position.Pgain -= 0.0001;
			break;
		case 3:
			PID_CONTROL_DATA._pid_position.Igain -= 0.00001;
			break;
		case 4:
			PID_CONTROL_DATA._pid_position.Dgain -= 0.01;
			break;
		}
		break;
	}
	break;
	case 3:
		switch(menu_state.middle_state)
				{
				case 1:
					switch (menu_state.line_state) {
							case 2:
								//X Value
								auto_setpoints.straight_setpoint -=0.25;
								break;
							case 3:
								break;
							case 4:
								break;
							}
				case 2:
					switch (menu_state.line_state) {
							case 2:
								//X Value
								auto_setpoints.X_square -=0.25;
								break;
							case 3:
								//Y Value
								auto_setpoints.Y_square -=0.25;
								break;
							case 4:
								break;
							}
				case 3:
					switch (menu_state.line_state) {
							case 2:
								break;
							case 3:
								break;
							case 4:
								break;
							}


				}
	break;
	}

}

/*****************************************************************************
 * disable ints_buttons if robot operation is inputted
 *
 * No arguments
 *
 *
 * This function will disable up and down buttons while robot is running
 * Only the main button can take the robot out of automation
 *
 *
 *
 ****************************************************************************/
void disable_ints_buttons() {
	GPIO_disableInt(5);
	GPIO_disableInt(6);
}

/*****************************************************************************
 * Re-anable the interrupts if automation state is changed back to configure
 *
 * No arguments
 *
 *
 * This function will disable up and down buttons while robot is running
 * Only the main button can take the robot out of automation
 *
 *
 *
 ****************************************************************************/
void enable_ints_buttons() {
	GPIO_enableInt(5);
	GPIO_enableInt(6);
}

/*****************************************************************************
 * Debounce button called when button is pressed
 *
 * No arguments
 *
 * This function will call a oneshot clock to call the button that was pressed
 *
 *
 *
 ****************************************************************************/
void up_button_deb(unsigned int index) {
	Clock_start(clock_up_btn);
	GPIO_clearInt(3);
//clear the interrupt of the button
}
/*****************************************************************************
 * Debounce button called when button is pressed
 *
 * No arguments
 *
 * This function will call a oneshot clock to call the button that was pressed
 *
 *
 *
 ****************************************************************************/
void down_button_deb(unsigned int index) {
	Clock_start(clock_down_btn);
	GPIO_clearInt(4);
	//clear the interrupt of the button
}

/*****************************************************************************
 * Debounce button called when button is pressed
 *
 * No arguments
 *
 * This function will call a oneshot clock to call the button that was pressed
 *
 *
 *
 ****************************************************************************/
void main_button_deb(unsigned int index) {
	Clock_start(clock_main_btn);
	GPIO_clearInt(5);
	//clear the interrupt of the button
}
