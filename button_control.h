/*
 * button_control.h
 *
 *  Created on: Feb 12, 2017
 *      Author: c_ker
 */

#ifndef BUTTON_CONTROL_H_
#define BUTTON_CONTROL_H_

#include "project_types.h"

void init_main_state();
void add_value_item();
void neg_value_item();

void main_button_deb(unsigned int index);
void down_button_deb(unsigned int index);
void up_button_deb(unsigned int index);

void disable_ints_buttons();
void enable_ints_buttons();


void refresh_debounce();

void init_buttons();

#endif /* BUTTON_CONTROL_H_ */
