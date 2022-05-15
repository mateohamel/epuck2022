/*
 * selection.c
 *
 *  Created on: 14 mai 2022
 *      Author: hamel
 */

#include "selection.h"
#include <motors.h>
#include <math.h>

//DEFINE

#define ON 1
#define OFF 0

extern messagebus_t bus;

//INTERNAL FUNCTION

void translation(void){

	switch(get_instruction_flow(0)){

	case(NO_INSTRUCTION) :
		break;
	case(NORTH) :
		set_route(STRAIGHT, 0);
		increase_route_counter();
		break;

	case(WEST) :
		set_route(LEFT, 0);
		increase_route_counter();
		set_route(STRAIGHT, 1);
		increase_route_counter();
		break;

	case(SOUTH) :
		set_route(RIGHT, 0);
		increase_route_counter();
		set_route(RIGHT, 1);
		increase_route_counter();
		set_route(STRAIGHT, 2);
		increase_route_counter();
		break;

	case(EST) :
		set_route(RIGHT, 0);
		increase_route_counter();
		set_route(STRAIGHT, 1);
		increase_route_counter();
		break;
	}

	for(int i=1; i<get_instruction_counter() ; i++){

		switch(get_instruction_flow(i)-get_instruction_flow(i-1)){

		case -3 :
			set_route(RIGHT, get_route_counter());
			increase_route_counter();
			set_route(STRAIGHT, get_route_counter());
			increase_route_counter();
			break;

		case -2 :
			set_route(RIGHT, get_route_counter());
			increase_route_counter();
			set_route(RIGHT, get_route_counter());
			increase_route_counter();
			set_route(STRAIGHT, get_route_counter());
			increase_route_counter();
			break;

		case -1 :
			set_route( LEFT, get_route_counter());
			increase_route_counter();
			set_route(STRAIGHT, get_route_counter());
			increase_route_counter();
			break;

		case 0 :
			set_route(STRAIGHT, get_route_counter());
			increase_route_counter();
			break;

		case 1 :
			set_route(RIGHT, get_route_counter());
			increase_route_counter();
			set_route(STRAIGHT, get_route_counter());
			increase_route_counter();
			break;

		case 2 :
			set_route(RIGHT, get_route_counter());
			increase_route_counter();
			set_route(RIGHT, get_route_counter());
			increase_route_counter();
			set_route(STRAIGHT, get_route_counter());
			increase_route_counter();
			break;

		case 3 :
			set_route(LEFT, get_route_counter());
			increase_route_counter();
			set_route(STRAIGHT, get_route_counter());
			increase_route_counter();
			break;
		}
	}
}

void Mode_Detection(imu_msg_t *imu_values){

	float threshold_zh = 16;
	float threshold_zl = 10;
	float threshold_xy = 2.5;
	//create a pointer to the array for shorter name
	float *accell = imu_values->acceleration;
    static uint8_t counter = 0;
    static bool current_state = false;

	if(fabs(accell[Z_AXIS]) < threshold_zh && fabs(accell[Z_AXIS]) > threshold_zl && !(fabs(accell[X_AXIS]) > threshold_xy || fabs(accell[Y_AXIS]) > threshold_xy )){
		if(get_mode() == MODE_1){ //MODE_1
			if(counter == 8 && !current_state){
				chThdSleepMilliseconds(500);
				set_mode(MODE_2);
				palWritePad(GPIOB, GPIOB_LED_BODY, ON ? 0 : 1);
				translation();
				counter = 0;
				current_state = true;
			}else{
				if(counter > 8){
					counter = 0;
				}
				++counter;
			}
		}else{ //MODE_2 or MODE_3
			if(counter == 8 && !current_state){
				set_mode(MODE_1);
				set_instruction_counter(0);
				set_instruction_flow(0,NO_INSTRUCTION);
				palWritePad(GPIOB, GPIOB_LED_BODY, OFF ? 0 : 1);
				counter = 0;
				current_state = true;
				left_motor_set_speed(0);
				right_motor_set_speed(0);
			}else{
				if(counter > 8){
					counter = 0;
				}
				++counter;
			}
		}
	}else{
		current_state = false;
	}
}

static THD_WORKING_AREA(waModeSelectionThread, 128);
static THD_FUNCTION(ModeSelectionThread, arg) {

	chRegSetThreadName(__FUNCTION__);
	(void)arg;

	messagebus_topic_t *imu_topic = messagebus_find_topic_blocking(&bus, "/imu");
	imu_msg_t imu_values;

	while(1){

		//wait for new measures to be published
		messagebus_topic_wait(imu_topic, &imu_values, sizeof(imu_values));

		Mode_Detection(&imu_values);
		chThdSleepMilliseconds(10);
	}
}

//EXTERNAL FUNCTION

void mode_select_init(void){
    chThdCreateStatic(waModeSelectionThread, sizeof(waModeSelectionThread), NORMALPRIO, ModeSelectionThread, NULL);
}



