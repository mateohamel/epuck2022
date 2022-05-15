/*
 * move.c
 *
 *  Created on: 9 mai 2022
 *      Author: Romane
 */


#include "ch.h"
#include "hal.h"
#include "selector.h"
#include <motors.h>
#include "move.h"
#include "globals.h"


//DEFINE

#define TURNING_SPEED 158
#define SPEED_FACTOR 60
#define SPEED_OFFSET 140

//INTERNAL FUNCTION

void go(direction dir, uint16_t motor_speed){

	switch(dir){

	case(NO_DIRECTION):
		break;
	case(LEFT):
		left_motor_set_speed(-TURNING_SPEED);
		right_motor_set_speed(TURNING_SPEED);
		break;

	case(RIGHT):
		left_motor_set_speed(TURNING_SPEED);
		right_motor_set_speed(-TURNING_SPEED);
		break;

	case(STRAIGHT):
		left_motor_set_speed(motor_speed);
		right_motor_set_speed(motor_speed);
		break;
	}
}

static THD_WORKING_AREA(waInstructionExecutionThread, 128);
static THD_FUNCTION(InstructionExecutionThread, arg) {

	chRegSetThreadName(__FUNCTION__);
	(void)arg;


	while(1){
		if(get_mode() == MODE_2){
			//function execute instruction
			uint8_t i = 0;

			while((i < get_route_counter()) && (get_mode() == MODE_2)){
		        uint16_t motor_speed = get_selector()*SPEED_FACTOR + SPEED_OFFSET;
		        go(get_route(i), motor_speed);
		        i++;
		        chThdSleepMilliseconds(2000);
			}
		}
		left_motor_set_speed(0);
		right_motor_set_speed(0);
		set_route_counter(0);

		chThdSleepMilliseconds(2000);
	}
}

//EXTERNAL FUNCTION

void move_init(void){
	chThdCreateStatic(waInstructionExecutionThread, sizeof(waInstructionExecutionThread), NORMALPRIO, InstructionExecutionThread, NULL);
}
