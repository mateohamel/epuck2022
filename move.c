/*
 * move.c
 *
 *  Created on: 9 mai 2022
 *      Author: Romane
 */


#include "ch.h"
#include "hal.h"
#include <math.h>
#include <usbcfg.h> // ??
#include <chprintf.h> // ??
#include "selector.h"



#include <main.h>
#include <motors.h>

#define TURNING_SPEED 158
#define SPEED_FACTOR 60



typedef enum {NORTH, EST, SOUTH, WEST} instruction;

typedef enum {FORWARD, RIGHT,LEFT} direction;


static direction route[31];
static uint8_t size = 0;

void go(direction dir, uint8_t motor_speed){

	switch(dir){

	case(LEFT) :
		left_motor_set_speed(-TURNING_SPEED);
		right_motor_set_speed(TURNING_SPEED);
		break;

	case(RIGHT) :
		left_motor_set_speed(TURNING_SPEED);
		right_motor_set_speed(-TURNING_SPEED);
		break;

	case(FORWARD) :
		left_motor_set_speed(motor_speed);
		right_motor_set_speed(motor_speed);
		break;
	}
}

void translation(instruction Instruction_Flow[Instruction_Counter]){

	switch(Instruction_Flow[0]){

	case(NORTH) :
		route[0] = FORWARD;
		size++;
		break;

	case(WEST) :
		route[0] = LEFT;
		size++;
		route[1] = FORWARD;
		size++;
		break;

	case(SOUTH) :
		route[0] = RIGHT;
		size++;
		route[1] = RIGHT;
		size++;
		route[2] = FORWARD;
		size++;
		break;

	case(EST) :
		route[0] = RIGHT;
		size++;
		route[1] = FORWARD;
		size++;
		break;
	}

	for(int i=1; i<Instruction_Counter ; i++){

		switch(Instruction_Flow[i]-Instruction_Flow[i-1]){

		case -3 :
			route[size] = RIGHT;
			size++;
			route[size] = FORWARD;
			size++;
			break;

		case -2 :
			route[size] = RIGHT;
			size++;
			route[size] = RIGHT;
			size++;
			route[size] = FORWARD;
			size++;
			break;

		case -1 :
			route[size] = LEFT;
			size++;
			route[size] = FORWARD;
			size++;
			break;

		case 0 :
			route[size] = FORWARD;
			size++;
			break;

		case 1 :
			route[size] = RIGHT;
			size++;
			route[size] = FORWARD;
			size++;
			break;

		case 2 :
			route[size] = RIGHT;
			size++;
			route[size] = RIGHT;
			size++;
			route[size] = FORWARD;
			size++;
			break;

		case 3 :
			route[size] = LEFT;
			size++;
			route[size] = FORWARD;
			size++;
			break;
		}
	}
}



// maintenant il faut creer un thread recurrent toutes les 2 secondes qui go(route[i])
// tant que i est inferieur a size. sinon, rien ?


//static THD_WORKING_AREA(waMove, 128);
//static THD_FUNCTION(Move, arg) {
//
	//uint8_t i = 0;
	//while(i < size){
		//float time;
    //    time = chVTGetSystemTime();
   //     uint8_t motor_speed = get_selector()*SPEED_FACTOR;
  //      go(route[i], motor_speed);
 //       i++;
//		chThdSleepUntilWindowed(time, time + MS2ST(2000000));
//	}
//	size = 0;
	//chThdSleepUntilWindowed(time, time + MS2ST(2000000));
//}


static THD_WORKING_AREA(waInstructionExecutionThread, 128);
static THD_FUNCTION(InstructionExecutionThread, arg) {

	chRegSetThreadName(__FUNCTION__);
	(void)arg;

	while(1){

		if(Mode == MODE_2){
			//function execute instruction
			uint8_t i = 0;
			while(i < size){
				uint32) time;
		        time = chVTGetSystemTime();
		        uint8_t motor_speed = get_selector()*SPEED_FACTOR;
		        go(route[i], motor_speed);
		        i++;
				chThdSleepUntilWindowed(time, time + MS2ST(2000000));
			}

			size = 0;

		}
		chThdSleepMilliseconds(2000);
	}
}


void move_init(void){
	chThdCreateStatic(waInstructionExecutionThread, sizeof(waInstructionExecutionThread), NORMALPRIO, InstructionExecutionThread, NULL);
	translation(Instruction_Flow);
	chThdCreateStatic(waMove, sizeof(waMove), NORMALPRIO, Move, NULL);

}
