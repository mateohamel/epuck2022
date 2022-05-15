/**
 * @file    m_execute_dir.c
 * @brief   Module File. Handles the movement of the robot according to the given instructions.
 */


// e-puck main processor headers

#include <motors.h>
#include <selector.h>


// Module headers

#include "m_execute_dir.h"
#include "m_globals.h"


/*===========================================================================*/
/* Module constants.                                                         */
/*===========================================================================*/

#define TURNING_SPEED 159
#define SPEED_FACTOR 60
#define SPEED_OFFSET 140


/*===========================================================================*/
/* Module local functions.                                                   */
/*===========================================================================*/

/**
 * @brief				function that sets the motors speed acording to the direction in which
 * 						the robot has to go
 *
 * @param 				motor_speed, the speed in which the robot will go straight
 * @param 				dir, the direction in which we want it to go
 *
 * @return              none
 *
*/

void go(direction dir, uint16_t motor_speed){

	switch(dir){

	case NO_DIRECTION :
		break;
	case LEFT :
		left_motor_set_speed(-TURNING_SPEED);
		right_motor_set_speed(TURNING_SPEED);
		break;

	case RIGHT :
		left_motor_set_speed(TURNING_SPEED);
		right_motor_set_speed(-TURNING_SPEED);
		break;

	case STRAIGHT :
		left_motor_set_speed(motor_speed);
		right_motor_set_speed(motor_speed);
		break;
	}
}


/*===========================================================================*/
/* Module threads.                                                           */
/*===========================================================================*/

/**
 * @brief               Thread which is in charge of moving the robot according to the given instruction.
 *
 * @return              none
 *
*/

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


/*===========================================================================*/
/* Module exported functions.                                                */
/*===========================================================================*/

/**
 * @brief               Initializes the Instruction execution thread.
 *
 * @return              none
 *
*/

void move_init(void){
	chThdCreateStatic(waInstructionExecutionThread, sizeof(waInstructionExecutionThread), NORMALPRIO, InstructionExecutionThread, NULL);
}
