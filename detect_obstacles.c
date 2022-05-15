/*
 * move.c
 *
 *  Created on: 9 mai 2022
 *      Author: Romane
 */


#include "hal.h"
#include <motors.h>
#include <sensors/proximity.h>
#include <msgbus/messagebus.h>
#include "globals.h"

//DEFINE

#define ON 1
#define OFF 0

extern messagebus_t bus;

//INTERNAL FUNCTION

void obstacle_detection(proximity_msg_t *prox_values){

	//arrêter les moteurs si IR1 ou IR8 détectent un obstacle
	if ((prox_values->ambient[0] - prox_values->reflected[0] > 100) && (prox_values->ambient[7] - prox_values->reflected[7] > 100)){
		left_motor_set_speed(0);
		right_motor_set_speed(0);
		set_mode(MODE_3);
		set_instruction_counter(0);
		set_instruction_flow(0, NO_INSTRUCTION);
	}
}


static THD_WORKING_AREA(waDetectObstaclesThread, 128);
static THD_FUNCTION(DetectObstaclesThread, arg) {

	chRegSetThreadName(__FUNCTION__);
	(void)arg;

	messagebus_topic_t *prox_topic = messagebus_find_topic_blocking(&bus, "/proximity");
	proximity_msg_t prox_values;
	uint8_t bodyled_state = OFF;

	while(1){
		switch(get_mode()){
		case INIT:
			break;
		case MODE_1:
			break;
		case MODE_2:
			bodyled_state = OFF;
			messagebus_topic_wait(prox_topic, &prox_values, sizeof(prox_values));
			obstacle_detection(&prox_values);
			break;
		case MODE_3:
			if(bodyled_state == OFF){
				bodyled_state = ON;
			}else{
				bodyled_state = OFF;
			}
			palWritePad(GPIOB, GPIOB_LED_BODY, bodyled_state ? 0 : 1);
			break;
		}
		chThdSleepMilliseconds(100);
	}
}


//EXTERNAL FUNCTION

void detect_obst_init(void){
	chThdCreateStatic(waDetectObstaclesThread, sizeof(waDetectObstaclesThread), NORMALPRIO, DetectObstaclesThread, NULL);
}
