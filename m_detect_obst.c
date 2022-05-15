/**
 * @file    m_detect_obst.c
 * @brief   Module File. Handles obstacles detection using the proximity sensor.
 */


// e-puck main processor headers

#include <motors.h>
#include <sensors/proximity.h>
#include <msgbus/messagebus.h>


// Module headers

#include "m_detect_obst.h"
#include "m_globals.h"


/*===========================================================================*/
/* Module constants.                                                         */
/*===========================================================================*/

#define ON 	1
#define OFF 0


/*===========================================================================*/
/* Bus related declarations.                                                 */
/*===========================================================================*/

extern messagebus_t bus;


/*===========================================================================*/
/* Module local functions.                                                   */
/*===========================================================================*/

/**
 * @brief				Compare the values measured by the proximity sensor to a threshold and set the robot in mode 3
 * 						if it is to close to an object.
 *
 * @param prox_values	Pointer to the message containing Proximity sensor measurements.
 *
 * @return              none
 *
*/

void obstacle_detection(proximity_msg_t *prox_values){

	//stop motors if IR1 or IR8 detect an obstacle
	if ((prox_values->ambient[0] - prox_values->reflected[0] > 100) && (prox_values->ambient[7] - prox_values->reflected[7] > 100)){
		left_motor_set_speed(NO_SPEED);
		right_motor_set_speed(NO_SPEED);
		set_mode(MODE_3);
		set_instruction_counter(0);
		set_instruction_flow(0, NO_INSTRUCTION);
	}
}


/*===========================================================================*/
/* Module threads.                                                           */
/*===========================================================================*/

/**
 * @brief               Thread which is in charge of detecting obstacles and changing the mode according to the results.
 *
 * @return              none
 *
*/

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


/*===========================================================================*/
/* Module exported functions.                                                */
/*===========================================================================*/

/**
 * @brief               Initializes obstacle detection using the proximity sensor thread.
 *
 * @return              none
 *
*/

void detect_obst_init(void){
	chThdCreateStatic(waDetectObstaclesThread, sizeof(waDetectObstaclesThread), NORMALPRIO, DetectObstaclesThread, NULL);
}
