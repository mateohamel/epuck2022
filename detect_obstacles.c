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
#include <motors.h>
#include <sensors/proximity.h>

#include <main.h>

#define SENSORS_THRESHOLD


static THD_WORKING_AREA(waDetectObstaclesThread, 128);
static THD_FUNCTION(DetectObstaclesThread, arg) {

	chRegSetThreadName(__FUNCTION__);
	(void)arg;
    calibrate_ir();
	int left_prox = 0;
	int right_prox = 0;
	while(1){

		if(Mode == MODE_2){ //? on garde ca ?

			left_prox = get_prox(7);
			right_prox = get_prox(0);
			if((left_prox < SENSORS_THRESHOLD)||(right_prox < SENSORS_THRESHOLD)){
				left_motor_set_speed(0);
				right_motor_set_speed(0);
				Mode = MODE_1;
			}

		}
		chThdSleepMilliseconds(1);
	}
}
