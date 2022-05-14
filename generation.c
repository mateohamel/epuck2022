/*
 * generation.c
 *
 *  Created on: 14 mai 2022
 *      Author: hamel
 */


#include "generation.h"
#include <msgbus/messagebus.h>
#include <sensors/imu.h>
#include <math.h>
#include <chsys.h>
#include <hal.h>


//DEFINE

#define NONE '0'
#define LED_1 '1'
#define LED_3 '3'
#define LED_5 '5'
#define LED_7 '7'

//INTERNAL FUNCTION

void led_charging(uint8_t *leds_tmp, uint8_t starting_led, uint8_t led_numbers) {

	uint8_t i;
	for(i=0; i < led_numbers; i=i+1){
		if(starting_led + i < 4){
			leds_tmp[starting_led + i] = 1;
		}else{
			leds_tmp[starting_led + i - 4] =1;
		}
	}
}

void show_gravity(imu_msg_t *imu_values){

    //we create variables for the led in order to turn them off at each loop and to
    //select which one to turn on
	uint8_t leds[4] = {0,0,0,0};

    //threshold value to not use the leds when the robot is too horizontal
    float threshold = 3;
    //create a pointer to the array for shorter name
    float *accel = imu_values->acceleration;
    static uint8_t counter = 0;
    static char previous_led = NONE;

    /*
    *   example 1 with trigonometry.
    */

    /*
    * Quadrant:
    *
    *       BACK
    *       ####
    *    #    0   #
    *  #            #
    * #-PI/2 TOP PI/2#
    * #      VIEW    #
    *  #            #
    *    # -PI|PI #
    *       ####
    *       FRONT
    */

    if(fabs(accel[X_AXIS]) > threshold || fabs(accel[Y_AXIS]) > threshold){

        chSysLock();
        //clock wise angle in rad with 0 being the back of the e-puck2 (Y axis of the IMU)
        float angle = atan2(accel[X_AXIS], accel[Y_AXIS]);
        chSysUnlock();

        //rotates the angle by 45 degrees (simpler to compare with PI and PI/2 than with 5*PI/4)
        angle += M_PI/4;

        //if the angle is greater than PI, then it has shifted on the -PI side of the quadrant
        //so we correct it
        if(angle > M_PI){
            angle = -2 * M_PI + angle;
        }

        if(angle >= 0 && angle < M_PI/2){
            if(previous_led == LED_5){
            	++counter;
            }else{
            	counter=0;
            	leds[2] = 0;
            }
            switch(counter){
            case 2:
            	led_charging(leds, 2, 1);
				break;
            case 3:
            	led_charging(leds, 2, 2);
				break;
            case 4:
            	led_charging(leds, 2, 3);
				break;
            case 5:
            	led_charging(leds, 2, 4);
				set_instruction_flow(get_instruction_counter(), SOUTH);
				if (get_instruction_counter() != 15){
					increase_instruction_counter();
				}
				counter = 0;
				break;
			}
        	previous_led = LED_5;
        }else if(angle >= M_PI/2 && angle < M_PI){
            if(previous_led == LED_7){
            	++counter;
            }else{
            	counter=0;
            	leds[3] = 0;
            }
            switch(counter){
            case 2:
            	led_charging(leds, 3, 1);
				break;
            case 3:
            	led_charging(leds, 3, 2);
				break;
            case 4:
            	led_charging(leds, 3, 3);
				break;
            case 5:
            	led_charging(leds, 3, 4);
				set_instruction_flow(get_instruction_counter(), WEST);
				if (get_instruction_counter != 15){
					increase_instruction_counter();
				}
				counter = 0;
				break;
			}
        	previous_led = LED_7;
        }else if(angle >= -M_PI && angle < -M_PI/2){
            if(previous_led == LED_1){
            	++counter;
            }else{
            	counter=0;
            	leds[0] = 0;
            }
			switch(counter){
			case 2:
            	led_charging(leds, 0, 1);
				break;
			case 3:
            	led_charging(leds, 0, 2);
				break;
			case 4:
            	led_charging(leds, 0, 3);
				break;
			case 5:
            	led_charging(leds, 0, 4);
				set_instruction_flow(get_instruction_counter(), NORTH);
				if (get_instruction_counter() != 15){
					increase_instruction_counter();
				}
				counter = 0;
				break;
            }
        	previous_led = LED_1;
        }else if(angle >= -M_PI/2 && angle < 0){
            if(previous_led == LED_3){
            	++counter;
            }else{
            	counter=0;
            	leds[1] = 0;
            }
			switch(counter){
			case 2:
            	led_charging(leds, 1, 1);
				break;
			case 3:
            	led_charging(leds, 1, 2);
				break;
			case 4:
            	led_charging(leds, 1, 3);
				break;
			case 5:
            	led_charging(leds, 1, 4);
				set_instruction_flow(get_instruction_counter(), EST);
				if (get_instruction_counter() != 15){
					increase_instruction_counter();
				}
				counter = 0;
				break;
			}
        	previous_led = LED_3;
        }
    }

    //we invert the values because a led is turned on if the signal is low
    palWritePad(GPIOD, GPIOD_LED1, leds[0] ? 0 : 1);
    palWritePad(GPIOD, GPIOD_LED3, leds[1] ? 0 : 1);
    palWritePad(GPIOD, GPIOD_LED5, leds[2] ? 0 : 1);
    palWritePad(GPIOD, GPIOD_LED7, leds[3] ? 0 : 1);

}

static THD_WORKING_AREA(waInstructionFlowThread, 128);
static THD_FUNCTION(InstructionFlowThread, arg) {

	chRegSetThreadName(__FUNCTION__);
	(void)arg;

	messagebus_topic_t *imu_topic = messagebus_find_topic_blocking(&bus, "/imu");
	imu_msg_t imu_values;

	while(1){

		if(get_mode() == MODE_1){
			//wait for new measures to be published
			messagebus_topic_wait(imu_topic, &imu_values, sizeof(imu_values));

			show_gravity(&imu_values);
		}
		chThdSleepMilliseconds(300);
	}
}

//EXTERNAL FUNCTION

void instruct_gen_init(void){
	chThdCreateStatic(waInstructionFlowThread, sizeof(waInstructionFlowThread), NORMALPRIO, InstructionFlowThread, NULL);
}

