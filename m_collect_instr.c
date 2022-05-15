/**
 * @file    m_collect_instr.c
 * @brief   Module File. Handles the retrieval of the instructions given by the user.
 */


// C library

#include <math.h>


// e-puck main processor headers

#include <sensors/imu.h>
#include <chsys.h>
#include <msgbus/messagebus.h>


// Module headers

#include <m_collect_instr.h>
#include "m_globals.h"


/*===========================================================================*/
/* Module constants.                                                         */
/*===========================================================================*/

#define NONE			'0'
#define LED_1			'1'
#define LED_3			'3'
#define LED_5			'5'
#define LED_7			'7'
#define XY_THRESHOLD	3     //threshold value to not use the leds when the robot is too horizontal


/*===========================================================================*/
/* Bus related declarations.                                                 */
/*===========================================================================*/

extern messagebus_t bus;


/*===========================================================================*/
/* Module local functions.                                                   */
/*===========================================================================*/

/**
 * @brief				Modify the LEDs array, put to 1 the starting led and a precise number of the following LEDs (clockwise).
 *
 * @param starting_led	index of 'leds_tmp' according to the measured angle.
 * @param led_numbers	represent how many LEDs to activate.
 *
 * @return              none
 *
*/

static void led_charging(uint8_t *leds_tmp, uint8_t starting_led, uint8_t led_numbers) {

	uint8_t i;
	for(i=0; i < led_numbers; i=i+1){
		if(starting_led + i < 4){
			leds_tmp[starting_led + i] = 1;
		}else{
			leds_tmp[starting_led + i - 4] =1;
		}
	}
}


/**
 * @brief				Set LEDs to 1 according to the counter to symbolize instruction charging.
 *
 * @param cardinal_dir	NO_INSTRUCTION, NORTH, EST, SOUTH, WEST
 * @param current_led	index of 'leds_tmp' according to the angle measured.
 *
 * @return              none
 *
*/

static bool led_counter(uint8_t *leds_tmp, uint8_t counter, uint8_t current_led, uint8_t cardinal_dir){
    switch(counter){
    case 2:
    	led_charging(leds_tmp, current_led, counter - 1);
    	return false;
    case 3:
    	led_charging(leds_tmp, current_led, counter - 1);
    	return false;
    case 4:
    	led_charging(leds_tmp, current_led, counter - 1);
    	return false;
    case 5:
    	led_charging(leds_tmp, current_led, counter - 1);
		set_instruction_flow(cardinal_dir, get_instruction_counter());
		if (get_instruction_counter() != 15){
			increase_instruction_counter();
		}
		return true;
	}
    return false;
}


/**
 * @brief				Retrieval of the accelerometer x and y-values and use of a threshold.
 * 						After a set counter, modification of instruction_flow using getters and setters.
 *
 * @param imu_values	pointer to the message containing the IMU measurements.
 *
 * @return              none
 *
*/

static void show_gravity(imu_msg_t *imu_values){

    //we create variables for the led in order to turn them off at each loop and to
    //select which one to turn on
	uint8_t leds[4] = {0,0,0,0};


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

    if(fabs(accel[X_AXIS]) > XY_THRESHOLD || fabs(accel[Y_AXIS]) > XY_THRESHOLD){

        chSysLock();
        //clockwise angle in rad with 0 being the back of the e-puck2 (Y axis of the IMU)
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
            if(led_counter(leds, counter, 2, SOUTH)){
            	counter = 0;
            }
        	previous_led = LED_5;
        }else if(angle >= M_PI/2 && angle < M_PI){
            if(previous_led == LED_7){
            	++counter;
            }else{
            	counter=0;
            	leds[3] = 0;
            }
            if(led_counter(leds, counter, 3, WEST)){
            	counter = 0;
            }
        	previous_led = LED_7;
        }else if(angle >= -M_PI && angle < -M_PI/2){
            if(previous_led == LED_1){
            	++counter;
            }else{
            	counter=0;
            	leds[0] = 0;
            }
            if(led_counter(leds, counter, 0, NORTH)){
            	counter = 0;
            }
        	previous_led = LED_1;
        }else if(angle >= -M_PI/2 && angle < 0){
            if(previous_led == LED_3){
            	++counter;
            }else{
            	counter=0;
            	leds[1] = 0;
            }
            if(led_counter(leds, counter, 1, EST)){
            	counter = 0;
            }
        	previous_led = LED_3;
        }
    }

    //we invert the values because the led is turned on if the signal is low
    palWritePad(GPIOD, GPIOD_LED1, leds[0] ? 0 : 1);
    palWritePad(GPIOD, GPIOD_LED3, leds[1] ? 0 : 1);
    palWritePad(GPIOD, GPIOD_LED5, leds[2] ? 0 : 1);
    palWritePad(GPIOD, GPIOD_LED7, leds[3] ? 0 : 1);

}


/*===========================================================================*/
/* Module threads.                                                           */
/*===========================================================================*/

/**
 * @brief               Thread in charge of the retrieval of the instructions given by the user.
 *
 * @return              none
 *
*/

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


/*===========================================================================*/
/* Module exported functions.                                                */
/*===========================================================================*/

/**
 * @brief               Initializes the Instruction generation thread.
 *
 * @return              none
 *
*/

void instruct_gen_init(void){
	chThdCreateStatic(waInstructionFlowThread, sizeof(waInstructionFlowThread), NORMALPRIO, InstructionFlowThread, NULL);
}

