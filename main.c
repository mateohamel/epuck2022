#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "ch.h"
#include "hal.h"
#include <main.h>
#include <chprintf.h>
#include <motors.h>
#include <sensors/imu.h>
#include <msgbus/messagebus.h>

#include <arm_math.h>

//define for accelerometer Instruction Flow
#define FORWARD 'F'
#define BACKWARD 'B'
#define LEFT 'L'
#define RIGHT 'R'
#define NONE '0'
#define LED_1 '1'
#define LED_3 '3'
#define LED_5 '5'
#define LED_7 '7'

char Instruction_Flow[15] = {0};
uint8_t Instruction_Counter = 0;

//define for Mode selection
#define MODE_1 1 //rentrée d'instruction
#define MODE_2 2 //excecution d'instruction

uint8_t Mode = 2;


messagebus_t bus;
MUTEX_DECL(bus_lock);
CONDVAR_DECL(bus_condvar);

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
//	uint8_t led1 = 0, led3 = 0, led5 = 0, led7 = 0;
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
//            	leds[2] = 1;
//				leds[3] = 0;
//				leds[0] = 0;
//				leds[1] = 0;
				break;
            case 3:
            	led_charging(leds, 2, 2);
//            	leds[2] = 1;
//            	leds[3] = 1;
//            	leds[0] = 0;
//				leds[1] = 0;
				break;
            case 4:
            	led_charging(leds, 2, 3);
//            	leds[2] = 1;
//            	leds[3] = 1;
//            	leds[0] = 1;
//				leds[1] = 0;
				break;
            case 5:
            	led_charging(leds, 2, 4);
//            	leds[2] = 1;
//            	leds[3] = 1;
//            	leds[0] = 1;
//				leds[1] = 1;
				Instruction_Flow[Instruction_Counter] = BACKWARD;
				++Instruction_Counter;
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
            	leds[3] = 1;
				leds[0] = 0;
				leds[1] = 0;
				leds[2] = 0;
				break;
            case 3:
	            leds[3] = 1;
				leds[0] = 1;
				leds[1] = 0;
				leds[2] = 0;
				break;
            case 4:
	            leds[3] = 1;
				leds[0] = 1;
				leds[1] = 1;
				leds[2] = 0;
				break;
            case 5:
	            leds[3] = 1;
				leds[0] = 1;
				leds[1] = 1;
				leds[2] = 1;
				Instruction_Flow[Instruction_Counter] = LEFT;
				++Instruction_Counter;
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
				leds[0] = 1;
				leds[1] = 0;
				leds[2] = 0;
				leds[3] = 0;
				break;
			case 3:
				leds[0] = 1;
				leds[1] = 1;
				leds[2] = 0;
				leds[3] = 0;
				break;
			case 4:
				leds[0] = 1;
				leds[1] = 1;
				leds[2] = 1;
				leds[3] = 0;
				break;
			case 5:
				leds[0] = 1;
				leds[1] = 1;
				leds[2] = 1;
				leds[3] = 1;
				Instruction_Flow[Instruction_Counter] = FORWARD;
				++Instruction_Counter;
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
				leds[1] = 1;
            	leds[2] = 0;
				leds[3] = 0;
				leds[0] = 0;
				break;
			case 3:
    			leds[1] = 1;
				leds[2] = 1;
				leds[3] = 0;
				leds[0] = 0;
				break;
			case 4:
				leds[1] = 1;
				leds[2] = 1;
				leds[3] = 1;
				leds[0] = 0;
				break;
			case 5:
				leds[1] = 1;
				leds[2] = 1;
				leds[3] = 1;
				leds[0] = 1;
				Instruction_Flow[Instruction_Counter] = RIGHT;
				++Instruction_Counter;
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

void Mode_Detection(imu_msg_t *imu_values){

	float threshold_zh = 16;
	float threshold_zl = 10;
	float threshold_xy = 2.5;
	//create a pointer to the array for shorter name
	float *accell = imu_values->acceleration;
    static uint8_t counter = 0;

	if(fabs(accell[2/*Z-AXIS*/]) < threshold_zh && fabs(accell[2/*Z-AXIS*/]) > threshold_zl && !(fabs(accell[X_AXIS]) > threshold_xy || fabs(accell[Y_AXIS]) > threshold_xy )){
		if(Mode == 1){
			if(counter == 8){
				Mode = MODE_2;
				uint8_t bodyled =1;
				palWritePad(GPIOB, GPIOB_LED_BODY, bodyled ? 0 : 1);
				counter = 0;
			}else{
			++counter;
			}
		}else{
			if(counter == 8){
				Mode = MODE_1;
				uint8_t bodyled =0;
				palWritePad(GPIOB, GPIOB_LED_BODY, bodyled ? 0 : 1);
				counter = 0;
			}else{
				++counter;
			}
		}
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


static THD_WORKING_AREA(waInstructionFlowThread, 128);
static THD_FUNCTION(InstructionFlowThread, arg) {

	chRegSetThreadName(__FUNCTION__);
	(void)arg;

	messagebus_topic_t *imu_topic = messagebus_find_topic_blocking(&bus, "/imu");
	imu_msg_t imu_values;

	while(1){

		if(Mode == MODE_1){
			//wait for new measures to be published
			messagebus_topic_wait(imu_topic, &imu_values, sizeof(imu_values));

			show_gravity(&imu_values);
		}
		chThdSleepMilliseconds(400);
	}
}


int main(void)
{

    halInit();
    chSysInit();

    imu_start(); //appel i2c_start() + lance la thread du imu.

    chThdCreateStatic(waModeSelectionThread, sizeof(waModeSelectionThread), NORMALPRIO, ModeSelectionThread, NULL);
	chThdCreateStatic(waInstructionFlowThread, sizeof(waInstructionFlowThread), NORMALPRIO, InstructionFlowThread, NULL);


    /** Inits the Inter Process Communication bus. */
    messagebus_init(&bus, &bus_lock, &bus_condvar);

    while(1){
    	chThdSleepMilliseconds(100);
    }

}

#define STACK_CHK_GUARD 0xe2dee396
uintptr_t __stack_chk_guard = STACK_CHK_GUARD;

void __stack_chk_fail(void)
{
    chSysHalt("Stack smashing detected");
}
