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
#include <sensors/proximity.h>
#include <math.h>

#include "selector.h"

#include <arm_math.h>

//define for accelerometer Instruction Flow
#define NONE '0'
#define LED_1 '1'
#define LED_3 '3'
#define LED_5 '5'
#define LED_7 '7'
#define TURNING_SPEED 158
#define SPEED_FACTOR 60
#define SENSORS_THRESHOLD 5



typedef enum {BLANK, NORTH, EST, SOUTH, WEST} instruction;

typedef enum {NOP, FORWARD, RIGHT,LEFT} direction;

instruction Instruction_Flow[15] = {0};
static uint8_t Instruction_Counter = 0;

//define for Mode selection
#define MODE_INIT 0 //mode de démarrage
#define MODE_1 1 //rentrée d'instruction
#define MODE_2 2 //excecution d'instruction
#define MODE_3 3 //obstacle detecté

uint8_t Mode = MODE_INIT;

#define ON 1
#define OFF 0

messagebus_t bus;
MUTEX_DECL(bus_lock);
CONDVAR_DECL(bus_condvar);

static direction route[35];
static uint8_t size = 0;

instruction get_instruction_flow(uint8_t index){
	return g_instruction_flow[index];
}

void set_instruction_flow(instruction new_instruction, uint8_t index){
	g_instruction_flow[index] = new_instruction;
	return;
}


uint8_t get_instruction_counter(void){
	return g_instruction_counter;
}

void set_instruction_counter(uint8_t new_instruction_counter){
	g_instruction_counter = new_instruction_counter;
	return;
}

void increase_instruction_counter(void){
	g_instruction_counter++;
	return;
}


direction get_route(uint8_t index){
	return g_route[index];
}

void set_route(direction new_direction, uint8_t index){
	g_route[index] = new_direction;
	return;
}


uint8_t get_route_counter(void){
	return g_route_counter;
}

void set_route_counter(uint8_t new_route_counter){
	g_route_counter = new_route_counter;
	return;
}

void increase_route_counter(void){
	g_route_counter++;
	return;
}


mode get_mode(void){
	return g_mode;
}

void set_mode(mode new_mode){
	g_mode = new_mode;
	return;
}


void go(direction dir, uint8_t motor_speed){

	switch(dir){

	case(NOP) :
		break;
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
				break;
            case 3:
            	led_charging(leds, 2, 2);
				break;
            case 4:
            	led_charging(leds, 2, 3);
				break;
            case 5:
            	led_charging(leds, 2, 4);
				Instruction_Flow[Instruction_Counter] = SOUTH;
				if (Instruction_Counter != 15){
					++Instruction_Counter;
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
				Instruction_Flow[Instruction_Counter] = WEST;
				if (Instruction_Counter != 15){
					++Instruction_Counter;
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
				Instruction_Flow[Instruction_Counter] = NORTH;
				if (Instruction_Counter != 15){
					++Instruction_Counter;
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
				Instruction_Flow[Instruction_Counter] = EST;
				if (Instruction_Counter != 15){
					++Instruction_Counter;
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

void translation(void){

	switch(Instruction_Flow[0]){

	case(BLANK) :
		break;
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

void obstacle_detection(proximity_msg_t *prox_values){

	//arrêter les moteurs si IR1 ou IR8 détectent un obstacle
	if ((prox_values->ambient[0] - prox_values->reflected[0] > 100) && (prox_values->ambient[7] - prox_values->reflected[7] > 100)){
		left_motor_set_speed(0);
		right_motor_set_speed(0);
		Mode = MODE_3;
		Instruction_Counter = 0;
		Instruction_Flow[0] = BLANK;
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

	if(fabs(accell[2/*Z-AXIS*/]) < threshold_zh && fabs(accell[2/*Z-AXIS*/]) > threshold_zl && !(fabs(accell[X_AXIS]) > threshold_xy || fabs(accell[Y_AXIS]) > threshold_xy )){
		if(Mode == MODE_1){ //MODE_1
			if(counter == 8 && !current_state){
				chThdSleepMilliseconds(500);
				Mode = MODE_2;
				translation();
				palWritePad(GPIOB, GPIOB_LED_BODY, ON ? 0 : 1);
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
				Mode = MODE_1;
				Instruction_Counter = 0;
				Instruction_Flow[0] = BLANK;
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
		chThdSleepMilliseconds(300);
	}
}


static THD_WORKING_AREA(waInstructionExecutionThread, 128);
static THD_FUNCTION(InstructionExecutionThread, arg) {

	chRegSetThreadName(__FUNCTION__);
	(void)arg;


	while(1){
		if(Mode == MODE_2){
			//function execute instruction
			uint8_t i = 0;

			while((i < size) && (Mode == MODE_2)){
		        uint8_t motor_speed = get_selector()*SPEED_FACTOR;
		        go(route[i], motor_speed);
		        i++;
		        chThdSleepMilliseconds(2000);
			}
		}
		left_motor_set_speed(0);
		right_motor_set_speed(0);
		size = 0;

		chThdSleepMilliseconds(2000);
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
		switch(Mode){
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


int main(void)
{
    halInit();
    chSysInit();

    /** Inits the Inter Process Communication bus. */
    messagebus_init(&bus, &bus_lock, &bus_condvar);

    imu_start(); //appel i2c_start() + lance la thread du imu.
    proximity_start();
    motors_init();



    chThdSleepMilliseconds(2000);

    chThdCreateStatic(waModeSelectionThread, sizeof(waModeSelectionThread), NORMALPRIO, ModeSelectionThread, NULL); //j'arrive pas a changer la prio sans que ca bug
	chThdCreateStatic(waInstructionFlowThread, sizeof(waInstructionFlowThread), NORMALPRIO, InstructionFlowThread, NULL);
	chThdCreateStatic(waInstructionExecutionThread, sizeof(waInstructionExecutionThread), NORMALPRIO, InstructionExecutionThread, NULL);
	chThdCreateStatic(waDetectObstaclesThread, sizeof(waDetectObstaclesThread), NORMALPRIO, DetectObstaclesThread, NULL);


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
