/**
 * @file    m_select_mode.c
 * @brief   Module File. Handles the interactions between the different modes.
 */

// C library

#include <math.h>


// e-puck main processor headers

#include <sensors/imu.h>
#include <motors.h>
#include <msgbus/messagebus.h>


// Module headers

#include "m_select_mode.h"
#include "m_globals.h"


/*===========================================================================*/
/* Module constants.                                                         */
/*===========================================================================*/

#define ON 					1
#define OFF					0
#define	W_TO_N				-3
#define	S_TO_N_OR_W_TO_E	-2
#define	TO_THE_LEFT			-1
#define	GO_STRAIGHT			0
#define	TO_THE_RIGHT		1
#define	N_TO_S_OR_E_TO_W	2
#define	N_TO_W				3
#define ZH_THRESHOLD		16
#define ZL_THRESHOLD		9
#define XY_THRESHOLD		3
#define DELAY				8



/*===========================================================================*/
/* Bus related declarations.                                                 */
/*===========================================================================*/

extern messagebus_t bus;


/*===========================================================================*/
/* Module local functions.                                                   */
/*===========================================================================*/

/**
 * @brief               Will add to g_route the directions to go to, depending of turning left or right
 *
 * @param	dir			left or right
 *
 * @return              none
 *
*/

void turn(direction dir){
	set_route(dir, get_route_counter());
	increase_route_counter();
	set_route(STRAIGHT, get_route_counter());
	increase_route_counter();
}

/**
 * @brief               Will add to g_route the directions to go to in order to turn back
 *
 * @return              none
 *
*/

void turn_back(void){
	set_route(RIGHT, get_route_counter());
	increase_route_counter();
	set_route(RIGHT, get_route_counter());
	increase_route_counter();
	set_route(STRAIGHT, get_route_counter());
	increase_route_counter();
}

/**
 * @brief               Will add to g_route the direction to go to in order to go straight
 *
 * @return              none
 *
*/

void go_straight(void){
	set_route(STRAIGHT, get_route_counter());
	increase_route_counter();
}



/**
 * @brief               Translation of instructions gotten from accelerometer into actual directions
 * 						in which the robot will have to go.
 * 						It is modifying the global variable 'route' using getter and setter.
 *
 * @return              none
 *
*/

static void translation(void){

	switch(get_instruction_flow(0)){

	case NO_INSTRUCTION :
		break;

	case NORTH :
		go_straight();
		break;

	case WEST :
		turn(LEFT);
		break;

	case SOUTH :
		turn_back();
		break;

	case EST :
		turn(RIGHT);
		break;
	}

	for(int i=1; i<get_instruction_counter() ; i++){

		switch(get_instruction_flow(i)-get_instruction_flow(i-1)){

		case W_TO_N :
			turn(RIGHT);
			break;

		case S_TO_N_OR_W_TO_E :
			turn_back();
			break;

		case TO_THE_LEFT :
			turn(LEFT);
			break;

		case GO_STRAIGHT :
			go_straight();
			break;

		case TO_THE_RIGHT :
			turn(RIGHT);
			break;

		case N_TO_S_OR_E_TO_W :
			turn_back();
			break;

		case N_TO_W :
			turn(LEFT);
			break;
		}
	}
}


/**
 * @brief               Using the z-values of the accelerometer to detect and change the robot's mode.
 * 						When changing the mode, conditions are modified using getters and setters.
 *
 * @param imu_values	pointer to the message containing the IMU measurement.
 *
 * @return              none
 *
*/

static void Mode_Detection(imu_msg_t *imu_values){


	//create a pointer to the array for shorter name
	float *accell = imu_values->acceleration;
    static uint8_t counter = 0;
    static bool current_state = false;

	if(fabs(accell[Z_AXIS]) < ZH_THRESHOLD && fabs(accell[Z_AXIS]) > ZL_THRESHOLD && !(fabs(accell[X_AXIS]) > XY_THRESHOLD || fabs(accell[Y_AXIS]) > XY_THRESHOLD )){
		if(get_mode() == MODE_1){ //MODE_1
			if(counter == DELAY && !current_state){
				chThdSleepMilliseconds(500);
				set_mode(MODE_2);
				palWritePad(GPIOB, GPIOB_LED_BODY, ON ? 0 : 1);
				translation();
				counter = 0;
				current_state = true;
			}else{
				if(counter > DELAY){
					counter = 0;
				}
				++counter;
			}
		}else{ //MODE_2 or MODE_3
			if(counter == DELAY && !current_state){
				set_mode(MODE_1);
				set_instruction_counter(0);
				set_instruction_flow(0,NO_INSTRUCTION);
				palWritePad(GPIOB, GPIOB_LED_BODY, OFF ? 0 : 1);
				counter = 0;
				current_state = true;
				left_motor_set_speed(NO_SPEED);
				right_motor_set_speed(NO_SPEED);
			}else{
				if(counter > DELAY){
					counter = 0;
				}
				++counter;
			}
		}
	}else{
		current_state = false;
	}
}


/*===========================================================================*/
/* Module threads.                                                           */
/*===========================================================================*/

/**
 * @brief               Thread in charge of detecting a mode transition and initializing the robot in consequences.
 *
 * @return              none
 *
*/

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


/*===========================================================================*/
/* Module exported functions.                                                */
/*===========================================================================*/

/**
 * @brief               Initializes the mode selection thread
 *
 * @return              none
 *
*/

void mode_select_init(void){
    chThdCreateStatic(waModeSelectionThread, sizeof(waModeSelectionThread), NORMALPRIO, ModeSelectionThread, NULL);
}



