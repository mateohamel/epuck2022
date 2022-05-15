/**
 * @file    m_globals.c
 * @brief   Complementary File. Creation of getters and setters of our global variables.
 * 			Globals source and header files are used to give robustness to the project even with
 * 			the use of global variables.
 */


// e-puck main processor headers

#include <ch.h>
#include <msgbus/messagebus.h>


// Module headers

#include <m_globals.h>


/*===========================================================================*/
/* Project global variables.                                                 */
/*===========================================================================*/

/** Array containing the instructions given to the e-puck. */
instruction g_instruction_flow[MAX_INSTRUCTIONS] = {0};


/** Counter keeping track of how many instructions were given.  */
uint8_t g_instruction_counter = 0;


/** Array containing the route the e-puck has to follow.  */
direction g_route[MAX_DIRECTIONS];


/** Counter keeping track of how many directions have to be followed.  */
uint8_t g_route_counter = 0;


/** Variable representing the current mode of the robot.	*/
mode g_mode = INIT;


/*===========================================================================*/
/* Bus related declarations.                                                 */
/*===========================================================================*/

messagebus_t bus;
MUTEX_DECL(bus_lock);
CONDVAR_DECL(bus_condvar);


/*===========================================================================*/
/* Module exported functions.                                                */
/*===========================================================================*/

/**
 * @brief               Getter and Setter for the table of instruction: 'g_instruction_flow'.
 *
 * @return              Getter: g_instruction_flow		Setter: none
 */

instruction get_instruction_flow(uint8_t index){
	return g_instruction_flow[index];
}

void set_instruction_flow(instruction new_instruction, uint8_t index){
	g_instruction_flow[index] = new_instruction;
	return;
}


/**
 * @brief               Getter and Setter for the counter of the previous table: 'g_instruction_counter'.
 * 						The increase function increments the counter by one.
 *
 * @return              Getter: g_instruction_counter 	Setter: none
 */

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


/**
 * @brief               Getter and Setter for the table of directions: 'g_route'.
 *
 * @return              Getter: g_route					Setter: none
 */

direction get_route(uint8_t index){
	return g_route[index];
}

void set_route(direction new_direction, uint8_t index){
	g_route[index] = new_direction;
	return;
}


/**
 * @brief               Getter and Setter for the counter of the previous table: 'g_route_counter'.
 * 						The increase function increments the counter by one.
 *
 * @return              Getter: g_route_counter 	Setter: none
 */

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


/**
 * @brief               Getter and Setter for the current mode of the robot: 'g_mode'.
 *
 * @return              Getter: g_mode					Setter: none
 */

mode get_mode(void){
	return g_mode;
}

void set_mode(mode new_mode){
	g_mode = new_mode;
	return;
}


/**
 * @brief               Initialize the Inter Process Communication bus.
 *
 * @return              none
 */

void msgbus_init(void){
    messagebus_init(&bus, &bus_lock, &bus_condvar);
}
