/*
 * globals.c
 *
 *  Created on: 14 mai 2022
 *      Author: Romane
 */

#include "msgbus/messagebus.h"
#include "ch.h"
#include "globals.h"

/** Array containing the instructions given to the e-puck. */
instruction g_instruction_flow[MAX_INSTRUCTIONS] = {0};

/** Counter keeping track of how many instructions were given.  */
uint8_t g_instruction_counter = 0;

/** Array containing the route the e-puck has to follow.  */
direction g_route[MAX_DIRECTIONS];

/** Counter keeping track of how many directions have to be followed.  */
uint8_t g_route_counter = 0;

mode g_mode = INIT;

messagebus_t bus;
MUTEX_DECL(bus_lock);
CONDVAR_DECL(bus_condvar);

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

void msgbus_init(void){
	/** Inits the Inter Process Communication bus. */
    messagebus_init(&bus, &bus_lock, &bus_condvar);
}
