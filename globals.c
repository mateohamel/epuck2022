/*
 * globals.c
 *
 *  Created on: 14 mai 2022
 *      Author: Romane
 */



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
