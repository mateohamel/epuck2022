/*
 * supplementary functions.c
 *
 *  Created on: 15 mai 2022
 *      Author: Romane
 */


void turn_right(void){
	set_route(RIGHT, get_route_counter());
	increase_route_counter();
	set_route(STRAIGHT, get_route_counter());
	increase_route_counter();
}

void turn_left(void){
	set_route(LEFT, get_route_counter());
	increase_route_counter();
	set_route(STRAIGHT, get_route_counter());
	increase_route_counter();
}

void turn_back(void){
	set_route(RIGHT, get_route_counter());
	increase_route_counter();
	set_route(RIGHT, get_route_counter());
	increase_route_counter();
	set_route(STRAIGHT, get_route_counter());
	increase_route_counter();
}

void go_straight(void){
	set_route(STRAIGHT, get_route_counter());
	increase_route_counter();
}
