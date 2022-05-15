#ifndef GLOBALS_H
#define GLOBALS_H

#include "msgbus/messagebus.h"

#ifdef __cplusplus
extern "C" {
#endif



/** Enum used for the the instructions. */
typedef enum {NO_INSTRUCTION, NORTH, EST, SOUTH, WEST} instruction;

/** Enum used for the direction in which the e-puck has to go.  */
typedef enum {NO_DIRECTION, STRAIGHT, RIGHT,LEFT} direction;

/** Enum used for the modes .  */
typedef enum {INIT, MODE_1, MODE_2, MODE_3} mode;

#define MAX_INSTRUCTIONS	10
#define MAX_DIRECTIONS		3 * MAX_INSTRUCTIONS


instruction get_instruction_flow(uint8_t index);
void set_instruction_flow(instruction new_instruction, uint8_t index);

uint8_t get_instruction_counter(void);
void set_instruction_counter(uint8_t new_instruction_counter);
void increase_instruction_counter(void);

direction get_route(uint8_t index);
void set_route(direction new_direction, uint8_t index);

uint8_t get_route_counter(void);
void set_route_counter(uint8_t new_route_counter);
void increase_route_counter(void);

mode get_mode(void);
void set_mode(mode new_mode);

void msgbus_init(void);

#ifdef __cplusplus
}
#endif

#endif /* GLOBALS_H */
