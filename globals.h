#ifndef GLOBALS_H
#define GLOBALS_H

#ifdef __cplusplus
extern "C" {
#endif


#define MAX_INSTRUCTIONS	10
#define MAX_DIRECTIONS		3 * MAX_INSTRUCTIONS

/** Enum used for the the instructions. */
typedef enum {NO_INSTRUCTION, NORTH, EST, SOUTH, WEST} instruction;

/** Enum used for the direction in which the e-puck has to go.  */
typedef enum {NO_DIRECTION, FORWARD, RIGHT,LEFT} direction;

/** Enum used for the modes .  */
typedef enum {INIT, MODE_1, MODE_2, MODE_3} mode;

/** Array containing the instructions given to the e-puck. */
instruction g_instruction_flow[MAX_INSTRUCTIONS] = {0};

/** Counter keeping track of how many instructions were given.  */
static uint8_t g_instruction_counter = 0;

/** Array containing the route the e-puck has to follow.  */
static direction g_route[MAX_DIRECTIONS];

/** Counter keeping track of how many directions have to be followed.  */
static uint8_t g_route_counter = 0;

static mode g_mode = INIT;


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


#ifdef __cplusplus
}
#endif

#endif /* GLOBALS_H */
