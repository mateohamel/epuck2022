/**
 * @file    m_globals.h
 * @brief   External declarations of getters and setters of our global variables.
 * 			Globals source and header files are used to give robustness to the project even with
 * 			the use of global variables.
 */

#ifndef GLOBALS_H
#define GLOBALS_H


#ifdef __cplusplus
extern "C" {
#endif


/*===========================================================================*/
/* Projects constants.                                                         */
/*===========================================================================*/

/** Enum used for the the instructions. */
typedef enum {NO_INSTRUCTION, NORTH, EST, SOUTH, WEST} instruction;


/** Enum used for the direction in which the e-puck has to go.  */
typedef enum {NO_DIRECTION, STRAIGHT, RIGHT,LEFT} direction;


/* Enum used for the mode .  									*/
/*  INIT  : 	Initial mode of the robot at startup. 			*/
/*  MODE_1: The retrieval of instructions given by the user.	*/
/*  MODE_2: The execution of the instructions given.			*/
/*  MODE_3: The Panic mode when an obstacle has been detected.	*/
typedef enum {INIT, MODE_1, MODE_2, MODE_3} mode;


#define MAX_INSTRUCTIONS 	10
#define MAX_DIRECTIONS 		3 * MAX_INSTRUCTIONS // the longest instruction is turning back
												 // and it takes 3 directions


/*===========================================================================*/
/* External declarations.                                                    */
/*===========================================================================*/

/**
 * @brief               Getter and Setter for the table of instruction: 'g_instruction_flow'.]
 *
 * @return              Getter: g_instruction_flow		Setter: none
 */

instruction get_instruction_flow(uint8_t index);
void set_instruction_flow(instruction new_instruction, uint8_t index);


/**
 * @brief               Getter and Setter for the counter of the previous table: 'g_instruction_counter'.
 * 						The increase function ables to increment the counter.
 *
 * @return              Getter: g_instruction_counter 	Setter: none
 */

uint8_t get_instruction_counter(void);
void set_instruction_counter(uint8_t new_instruction_counter);
void increase_instruction_counter(void);


/**
 * @brief               Getter and Setter for the table of directions: 'g_route'.
 *
 * @return              Getter: g_route					Setter: none
 */

direction get_route(uint8_t index);
void set_route(direction new_direction, uint8_t index);


/**
 * @brief               Getter and Setter for the counter of the previous table: 'g_route_counter'.
 * 						The increase function ables to increment the counter.
 *
 * @return              Getter: g_route_counter 	Setter: none
 */

uint8_t get_route_counter(void);
void set_route_counter(uint8_t new_route_counter);
void increase_route_counter(void);


/**
 * @brief               Getter and Setter for the current mode of the robot: 'g_mode'.
 *
 * @return              Getter: g_mode					Setter: none
 */

mode get_mode(void);
void set_mode(mode new_mode);


/**
 * @brief               Initialize the Inter Process Communication bus.
 *
 * @return              none
 */

void msgbus_init(void);


#ifdef __cplusplus
}
#endif

#endif /* GLOBALS_H */
