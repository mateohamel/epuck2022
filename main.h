#ifndef MAIN_H
#define MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

#include "camera/dcmi_camera.h"
#include "msgbus/messagebus.h"
#include "parameter/parameter.h"


/** Robot wide IPC bus. */
extern messagebus_t bus;

extern parameter_namespace_t parameter_root;


//typedef enum {BLANK, NORTH, EST, SOUTH, WEST} instruction;
//
//typedef enum {NOP, FORWARD, RIGHT,LEFT} direction;
//
//instruction Instruction_Flow[15];
//uint8_t Instruction_Counter;
//
//direction route[35];
//uint8_t size;
//
////define for Mode selection
//#define MODE_INIT 0 //mode de démarrage
//#define MODE_1 1 //rentrée d'instruction
//#define MODE_2 2 //excecution d'instruction
//#define MODE_3 3 //obstacle detecté
//
//uint8_t Mode;


#ifdef __cplusplus
}
#endif

#endif
