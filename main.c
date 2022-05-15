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

#include "selection.h"
#include "generation.h"
#include "detect_obstacles.h"
#include "move.h"
#include "globals.h"



int main(void)
{
    halInit();
    chSysInit();
    motors_init();

    msgbus_init();
	proximity_start();
    imu_start();
    chThdSleepMilliseconds(2000);

    mode_select_init();
    instruct_gen_init();
    move_init();
    detect_obst_init();

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
