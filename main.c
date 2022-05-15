/**
 * @file    main.c
 * @brief   Main file. Handles interactions between modules.
 */


// e-puck main processor headers

#include <sensors/imu.h>
#include <sensors/proximity.h>
#include <motors.h>


// Module headers

#include "main.h"
#include "m_collect_instr.h"
#include "m_detect_obst.h"
#include "m_execute_dir.h"
#include "m_globals.h"
#include "m_select_mode.h"

/*===========================================================================*/
/* Module local functions.                                                   */
/*===========================================================================*/

/**
 * @brief             Initializes every module.
 */

static void modules_init(void){
    halInit();
    chSysInit();
    motors_init();
    msgbus_init();
	proximity_start();
    imu_start();
}


/**
 * @brief             Initializes every Thread.
 */

static void threads_init(void){
    mode_select_init();
    instruct_gen_init();
    move_init();
    detect_obst_init();
}


/*===========================================================================*/
/* Main function.                                                   		 */
/*===========================================================================*/

int main(void){
	modules_init();
    chThdSleepMilliseconds(2000);
    threads_init();

    while(1){
    	chThdSleepMilliseconds(100);
    }
}


#define STACK_CHK_GUARD 0xe2dee396
uintptr_t __stack_chk_guard = STACK_CHK_GUARD;

void __stack_chk_fail(void){
    chSysHalt("Stack smashing detected");
}
