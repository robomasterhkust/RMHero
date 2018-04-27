//
// Created by XU Xinyuan on 22/4/2018.
//
#include <main.h>
#include "ch.h"
#include "hal.h"
#include "auto_fetch_task.h"



thread_reference_t auto_fetch_thread_ref = NULL;
static volatile  Auto_Fetch_Task_Struct AFT_struct;

volatile Auto_Fetch_Task_Struct * get_AFT_struct(){
    return &AFT_struct;
}

void load_state(){
    AFT_struct.BACK_SWITCH = palReadPad(GPIOA,GPIOA_PIN0_EXTI);
    AFT_struct.LEFT_SWITCH = palReadPad(GPIOA,GPIOA_PIN1_EXIT);
    AFT_struct.RIGHT_SWITCH = palReadPad(GPIOA,GPIOA_PIN3_EXIT);
    AFT_struct.running = (AFT_struct.BACK_SWITCH== 1)&&(AFT_struct.LEFT_SWITCH== 1);
}



void chassis_move(){
    LEDG6_ON();
    chThdSleepMilliseconds(100);
    LEDG6_OFF();
    chThdSleepMilliseconds(100);
}

void fetch_process(){
    LEDG5_ON();
    chThdSleepMilliseconds(1000);
    LEDG5_OFF();
}

/*TODO:use CAN to estimate position, double integral IMU is not fesible
 *
 * */
static THD_WORKING_AREA(auto_fetch_thread_position_wa, 512);
static THD_FUNCTION(auto_fetch_thread_position,ptr){
    LEDG2_TOGGLE();
    float vx=0;
    AFT_struct.x_i=0;
    while(!chThdShouldTerminateX()){
        AFT_struct.p_IMU = imu_get();
        vx += AFT_struct.p_IMU->accelData[X]*INTEGRAL_dt_ms/1000; // m/s=mm/ms
        AFT_struct.x_i += vx * INTEGRAL_dt_ms;
        chThdSleepMilliseconds(INTEGRAL_dt_ms);
    }
}



void aft_process_controller(){
    thread_t* p_position_thread= chThdCreateStatic(auto_fetch_thread_position_wa, sizeof(auto_fetch_thread_position_wa),
                           NORMALPRIO+1, auto_fetch_thread_position, NULL);
    while(AFT_struct.x_i<AFT_X_SETPOINT_mm)chassis_move();
    fetch_process();
    while(AFT_struct.x_i<AFT_X_SETPOINT_mm)chassis_move();
    fetch_process();
    while(AFT_struct.x_i<AFT_X_SETPOINT_mm)chassis_move();
    fetch_process();
    chThdTerminate(p_position_thread);
}

static THD_WORKING_AREA(auto_fetch_thread_wa, 512);
static THD_FUNCTION(auto_fetch_thread, ptr)
{
    LEDG8_TOGGLE();
    while (!chThdShouldTerminateX()) {
        chSysLock();
        chThdSuspendS(&auto_fetch_thread_ref);
        chSysUnlock();
        auto_fetch_thread_ref = NULL;
        load_state();//read switch state and determine whether start process.
        AFT_struct.running = true; //FOR DEBUG!!!
        if(AFT_struct.running) aft_process_controller();
        AFT_struct.running = false;
    }
}

void auto_fetch_task_init(void)
{
    AFT_struct.init = true;
    AFT_struct.running = false;
    chThdCreateStatic(auto_fetch_thread_wa, sizeof(auto_fetch_thread_wa),
                    NORMALPRIO, auto_fetch_thread, NULL);
}