#include "ch.h"
#include "hal.h"

#include "params.h"
#include "canBusProcess.h"
#include "dbus.h"

#include "math_misc.h"

#include "preload.h"


#define PRELOAD_CAN &CAND1
#define PRELOAD_CAN_EID 0x200

#define BOOST_CURRENT   -6000

#define preload_canBoost() \
    (can_motorSetCurrent(PRELOAD_CAN, PRELOAD_CAN_EID,\
        BOOST_CURRENT, 0, 0, 0))

#define preload_canUpdate() \
    (can_motorSetCurrent(PRELOAD_CAN, PRELOAD_CAN_EID,\
        preload_set_speed, 0, 0, 0))

#define preload_canStop() \
    (can_motorSetCurrent(PRELOAD_CAN, PRELOAD_CAN_EID,\
        0, 0, 0, 0))

static ChassisEncoder_canStruct*    preload_encode;
static BarrelStatus_canStruct*      heat_encode;
static RC_Ctl_t* pRC;

/*static*/ preload_states preload_state;

static volatile int16_t preload_set_speed;

static lpfilterStruct lp_spd_preload;

static preload_pid_struct preload_pid;
static preloadPosStruct   preload_motor;


//static thread_reference_t PreloadPID_thread_handler = NULL;

static uint8_t PreLoad_Bullet_Out;

void set_bullet_out(void){
    PreLoad_Bullet_Out = 1;
}



static void preload_encoder_update(void){
    if(preload_encode->updated){
        preload_encode->updated = false;

        preload_motor._pos = (float) preload_encode->total_ecd;
        preload_motor._speed = (float) preload_encode->raw_speed;
        preload_motor._speed = lpfilter_apply(&lp_spd_preload, preload_motor._speed);

        preload_motor._wait_count = 1;
    }
    else{
        preload_motor._wait_count++;
        if(preload_motor._wait_count > PreLoad_UnConnected_Max){
            preload_canStop();
            preload_motor._wait_count = 1;
            preload_state = PreLoadError;
        }
    }
}


#define PreLoad_OUTPUT_MAX (6000.0f)
static volatile int16_t preload_controlPos
        (const preloadPosStruct* const motor, preload_pid_struct* const controller)
{
    float error = motor->pos_sp - motor->_pos;

    controller->inte += error * controller->ki;
    controller->inte = boundOutput(controller->inte, controller->inte_max);


    float output =
            error*controller->kp + controller->inte - motor->_speed * controller->kd;


    output = output >  (PreLoad_OUTPUT_MAX)?  (PreLoad_OUTPUT_MAX):output;
    output = output < (-PreLoad_OUTPUT_MAX)? (-PreLoad_OUTPUT_MAX):output;

    return (int16_t) output;
}

#define FEEDER_UPDATE_PERIOD_US 1000000/PreLoad_CONTROL_FREQ
static THD_WORKING_AREA(preload_pid_control_wa, 512);
static THD_FUNCTION(preload_pid_control, p){

    while(preload_state == PreLoadIniting || preload_state == PreLoadUnInited){
        chThdSleepMilliseconds(5);
    }
    uint32_t tick = chVTGetSystemTimeX();
    while(!chThdShouldTerminateX()) {
        tick += US2ST(FEEDER_UPDATE_PERIOD_US);
        if (tick > chVTGetSystemTimeX())
            chThdSleepUntil(tick);
        else
            tick = chVTGetSystemTimeX();

        preload_encoder_update();

        if(preload_state == PreLoadError){
            chThdSleepMilliseconds(5);
            continue;
        }
        if(preload_state == PreLoadBoost){
            preload_pid.inte = 0.0f;
        }
        else{
            preload_set_speed = preload_controlPos(&preload_motor, &preload_pid);
            preload_canUpdate();
        }

    }
}


uint8_t prev_s = 0;

/*static */int bullet_time_out;

//extern systime_t bullet_out_time;

static THD_WORKING_AREA(preload_control_wa, 512);
static THD_FUNCTION(preload_control, p){

    (void) p;

    preload_state = PreLoadIniting;
    while( !(preload_encode->updated) ){
        chThdSleepMilliseconds(5);
    }
    preload_encoder_update();
    preload_motor.pos_sp = preload_motor._pos;
    preload_state = PreLoadStop;

    while(!chThdShouldTerminateX()){
        if(pRC->mouse.LEFT || (pRC->rc.s1 == 2 && prev_s == 3) ){
            systime_t start_time = chVTGetSystemTimeX();
            preload_state = PreLoadBoost;
            PreLoad_Bullet_Out = 0;
            preload_canBoost();
            while( PreLoad_Bullet_Out == 0 && ST2MS(chVTGetSystemTimeX() - start_time) < 1000 ){
                preload_canBoost();
                chThdSleepMicroseconds(500);
            }
            //bullet_time_out = ST2MS(bullet_out_time - start_time);
            preload_encoder_update();
            preload_motor.pos_sp = preload_motor._pos;
            preload_pid.inte = 0.0f;
            can_motorSetCurrent(PRELOAD_CAN, PRELOAD_CAN_EID, (BOOST_CURRENT*2/3), 0, 0, 0);
            chThdSleepMicroseconds(500);
            can_motorSetCurrent(PRELOAD_CAN, PRELOAD_CAN_EID, (BOOST_CURRENT/3), 0, 0, 0);
            chThdSleepMicroseconds(500);
            can_motorSetCurrent(PRELOAD_CAN, PRELOAD_CAN_EID, 0, 0, 0, 0);

            preload_state = PreLoadStop;
            //chThdResumeS(&PreloadPID_thread_handler, MSG_OK);
        }
        else{
            preload_state = PreLoadStop;
        }
        prev_s = pRC->rc.s1;
        chThdSleepMilliseconds(1);
    }



}


void preloadInit(void){

    preload_state = PreLoadUnInited;

    pRC = RC_get();
    preload_encode = can_getfeederMotor();
    heat_encode = can_getHeatValue();

    preload_pid.kp = 0.12f;
    preload_pid.ki = 0.0005f;
    preload_pid.kd = 0.5f;
    preload_pid.inte = 0.0f;
    preload_pid.inte_max = 100.0f;

    preload_motor.pos_sp = 0.0f;
    preload_motor._pos = 0.0f;
    preload_motor.speed_sp = 0.0f;
    preload_motor._speed = 0.0f;
    preload_motor._wait_count = 0;

    lpfilter_init(&lp_spd_preload, PreLoad_CONTROL_FREQ, 24);


    chThdCreateStatic(preload_control_wa, sizeof(preload_control_wa),
                      NORMALPRIO+2, preload_control, NULL);
    chThdCreateStatic(preload_pid_control_wa, sizeof(preload_pid_control_wa),
                      NORMALPRIO+2, preload_pid_control, NULL);

}