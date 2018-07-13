
#include "ch.h"
#include "hal.h"

//#include "params.h"
#include "canBusProcess.h"
#include "dbus.h"

#include "math_misc.h"

#include "feeder.h"
#include "preload.h"

#define FEEDER_CAN &CAND1
#define FEEDER_CAN_EID 0x200

static RC_Ctl_t * p_rc;

#define feeder_canUpdate() \
    (can_motorSetCurrent(FEEDER_CAN, FEEDER_CAN_EID,\
        feeder_set_speed, 0, 0, 0))

#define feeder_canStop() \
    (can_motorSetCurrent(FEEDER_CAN, FEEDER_CAN_EID,\
        0, 0, 0, 0))


//static volatile int16_t feeder_set_speed;

static int16_t bullet_count = 0;
//static uint8_t bullet_threshold;

//static ChassisEncoder_canStruct*    feeder_encode;
//static BarrelStatus_canStruct*      heat_encode;

//static feeder_pid_struct feeder_pid;
//static feeder_motorPosStruct feeder_motor;

//static lpfilterStruct lp_spd_feeder;

//static systime_t bullet_in_time;
static systime_t bullet_out_time;

//static feeder_states feeder_state;

//#define BULLET_IN_TIME_OUT 20U
#define BULLET_OUT_TIME_OUT 20U
/*void bullet_in(void){
    if( ST2MS( chVTGetSystemTimeX() - bullet_in_time) > BULLET_IN_TIME_OUT ){
        if(bullet_count < 0)
            bullet_count = 0;
        else
            bullet_count++;
    }
    bullet_in_time = chVTGetSystemTimeX();
}*/

void bullet_out(void){
    if( (ST2MS( chVTGetSystemTimeX() - bullet_out_time)) > BULLET_OUT_TIME_OUT ){
        set_bullet_out();
        bullet_count++;
        /*if(bullet_count < 0)
            bullet_count = 0;*/
    }
    bullet_out_time = chVTGetSystemTimeX();
}

/*
static void feeder_encoder_update(void){
    if(feeder_encode->updated){
        feeder_encode->updated = false;

        feeder_motor._pos = (float) feeder_encode->total_ecd;
        feeder_motor._speed = (float) feeder_encode->raw_speed;
        feeder_motor._speed = lpfilter_apply(&lp_spd_feeder, feeder_motor._speed);

        feeder_motor._wait_count = 1;
    }
    else{
        feeder_motor._wait_count++;
        if(feeder_motor._wait_count > UNCONNECTED_MAX){
            feeder_canStop();
            feeder_motor._wait_count = 1;
            feeder_state = FEEDER_ERROR;
        }
    }
}*/


/*#define FEEDER_OUTPUT_MAX (1000.0f)
static volatile int16_t gripper_controlPos
        (const feeder_motorPosStruct* const motor, feeder_pid_struct* const controller)
{
    float error = motor->pos_sp - motor->_pos;

    controller->inte += error * controller->ki;
    controller->inte = boundOutput(controller->inte, controller->inte_max);


    float output =
            error*controller->kp + controller->inte - motor->_speed * controller->kd;


    output = output >  (FEEDER_OUTPUT_MAX)?  (FEEDER_OUTPUT_MAX):output;
    output = output < (-FEEDER_OUTPUT_MAX)? (-FEEDER_OUTPUT_MAX):output;

    return (int16_t) output;
}*/

static inline void Can_send_bullet_mouse(CANDriver *const CANx, const uint16_t SID)
{
    CANTxFrame txmsg;
    Can_send_bullet_mouse_struct txCan;

    txmsg.IDE = CAN_IDE_STD;
    txmsg.SID = CAN_BULLET_SID;
    txmsg.RTR = CAN_RTR_DATA;
    txmsg.DLC = 0x08;

    chSysLock();
    txCan.RIGHT = p_rc->rc.channel0;
    txCan.LEFT = p_rc->rc.channel1;
    txCan.bullet = bullet_count;

    memcpy(&(txmsg.data8), &txCan ,8);
    chSysUnlock();

    canTransmit(CANx, CAN_ANY_MAILBOX, &txmsg, MS2ST(100));
}



#define FEEDER_UPDATE_PERIOD_US 1000000/GRIPPER_CONTROL_FREQ
static THD_WORKING_AREA(feeder_pid_control_wa, 512);
static THD_FUNCTION(feeder_pid_control, p){
    (void) p;

    /*while(feeder_state != FEEDER_INITED){
        chThdSleepMilliseconds(5);
    }*/
    uint32_t tick = chVTGetSystemTimeX();
    while(true){
        tick += US2ST(FEEDER_UPDATE_PERIOD_US);
        if(tick > chVTGetSystemTimeX())
            chThdSleepUntil(tick);
        else
        {
            tick = chVTGetSystemTimeX();
        }

        Can_send_bullet_mouse(BULLET_CAN, CAN_BULLET_SID);

        /*feeder_encoder_update();

        feeder_set_speed = gripper_controlPos(&feeder_motor, &feeder_pid);
        if(feeder_state == FEEDER_INITED)
            feeder_canUpdate();
        else
            feeder_canStop();*/
    }
}


/*static THD_WORKING_AREA(feeder_control_wa, 256);
static THD_FUNCTION(feeder_control, p){
    (void) p;

    feeder_state = FEEDER_INITING;
    while( !(feeder_encode->updated) ){
        chThdSleepMilliseconds(5);
    };

    feeder_encoder_update();
    feeder_motor.pos_sp = feeder_motor._pos;
    feeder_state = FEEDER_INITED;

    while(!chThdShouldTerminateX()){

        switch(heat_encode->heatLimit){
            case HEATLIMIT_LVL_1:{
                bullet_threshold = BULLET_NUM_1;
            }
            case HEATLIMIT_LVL_2:{
                bullet_threshold = BULLET_NUM_2;
            }
            case HEATLIMIT_LVL_3:{
                bullet_threshold = BULLET_NUM_3;
            }
        }

        if(bullet_count >= bullet_threshold){
            chThdSleepMilliseconds(5);
            continue;
        }
        else{
            feeder_motor.pos_sp += FeederMotorStep;
        }

        chThdSleepMilliseconds(5);
    }
}*/

void feederInit(void){

    //feeder_state = FEEDER_UINITED;
    //bullet_in_time = chVTGetSystemTimeX();
    bullet_out_time = chVTGetSystemTimeX();

    /*feeder_encode = can_getfeederMotor();
    heat_encode = can_getHeatValue();
    bullet_threshold = BULLET_NUM_1;
    feeder_motor.pos_sp = 0.0f;
    feeder_motor._pos = 0.0f;
    feeder_motor._wait_count = 0;
    feeder_motor.speed_sp = 0.0f;
    feeder_motor._speed = 0.0f;

    feeder_pid.kp = 0.001f;
    feeder_pid.ki = 0.0f;
    feeder_pid.kd = 0.0f;
    feeder_pid.inte = 0.0f;
    feeder_pid.inte_max = 0.0f;

    lpfilter_init(&lp_spd_feeder, GRIPPER_CONTROL_FREQ, 24);

    chThdCreateStatic(feeder_control_wa, sizeof(feeder_control_wa),
                      NORMALPRIO-2, feeder_control, NULL);
    */

    p_rc = RC_get();
    chThdCreateStatic(feeder_pid_control_wa, sizeof(feeder_pid_control_wa),
                      NORMALPRIO-2, feeder_pid_control, NULL);


}
