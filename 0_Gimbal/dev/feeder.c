#include "ch.h"
#include "hal.h"

//#include "params.h"
#include "canBusProcess.h"
#include "dbus.h"

#include "math_misc.h"

#include "feeder.h"
#include "preload.h"

static RC_Ctl_t * p_rc;

static int16_t bullet_count = 0;
static systime_t bullet_out_time;

#define BULLET_OUT_TIME_OUT 20U

void bullet_out(void){
    if( (ST2MS( chVTGetSystemTimeX() - bullet_out_time)) > BULLET_OUT_TIME_OUT ){
        set_bullet_out();
        bullet_count++;
        /*if(bullet_count < 0)
            bullet_count = 0;*/
    }
    bullet_out_time = chVTGetSystemTimeX();
}

static inline void Can_send_bullet_mouse(CANDriver *const CANx, const uint16_t SID)
{
    CANTxFrame txmsg;
    Can_send_bullet_mouse_struct txCan;

    txmsg.IDE = CAN_IDE_STD;
    txmsg.SID = CAN_BULLET_SID;
    txmsg.RTR = CAN_RTR_DATA;
    txmsg.DLC = 0x08;

    chSysLock();
    txCan.RIGHT = p_rc->mouse.RIGHT;
    txCan.LEFT = p_rc->mouse.LEFT;
    txCan.bullet = bullet_count;

    memcpy(&(txmsg.data8), &txCan ,8);
    chSysUnlock();

    canTransmit(CANx, CAN_ANY_MAILBOX, &txmsg, MS2ST(100));
}



#define FEEDER_UPDATE_PERIOD_US 1000000/GRIPPER_CONTROL_FREQ
static THD_WORKING_AREA(feeder_pid_control_wa, 512);
static THD_FUNCTION(feeder_pid_control, p){
    (void) p;

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
    }
}

void feederInit(void){

    bullet_out_time = chVTGetSystemTimeX();

    p_rc = RC_get();
    chThdCreateStatic(feeder_pid_control_wa, sizeof(feeder_pid_control_wa),
                      NORMALPRIO-2, feeder_pid_control, NULL);


}
