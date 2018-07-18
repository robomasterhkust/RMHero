
#include "ch.h"
#include "hal.h"

#include "HeroPWM.h"
#include "dbus.h"

#include "barrelStatus.h"




static int bullet_in_count;
static int bullet_num;
static systime_t bullet_in_time;



#define BULLET_IN_TIME_OUT 20U
void bullet_in(void){
    if( ST2MS( chVTGetSystemTimeX() - bullet_in_time) > BULLET_IN_TIME_OUT ){
        //if(bullet_in_count < 0)
          //  bullet_in_count = 0;
        //else
            bullet_in_count++;
    }
    bullet_in_time = chVTGetSystemTimeX();
}



extern PWMDriver PWMD4;

static int set_feeder;

static int threshold;


static uint8_t mode;

static volatile Can_send_bullet_mouse_struct* p_bullet_out;

static pBarrelStatus p_heat;


static const PWMConfig pwm4cfg = {
        500000,
        1000,
        NULL,
        {
                {PWM_OUTPUT_ACTIVE_HIGH, NULL},
                {PWM_OUTPUT_ACTIVE_HIGH, NULL},
                {PWM_OUTPUT_DISABLED, NULL},
                {PWM_OUTPUT_DISABLED, NULL}
        },
        0,
        0
};

static THD_WORKING_AREA(pwm_thd_wa, 512);
static THD_FUNCTION(pwm_thd, arg) {
    (void)arg;

    pwmEnableChannel(&PWMD4, 0, PWM_PERCENTAGE_TO_WIDTH(&PWMD4, 5000));
    pwmEnableChannel(&PWMD4, 1, PWM_PERCENTAGE_TO_WIDTH(&PWMD4, 7500-150));
    chThdSleepMilliseconds(1000);
    while(1){
        pwmEnableChannel(&PWMD4, 0, PWM_PERCENTAGE_TO_WIDTH(&PWMD4, 5700));
        pwmEnableChannel(&PWMD4, 1, PWM_PERCENTAGE_TO_WIDTH(&PWMD4, set_feeder-150));

        chThdSleepMilliseconds(10);
    }
}




systime_t reverse_start_time;

static THD_WORKING_AREA(pwm_control_thd_wa, 512);
static THD_FUNCTION(pwm_control_thd, arg) {


    uint32_t tick = chVTGetSystemTimeX();
    while(1){

        tick += MS2ST(2);
        if(tick > chVTGetSystemTimeX())
            chThdSleepUntil(tick);
        else
        {
            tick = chVTGetSystemTimeX();
        }

        switch (p_heat->heatLimit){
            case HEATLIMIT_LVL_1:
                threshold = 2;
                break;
            case HEATLIMIT_LVL_2:
                threshold = 2;
                break;
            case HEATLIMIT_LVL_3:
                threshold = 2;
                break;
        }


        bullet_num = bullet_in_count - p_bullet_out->bullet;

        if(bullet_num < threshold){
            if(mode == 1 && ST2MS( chVTGetSystemTimeX() - reverse_start_time ) < 1000){
                set_feeder = 6000;
            }
            else if(mode == 1 && ST2MS( chVTGetSystemTimeX() - reverse_start_time ) >= 1000){
                set_feeder = 8000;
            }
            else{
                mode = 1;
                reverse_start_time = chVTGetSystemTimeX();
                set_feeder = 6000;
            }
        }
        else{
            mode = 0;
            set_feeder = 7500;
        }


    }


}

void HeroPWM_init(void)
{

    bullet_in_time = chVTGetSystemTimeX();

    mode = 0;

    bullet_in_count = 0;
    set_feeder = 7500;
    threshold = 2;
    bullet_num = 0;

    p_bullet_out = can_get_sent_bullet_mouse();
    p_heat = barrelStatus_get();

    pwmStart(&PWMD4,&pwm4cfg);

    chThdCreateStatic(pwm_thd_wa, sizeof(pwm_thd_wa), NORMALPRIO + 1, pwm_thd, NULL);
    chThdCreateStatic(pwm_control_thd_wa, sizeof(pwm_control_thd_wa), NORMALPRIO + 1, pwm_control_thd, NULL);

}
