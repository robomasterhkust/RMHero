
#include "ch.h"
#include "hal.h"

#include "shoot.h"
#include "dbus.h"
#include "gimbal.h"


static RC_Ctl_t* rc;

extern PWMDriver PWMD9;
extern PWMDriver PWMD4;

int shooter_set_speed;
int last_set_speed;
int shoot_off;
systime_t shoot_off_tick;

int get_shooter_speed(void){
    if( ST2MS(chVTGetSystemTimeX() - shoot_off_tick) > 300 && shooter_set_speed != 5000){
        return 1;
    }
    return 0;
}

static const PWMConfig pwm9cfg = {
        500000,
        1000,
        NULL,
        {
          {PWM_OUTPUT_ACTIVE_HIGH, NULL},
          {PWM_OUTPUT_DISABLED, NULL},
          {PWM_OUTPUT_DISABLED, NULL},
          {PWM_OUTPUT_DISABLED, NULL}
        },
        0,
        0
};

static const PWMConfig pwm4cfg = {
        500000,
        1000,
        NULL,
        {
                {PWM_OUTPUT_ACTIVE_HIGH, NULL},
                {PWM_OUTPUT_DISABLED, NULL},
                {PWM_OUTPUT_DISABLED, NULL},
                {PWM_OUTPUT_DISABLED, NULL}
        },
        0,
        0
};

static THD_WORKING_AREA(pwm_thd_wa, 512);
static THD_FUNCTION(pwm_thd, arg) {
    (void)arg;

    pwmEnableChannel(&PWMD9, 0, PWM_PERCENTAGE_TO_WIDTH(&PWMD9, 5000));
    pwmEnableChannel(&PWMD9, 1, PWM_PERCENTAGE_TO_WIDTH(&PWMD9, 5000));
    chThdSleepMilliseconds(50);

    while(1){
        last_set_speed = shooter_set_speed;
        if(get_screen_mode() == 1){
            shooter_set_speed = 5000;
        }
        else{
            shooter_set_speed = 8500;
        }
        if(last_set_speed = 5000 && shooter_set_speed != 5000){
            shoot_off_tick = chVTGetSystemTimeX();
        }
        pwmEnableChannel(&PWMD9, 0, PWM_PERCENTAGE_TO_WIDTH(&PWMD9, shooter_set_speed));
        chThdSleepMilliseconds(200);
    }
}

void shooter_init(void)
{

    shoot_off_tick = chVTGetSystemTimeX();
    last_set_speed = shooter_set_speed = shoot_off = 0;
    rc = RC_get();

    pwmStart(&PWMD9,&pwm9cfg);
    pwmStart(&PWMD4,&pwm4cfg);

    chThdCreateStatic(pwm_thd_wa, sizeof(pwm_thd_wa), NORMALPRIO + 1, pwm_thd, NULL);

}
