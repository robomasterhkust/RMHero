
#include "ch.h"
#include "hal.h"

#include "shoot.h"
#include "dbus.h"
#include "gimbal.h"


static RC_Ctl_t* rc;

extern PWMDriver PWMD9;
extern PWMDriver PWMD4;

int shooter_set_speed;

int get_shooter_speed(void){
    return shooter_set_speed;
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
        if(get_screen_mode() == 1){
            pwmEnableChannel(&PWMD9, 0, PWM_PERCENTAGE_TO_WIDTH(&PWMD9, 5000));
        }
        else{
            pwmEnableChannel(&PWMD9, 0, PWM_PERCENTAGE_TO_WIDTH(&PWMD9, 8500));
        }

        chThdSleepMilliseconds(200);
    }
}

void shooter_init(void)
{


    rc = RC_get();

    pwmStart(&PWMD9,&pwm9cfg);
    pwmStart(&PWMD4,&pwm4cfg);

    chThdCreateStatic(pwm_thd_wa, sizeof(pwm_thd_wa), NORMALPRIO + 1, pwm_thd, NULL);

}
