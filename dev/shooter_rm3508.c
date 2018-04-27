#include "stdint.h"
#include "shooter_rm3508.h"
#include "ch.h"
#include "hal.h"

#include "dbus.h"
#include "judge.h"

RC_Ctl_t *p_dbus_shooter;
game_fb_t* p_game_fb;
projectile_fb_t* p_projectile;
power_fb_t* p_power_fb;

judge_fb_t* p_judge_fb; 

uint8_t level = 1;
uint16_t golfheat;
float bulletSpeed;

float speed_max = 16.5f;
float heat_max[4] = {0, 80, 160, 320};

extern PWMDriver PWMD12;

static PWMConfig test_pwm12cfg = {
        500000,                                    /* 1MHz PWM clock frequency.   */
        1000,                                       /* Initial PWM period 1ms.     */
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

void shooter_info_update(void);

static THD_WORKING_AREA(shooter_wa, 512);

static THD_FUNCTION(shooter_thd, p)
{
    judge_fb_t judge_fb = judgeDataGet();
    p_judge_fb = &judge_fb;
    p_projectile = &(p_judge_fb->projectileInfo);
    p_game_fb = &(p_judge_fb->gameInfo);
    p_power_fb = &(p_judge_fb->powerInfo);

    

    int pwm_width = 6000;

    pwmEnableChannel(&PWMD12, 0, PWM_PERCENTAGE_TO_WIDTH(&PWMD12, pwm_width));
    pwmEnableChannel(&PWMD12, 1, PWM_PERCENTAGE_TO_WIDTH(&PWMD12, pwm_width));

    while (!chThdShouldTerminateX()) {
/**
 * Comment: RC control, the other is slow rotation
 */
/*if(p_dbus_shooter->rc.s2 == 1){
    pwmEnableChannel(&PWMD12, 0, PWM_PERCENTAGE_TO_WIDTH(&PWMD12, 7200));
    pwmEnableChannel(&PWMD12, 1, PWM_PERCENTAGE_TO_WIDTH(&PWMD12, 7200));
    //pwmEnableChannel(&PWMD12, 2, PWM_PERCENTAGE_TO_WIDTH(&PWMD12, 6000));
    //pwmEnableChannel(&PWMD12, 3, PWM_PERCENTAGE_TO_WIDTH(&PWMD12, 6000));
    chThdSleepMilliseconds(200);
}
else{
    pwmEnableChannel(&PWMD12, 0, PWM_PERCENTAGE_TO_WIDTH(&PWMD12, 5000));
    pwmEnableChannel(&PWMD12, 1, PWM_PERCENTAGE_TO_WIDTH(&PWMD12, 5000));
    //pwmEnableChannel(&PWMD12, 2, PWM_PERCENTAGE_TO_WIDTH(&PWMD12, 6000));
    //pwmEnableChannel(&PWMD12, 3, PWM_PERCENTAGE_TO_WIDTH(&PWMD12, 6000));
    chThdSleepMilliseconds(200);
}*/     shooter_info_update();
        
        while((bulletSpeed>=speed_max)||golfheat>heat_max[level]){
            pwm_width -= 100;
            pwmEnableChannel(&PWMD12, 0, PWM_PERCENTAGE_TO_WIDTH(&PWMD12, pwm_width));
            pwmEnableChannel(&PWMD12, 1, PWM_PERCENTAGE_TO_WIDTH(&PWMD12, pwm_width));
            shooter_info_update();
        }

        if(pwm_width<6000){
            pwm_width += 100;
            pwmEnableChannel(&PWMD12, 0, PWM_PERCENTAGE_TO_WIDTH(&PWMD12, pwm_width));
            pwmEnableChannel(&PWMD12, 1, PWM_PERCENTAGE_TO_WIDTH(&PWMD12, pwm_width));
        }
        shooter_info_update();
    }
}

void shooter_info_update(void){

    bulletSpeed = p_projectile->bulletSpeed;
    golfheat = p_power_fb->shooterHeat1;
    level = p_game_fb->robotLevel;

}


void shooter_rm3508_init(void)
{

    pwmStart(&PWMD12, &test_pwm12cfg);

    int i = 0;
    for (i = 0; i < 120; i++) {
//        pwmStop(&PWMD12);
//        pwmStart(&PWMD12, &test_pwm12cfg);
        pwmEnableChannel(&PWMD12, 0, PWM_PERCENTAGE_TO_WIDTH(&PWMD12, i * 50));
        pwmEnableChannel(&PWMD12, 1, PWM_PERCENTAGE_TO_WIDTH(&PWMD12, i * 50));
        //pwmEnableChannel(&PWMD12, 2, PWM_PERCENTAGE_TO_WIDTH(&PWMD12, i * 50));
        //pwmEnableChannel(&PWMD12, 3, PWM_PERCENTAGE_TO_WIDTH(&PWMD12, i * 50));
        chThdSleepMilliseconds(50);
    }

    p_dbus_shooter = RC_get();


    chThdCreateStatic(shooter_wa, sizeof(shooter_wa),
                      NORMALPRIO, shooter_thd, NULL);
}
