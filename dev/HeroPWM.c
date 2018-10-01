
#include "ch.h"
#include "hal.h"

#include "HeroPWM.h"
#include "dbus.h"

#include "keyboard.h"

#include "barrelStatus.h"


#define FEEDER_CAN &CAND1
#define FEEDER_EID 0x1ff


static int bullet_in_count;
static int bullet_num;
static systime_t bullet_in_time;

static uint8_t bool_bullet_in;

int* p_bit_map;

int bullet_num_get(){
    return bullet_num;
}


#define BULLET_IN_TIME_OUT 50U
void bullet_in(void){
    if( ST2MS( chVTGetSystemTimeX() - bullet_in_time) > BULLET_IN_TIME_OUT ){
        //if(bullet_in_count < 0)
          //  bullet_in_count = 0;
        //else
            bullet_in_count++;
    }
    bullet_in_time = chVTGetSystemTimeX();
}


static int set_feeder;

static int threshold;


static uint8_t mode;

static volatile Can_send_bullet_mouse_struct* p_bullet_out;

static pBarrelStatus p_heat;

static volatile ChassisEncoder_canStruct* p_motors;

static volatile Feeder_motorPosStruct Feeder_Motors[Feeder_Motor_NUM];

static lpfilterStruct lp_feeder[Feeder_Motor_NUM];

static feeder_pid_controller_t feeder_pid_struct[Feeder_Motor_NUM];

static const int16_t outputmax[Feeder_Motor_NUM] = {10000, 12000};

static feeder_output_struct feeder_output;



static void hero_feeder_encoderUpdate(void)
{

    uint8_t i;
    for (i = 0; i < Feeder_Motor_NUM; i++)
    {
        if(p_motors[i].updated)
        {
            //Check validiaty of can connection
            p_motors[i].updated = false;

            Feeder_Motors[i]._speed = (float) p_motors[i].raw_speed;
            Feeder_Motors[i]._speed = lpfilter_apply(&lp_feeder[i], Feeder_Motors[i]._speed);

            Feeder_Motors[i]._wait_count = 1;
        }
        else
        {
            Feeder_Motors[i]._wait_count++;
            if(Feeder_Motors[i]._wait_count > 100)
            {
                Feeder_Motors[i]._wait_count = 1;
                can_motorSetCurrent(FEEDER_CAN, FEEDER_EID, 0, 0, 0, 0);
            }
        }

    }

}



static int16_t Feeder_controlSpeed
        (const Feeder_motorPosStruct* const motor, feeder_pid_controller_t* const controller,
         const int16_t output_max)
{
    float error = motor->speed_sp - motor->_speed;
    float output;

    controller->error_int += error;
    if(controller->error_int > 30000.0f)
        controller->error_int = 30000.0f;
    if(controller->error_int < -30000.0f)
        controller->error_int = -30000.0f;

    output = error*controller->kp + (controller->error_int) * controller->ki;  /*- motor->_speed * controller->kd*/;

    output = output >  ((float)output_max)?  ((float)output_max):output;
    output = output < ((float)-output_max)? ((float)-output_max):output;

    return (int16_t) output;
}



uint8_t selector_error;

uint16_t selector_error_count;

static int16_t target_speed[2];

#define FEEDER_UPDATE_PERIOD_US  1000000/FEEDER_CONTROL_FREQ

static THD_WORKING_AREA(pwm_thd_wa, 512);
static THD_FUNCTION(pwm_thd, arg) {
    (void)arg;


    int16_t output[Feeder_Motor_NUM];

    uint32_t tick = chVTGetSystemTimeX();
    while(1){

        tick += MS2ST(2);
        if(tick > chVTGetSystemTimeX())
            chThdSleepUntil(tick);
        else
        {
            tick = chVTGetSystemTimeX();
        }


        hero_feeder_encoderUpdate();



        float diff = Feeder_Motors[Selector]._speed - target_speed[Selector];


        if(selector_error == 0){
            if( diff > -200.0f && diff < 200.0f){
                //selector_error_count++;
                if(selector_error_count > 0)
                    selector_error_count--;
            }
            else
                selector_error_count++;
        }


        if(selector_error_count >= 500){
            selector_error = 1;
            selector_error_count = 0;
        }

        uint8_t i;
        for( i = 0 ; i < Feeder_Motor_NUM ; i++){
            Feeder_Motors[i].speed_sp = target_speed[i];
            feeder_output.output[i] = Feeder_controlSpeed(&Feeder_Motors[i], &feeder_pid_struct[i], outputmax[i]);
        }

        can_motorSetCurrent(FEEDER_CAN, FEEDER_EID, feeder_output.output[0], feeder_output.output[1], 0, 0);


    }
}


static uint8_t last_R_press = 0;
static uint8_t R_press = 0;

systime_t reverse_start_time;

static THD_WORKING_AREA(pwm_control_thd_wa, 512);
static THD_FUNCTION(pwm_control_thd, arg) {


    uint16_t reverse_count = 0;
    uint32_t tick = chVTGetSystemTimeX();
    while(1){

        tick += US2ST(FEEDER_UPDATE_PERIOD_US);
        if(tick > chVTGetSystemTimeX())
            chThdSleepUntil(tick);
        else
        {
            tick = chVTGetSystemTimeX();
        }


        if(p_heat->heatLimit <= HEATLIMIT_LVL_1){
            threshold = 2;
        }
        else if(p_heat->heatLimit <= HEATLIMIT_LVL_2){
            threshold = 3;
        }
        else if(p_heat->heatLimit <= HEATLIMIT_LVL_3){
            threshold = 4;
        }
        else{
            threshold = 2;
        }


        last_R_press = R_press;
        R_press = p_bit_map[KEY_R];

        if(last_R_press == 1 && R_press == 0){
            bullet_in_count = p_bullet_out->bullet;
        }
        if(bullet_in_count < p_bullet_out->bullet){
            bullet_in_count = p_bullet_out->bullet;
        }


        bullet_num = bullet_in_count - p_bullet_out->bullet;

        if(bullet_num < threshold){
            target_speed[Selector] = 700;
        }
        else{
            target_speed[Selector] = 0;
        }

        if(target_speed[Selector] == 700){
            if(selector_error == 1){
                reverse_count++;
                if(reverse_count == 800){
                    selector_error = 0;
                    reverse_count = 0;
                }
                target_speed[Selector] = -700;
            }
            else{
                target_speed[Selector] = 700;
            }
        }
    }


}

void HeroPWM_init(void)
{

    bool_bullet_in = 0;

    bullet_in_time = chVTGetSystemTimeX();

    p_bit_map = Bitmap_get();

    uint8_t i;
    for(i = 0 ; i < Feeder_Motor_NUM ; i++){
        lpfilter_init( &(lp_feeder[i]), FEEDER_CONTROL_FREQ, 24);

    }
    feeder_pid_struct[0].kp = 3.5f;
    feeder_pid_struct[0].ki = 0.01f;
    feeder_pid_struct[0].kd = 0.0f;
    feeder_pid_struct[0].error_int = 0.0f;
    feeder_pid_struct[0].error_int_max = 0.0f;

    feeder_pid_struct[1].kp = 10.0f;
    feeder_pid_struct[1].ki = 0.0f;
    feeder_pid_struct[1].kd = 0.0f;
    feeder_pid_struct[1].error_int = 0.0f;
    feeder_pid_struct[1].error_int_max = 0.0f;


    target_speed[0] = 0;
    target_speed[1] = 0;


    selector_error_count = 0;
    selector_error = 0;
    mode = 0;

    bullet_in_count = 0;
    set_feeder = 7500;
    threshold = 2;
    bullet_num = 0;

    p_bullet_out = can_get_sent_bullet_mouse();
    p_heat = barrelStatus_get();

    p_motors = can_getChassisMotor();


    chThdCreateStatic(pwm_thd_wa, sizeof(pwm_thd_wa), NORMALPRIO + 1, pwm_thd, NULL);
    chThdCreateStatic(pwm_control_thd_wa, sizeof(pwm_control_thd_wa), NORMALPRIO + 1, pwm_control_thd, NULL);

}
