
#include "ch.h"
#include "hal.h"

#include "canBusProcess.h"
#include "dbus.h"
#include "math_misc.h"
#include "hero_gripper.h"

#include "keyboard.h"

#define HERO_GRIPPER_CAN &CAND1
#define HERO_GRIPPER_EID 0x1ff



float pos_sp__,pos__;uint8_t init_count = 0;
int stall__;
volatile float offset__;
volatile float prev__;
volatile float spin1,spin2,spin3;




static volatile Can_send_bullet_mouse_struct* p_mouse;
static volatile Gimbal_Send_Dbus_canStruct* p_rc;

static volatile uint8_t gripper_start;

static float offset[GRIPPER_MOTOR_NUM];
static hero_gripper gripper;

static lpfilterStruct lp_gripper[GRIPPER_MOTOR_NUM];

static ChassisEncoder_canStruct* hero_gripper_encoders;

static float lift_sp[3];
static float spin_sp[4];
static float hand_sp[2];

static gripper_pid_controller_t controllers[GRIPPER_MOTOR_NUM];
static gripper_pid_controller_t speed_controller;
static gripper_motorPosStruct   gripper_motors[GRIPPER_MOTOR_NUM];

static const int16_t gripper_output_max[GRIPPER_MOTOR_NUM] = {8000, 16000, 5000};
static const float GEAR_RATIO[GRIPPER_MOTOR_NUM] = {36.0f, 19.0f, 19.0f};
static const float Error_tolerance[GRIPPER_MOTOR_NUM] = {10.f, 12.0f, 180.0f};





uint8_t _state = GRIPPER_INITING; //= GRIPPER_UNINIT;
/*static */gripper_state running_state;
static uint8_t in_position[GRIPPER_MOTOR_NUM] = {0,0,0};
static uint8_t in_pos = 0;




#define STALL_COUNT_MAX 200U


static void hero_gripper_encoderUpdate(void);

static void hero_gripper_calibrate(void){
    bool init_state[GRIPPER_MOTOR_NUM] = {0,0,0};

    float prev_pos[GRIPPER_MOTOR_NUM];


    int stall_count[GRIPPER_MOTOR_NUM] = {0,0,0};

    int stall_max[GRIPPER_MOTOR_NUM] = {300,100 ,300};

    hero_gripper_encoderUpdate();


    const float motor_step[GRIPPER_MOTOR_NUM] = {-1000.0f, 50.0f, 1000.0f};
    while(init_count < GRIPPER_MOTOR_NUM)
    {

        stall__ = stall_count[2];
        uint8_t i;
        init_count = 0;
        for(i = 0; i < GRIPPER_MOTOR_NUM; i++){
            if(stall_count[i] < stall_max[i]){
                gripper_motors[i].pos_sp += motor_step[i];
                if(motor_step[i] > 0){
                    if(gripper_motors[i]._pos - prev_pos[i] < motor_step[i] * 0.01f){
                        stall_count[i]++;

                    }
                    else if(stall_count[i] > 10){
                        stall_count[i] -= 10;
                    }
                    else{
                        stall_count[i] = 0;
                    }
                }
                else{
                    if(gripper_motors[i]._pos - prev_pos[i] > ((motor_step[i] * 0.01f))  ){
                        stall_count[i]++;
                    }
                    else if(stall_count[i] > 10){
                        stall_count[i] -= 10;
                    }
                    else{
                        stall_count[i] = 0;
                    }
                }
                prev_pos[i] = gripper_motors[i]._pos;
                prev__ = prev_pos[1];
                stall__ = stall_count[1];

            }

            else{
                init_state[i] = true;
                in_position[i] = 1;
                offset[i] = gripper_motors[i]._pos;
            }

            init_count += init_state[i] ? 1 : 0;
        }

        chThdSleepMilliseconds(2);
    }

/*    gripper_motors[GRIPPER_LIFT_MOTOR].pos_sp = offset[GRIPPER_LIFT_MOTOR] + 60.0f * 19.0f / 360.0f * 8192.0f;
    gripper_motors[GRIPPER_ARM_MOTOR].pos_sp = offset[GRIPPER_ARM_MOTOR] + 60.0f * 19.0f / 360.0f * 8192.0f;
    gripper_motors[GRIPPER_HAND_MOTOR].pos_sp = offset[GRIPPER_HAND_MOTOR];
    offset__ = offset[GRIPPER_ARM_MOTOR];*/

    _state = GRIPPER_RUNNING;
    running_state = down_1_open;


}



static void hero_gripper_encoderUpdate(void)
{

    uint8_t i;
    uint8_t in_pos_count = 0;
    for (i = 0; i < GRIPPER_MOTOR_NUM; i++)
    {
        if(hero_gripper_encoders[i].updated)
        {
            //Check validiaty of can connection
            hero_gripper_encoders[i].updated = false;

            gripper_motors[i]._speed = (float) hero_gripper_encoders[i].raw_speed;
            gripper_motors[i]._speed = lpfilter_apply(&lp_gripper[i], gripper_motors[i]._speed);
            gripper_motors[i]._pos = (float) hero_gripper_encoders[i].total_ecd;

            pos_sp__ = gripper_motors[i].pos_sp;
            pos__ = gripper_motors[i]._pos;


            gripper_motors[i]._wait_count = 1;
        }
        else
        {
            gripper_motors[i]._wait_count++;
            if(gripper_motors[i]._wait_count > GRIPPER_CONNECTION_ERROR_COUNT)
            {
                gripper_motors[i]._wait_count = 1;
                can_motorSetCurrent(HERO_GRIPPER_CAN, HERO_GRIPPER_EID, 0, 0, 0, 0);
            }
        }

        float diff = gripper_motors[i]._pos - gripper_motors[i].pos_sp;
        if( diff < Error_tolerance[i] * GEAR_RATIO[i] / 360.0f * 8192.0f && diff > -Error_tolerance[i] * GEAR_RATIO[i] / 360.0f * 8192.0f )
        {
            in_position[i] = 1;
            in_pos_count++;
        }
        else
            in_position[i] = 0;

    }
    if(in_pos_count == GRIPPER_MOTOR_NUM)
        in_pos = 1;
    else
        in_pos = 0;

}


static int16_t gripper_controlSpeed
        (const gripper_motorPosStruct* const motor, gripper_pid_controller_t* const controller,
         const int16_t output_max)
{
    float error = motor->speed_sp - motor->_speed;
    float output;

    controller->error_int += error ;
    controller->error_int = boundOutput(controller->error_int, controller->error_int_max);


    output = error*controller->kp + controller->error_int * controller->ki - motor->_speed * controller->kd;


    output = output >  ((float)output_max)?  ((float)output_max):output;
    output = output < ((float)-output_max)? ((float)-output_max):output;

    return (int16_t) output;
}



static int16_t gripper_controlPos
        (gripper_motorPosStruct* const motor, gripper_pid_controller_t* const controller,
         const int16_t output_max)
{
    if(running_state == up_4_close){
        float error = motor->pos_sp - motor->_pos;
        float output;

        controller->error_int += error * 0.01f;
        controller->error_int = boundOutput(controller->error_int, controller->error_int_max);

        output = error*0.1f + controller->error_int - motor->_speed * 1.0f;


        output = output >  ((float)1000)?  ((float)1000):output;
        output = output < ((float)-1000)? ((float)-1000):output;

        motor->speed_sp = output;

        return (int16_t) gripper_controlSpeed(motor, &speed_controller, output_max);
    }
    else{
        float error = motor->pos_sp - motor->_pos;
        float output;

        controller->error_int += error * controller->ki;
        controller->error_int = boundOutput(controller->error_int, controller->error_int_max);

        if(motor->_speed > -1000.0f && motor->_speed < 1000.0f){
            output =
                    error*controller->kp + controller->error_int ;
        }
        else{
            output =
                    error*controller->kp + controller->error_int - motor->_speed * controller->kd;
        }


        output = output >  ((float)output_max)?  ((float)output_max):output;
        output = output < ((float)-output_max)? ((float)-output_max):output;

        return (int16_t) output;
    }


}


int16_t output_max[GRIPPER_MOTOR_NUM];
#define GRIPPER_UPDATE_PERIOD_US  1000000/GRIPPER_CONTROL_FREQ
static THD_WORKING_AREA(hero_gripper_control_wa, 512);
static THD_FUNCTION(hero_gripper_control, p)
{
    (void)p;

    uint32_t tick = chVTGetSystemTimeX();
    while(true){

        tick += US2ST(GRIPPER_UPDATE_PERIOD_US);
        if(tick > chVTGetSystemTimeX())
            chThdSleepUntil(tick);
        else
        {
            tick = chVTGetSystemTimeX();
        }

        hero_gripper_encoderUpdate();

        uint8_t i;
        for(i = 0; i < GRIPPER_MOTOR_NUM; i++){


            if( _state == GRIPPER_INITING)
            {
                output_max[GRIPPER_LIFT_MOTOR] = 2000;
                output_max[GRIPPER_ARM_MOTOR]  = 2500;
                output_max[GRIPPER_HAND_MOTOR] = 1500;
            }

            else {
                output_max[GRIPPER_LIFT_MOTOR] = gripper_output_max[GRIPPER_LIFT_MOTOR];
                output_max[GRIPPER_ARM_MOTOR]  = gripper_output_max[GRIPPER_ARM_MOTOR];
                output_max[GRIPPER_HAND_MOTOR] = gripper_output_max[GRIPPER_HAND_MOTOR];
            }

            gripper.output[i] = gripper_controlPos(&gripper_motors[i], &controllers[i], output_max[i]);

        }
        can_motorSetCurrent(HERO_GRIPPER_CAN, HERO_GRIPPER_EID,
                            gripper.output[GRIPPER_LIFT_MOTOR] , gripper.output[GRIPPER_ARM_MOTOR], gripper.output[GRIPPER_HAND_MOTOR], 0);

    }
}



uint8_t LEFT , RIGHT;
uint8_t prev_LEFT = 0, prev_RIGHT = 0;



static THD_WORKING_AREA(hero_rc_gripper_control_wa, 512);
static THD_FUNCTION(hero_rc_gripper_control, p)
{

    (void)p;


    chThdSleepMilliseconds(1000);

    hero_gripper_calibrate();


    uint8_t key_press[2] = {0,0};


    uint8_t s1, s2, prev_s2;


    systime_t down_start_time;
    systime_t step_start_time = chVTGetSystemTime();

    while(1){

        keyboard_hero_gripper_process(&gripper_start, key_press, p_rc);

        prev_LEFT = LEFT;
        prev_RIGHT = RIGHT;

        LEFT = p_mouse->LEFT;
        RIGHT = p_mouse->RIGHT;

        if(gripper_start == 0){
            running_state = down_1_open;
            gripper_motors[GRIPPER_LIFT_MOTOR].pos_sp = offset[GRIPPER_LIFT_MOTOR]  - lift_sp[LIFT_DOWN];
            gripper_motors[GRIPPER_ARM_MOTOR].pos_sp  = offset[GRIPPER_ARM_MOTOR]   - spin_sp[ARM_1];
            gripper_motors[GRIPPER_HAND_MOTOR].pos_sp = offset[GRIPPER_HAND_MOTOR]  - hand_sp[HAND_OPEN];
            chThdSleepMilliseconds(20);
            continue;
        }

        if(prev_RIGHT == 0 && RIGHT == 1){
            running_state = middle_2_open;
            gripper_motors[GRIPPER_LIFT_MOTOR].pos_sp = offset[GRIPPER_LIFT_MOTOR]  - lift_sp[LIFT_MIDDLE];
            gripper_motors[GRIPPER_ARM_MOTOR].pos_sp  = offset[GRIPPER_ARM_MOTOR]   - spin_sp[ARM_2];
            gripper_motors[GRIPPER_HAND_MOTOR].pos_sp = offset[GRIPPER_HAND_MOTOR]  - hand_sp[HAND_OPEN];
            continue;
        }

        //chSysLock();
        switch(running_state){
            case down_1_open:
            {
                running_state = middle_2_open;
                step_start_time = chVTGetSystemTime();
            }break;
            case middle_2_open:
            {
                //if(in_pos == 1){
                if(prev_LEFT == 0 && LEFT == 1){
                    running_state = middle_3_open;
                    step_start_time = chVTGetSystemTime();
                }
                //}

            }break;
            case middle_3_open:
            {
                if(in_pos == 0 && ( ST2MS(chVTGetSystemTime() - step_start_time)  > 1000 ) ){
                    running_state = middle_2_open;
                    step_start_time = chVTGetSystemTime();
                    break;
                }
                if(in_pos == 1 && ( prev_LEFT == 0 && LEFT == 1) ){
                    in_pos = 0;
                    running_state = middle_3_close;
                    step_start_time = chVTGetSystemTime();
                    break;
                }
            }break;
            case middle_3_close:
            {
                if(in_pos == 0 && ( ST2MS(chVTGetSystemTime() - step_start_time)  > 1000 ) ){
                    running_state = middle_3_open;
                    step_start_time = chVTGetSystemTime();
                    break;
                }
                if(in_pos == 1 && ( ST2MS(chVTGetSystemTime() - step_start_time) >  300) ){
                    in_pos = 0;
                    running_state = up_3_close1;
                    step_start_time = chVTGetSystemTime();
                    break;
                }
            }break;
            case up_3_close1:
            {
                if(in_pos == 0 && ( ST2MS(chVTGetSystemTime() - step_start_time)  > 1000 ) ){
                    running_state = middle_3_close;
                    step_start_time = chVTGetSystemTime();
                    break;
                }
                if(in_pos == 1 && ( ST2MS(chVTGetSystemTime() - step_start_time) >  300) ){
                    in_pos = 0;
                    running_state = up_4_close;
                    step_start_time = chVTGetSystemTime();
                    break;
                }
            }break;
            case up_4_close:
            {
                if(in_pos == 0 && ( ST2MS(chVTGetSystemTime() - step_start_time)  > 6000 ) ){
                    running_state = up_3_close1;
                    step_start_time = chVTGetSystemTime();
                    break;
                }
                if(in_pos == 1 && ( ST2MS(chVTGetSystemTime() - step_start_time) >  300) ){
                    in_pos = 0;
                    running_state = up_3_close2;
                    step_start_time = chVTGetSystemTime();
                    break;
                }
            }break;
            case up_3_close2:
            {
                if(in_pos == 0 && ( ST2MS(chVTGetSystemTime() - step_start_time)  > 1000 ) ){
                    running_state = middle_2_open;
                    step_start_time = chVTGetSystemTime();
                    break;
                }
                if(in_pos == 1 && ( ST2MS(chVTGetSystemTime() - step_start_time) >  300) ){
                    in_pos = 0;
                    running_state = up_3_open;
                    step_start_time = chVTGetSystemTime();
                    break;
                }
            }break;
            case up_3_open:
            {
                if(in_pos == 1 && ( ST2MS(chVTGetSystemTime() - step_start_time) >  300) ){
                    in_pos = 0;
                    running_state = middle_2_open;
                    step_start_time = chVTGetSystemTime();
                    break;
                }
                else if( ST2MS(chVTGetSystemTime() - step_start_time) >  2000){
                    running_state = middle_2_open;
                    step_start_time = chVTGetSystemTime();
                    break;
                }
            }break;
        }
        //chSysUnlock();

        chSysLock();
        switch(running_state){
            case down_1_open:
            {
                gripper_motors[GRIPPER_LIFT_MOTOR].pos_sp = offset[GRIPPER_LIFT_MOTOR]  - lift_sp[LIFT_DOWN];
                gripper_motors[GRIPPER_ARM_MOTOR].pos_sp  = offset[GRIPPER_ARM_MOTOR]   - spin_sp[ARM_1];
                gripper_motors[GRIPPER_HAND_MOTOR].pos_sp = offset[GRIPPER_HAND_MOTOR]  - hand_sp[HAND_OPEN];
            }break;
            case middle_2_open:
            {
                gripper_motors[GRIPPER_LIFT_MOTOR].pos_sp = offset[GRIPPER_LIFT_MOTOR]  - lift_sp[LIFT_MIDDLE];
                gripper_motors[GRIPPER_ARM_MOTOR].pos_sp  = offset[GRIPPER_ARM_MOTOR]   - spin_sp[ARM_2];
                gripper_motors[GRIPPER_HAND_MOTOR].pos_sp = offset[GRIPPER_HAND_MOTOR]  - hand_sp[HAND_OPEN];
            }break;
            case middle_3_open:
            {
                gripper_motors[GRIPPER_LIFT_MOTOR].pos_sp = offset[GRIPPER_LIFT_MOTOR]  - lift_sp[LIFT_MIDDLE];
                gripper_motors[GRIPPER_ARM_MOTOR].pos_sp  = offset[GRIPPER_ARM_MOTOR]   - spin_sp[ARM_3];
                gripper_motors[GRIPPER_HAND_MOTOR].pos_sp = offset[GRIPPER_HAND_MOTOR]  - hand_sp[HAND_OPEN];
            }break;
            case middle_3_close:
            {
                gripper_motors[GRIPPER_LIFT_MOTOR].pos_sp = offset[GRIPPER_LIFT_MOTOR]  - lift_sp[LIFT_MIDDLE];
                gripper_motors[GRIPPER_ARM_MOTOR].pos_sp  = offset[GRIPPER_ARM_MOTOR]   - spin_sp[ARM_3];
                gripper_motors[GRIPPER_HAND_MOTOR].pos_sp = offset[GRIPPER_HAND_MOTOR]  - hand_sp[HAND_CLOSE];
            }break;
            case up_3_close1:
            {
                gripper_motors[GRIPPER_LIFT_MOTOR].pos_sp = offset[GRIPPER_LIFT_MOTOR]  - lift_sp[LIFT_UP];
                gripper_motors[GRIPPER_ARM_MOTOR].pos_sp  = offset[GRIPPER_ARM_MOTOR]   - spin_sp[ARM_3];
                gripper_motors[GRIPPER_HAND_MOTOR].pos_sp = offset[GRIPPER_HAND_MOTOR]  - hand_sp[HAND_CLOSE];
            }break;
            case up_4_close:
            {
                gripper_motors[GRIPPER_LIFT_MOTOR].pos_sp = offset[GRIPPER_LIFT_MOTOR]  - lift_sp[LIFT_UP];
                gripper_motors[GRIPPER_ARM_MOTOR].pos_sp  = offset[GRIPPER_ARM_MOTOR]   - spin_sp[ARM_4];
                gripper_motors[GRIPPER_HAND_MOTOR].pos_sp = offset[GRIPPER_HAND_MOTOR]  - hand_sp[HAND_CLOSE];
            }break;
            case up_3_close2:
            {
                gripper_motors[GRIPPER_LIFT_MOTOR].pos_sp = offset[GRIPPER_LIFT_MOTOR]  - lift_sp[LIFT_UP];
                gripper_motors[GRIPPER_ARM_MOTOR].pos_sp  = offset[GRIPPER_ARM_MOTOR]   - spin_sp[ARM_3];
                gripper_motors[GRIPPER_HAND_MOTOR].pos_sp = offset[GRIPPER_HAND_MOTOR]  - hand_sp[HAND_CLOSE];
            }break;
            case up_3_open:
            {
                gripper_motors[GRIPPER_LIFT_MOTOR].pos_sp = offset[GRIPPER_LIFT_MOTOR]  - lift_sp[LIFT_UP];
                gripper_motors[GRIPPER_ARM_MOTOR].pos_sp  = offset[GRIPPER_ARM_MOTOR]   - spin_sp[ARM_3];
                gripper_motors[GRIPPER_HAND_MOTOR].pos_sp = offset[GRIPPER_HAND_MOTOR]  - hand_sp[HAND_OPEN];
            }break;
        }
        chSysUnlock();

        prev_s2 = s2;
        chThdSleepMilliseconds(20);
    }
}



#define GRIPPER_ERROR_INT_MAX 3000
void hero_gripper_Init(void){

    memset(&gripper_motors, 0, sizeof(gripper_motorPosStruct) * GRIPPER_MOTOR_NUM);

    hero_gripper_encoders = can_getChassisMotor();

    p_mouse = can_get_sent_bullet_mouse();
    p_rc = can_get_sent_dbus();

    gripper_start = 0;


    uint8_t i;
    for (i = 0; i < GRIPPER_MOTOR_NUM; i++) {
        lpfilter_init( &(lp_gripper[i]), GRIPPER_CONTROL_FREQ, 24);
        gripper_motors[i].speed_sp = 0.0f;
        gripper_motors[i]._speed = 0.0f;
        gripper_motors[i].pos_sp = 0.0f;
        gripper_motors[i]._pos = 0.0f;

        controllers[i].error_int = 0.0f;
        controllers[i].error_int_max = GRIPPER_ERROR_INT_MAX;
    }


    //gripper_motors[i].pos_sp = gripper_motors[i]._pos;

    controllers[GRIPPER_LIFT_MOTOR].kp = 0.50f;
    controllers[GRIPPER_LIFT_MOTOR].ki = 0.0015f;
    controllers[GRIPPER_LIFT_MOTOR].kd = 5.0f;

    controllers[GRIPPER_ARM_MOTOR].kp  = 0.70f;
    controllers[GRIPPER_ARM_MOTOR].ki  = 0.0001f;
    controllers[GRIPPER_ARM_MOTOR].kd  = 7.0f;

    speed_controller.kp = 7.0f;
    speed_controller.ki = 0.01f;
    speed_controller.kd = 0.0f;
    speed_controller.error_int = 0.0f;
    speed_controller.error_int_max = 1500000.0f;

    controllers[GRIPPER_HAND_MOTOR].kp = 0.30f;
    controllers[GRIPPER_HAND_MOTOR].ki = 0.0001f;
    controllers[GRIPPER_HAND_MOTOR].kd = 5.0f;

    lift_sp[LIFT_DOWN]      = -30.0f * 19.0f / 360.0f * 8192.0f;
    lift_sp[LIFT_MIDDLE]    = -180.0f * 19.0f / 360.0f * 8192.0f;
    lift_sp[LIFT_UP]        = -520.0f * 19.0f / 360.0f * 8192.0f;

    spin_sp[ARM_1]          = 10.0f * 19.2f / 360.0f * 8192.0f;
    spin_sp[ARM_2]          = 90.0f * 19.2f / 360.0f * 8192.0f;
    spin_sp[ARM_3]          = 183.0f * 19.2f / 360.0f * 8192.0f;
    spin_sp[ARM_4]          = 2.0f * 19.2f / 360.0f * 8192.0f;


    hand_sp[HAND_OPEN]      =  6.7f * 360.0f * 36.0f / 360.0f * 8192.0f;
    hand_sp[HAND_CLOSE]     =  4.0f * 360.0f * 36.0f / 360.0f * 8192.0f;



    _state = GRIPPER_INITING;

    chThdCreateStatic(hero_gripper_control_wa, sizeof(hero_gripper_control_wa),
                      NORMALPRIO, hero_gripper_control, NULL);
    chThdCreateStatic(hero_rc_gripper_control_wa, sizeof(hero_rc_gripper_control_wa),
                      NORMALPRIO + 1, hero_rc_gripper_control, NULL);

}