#ifndef HERO_GRIPPER_H
#define HERO_GRIPPER_H

void hero_gripper_Init(void);


#define GRIPPER_UNINIT  1U
#define GRIPPER_INITING 2U
#define GRIPPER_RUNNING 3U



#define GRIPPER_MOTOR_NUM 3

#define GRIPPER_LIFT_MOTOR 0
#define GRIPPER_ARM_MOTOR  1
#define GRIPPER_HAND_MOTOR 2

#define LIFT_DOWN       0
#define LIFT_MIDDLE     1
#define LIFT_UP         2

#define ARM_1           0
#define ARM_2           1
#define ARM_3           2

#define HAND_CLOSE      0
#define HAND_OPEN       1

#define GRIPPER_CONNECTION_ERROR_COUNT 10U

#define GRIPPER_CONTROL_FREQ 500U

#define LIFT_GEAR_RATIO 19.2f
#define ARM_GEAR_RATIO  19.2f
#define HAND_GEAR_RATIO 36.0f






typedef struct{
    float speed_sp;
    float _speed;
    float pos_sp;
    float _pos;
    uint8_t _wait_count;

} gripper_motorPosStruct;

typedef enum {
    down_1_open,                    // initial state, go to middle_2_open

    middle_2_open,                  // go to middle_3_open
    middle_3_open,                  // go to middle_3_close
    middle_3_close,                 // go to up_3_close1
    up_3_close1,                    // go to up_1_close
    up_1_close,                     // go to up_3_close2
    up_3_close2,                    // go to up_3_open
    up_3_open                       // go to middle_2_open

} gripper_state;


typedef struct{
    float output[GRIPPER_MOTOR_NUM];
}hero_gripper;


typedef struct{
    float kp;
    float ki;
    float kd;
    float error_int;
    float error_int_max;
} __attribute__((packed)) gripper_pid_controller_t;




#endif