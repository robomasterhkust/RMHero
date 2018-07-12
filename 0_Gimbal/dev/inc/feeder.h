#ifndef FEEDER_H
#define FEEDER_H


#define BULLET_NUM_1 2U
#define BULLET_NUM_2 3U
#define BULLET_NUM_3 4U

#define HEATLIMIT_LVL_1 3200
#define HEATLIMIT_LVL_2 6400
#define HEATLIMIT_LVL_3 12800

#define FeederMotorStep 5000.0f//294912.0f   //8192.0f*360.0f

#define GRIPPER_CONTROL_FREQ 500U
#define UNCONNECTED_MAX 50U

typedef struct{
    float kp;
    float ki;
    float kd;
    float inte_max;
    float inte;
} __attribute__((packed)) feeder_pid_struct;

typedef struct{
    float speed_sp;
    float _speed;
    float pos_sp;
    float _pos;
    uint8_t _wait_count;

} feeder_motorPosStruct;

typedef enum{
    FEEDER_UINITED,
    FEEDER_INITING,
    FEEDER_INITED,
    FEEDER_ERROR
} feeder_states;

void feederInit(void);
void bullet_out(void);
void bullet_in(void);


#endif