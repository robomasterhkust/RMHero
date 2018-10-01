#ifndef _HEROPWM_H_
#define _HEROPWM_H_




#define Feeder_Motor_NUM 2

#define FEEDER_CONTROL_FREQ 500

#define Selector 0
#define Feeder   1








typedef struct{
    float speed_sp;
    float _speed;
    uint8_t _wait_count;

} Feeder_motorPosStruct;

typedef struct{
    float kp;
    float ki;
    float kd;
    float error_int;
    float error_int_max;
} __attribute__((packed)) feeder_pid_controller_t;

typedef struct {
    float output[2];
} feeder_output_struct;



void HeroPWM_init(void);
int bullet_num_get(void);
void bullet_in(void);

#endif
