#ifndef PRELOAD_H
#define PRELOAD_H



#define PreLoad_CONTROL_FREQ 1000U

#define PreLoad_UnConnected_Max 50U


typedef enum{
    PreLoadUnInited,
    PreLoadIniting,
    PreLoadStop,
    PreLoadBoost,
    PreLoadError
} preload_states;


typedef struct{
    float kp;
    float ki;
    float kd;
    float inte_max;
    float inte;
} __attribute__((packed)) preload_pid_struct;

typedef struct{
    float speed_sp;
    float _speed;
    float pos_sp;
    float _pos;
    uint8_t _wait_count;

} preloadPosStruct;






void preloadInit(void);
void set_bullet_out(void);



#endif PRELOAD_H
