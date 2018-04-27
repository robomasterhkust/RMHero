//
// Created by XU Xinyuan on 22/4/2018.
//

#ifndef RM_CHIBIOS_AUTO_FETCH_TASK_H
#define RM_CHIBIOS_AUTO_FETCH_TASK_H

#define BACK_SWITCH state_PA0
#define LEFT_SWITCH state_PA1
#define RIGHT_SWITCH state_PA3

#define AFT_X_SETPOINT_mm 100
#define INTEGRAL_dt_ms 50
#define XXY_FILTER 0.10

typedef struct {
    bool init;
    bool running;
    IMUStruct* p_IMU;
    RC_Ctl_t* p_RC;

    bool debug;

    systime_t time;
    uint32_t BACK_SWITCH;
    uint32_t LEFT_SWITCH;
    uint32_t RIGHT_SWITCH;

    float_t x_i;
    float_t y_i;
}Auto_Fetch_Task_Struct;

volatile Auto_Fetch_Task_Struct * get_AFT_struct(void);
#ifdef __cplusplus
extern "C" {
#endif

//#define AF_BACK_SWITCH 0U
//#define AF_LEFT_SWITCH 1U
//#define AF_RIGHT_SWITCH 2U




void auto_fetch_task_init(void);

#ifdef __cplusplus
}
#endif

#endif //RM_CHIBIOS_AUTO_FETCH_TASK_H
