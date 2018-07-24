#ifndef _CAN_BUS_PROCESS_H_
#define _CAN_BUS_PROCESS_H_

#include "stdint.h"
#include "stdbool.h"
#include "can_lld.h"
#include "string.h"

#define GIMBAL_MOTOR_NUM  2U
#define CHASSIS_MOTOR_NUM 4U
/* CAN Bus 1 or 2 */
#define CAN_FEEDER_FEEDBACK_MSG_ID                  0x201
#define CAN_PRELOAD_FEEDBACK_MSG_ID                 0x207
#define CAN_GIMBAL_YAW_FEEDBACK_MSG_ID              0x205
#define CAN_GIMBAL_PITCH_FEEDBACK_MSG_ID            0x206

#define CAN_DBUS_ID                                 0x001
#define CAN_CHASSIS_SEND_BARREL_ID                  0x002
#define CAN_NVIDIA_TX2_BOARD_ID                     0x103

#define CAN_BULLET_SID                              0x123

#define CAN_ENCODER_RANGE           8192            // 0x2000

typedef enum
{
  GIMBAL_YAW = 0,
  GIMBAL_PITCH
}gimbal_num_t;

typedef struct {
  uint16_t raw_angle;
  int16_t  raw_current;
  int16_t  current_setpoint;

  int32_t round_count;
  float radian_angle; // Continuous

  bool updated;
} GimbalEncoder_canStruct;

typedef struct {
  uint16_t raw_angle;
  int16_t  raw_speed;
  int16_t act_current;
  uint8_t temperature;

  int32_t round_count;
  int32_t total_ecd;
  float radian_angle; // Continuous

  bool updated;
} ChassisEncoder_canStruct;

typedef struct{
    uint16_t channel0;
    uint16_t channel1;
    uint8_t  s1;
    uint8_t  s2;
    uint16_t key_code;
} dbus_tx_canStruct;

typedef struct{
    uint8_t LEFT;
    uint8_t RIGHT;
    int16_t bullet;
} Can_send_bullet_mouse_struct;

typedef struct{
    uint16_t heatLimit;
    uint16_t currentHeatValue;
    uint8_t  chassis_mode;
} BarrelStatus_canStruct;

typedef struct {
    double py;
    double pz;
    double vy;
    double vz;
    bool updated;
} Ros_msg_canStruct;

volatile GimbalEncoder_canStruct* can_getGimbalMotor(void);
volatile ChassisEncoder_canStruct* can_getfeederMotor(void);
volatile ChassisEncoder_canStruct* can_getPreloadMotor(void);
volatile BarrelStatus_canStruct *can_getHeatValue(void);
volatile Ros_msg_canStruct *can_get_ros_msg(void);

void can_processInit(void);
void can_motorSetCurrent(CANDriver *const CANx,
  const uint16_t EID,
  const int16_t cm1_iq,
  const int16_t cm2_iq,
  const int16_t cm3_iq,
  const int16_t cm4_iq);

#endif
