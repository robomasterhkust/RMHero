#ifndef _MAIN_H_
#define _MAIN_H_

#include "ch.h"
#include "hal.h"

#include "math_misc.h"

#include "usbcfg.h"
#include "flash.h"
#include "chprintf.h"

#include "canBusProcess.h"
#include "dbus.h"
#include "params.h"
#include "sdlog.h"

#include "attitude.h"
#include "calibrate_sensor.h"

#include "gimbal.h"
#include "shoot.h"
#include "feeder.h"

#include "preload.h"

#include "system_error.h"

#include "exti.h"

typedef enum {
  INIT_DUMMY = 0,
  INIT_SEQUENCE_3_RETURN_1 = 1,
  INIT_SEQUENCE_3_RETURN_2 = 2,
  INIT_SEQUENCE_3_RETURN_3 = 4,
  INIT_ATTITUDE_COMPLETE = 16,
  INIT_COMPLETE = 32
} system_init_state_t;

void shellStart(void);

uint8_t power_check(void);
bool power_failure(void);

#endif
