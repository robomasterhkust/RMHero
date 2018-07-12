cmake_minimum_required(VERSION 3.10)
project(0_Gimbal C)

set(CMAKE_C_STANDARD 11)

include_directories(dev)
include_directories(dev/inc)
include_directories(misc)

add_executable(0_Gimbal
        dev/inc/adis16265.h
        dev/inc/attitude.h
        dev/inc/bullet_cnt.h
        dev/inc/calibrate_sensor.h
        dev/inc/canBusProcess.h
        dev/inc/dbus.h
        dev/inc/exti.h
        dev/inc/feeder.h
        dev/inc/flash.h
        dev/inc/gimbal.h
        dev/inc/imu_temp.h
        dev/inc/ist8310.h
        dev/inc/main.h
        dev/inc/math_misc.h
        dev/inc/mavlink_comm.h
        dev/inc/mpu6500.h
        dev/inc/params.h
        dev/inc/rune.h
        dev/inc/sdlog.h
        dev/inc/shoot.h
        dev/inc/shooter.h
        dev/inc/usbcfg.h
        dev/adis16265.c
        dev/attitude.c
        dev/bullet_cnt.c
        dev/calibrate_sensor.c
        dev/canBusProcess.c
        dev/chconf.h
        dev/dbus.c
        dev/exti.c
        dev/feeder.c
        dev/flash.c
        dev/gimbal.c
        dev/halconf.h
        dev/imu_temp.c
        dev/ist8310.c
        dev/main.c
        dev/math_misc.c
        dev/mavlink_comm.c
        dev/mavlink_topic.c
        dev/mcuconf.h
        dev/mpu6500.c
        dev/params.c
        dev/rune.c
        dev/sdlog.c
        dev/shellcfg.c
        dev/shoot_pwm.c
        dev/shooter.c
        dev/usbcfg.c
        "misc/1.8 TFT_ascii.h"
        misc/gimbal_sys_iden.c
        misc/tft_display.c
        misc/tft_display.h)
