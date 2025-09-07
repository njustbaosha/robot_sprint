#ifndef __CONTROLLER_H
#define __CONTROLLER_H

// #include "Motor.h"

#include "Lib_pormise.h"
#include "Kinematic.h"
#include "pid.h"
#include "motor_def.h"
#include "ZDTstepmotor.h"

typedef struct _ctrl
{
    StepMotorZDT_t *zdt_mot[4];
    float target_speed[4];
    float current_speed[4];
    float target_distance[4];
    float current_distance[4];
    SimpleStatus_t status;
    pid_t pid_x;
    pid_t pid_yaw;
    Kinematic_t *kinematic;
    void (*setmotor_speed)(struct _ctrl *, float *);
    void (*setmotor_pos_vel)(struct _ctrl *, float *, float *);
    void (*Controller_MotorUpdate)(struct _ctrl *, uint16_t);
} Controller_t;

// 初始化函数
void Controller_Init(Controller_t *controller, StepMotorZDT_t **_zdt_motor, Kinematic_t *kinematic);

void Controller_setMotorTargetSpeed(Controller_t *controller, float *target_vel);

void Controller_KinematicAndControlUpdateWithYaw(Controller_t *controller, uint16_t dt, float yaw);

#endif