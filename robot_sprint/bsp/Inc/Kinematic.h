/**
 * @file Kinematic.h
 * @author your name (you@domain.com)
 * @brief 本库为四轮差速底盘解算库
 * @version 0.1
 * @date 2025-09-06
 * @copyright Copyright (c) 2025
 *
 */
/**
 *
 *       1 ---------  3
 *        |    /\    |
 *        |   /  \   |
 *        |    ||    |
 *       2 ---------  4
 */

#ifndef __KINEMATIC_H
#define __KINEMATIC_H
#include "stdint.h"
#include "math.h"
#include "main.h"

typedef struct
{
    float linear;  // 线速度
    float omega; // 角速度
} vel_t;

typedef struct
{
    float distance;
    float yaw;
} odom_t;

typedef struct
{
    uint16_t c;                    // 轮间距
    vel_t current_velocity;        // 当前线速度和航向角速度
    vel_t target_velocity;         // 目标线速度和航向角速度
    odom_t current_odom;            // 当前里程计数据
    odom_t target_odom;
    odom_t odom_error;
    float velocity_right; // 右轮速度
    float velocity_left;  // 左轮速度
    float yaw;            // 航向角
} Kinematic_t;

void Kinematic_init(Kinematic_t *_Kinematic, float _c);

void Kinematic_cal_speed(float target_omega, float target_velocity_linear, Kinematic_t *_Kinematic);
// void Kinematic_updateCalculationUpdate(Kinematic_t *_Kinematic, float dt);
void Kinematic_updateCalculationUpdate_withyaw(Kinematic_t *_Kinematic, float dt);

#endif
