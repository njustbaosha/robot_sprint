#include "Kinematic.h"
#include <math.h>


/**
 * @brief Normalize angle to [-PI, PI]
 * 
 * @param rad 
 * @return float 
 */
static float normalRad(float rad) {
  if (rad > PI) {
    rad -= 2 * PI;
  } else if (rad < -PI) {
    rad += 2 * PI;
  }
  return rad;
}

/**
 * @brief 初始化运动学参数
 * 
 * @param _Kinematic 
 * @param _c 
 */
void Kinematic_init(Kinematic_t *_Kinematic,float _c)
{
    _Kinematic->c = _c;
    _Kinematic->current_velocity = (vel_t){0.0f, 0.0f};
    _Kinematic->target_velocity = (vel_t){0.0f, 0.0f};
    _Kinematic->velocity_right = 0.0f;
    _Kinematic->velocity_left = 0.0f;
    _Kinematic->current_odom = (odom_t){0.0f, 0.0f};
    _Kinematic->target_odom = (odom_t){0.0f, 0.0f};
    _Kinematic->odom_error = (odom_t){0.1f, 0.1f};
}




/**
 * @brief 根据目标速度和角速度计算左右轮速度
 * 
 * @param _Kinematic 
 * @param _c 
 */
void Kinematic_cal_speed(float target_omega, float target_velocity_linear, Kinematic_t *_Kinematic)
{   
    // 计算左右轮速度
    _Kinematic->target_velocity = (vel_t){
        .linear = target_velocity_linear,
        .omega = target_omega
    };
    _Kinematic->velocity_right = _Kinematic->target_velocity.linear + ((float)(_Kinematic->c) / 2.0f) * _Kinematic->target_velocity.omega;
    _Kinematic->velocity_left  = _Kinematic->target_velocity.linear - ((float)(_Kinematic->c) / 2.0f) * _Kinematic->target_velocity.omega;

}

// void Kinematic_updateCalculationUpdate(Kinematic_t *_Kinematic, float dt)
// {
//         float delta_t = (float)dt / 1000;
//     // 更新当前线速度和角速度
//     float dx =  _Kinematic->current_velocity.linear * delta_t;

//     // 更新里程计数据
//     _Kinematic->current_odom += _Kinematic->current_velocity.linear * dt;
// }

/**
 * @brief 更新运动学模型
 * 
 * @param _Kinematic 
 * @param dt 
 */
void Kinematic_updateCalculationUpdate_withyaw(Kinematic_t *_Kinematic, float dt)
{
    float delta_t = (float)dt / 1000;
    // 更新当前线速度和角速度
    float dx =  _Kinematic->current_velocity.linear * delta_t;
    float dyaw = _Kinematic->current_velocity.omega * delta_t;
    // 更新里程计数据
    _Kinematic->current_odom.distance += dx;
    _Kinematic->current_odom.yaw += dyaw;
    _Kinematic->current_odom.yaw = normalRad(_Kinematic->current_odom.yaw);
}

void Kinematic_resetOdom(Kinematic_t *_Kinematic)
{
    _Kinematic->current_odom = (odom_t){0.0f, 0.0f};
    _Kinematic->target_odom = (odom_t){0.0f, 0.0f};
    _Kinematic->odom_error = (odom_t){0.0f, 0.0f};
}
void Kinematic_forward(float *current_speed, Kinematic_t *_Kinematic)
{
    // 计算当前线速度和角速度
    _Kinematic->current_velocity = (vel_t){
        .linear = (current_speed[0] + current_speed[1] + current_speed[2] + current_speed[3]) / 4.0f,
        .omega = (current_speed[0] + current_speed[1] - current_speed[2] - current_speed[3]) / (float)(_Kinematic->c)
    };
}
