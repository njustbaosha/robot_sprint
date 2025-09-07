#include "controller.h"
/**
 * @brief 规范化角度到[-PI, PI]
 * 
 * @param rad 
 * @return float 
 */
static float normalRad(float rad)
{
    if (rad > PI)
    {
        rad -= 2 * PI;
    }
    else if (rad < -PI)
    {
        rad += 2 * PI;
    }
    return rad;
}

// 绝对式，速度只是一个参考，调用内部函数
/**
 * @brief   设置电机以指定位置和速度运动
 * 
 * @param controller 
 * @param target_pos 
 * @param target_vel 
 */
void Controller_setMotor_Targetpos_vel(Controller_t *controller, float *target_pos, float *target_vel)
{
    for (int i = 0; i < 4; i++)
    {
        controller->zdt_mot[i]->motor_controller_t.set.velocity = target_vel[i];
        controller->zdt_mot[i]->motor_controller_t.set.deg_pos = target_pos[i];
    }
    set_speed_pos_target(controller->zdt_mot[0], target_vel[0], target_pos[0]);
    set_speed_pos_target(controller->zdt_mot[1], target_vel[1], target_pos[1]);
    set_speed_pos_target(controller->zdt_mot[2], target_vel[2], target_pos[2]);
    set_speed_pos_target(controller->zdt_mot[3], target_vel[3], target_pos[3]);
}
/**
 * @brief  更新电机速度
 * 
 * @param controller 
 * @param dt 
 */
void ZDTController_MotorUpdate(Controller_t *controller, uint16_t dt)
{
    for (int i = 0; i < 4; i++)
    {
        // 更新电机速度
        controller->current_speed[i] = get_linear_speed(controller->zdt_mot[i]);
    }
}
/**
 * @brief   设置电机以指定速度运动
 * 
 * @param controller 
 * @param target_vel 
 */
void Controller_setMotorTargetSpeed(Controller_t *controller, float *target_vel)
{
    for (int i = 0; i < 4; i++)
    {
        controller->zdt_mot[i]->motor_controller_t.set.velocity = target_vel[i];
    }
    set_speed_target(controller->zdt_mot[0], target_vel[0]);
    set_speed_target(controller->zdt_mot[1], target_vel[1]);
    set_speed_target(controller->zdt_mot[2], target_vel[2]);
    set_speed_target(controller->zdt_mot[3], target_vel[3]);
}

void Controller_Init(Controller_t *controller, StepMotorZDT_t **_zdt_motor, Kinematic_t *kinematic)
{
    for (int i = 0; i < 4; i++)
    {
        controller->zdt_mot[i] = _zdt_motor[i];
    }

    controller->kinematic = kinematic;
    // 初始化PID参数
    uint32_t init_x_maxout = 4;
    uint32_t init_yaw_maxout = 3;
    PID_struct_init(&controller->pid_x, POSITION_PID, init_x_maxout, 20, 0.8f, 0.05f, 0.0f);
    PID_struct_init(&controller->pid_yaw, POSITION_PID, init_yaw_maxout, 20, 0.8f, 0.05f, 0.0f);

    SimpleStatus_t_init(&controller->status);
    // 设置函数指针
    controller->setmotor_speed = Controller_setMotorTargetSpeed;
    controller->Controller_MotorUpdate = ZDTController_MotorUpdate;
    controller->setmotor_pos_vel = Controller_setMotor_Targetpos_vel;

    for (int i = 0; i < 4; i++)
    {
        controller->target_speed[i] = 0;
        controller->current_speed[i] = 0;
        controller->target_distance[i] = 0;
        controller->current_distance[i] = 0;
    }   
}


void Controller_control_update(Controller_t *controller, float *current_odom)
{
    // 计算x方向速度控制量
    controller->pid_x.set[0] = controller->kinematic->target_velocity.linear;
    controller->pid_x.get[0] = controller->kinematic->current_velocity.linear;
    controller->pid_x.err[0] = controller->pid_x.set[0] - controller->pid_x.get[0];
    // controller->pid_x.f_param_init(&controller->pid_x, POSITION_PID, controller->pid_x.MaxOutput, controller->pid_x.IntegralLimit, controller->pid_x.p, controller->pid_x.i, controller->pid_x.d);
    controller->pid_x.pos_out = pid_calc(&controller->pid_x, controller->pid_x.get[0], controller->pid_x.set[0]);   

    // 计算yaw方向速度控制量

    controller->pid_yaw.set[0] = controller->kinematic->target_velocity.omega;
    controller->pid_yaw.get[0] = controller->kinematic->current_velocity.omega;
    controller->pid_yaw.err[0] = controller->pid_yaw.set[0] - controller->pid_yaw.get[0];
    // controller->pid_yaw.f_param_init(&controller->pid_yaw, POSITION_PID, controller->pid_yaw.MaxOutput, controller->pid_yaw.IntegralLimit, controller->pid_yaw.p, controller->pid_yaw.i, controller->pid_yaw.d);
    controller->pid_yaw.pos_out = pid_calc(&controller->pid_yaw, controller->pid_yaw.get[0], controller->pid_yaw.set[0]);
    // 计算目标速度
    float target_vx = controller->pid_x.pos_out;
    float target_vyaw = controller->pid_yaw.pos_out;

    // 计算左右轮速度
    Kinematic_cal_speed(target_vyaw, target_vx, controller->kinematic);

    // 更新目标速度数组
    controller->target_speed[0] = controller->kinematic->velocity_left; // 左前
    controller->target_speed[1] = controller->kinematic->velocity_left;  // 左后
    controller->target_speed[2] = controller->kinematic->velocity_right;  // 右前
    controller->target_speed[3] = controller->kinematic->velocity_right; // 右后
}


void Controller_KinematicAndControlUpdateWithYaw(Controller_t *controller, uint16_t dt, float yaw)
{
    Kinematic_forward(controller->current_speed, &controller->kinematic->current_velocity, controller->kinematic);
    Kinematic_CalculationUpdateWithYaw(dt, controller->kinematic, &controller->kinematic->current_velocity, &controller->kinematic->current_odom, yaw);
    Controller_control_update(controller, &controller->kinematic->current_odom);
    Controller_StatusUpdate(controller, &controller->kinematic->current_odom);
    // 这个函数随电机改变
    controller->setmotor_speed(controller, controller->target_speed);
}