#include "gimbal.h"
#include "robot_def.h"
#include "dji_motor.h"
#include "dmmotor.h"
#include "ins_task.h"
#include "message_center.h"
#include "general_def.h"
#include "bmi088.h"
#include <math.h>

static attitude_t *gimba_IMU_data; // 云台IMU数据
static DJIMotorInstance *yaw_motor;
static DMMotorInstance  *pitch_motor; // 后续增加DM4310电机实例

static Publisher_t *gimbal_pub;                   // 云台应用消息发布者(云台反馈给cmd)
static Subscriber_t *gimbal_sub;                  // cmd控制消息订阅者
static Gimbal_Upload_Data_s gimbal_feedback_data; // 回传给cmd的云台状态信息
static Gimbal_Ctrl_Cmd_s gimbal_cmd_recv;         // 来自cmd的控制信息

static BMI088Instance *bmi088; // 云台IMU

static float pitch_gravity_ff; // pitch重力前馈力矩(N·m),= dbg_gravity_ff_k * cos(IMU Pitch)

// EmberProbe调试镜像变量:堆指针(pitch_motor/gimba_IMU_data)无法被采样工具解引用,
// 在任务中把关键字段镜像到这些static变量上供监视采样
static volatile float dbg_pitch_ref;       // 速度环参考值
static volatile float dbg_pitch_gyro;      // IMU Gyro[0] 速度反馈
static volatile float dbg_pitch_output;    // 速度环PID输出(力矩指令)
static volatile float dbg_pitch_torque;    // 电机实际力矩反馈
static volatile float dbg_pitch_err;       // 速度环误差
static volatile float dbg_pitch_iterm;     // 速度环积分项
static volatile float dbg_pitch_angle;     // IMU Pitch角度
static volatile Motor_Working_Type_e dbg_pitch_stop; // 使能状态

// PID参数镜像(双向同步):GimbalInit时从电机实例读入初值,GimbalTask每周期写回,
// EmberProbe修改镜像变量后下一周期即生效,实现在线调参
static volatile float dbg_speed_kp, dbg_speed_ki, dbg_speed_kd; // 速度环PID增益
static volatile float dbg_speed_maxout, dbg_speed_ilim;         // 速度环输出限幅/积分限幅
static volatile float dbg_angle_kp, dbg_angle_ki, dbg_angle_kd; // 角度环PID增益(串级启用后用)
static volatile float dbg_angle_maxout, dbg_angle_ilim;         // 角度环输出限幅/积分限幅
static volatile float dbg_gravity_ff_k; // 重力前馈增益(N·m),在线可调,静止时调到dbg_pitch_output均值≈0
static volatile float dbg_angle_err;    // 角度环误差(rad)
static volatile float dbg_angle_out;    // 角度环输出(=速度环参考,rad/s)

// yaw轴调试镜像(GM6020,输出为电压指令量纲)
static volatile float dbg_yaw_ref;       // 速度环参考(rad/s,单环调试时=摇杆直接映射)
static volatile float dbg_yaw_gyro;      // IMU Gyro[2] 速度反馈(rad/s)
static volatile float dbg_yaw_output;    // 速度环输出(电压指令)
static volatile float dbg_yaw_angle;     // IMU YawTotalAngle(rad)
static volatile float dbg_yaw_current;   // 实际电流反馈
static volatile Motor_Working_Type_e dbg_yaw_stop; // 使能状态
// yaw参数镜像(双向同步,同pitch机制)
static volatile float dbg_yaw_speed_kp, dbg_yaw_speed_ki, dbg_yaw_speed_kd; // yaw速度环增益
static volatile float dbg_yaw_angle_kp, dbg_yaw_angle_ki, dbg_yaw_angle_kd; // yaw角度环增益(串级启用后用)
static volatile float dbg_yaw_angle_maxout; // yaw角度环输出限幅(速度参考rad/s)

void GimbalInit()
{   
    gimba_IMU_data = INS_Init(); // IMU先初始化,获取姿态数据指针赋给yaw电机的其他数据来源
    // YAW
    Motor_Init_Config_s yaw_config = {
        .can_init_config = {
            .can_handle = &hcan2,
            .tx_id = 2,
        },
        .controller_param_init_config = {
            .angle_PID = {
                .Kp = 5, // 角度环起步,输出为速度参考(rad/s),串级启用后调
                .Ki = 0,
                .Kd = 0,
                .DeadBand = 0, // 原0.1rad(约5.7°)死区太大,会造成"没到就停",去除;防参考漂移靠robot_cmd摇杆死区
                .Improve = PID_Trapezoid_Intergral | PID_Integral_Limit | PID_Derivative_On_Measurement,
                .IntegralLimit = 100,
                .MaxOut = 10, // 限制速度参考幅值(rad/s),原500无意义
            },
            .speed_PID = {
                .Kp = 20,  // 速度环起步,在线逐步翻倍验证,GM6020电压指令量纲(±20000限幅)
                .Ki = 0,   // Kp稳了再加,起步5~50,注意IntegralLimit占比
                .Kd = 0,
                .Improve = PID_Trapezoid_Intergral | PID_Integral_Limit | PID_Derivative_On_Measurement,
                .IntegralLimit = 3000,
                .MaxOut = 20000,
            },
            .other_angle_feedback_ptr = &gimba_IMU_data->YawTotalAngle,
            .other_speed_feedback_ptr = &gimba_IMU_data->Gyro[2],
        },
        .controller_setting_init_config = {
            .angle_feedback_source = OTHER_FEED,
            .speed_feedback_source = OTHER_FEED,
            .outer_loop_type = SPEED_LOOP,            // 临时:单速度环调试yaw(同pitch流程),速度环稳后切回ANGLE_LOOP
            .close_loop_type = SPEED_LOOP,            // 临时:单速度环,串级时改回ANGLE_LOOP | SPEED_LOOP
            .motor_reverse_flag = MOTOR_DIRECTION_REVERSE, // 模板值,解锁打杆验证方向,若正反馈改为NORMAL
        },
        .motor_type = GM6020};
    // PITCH
    Motor_Init_Config_s pitch_config = {
        .can_init_config = {
            .can_handle = &hcan2,
            .tx_id = 1,
            .rx_id = 1,
        },
        .controller_param_init_config = {
            .angle_PID = {
                .Kp = 0.07, // 角度环起步值,输出为速度参考(rad/s),误差0.5rad时给1.5rad/s
                .Ki = 0.005,
                .Kd = 0.0005,
                .Improve = PID_Trapezoid_Intergral | PID_Integral_Limit | PID_Derivative_On_Measurement,
                .IntegralLimit = 100,
                .MaxOut = 3, // 限制速度参考幅值(rad/s),与速度环能力匹配
            },
            .speed_PID = {
                .Kp = 2.3,  
                .Ki = 0, 
                .Kd = 0,
                .Improve = PID_Trapezoid_Intergral | PID_Integral_Limit | PID_Derivative_On_Measurement,
                .IntegralLimit = 5,
                .MaxOut = 3,
            },
            .other_angle_feedback_ptr = &gimba_IMU_data->Pitch,
            .other_speed_feedback_ptr = &gimba_IMU_data->Gyro[0],
            .speed_feedforward_ptr = &pitch_gravity_ff, // 重力前馈力矩,叠加在速度环输出之后
        },
        .controller_setting_init_config = {
            .angle_feedback_source = OTHER_FEED,  // 角度反馈来自IMU的Pitch(other_angle_feedback_ptr),而非电机编码器
            .speed_feedback_source = OTHER_FEED,  // 速度反馈来自IMU的Gyro[0](other_speed_feedback_ptr),即机体x轴角速度
            .outer_loop_type = ANGLE_LOOP,        // 最外层为角度环(串级:角度环→速度环→力矩)
            .close_loop_type = ANGLE_LOOP | SPEED_LOOP, // 角度环+速度环双环串级,由MCU端PID计算后输出力矩
            .feedforward_flag = SPEED_FEEDFORWARD, // 启用速度(力矩)前馈通道
            .motor_reverse_flag = MOTOR_DIRECTION_REVERSE, // 实测正力矩使Gyro[0]反向,闭环正反馈,输出整体取反修正方向
        },
        .motor_type = DM4310,
    };
    // 电机对total_angle闭环,上电时为零,会保持静止,收到遥控器数据再动
    yaw_motor = DJIMotorInit(&yaw_config);
    pitch_motor = DMMotorInit(&pitch_config);

    // PID参数镜像读入初值,此后以镜像为准(GimbalTask每周期写回电机实例)
    dbg_speed_kp = pitch_motor->speed_PID.Kp;
    dbg_speed_ki = pitch_motor->speed_PID.Ki;
    dbg_speed_kd = pitch_motor->speed_PID.Kd;
    dbg_speed_maxout = pitch_motor->speed_PID.MaxOut;
    dbg_speed_ilim = pitch_motor->speed_PID.IntegralLimit;
    dbg_angle_kp = pitch_motor->angle_PID.Kp;
    dbg_angle_ki = pitch_motor->angle_PID.Ki;
    dbg_angle_kd = pitch_motor->angle_PID.Kd;
    dbg_angle_maxout = pitch_motor->angle_PID.MaxOut;
    dbg_angle_ilim = pitch_motor->angle_PID.IntegralLimit;
    dbg_gravity_ff_k = 0.15f; // 重力前馈增益初值,上电后用EmberProbe在线标定

    // yaw参数镜像读入初值(同pitch机制,此后以镜像为准)
    dbg_yaw_speed_kp = yaw_motor->motor_controller.speed_PID.Kp;
    dbg_yaw_speed_ki = yaw_motor->motor_controller.speed_PID.Ki;
    dbg_yaw_speed_kd = yaw_motor->motor_controller.speed_PID.Kd;
    dbg_yaw_angle_kp = yaw_motor->motor_controller.angle_PID.Kp;
    dbg_yaw_angle_ki = yaw_motor->motor_controller.angle_PID.Ki;
    dbg_yaw_angle_kd = yaw_motor->motor_controller.angle_PID.Kd;
    dbg_yaw_angle_maxout = yaw_motor->motor_controller.angle_PID.MaxOut;

    gimbal_pub = PubRegister("gimbal_feed", sizeof(Gimbal_Upload_Data_s));
    gimbal_sub = SubRegister("gimbal_cmd", sizeof(Gimbal_Ctrl_Cmd_s));
}

/* 机器人云台控制核心任务,后续考虑只保留IMU控制,不再需要电机的反馈 */
void GimbalTask()
{
    // 获取云台控制数据
    // 后续增加未收到数据的处理
    DMMotorEnable(pitch_motor);
    SubGetMessage(gimbal_sub, &gimbal_cmd_recv);
    // @todo:现在已不再需要电机反馈,实际上可以始终使用IMU的姿态数据来作为云台的反馈,yaw电机的offset只是用来跟随底盘
    // 根据控制模式进行电机反馈切换和过渡,视觉模式在robot_cmd模块就已经设置好,gimbal只看yaw_ref和pitch_ref
    switch (gimbal_cmd_recv.gimbal_mode)
    {
    // 停止
    case GIMBAL_ZERO_FORCE:
        DJIMotorStop(yaw_motor);
        DMMotorStop(pitch_motor);
        break;
    // 使用陀螺仪的反馈,底盘根据yaw电机的offset跟随云台或视觉模式采用
    case GIMBAL_GYRO_MODE: // 后续只保留此模式
        DJIMotorEnable(yaw_motor);
        DMMotorEnable(pitch_motor);
        DJIMotorChangeFeed(yaw_motor, ANGLE_LOOP, OTHER_FEED);
        DJIMotorChangeFeed(yaw_motor, SPEED_LOOP, OTHER_FEED);
        DJIMotorSetRef(yaw_motor, gimbal_cmd_recv.yaw); // yaw和pitch会在robot_cmd中处理好多圈和单圈
        DMMotorSetRef(pitch_motor, gimbal_cmd_recv.pitch); // pitch角度参考(rad),robot_cmd中做角度增量和软件限位
        break;
    // 云台自由模式,使用编码器反馈,底盘和云台分离,仅云台旋转,一般用于调整云台姿态(英雄吊射等)/能量机关
    case GIMBAL_FREE_MODE: // 后续删除,或加入云台追地盘的跟随模式(响应速度更快)
        DJIMotorEnable(yaw_motor);
        DJIMotorChangeFeed(yaw_motor, ANGLE_LOOP, OTHER_FEED);
        DJIMotorChangeFeed(yaw_motor, SPEED_LOOP, OTHER_FEED);
        DJIMotorSetRef(yaw_motor, gimbal_cmd_recv.yaw); // yaw和pitch会在robot_cmd中处理好多圈和单圈
        DMMotorSetRef(pitch_motor, gimbal_cmd_recv.pitch); // pitch角度参考(rad),robot_cmd中做角度增量和软件限位
        break;
    default:
        break;
    }

    // 在合适的地方添加pitch重力补偿前馈力矩
    // 根据IMU姿态/pitch电机角度反馈计算出当前配重下的重力矩
    // ...

    // 设置反馈数据,主要是imu和yaw的ecd
    gimbal_feedback_data.gimbal_imu_data = *gimba_IMU_data;
    gimbal_feedback_data.yaw_motor_single_round_angle = yaw_motor->measure.angle_single_round;

    // 更新调试镜像变量
    dbg_pitch_ref = pitch_motor->pid_ref;
    dbg_pitch_gyro = gimba_IMU_data->Gyro[0];
    dbg_pitch_output = pitch_motor->speed_PID.Output;
    dbg_pitch_torque = pitch_motor->measure.torque;
    dbg_pitch_err = pitch_motor->speed_PID.Err;
    dbg_pitch_iterm = pitch_motor->speed_PID.ITerm;
    dbg_pitch_angle = gimba_IMU_data->Pitch;
    dbg_pitch_stop = pitch_motor->stop_flag;
    dbg_angle_err = pitch_motor->angle_PID.Err;
    dbg_angle_out = pitch_motor->angle_PID.Output;

    // yaw调试镜像更新
    dbg_yaw_ref = yaw_motor->motor_controller.pid_ref;
    dbg_yaw_gyro = gimba_IMU_data->Gyro[2];
    dbg_yaw_output = yaw_motor->motor_controller.speed_PID.Output;
    dbg_yaw_angle = gimba_IMU_data->YawTotalAngle;
    dbg_yaw_current = yaw_motor->measure.real_current;
    dbg_yaw_stop = yaw_motor->stop_flag;

    // yaw参数写回(同pitch机制)
    yaw_motor->motor_controller.speed_PID.Kp = dbg_yaw_speed_kp;
    yaw_motor->motor_controller.speed_PID.Ki = dbg_yaw_speed_ki;
    yaw_motor->motor_controller.speed_PID.Kd = dbg_yaw_speed_kd;
    yaw_motor->motor_controller.angle_PID.Kp = dbg_yaw_angle_kp;
    yaw_motor->motor_controller.angle_PID.Ki = dbg_yaw_angle_ki;
    yaw_motor->motor_controller.angle_PID.Kd = dbg_yaw_angle_kd;
    yaw_motor->motor_controller.angle_PID.MaxOut = dbg_yaw_angle_maxout;

    // pitch重力前馈:K*cos(Pitch),力矩量纲,由DMMotorControl叠加在速度环输出之后
    pitch_gravity_ff = dbg_gravity_ff_k * cosf(gimba_IMU_data->Pitch);

    // PID参数写回:EmberProbe修改镜像值后,下一控制周期生效
    pitch_motor->speed_PID.Kp = dbg_speed_kp;
    pitch_motor->speed_PID.Ki = dbg_speed_ki;
    pitch_motor->speed_PID.Kd = dbg_speed_kd;
    pitch_motor->speed_PID.MaxOut = dbg_speed_maxout;
    pitch_motor->speed_PID.IntegralLimit = dbg_speed_ilim;
    pitch_motor->angle_PID.Kp = dbg_angle_kp;
    pitch_motor->angle_PID.Ki = dbg_angle_ki;
    pitch_motor->angle_PID.Kd = dbg_angle_kd;
    pitch_motor->angle_PID.MaxOut = dbg_angle_maxout;
    pitch_motor->angle_PID.IntegralLimit = dbg_angle_ilim;

    // 推送消息
    PubPushMessage(gimbal_pub, (void *)&gimbal_feedback_data);
}