#ifndef __IMU_H__
#define __IMU_H__

#include "zf_common_headfile.h"


// 低通滤波器结构体定义
typedef struct {
    float alpha;      // 滤波系数 (0.0 ~ 1.0, 越小滤波效果越强)
    float prev_value; // 上一次滤波后的值
} low_pass_filter_t;

// 二阶IIR低通滤波器结构体定义（Butterworth biquad）
typedef struct {
    float x1; // x[n-1]
    float x2; // x[n-2]
    float y1; // y[n-1]
    float y2; // y[n-2]
} iir2_low_pass_filter_t;

#define IMU_TYPE_ICM42688 0
#define IMU_TYPE_MPU6050 1
#define IMU_TYPE_IMU660RB 2
#define IMU_TYPE_IMU660RC 3

#define IMU_TYPE       IMU_TYPE_ICM42688

/*
 * IMU board +X is mounted clockwise from the aircraft nose when viewed from
 * above. Positive values rotate the already sign-mapped sensor XY axes back
 * into the aircraft body frame. Gyro and accelerometer must use the same
 * correction.
 */
#define IMU_MOUNT_YAW_DEG 0.0f

// 定义IMU数据结构体
typedef struct {
    // 陀螺仪偏移量
    float gyro_offset[3];  // [0]=x, [1]=y, [2]=z
    float gyro_offset_actual[3];  // 映射到机体系后的陀螺仪零偏，单位 deg/s
    
    // 陀螺仪实际值
    float gyro_actual[3];  // 25 Hz低通后的机体系角速度，供姿态/Rate环使用
    float gyro_unfiltered[3]; // 已映射并去零偏的机体系角速度，供传感器帧积分
    // 陀螺仪原始值
    int16_t gyro_data[3];  // [0]=x, [1]=y, [2]=z
    //陀螺仪真实值（去偏移）
    int16_t gyro_raw[3];  // [0]=x, [1]=y, [2]=z

    // 加速度计偏移量
    float acc_offset[3];   // [0]=x, [1]=y, [2]=z]
    
    // 加速度计实际值
    int16_t acc_data[3];  // [0]=x, [1]=y, [2]=z
    // 加速度计原始值
    int16_t acc_raw[3];  // [0]=x, [1]=y, [2]=z
    
    // 加速度计实际值
    float acc_actual[3];   // [0]=x, [1]=y, [2]=z
    
    // 转角
    float turn_angle;
    
    // 四元数
    float quaternion[4];   // [0]=w, [1]=x, [2]=y, [3]=z
    
    // 欧拉角
    float roll, pitch, yaw;
    
    // 方向余弦矩阵
    float att_matrix[3][3];
    
    // 陀螺仪零偏
    float gyro_bias[3];
    
    // 积分项
    float ex_int, ey_int, ez_int;

    // 滤波器
    low_pass_filter_t gyro_filter[3];  // 陀螺仪XYZ轴滤波器
    low_pass_filter_t acc_filter[3];   // 加速度计XYZ轴滤波器
    iir2_low_pass_filter_t gyro_iir2_filter[3]; // 陀螺仪XYZ轴二阶IIR滤波器
    iir2_low_pass_filter_t acc_iir2_filter[3];  // 加速度计XYZ轴二阶IIR滤波器
    
    // 状态标志
    uint8_t cal_enable;
    // 复位标志
    uint8_t g_reset;  // 重力方向快速复位
    
    // 控制参数
    float gkp;  // 重力方向比例增益
    float gki;  // 重力方向积分增益
    
    // 死区参数
    float gacc_deadzone[3];  // 重力加速度死区    
} imu_data_t;

// 外部变量声明
extern imu_data_t imu_data;

extern uint32_t imu_bad_frame_count;
extern float imu_debug_ex;
extern float imu_debug_ey;
extern float imu_debug_ez;
extern float imu_debug_ex_int;
extern float imu_debug_ey_int;
extern float imu_debug_ez_int;
extern float imu_debug_kp;
extern float imu_debug_ki;
extern float imu_debug_acc_weight;
extern float imu_debug_gyro_rad_norm;
extern float imu_debug_acc_actual_z;
extern float imu_debug_gravity_z;
extern float imu_debug_linear_acc_body_x;
extern float imu_debug_linear_acc_body_y;
extern float imu_debug_linear_acc_body_z;
extern float imu_debug_att_m20;
extern float imu_debug_att_m21;
extern float imu_debug_att_m22;



// 系统时间函数
uint32 system_time_us(void);

// 滤波器函数
void low_pass_filter_init(low_pass_filter_t *filter, float alpha);
float low_pass_filter_update(low_pass_filter_t *filter, float raw_value);
void iir2_low_pass_filter_init(iir2_low_pass_filter_t *filter, float alpha);
float iir2_low_pass_filter_update(iir2_low_pass_filter_t *filter, float raw_value);

// 函数声明

// 初始化函数
void imu_data_init(void);

// 传感器校准函数
void imu_gyro_calibrate(void);
void imu_acc_calibrate(void);
void imu_calibrate(void);
void imu_get_gyro(int16_t *gyro_x, int16_t *gyro_y, int16_t *gyro_z);
void imu_get_acc(int16_t *acc_x, int16_t *acc_y, int16_t *acc_z);
float imu_gyro_transition(int16_t gyro_raw);
float imu_acc_transition(int16_t acc_raw);
float imu_acc_1g_raw(void);
uint8 imu_chip_id_is_ok(void);

// 数据处理函数
void imu_calc(void);
uint32 imu_get_bad_frame_count(void);
uint8 imu_attitude_is_valid(void);
uint8 imu_is_valid(void);

// 辅助功能函数
void imu_reset_turn_angle(void);

// 姿态获取函数
void imu_get_quaternion(float q[4]);
void imu_get_direction_cosine_matrix(float matrix[3][3]);

// 坐标转换函数
void imu_transform_vector_body_to_world(float body_vec[3], float world_vec[3]);
void imu_transform_vector_world_to_body(float world_vec[3], float body_vec[3]);


// 重力和加速度相关函数
void imu_get_gravity_vector(float gravity[3]);
void imu_get_linear_acceleration(float linear_acc[3]);

// 其他辅助函数
void imu_get_full_attitude(float *roll, float *pitch, float *yaw, float *quat);
void imu_set_parameters(float kp, float ki, float mag_kp);
void imu_start_quick_reset(uint8_t g_reset, uint8_t m_reset);

#endif
