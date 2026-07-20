#include "zf_common_headfile.h"
#include "attitude_history.h"



// 声明全局IMU数据实例
imu_data_t imu_data = {
    .gyro_offset = {0.0f, 0.0f, 0.0f},
    .gyro_offset_actual = {0.0f, 0.0f, 0.0f},
    .gyro_actual = {0.0f, 0.0f, 0.0f},
    .gyro_unfiltered = {0.0f, 0.0f, 0.0f},
    .gyro_raw = {0, 0, 0},
    .acc_offset = {0.0f, 0.0f, 0.0f},
    .acc_actual = {0.0f, 0.0f, 0.0f},
    .acc_raw = {0, 0, 0},
    .turn_angle = 0.0f,
    .quaternion = {1.0f, 0.0f, 0.0f, 0.0f},
    .roll = 0.0f,
    .pitch = 0.0f,
    .yaw = 0.0f,
    .gyro_bias = {0.0f, 0.0f, 0.0f},
    .ex_int = 0.0f,
    .ey_int = 0.0f,
    .ez_int = 0.0f,
    .cal_enable = 0,
    .g_reset = 0,
    .gkp = 0.5f,     // 降低比例增益，防止加速度计噪声导致解算剧烈震荡
    .gki = 0.0f,     // 先关闭积分项，避免静止小误差长期累积导致姿态慢漂
    .gacc_deadzone = {0.0f, 0.0f, 0.0f}    
};

static volatile uint8 imu_log_running = 0;
static volatile uint8 imu_log_full = 0;
static volatile uint16 imu_log_index = 0;
static imu_log_sample_t imu_log_buf[IMU_LOG_SAMPLE_COUNT];
static float imu_log_gyro_unfiltered[3] = {0.0f, 0.0f, 0.0f};
static float imu_log_acc_unfiltered[3] = {0.0f, 0.0f, 1.0f};

float imu_debug_ex = 0.0f;
float imu_debug_ey = 0.0f;
float imu_debug_ez = 0.0f;
float imu_debug_ex_int = 0.0f;
float imu_debug_ey_int = 0.0f;
float imu_debug_ez_int = 0.0f;
float imu_debug_kp = 0.0f;
float imu_debug_ki = 0.0f;
float imu_debug_acc_weight = 0.0f;
float imu_debug_gyro_rad_norm = 0.0f;
float imu_debug_acc_actual_z = 0.0f;
float imu_debug_gravity_z = 0.0f;
float imu_debug_linear_acc_body_x = 0.0f;
float imu_debug_linear_acc_body_y = 0.0f;
float imu_debug_linear_acc_body_z = 0.0f;
float imu_debug_att_m20 = 0.0f;
float imu_debug_att_m21 = 0.0f;
float imu_debug_att_m22 = 1.0f;

extern uint32_t duty;

static void imu_map_gyro_to_body(float raw_gyro_x, float raw_gyro_y, float raw_gyro_z,
                                 float *gyro_x, float *gyro_y, float *gyro_z);

static int16 imu_log_float_to_i16(float value)
{
    if(value > 32767.0f) {
        return 32767;
    }
    if(value < -32768.0f) {
        return -32768;
    }
    return (int16)value;
}

void imu_log_start(void)
{
    imu_log_index = 0;
    imu_log_full = 0;
    imu_log_running = 1;
}

uint8 imu_log_is_full(void)
{
    return imu_log_full;
}

void imu_log_sample_isr(void)
{
    uint16 index;

    if(!imu_log_running || imu_log_full) {
        return;
    }

    index = imu_log_index;
    if(index >= IMU_LOG_SAMPLE_COUNT) {
        imu_log_running = 0;
        imu_log_full = 1;
        return;
    }

    imu_log_buf[index].gx = imu_log_float_to_i16(imu_log_gyro_unfiltered[0] * IMU_LOG_GYRO_SCALE);
    imu_log_buf[index].gy = imu_log_float_to_i16(imu_log_gyro_unfiltered[1] * IMU_LOG_GYRO_SCALE);
    imu_log_buf[index].gz = imu_log_float_to_i16(imu_log_gyro_unfiltered[2] * IMU_LOG_GYRO_SCALE);
    imu_log_buf[index].ax = imu_log_float_to_i16(imu_log_acc_unfiltered[0] * IMU_LOG_ACC_SCALE);
    imu_log_buf[index].ay = imu_log_float_to_i16(imu_log_acc_unfiltered[1] * IMU_LOG_ACC_SCALE);
    imu_log_buf[index].az = imu_log_float_to_i16(imu_log_acc_unfiltered[2] * IMU_LOG_ACC_SCALE);
    imu_log_buf[index].bad = (uint16)imu_get_bad_frame_count();

    index++;
    imu_log_index = index;

    if(index >= IMU_LOG_SAMPLE_COUNT) {
        imu_log_running = 0;
        imu_log_full = 1;
    }
}

void imu_log_dump_task(void)
{
    uint16 count;

    if(!imu_log_full) {
        return;
    }

    count = imu_log_index;
    printf("IMU_LOG_BEGIN fs=500 count=%u duty=%lu gyro_scale=100 acc_scale=10000\r\n",
           count, (uint32)duty);

    for(uint16 i = 0; i < count; i++) {
        printf("%u,%d,%d,%d,%d,%d,%d,%u\r\n",
               i,
               imu_log_buf[i].gx,
               imu_log_buf[i].gy,
               imu_log_buf[i].gz,
               imu_log_buf[i].ax,
               imu_log_buf[i].ay,
               imu_log_buf[i].az,
               imu_log_buf[i].bad);
    }

    printf("IMU_LOG_END\r\n");
    imu_log_full = 0;
}


// 初始化方向余弦矩阵为单位矩阵
void init_att_matrix(void)
{
    for(int i = 0; i < 3; i++) {
        for(int j = 0; j < 3; j++) {
            imu_data.att_matrix[i][j] = (i == j) ? 1.0f : 0.0f;
        }
    }
}


// 低通滤波器初始化
void low_pass_filter_init(low_pass_filter_t *filter, float alpha)
{
    filter->alpha = alpha;
    filter->prev_value = 0.0f;
}

// 低通滤波器更新
float low_pass_filter_update(low_pass_filter_t *filter, float raw_value)
{
    filter->prev_value = filter->alpha * raw_value + (1.0f - filter->alpha) * filter->prev_value;
    return filter->prev_value;
}

// Fs=500Hz, Fc=25Hz 二阶 Butterworth IIR 低通滤波器系数
#define IIR2_B0             (0.02008337f)
#define IIR2_B1             (0.04016673f)
#define IIR2_B2             (0.02008337f)
#define IIR2_A1             (-1.56101808f)
#define IIR2_A2             (0.64135154f)
#define IIR2_SAMPLE_LIMIT   (2000.0f)

// 二阶IIR低通滤波器初始化
void iir2_low_pass_filter_init(iir2_low_pass_filter_t *filter, float alpha)
{
    (void)alpha;
    filter->x1 = 0.0f;
    filter->x2 = 0.0f;
    filter->y1 = 0.0f;
    filter->y2 = 0.0f;
}

// 二阶IIR低通滤波器更新（Butterworth biquad）
float iir2_low_pass_filter_update(iir2_low_pass_filter_t *filter, float raw_value)
{
    float output;

    if(raw_value > IIR2_SAMPLE_LIMIT || raw_value < -IIR2_SAMPLE_LIMIT) {
        raw_value = filter->x1;
    }

    output = IIR2_B0 * raw_value
           + IIR2_B1 * filter->x1
           + IIR2_B2 * filter->x2
           - IIR2_A1 * filter->y1
           - IIR2_A2 * filter->y2;

    filter->x2 = filter->x1;
    filter->x1 = raw_value;
    filter->y2 = filter->y1;
    filter->y1 = output;

    return output;
}


// 初始化滤波器
void init_filters(void)
{
    for(int i = 0; i < 3; i++) {
        low_pass_filter_init(&imu_data.gyro_filter[i], 0.8f);  // 陀螺仪轻微滤波
        low_pass_filter_init(&imu_data.acc_filter[i], 0.5f);   // 释放加速度计高频响应，防止旋转停止后延迟收敛
        iir2_low_pass_filter_init(&imu_data.gyro_iir2_filter[i], 0.0f); // 陀螺仪二阶Butterworth，Fs=500Hz，Fc=25Hz
        iir2_low_pass_filter_init(&imu_data.acc_iir2_filter[i], 0.0f);  // 加速度计二阶Butterworth，Fs=500Hz，Fc=25Hz
    }
}

// 四元数归一化函数
void normalizeQuaternion(float q[4])
{
    float norm = sqrt(q[0]*q[0] + q[1]*q[1] + q[2]*q[2] + q[3]*q[3]);
    if(norm > 0.0f)
    {
        q[0] /= norm;
        q[1] /= norm;
        q[2] /= norm;
        q[3] /= norm;
    }
}
// 初始化方向余弦矩阵为单位矩阵


// 从四元数构建方向余弦矩阵
void buildDirectionCosineMatrix(float q[4], float matrix[3][3])
{
    float q0 = q[0], q1 = q[1], q2 = q[2], q3 = q[3];
    
    // 构建方向余弦矩阵
    matrix[0][0] = 1 - 2*q2*q2 - 2*q3*q3;  // x_vec[X]
    matrix[0][1] = 2*q1*q2 - 2*q0*q3;      // x_vec[Y]
    matrix[0][2] = 2*q1*q3 + 2*q0*q2;      // x_vec[Z]
    
    matrix[1][0] = 2*q1*q2 + 2*q0*q3;      // y_vec[X]
    matrix[1][1] = 1 - 2*q1*q1 - 2*q3*q3;  // y_vec[Y]
    matrix[1][2] = 2*q2*q3 - 2*q0*q1;      // y_vec[Z]
    
    matrix[2][0] = 2*q1*q3 - 2*q0*q2;      // z_vec[X]
    matrix[2][1] = 2*q2*q3 + 2*q0*q1;      // z_vec[Y]
    matrix[2][2] = 1 - 2*q1*q1 - 2*q2*q2;  // z_vec[Z]
}
// 四元数微分方程
void quaternionRate(float q[4], float omega[3], float dt)
{
    float qDot[4];
    
    // 修复四元数微分方程中极其致命的叉乘正负号错误，这是导致快速旋转时姿态彻底发散的根源
    qDot[0] = 0.5f * (-q[1]*omega[0] - q[2]*omega[1] - q[3]*omega[2]);
    qDot[1] = 0.5f * ( q[0]*omega[0] + q[2]*omega[2] - q[3]*omega[1]);
    qDot[2] = 0.5f * ( q[0]*omega[1] - q[1]*omega[2] + q[3]*omega[0]);
    qDot[3] = 0.5f * ( q[0]*omega[2] + q[1]*omega[1] - q[2]*omega[0]);
    
    // 更新四元数
    q[0] += qDot[0] * dt;
    q[1] += qDot[1] * dt;
    q[2] += qDot[2] * dt;
    q[3] += qDot[3] * dt;
    
    // 归一化
    normalizeQuaternion(q);
}

// 坐标系转换函数：载体坐标转世界坐标（3D）
void body_to_world_trans(float b[3], float w[3])
{
    for(uint8_t i = 0; i < 3; i++)
    {
        float temp = 0;
        for(uint8_t j = 0; j < 3; j++)
        {
            temp += b[j] * imu_data.att_matrix[i][j];
        }
        w[i] = temp;
    }
}

// 坐标系转换函数：世界坐标转载体坐标（3D）
void world_to_body_trans(float w[3], float b[3])
{
    for(uint8_t i = 0; i < 3; i++)
    {
        float temp = 0;
        for(uint8_t j = 0; j < 3; j++)
        {
            temp += w[j] * imu_data.att_matrix[j][i];  // 转置矩阵
        }
        b[i] = temp;
    }
}

// 2D坐标转换：世界坐标平面XY转平面航向坐标XY
void world_to_heading_2d_trans(float w[3], float ref_ax[3], float h[3])
{
    h[0] =  w[0] *  ref_ax[0]  + w[1] * ref_ax[1];
    h[1] =  w[0] * (-ref_ax[1]) + w[1] * ref_ax[0];
    h[2] = w[2]; // Z轴保持不变
}

// 2D坐标转换：平面航向坐标XY转世界坐标平面XY
void heading_to_world_2d_trans(float h[3], float ref_ax[3], float w[3])
{
    w[0] = h[0] * ref_ax[0] + h[1] * (-ref_ax[1]);
    w[1] = h[0] * ref_ax[1] + h[1] *  ref_ax[0];
    w[2] = h[2]; // Z轴保持不变
}



// 将四元数转换为欧拉角
void quaternionToEuler(float q[4], float *roll, float *pitch, float *yaw)
{
    float q0 = q[0], q1 = q[1], q2 = q[2], q3 = q[3];
        
    // Pitch (Y轴旋转)
    float sinp = 2.0f * (q0 * q2 - q3 * q1);
   // 极限安全保护：由于浮点误差，sinp 可能轻微超过 1.0，导致 asin 算爆产生 NaN，此处必须钳位保护
    sinp = LIMIT(sinp, -1.0f, 1.0f);
    *pitch = asin(sinp) * 180.0f / PI;

    // Roll (X轴旋转)
    float sinr_cosp = 2.0f * (q0 * q1 + q2 * q3);
    float cosr_cosp = 1.0f - 2.0f * (q1 * q1 + q2 * q2);
    *roll = atan2(sinr_cosp, cosr_cosp) * 180.0f / PI;

    // Yaw (Z轴旋转)
    float siny_cosp = 2.0f * (q0 * q3 + q1 * q2);
    float cosy_cosp = 1.0f - 2.0f * (q2 * q2 + q3 * q3);
    // 保持标准右手系 yaw 极性，避免仅翻转加速度轴导致 roll/pitch 失真
    *yaw = atan2(siny_cosp, cosy_cosp) * 180.0f / PI;
}
void imu_gyro_calibrate(void)
{
    uint16_t cali_cnt=0;
    float gyro_x_sum=0.0f;
    float gyro_y_sum=0.0f;
    float gyro_z_sum=0.0f;
    for(cali_cnt=0;cali_cnt<2000;cali_cnt++)
    {
        float raw_gyro_x;
        float raw_gyro_y;
        float raw_gyro_z;
        float gyro_x;
        float gyro_y;
        float gyro_z;

        imu_get_gyro(&imu_data.gyro_data[0], &imu_data.gyro_data[1], &imu_data.gyro_data[2]);

        raw_gyro_x = imu_gyro_transition(imu_data.gyro_data[0]);
        raw_gyro_y = imu_gyro_transition(imu_data.gyro_data[1]);
        raw_gyro_z = imu_gyro_transition(imu_data.gyro_data[2]);
        imu_map_gyro_to_body(raw_gyro_x, raw_gyro_y, raw_gyro_z, &gyro_x, &gyro_y, &gyro_z);

        gyro_x_sum += gyro_x;
        gyro_y_sum += gyro_y;
        gyro_z_sum += gyro_z;
        system_delay_ms(2);
    }

    imu_data.gyro_offset[0]=0.0f;
    imu_data.gyro_offset[1]=0.0f;
    imu_data.gyro_offset[2]=0.0f;
    imu_data.gyro_offset_actual[0]=gyro_x_sum/2000.0f;
    imu_data.gyro_offset_actual[1]=gyro_y_sum/2000.0f;
    imu_data.gyro_offset_actual[2]=gyro_z_sum/2000.0f;
    imu_data.gyro_actual[2]=0.0f;
    imu_data.gyro_actual[0]=0.0f;
    imu_data.gyro_actual[1]=0.0f;

        // 重置四元数
    imu_data.quaternion[0] = 1.0f;
    imu_data.quaternion[1] = 0.0f;
    imu_data.quaternion[2] = 0.0f;
    imu_data.quaternion[3] = 0.0f;
    
    // 重置方向余弦矩阵
    init_att_matrix();
        

}

void imu_acc_calibrate(void)
{
    uint16_t cali_cnt=0;
    int32_t acc_z_raw_sum=0;
    int32_t acc_x_raw_sum=0;
    int32_t acc_y_raw_sum=0;
    for(cali_cnt=0;cali_cnt<2000;cali_cnt++)
    {
    
        imu_get_acc(&imu_data.acc_data[0], &imu_data.acc_data[1], &imu_data.acc_data[2]);
        acc_x_raw_sum+=imu_data.acc_data[0];
        acc_y_raw_sum+=imu_data.acc_data[1];
        acc_z_raw_sum+=imu_data.acc_data[2];
        system_delay_ms(1);
    }

    imu_data.acc_offset[2]=(float)acc_z_raw_sum/2000.0f - imu_acc_1g_raw();
    imu_data.acc_offset[0]=(float)acc_x_raw_sum/2000.0f;
    imu_data.acc_offset[1]=(float)acc_y_raw_sum/2000.0f;

    imu_data.acc_actual[2]=0.0f;
    imu_data.acc_actual[0]=0.0f;
    imu_data.acc_actual[1]=0.0f;

        

}

void imu_calibrate(void)
{
    imu_gyro_calibrate();
    imu_acc_calibrate();
}


// ==================== 姿态解算相关函数 ====================

// 简单的航向角积分
float get_yaw_from_gyro(float gyro_z, float dt)
{
    static float yaw_angle = 0.0f;
    yaw_angle += gyro_z * dt * 180.0f / 3.14159f;  // 弧度转度
    
    // 限制范围在 -180 ~ 180
    while(yaw_angle > 180.0f) yaw_angle -= 360.0f;
    while(yaw_angle < -180.0f) yaw_angle += 360.0f;
    
    return yaw_angle;
}

// Mahony互补滤波姿态解算
void imu_mahony_filter(float gyro[3], float acc[3], float dt)
{
    float q0 = imu_data.quaternion[0], q1 = imu_data.quaternion[1], 
          q2 = imu_data.quaternion[2], q3 = imu_data.quaternion[3];
    
    float norm;
    float vx, vy, vz;
    float ex = 0.0f, ey = 0.0f, ez = 0.0f;
    
    // 创建临时的陀螺仪数组，避免污染原始数据
    float gyro_temp[3] = {gyro[0], gyro[1], gyro[2]};

    // 归一化加速度计数据（增加合理范围校验）
    norm = sqrt(acc[0]*acc[0] + acc[1]*acc[1] + acc[2]*acc[2]);
    float adaptive_weight = 1.0f;
    
    if(norm > 0.001f) {
        // 先做归一化
        acc[0] /= norm;
        acc[1] /= norm;
        acc[2] /= norm;

        // 计算当前加速度偏离完美 1g (1.0f) 的误差
        float error_acc = ABS(norm - 1.0f); 
        //printf("acc_norm:%.3f, error_acc:%.3f\n", norm, error_acc);
        // 缓和自适应权重，防止普通飞行机动就完全丢弃加速度计
        if (error_acc < 0.15f) {
            adaptive_weight = 1.0f;       // 极度平稳，完全信任 (0.9g ~ 1.1g)
        } else if (error_acc < 0.35f) {
            adaptive_weight = 1.0f - (error_acc - 0.15f) * 5.0f; // 发生急加速，平滑衰减权重
        } else {
            adaptive_weight = 0.0f;     
        }

        vx = 2*(q1*q3 - q0*q2);
        vy = 2*(q0*q1 + q2*q3);
        vz = q0*q0 - q1*q1 - q2*q2 + q3*q3;

        ex = (acc[1]*vz - acc[2]*vy) * adaptive_weight;
        ey = (acc[2]*vx - acc[0]*vz) * adaptive_weight;
        // 六轴模式，重力矢量无法提供偏航(Yaw)基准。必须屏蔽强行修正，否则会把机动震动串扰进Yaw导致其随机乱飘。
        ez = 0.0f; 
    }

    float gyro_rad_norm = sqrt(gyro[0]*gyro[0] + gyro[1]*gyro[1] + gyro[2]*gyro[2]);

    // 误差积分 (极其关键：仅在基本平稳且未剧烈旋转时积分，彻底切断剧烈旋转停止瞬间的错误残量对 Yaw 零偏的污染)
    if(imu_data.gki > 0.0f && adaptive_weight > 0.9f && gyro_rad_norm < 1.0f) {
        ex = LIMIT(ex, -0.1f, 0.1f);
        ey = LIMIT(ey, -0.1f, 0.1f);
        ez = LIMIT(ez, -0.1f, 0.1f);
        
        imu_data.ex_int += ex * imu_data.gki * dt;
        imu_data.ey_int += ey * imu_data.gki * dt;
        imu_data.ez_int += ez * imu_data.gki * dt;
        
        imu_data.ex_int = LIMIT(imu_data.ex_int, -1.0f, 1.0f);
        imu_data.ey_int = LIMIT(imu_data.ey_int, -1.0f, 1.0f);
        imu_data.ez_int = LIMIT(imu_data.ez_int, -1.0f, 1.0f);
    }

    // 根据复位状态调整参数
    float kp_use = imu_data.gkp;
    
    // 重力方向快速复位
    if(imu_data.g_reset) {
        kp_use = 10.0f;  // 快速对齐时使用大增益
        if(ABS(ex) < 0.01f && ABS(ey) < 0.01f) {
            imu_data.g_reset = 0;  // 误差足够小时退出快速复位
        }
    }

    // 应用比例-积分校正（修改临时数组，不污染原始数据）
    // 重力方向校正
    imu_debug_ex = ex;
    imu_debug_ey = ey;
    imu_debug_ez = ez;
    imu_debug_ex_int = imu_data.ex_int;
    imu_debug_ey_int = imu_data.ey_int;
    imu_debug_ez_int = imu_data.ez_int;
    imu_debug_kp = kp_use;
    imu_debug_ki = imu_data.gki;
    imu_debug_acc_weight = adaptive_weight;
    imu_debug_gyro_rad_norm = gyro_rad_norm;

    gyro_temp[0] += kp_use*ex + imu_data.ex_int;
    gyro_temp[1] += kp_use*ey + imu_data.ey_int;
    gyro_temp[2] += kp_use*ez + imu_data.ez_int;
    
    // 使用陀螺仪积分获取航向角
    /*float gyro_yaw_rate = gyro_temp[2] * 180.0f / 3.14159f;  // rad/s -> deg/s
    imu_data.yaw += gyro_yaw_rate * dt;
    
    // 限制范围在 -180 ~ 180
    while(imu_data.yaw > 180.0f) imu_data.yaw -= 360.0f;
    while(imu_data.yaw < -180.0f) imu_data.yaw += 360.0f;*/

    // 四元数积分
    quaternionRate(imu_data.quaternion, gyro_temp, dt);
    
    // 更新方向余弦矩阵
    buildDirectionCosineMatrix(imu_data.quaternion, imu_data.att_matrix);
    
    // 更新欧拉角（roll和pitch从四元数计算，yaw用陀螺仪积分）
    quaternionToEuler(imu_data.quaternion, &imu_data.roll, &imu_data.pitch, &imu_data.yaw);
  
}
// ==================== 姿态解算相关函数 ====================
static uint16_t zupt_stable_cnt = 0;
static uint32 imu_last_sample_us = 0;
static uint32 imu_last_valid_sample_us = 0;
static uint8 imu_raw_frame_ready = 0;
 uint32 imu_bad_frame_count = 0;
static float imu_last_gyro_valid[3] = {0.0f, 0.0f, 0.0f};
static float imu_last_acc_valid[3] = {0.0f, 0.0f, 1.0f};

#define ICM42688_CHIP_ID_VALUE          (0x47)
#define IMU_GYRO_ABS_MAX_DPS      (1500.0f)
#define IMU_GYRO_SPIKE_DELTA_DPS  (400.0f)

#define IMU_ACC_ABS_MAX_G         (4.0f)
#define IMU_ACC_NORM_MIN_G        (0.25f)
#define IMU_ACC_NORM_MAX_G        (9.0f)
#define IMU_ACC_SPIKE_DELTA_G     (1.5f)
#define IMU_STATIC_GYRO_DPS       (0.12f)
#define IMU_STATIC_ACC_ERR_G      (0.03f)

static uint8 imu_gyro_frame_is_valid(const float gyro[3])
{
    for(int i = 0; i < 3; i++) {
        if(ABS(gyro[i]) > IMU_GYRO_ABS_MAX_DPS) {
            imu_raw_frame_ready = 0;
            return 0;
        }
    }

    imu_raw_frame_ready = 1;
    return 1;
}

static uint8 imu_acc_frame_is_valid(const float acc[3])
{
    float acc_norm = acc[0]*acc[0] + acc[1]*acc[1] + acc[2]*acc[2];

    if((acc_norm < IMU_ACC_NORM_MIN_G) ||
       (acc_norm > IMU_ACC_NORM_MAX_G)) {
        return 0;
    }

    for(int i = 0; i < 3; i++) {
        if(ABS(acc[i]) > IMU_ACC_ABS_MAX_G) {
            return 0;
        }
    }

    return 1;
}

static void imu_record_valid_frame(const float gyro[3], const float acc[3])
{

    // spike 检测只用于计数/日志，不要直接卡死更新
    if(imu_raw_frame_ready) {
        for(int i = 0; i < 3; i++) {
            if(ABS(gyro[i] - imu_last_gyro_valid[i]) > IMU_GYRO_SPIKE_DELTA_DPS ||
               ABS(acc[i] - imu_last_acc_valid[i]) > IMU_ACC_SPIKE_DELTA_G) {
                imu_bad_frame_count++;

                // 关键：不要 return 0
                // 允许这帧通过，否则会永远 stuck 在旧值
                break;
            }
        }
    }

    for(int i = 0; i < 3; i++) {
        imu_last_gyro_valid[i] = gyro[i];
        imu_last_acc_valid[i] = acc[i];
    }
}
void imu_get_gyro(int16_t *gyro_x, int16_t *gyro_y, int16_t *gyro_z)
{
#if IMU_TYPE == IMU_TYPE_ICM42688    
    icm42688_get_gyro();
    *gyro_x = icm42688_gyro_x;
    *gyro_y = icm42688_gyro_y;
    *gyro_z = icm42688_gyro_z;
#elif IMU_TYPE == IMU_TYPE_IMU660RB
    imu660rb_get_gyro();
    *gyro_x = imu660rb_gyro_x;
    *gyro_y = imu660rb_gyro_y;
    *gyro_z = imu660rb_gyro_z;
#endif

}
void imu_get_acc(int16_t *acc_x, int16_t *acc_y, int16_t *acc_z)
{   
#if IMU_TYPE == IMU_TYPE_ICM42688    
    icm42688_get_acc();
    *acc_x = icm42688_acc_x;
    *acc_y = icm42688_acc_y;
    *acc_z = icm42688_acc_z;    
#elif IMU_TYPE == IMU_TYPE_IMU660RB
    imu660rb_get_acc();
    *acc_x = imu660rb_acc_x;
    *acc_y = imu660rb_acc_y;
    *acc_z = imu660rb_acc_z;
#endif
}

float imu_gyro_transition(int16_t gyro_raw)
{
#if IMU_TYPE == IMU_TYPE_ICM42688
    return icm42688_gyro_transition(gyro_raw);
#elif IMU_TYPE == IMU_TYPE_IMU660RB
    return imu660rb_gyro_transition(gyro_raw);
#else
    return 0.0f;
#endif
}

float imu_acc_transition(int16_t acc_raw)
{
#if IMU_TYPE == IMU_TYPE_ICM42688
    return icm42688_acc_transition(acc_raw);
#elif IMU_TYPE == IMU_TYPE_IMU660RB
    return imu660rb_acc_transition(acc_raw);
#else
    return 0.0f;
#endif
}

float imu_acc_1g_raw(void)
{
#if IMU_TYPE == IMU_TYPE_ICM42688
    return icm42688_transition_factor[0];
#elif IMU_TYPE == IMU_TYPE_IMU660RB
    return 1.0f / imu660rb_acc_transition(1);
#else
    return 0.0f;
#endif
}

static void imu_map_gyro_to_body(float raw_gyro_x, float raw_gyro_y, float raw_gyro_z,
                                 float *gyro_x, float *gyro_y, float *gyro_z)
{
    *gyro_x = -raw_gyro_x;
    *gyro_y =  raw_gyro_y;
    *gyro_z = -raw_gyro_z;
}

uint8 imu_chip_id_is_ok(void)
{
#if IMU_TYPE == IMU_TYPE_ICM42688
    return (ICM42688_CHIP_ID_VALUE == icm42688_get_chip_id());
#elif IMU_TYPE == IMU_TYPE_IMU660RB
    return 1;
#else
    return 0;
#endif
}

void imu_calc(void)
{
    imu_update_cnt++;
    uint32 current_time_us = system_time_us();
    float dt = (current_time_us - imu_last_sample_us) / 1000000.0f; // 修复致命Bug：微秒转换秒
    if(dt <= 0.0f || dt > 0.1f)
    {
        dt = IMU_SAMPLE_TIME;
    }
    imu_last_sample_us = current_time_us;

    if(!imu_chip_id_is_ok()) {
        imu_bad_frame_count++;
        return;
    }

    imu_get_gyro(&imu_data.gyro_data[0], &imu_data.gyro_data[1], &imu_data.gyro_data[2]);
    imu_data.gyro_raw[2]=imu_data.gyro_data[2];
    imu_data.gyro_raw[0]=imu_data.gyro_data[0];
    imu_data.gyro_raw[1]=imu_data.gyro_data[1];

    float raw_gyro_x=imu_gyro_transition(imu_data.gyro_raw[0]);
    float raw_gyro_y=imu_gyro_transition(imu_data.gyro_raw[1]);
    float raw_gyro_z=imu_gyro_transition(imu_data.gyro_raw[2]);

    imu_get_acc(&imu_data.acc_data[0], &imu_data.acc_data[1], &imu_data.acc_data[2]);
    imu_data.acc_raw[2]=imu_data.acc_data[2]-imu_data.acc_offset[2];
    imu_data.acc_raw[0]=imu_data.acc_data[0]-imu_data.acc_offset[0];
    imu_data.acc_raw[1]=imu_data.acc_data[1]-imu_data.acc_offset[1];

    float raw_acc_x=imu_acc_transition(imu_data.acc_raw[0]);
    float raw_acc_y=imu_acc_transition(imu_data.acc_raw[1]);
    float raw_acc_z=imu_acc_transition(imu_data.acc_raw[2]);

    // Raw frame -> body frame. Gyro and acc must use the same axis map.
    float gyro_x;
    float gyro_y;
    float gyro_z;
    imu_map_gyro_to_body(raw_gyro_x, raw_gyro_y, raw_gyro_z, &gyro_x, &gyro_y, &gyro_z);
    gyro_x -= imu_data.gyro_offset_actual[0];
    gyro_y -= imu_data.gyro_offset_actual[1];
    gyro_z -= imu_data.gyro_offset_actual[2];

    float body_acc_x=-raw_acc_x;
    float body_acc_y= raw_acc_y;
    float body_acc_z=-raw_acc_z;


    float acc_x=-body_acc_x;
    float acc_y=-body_acc_y;
    float acc_z=-body_acc_z;
    float gyro_raw_frame[3] = {gyro_x, gyro_y, gyro_z};
    float acc_raw_frame[3] = {acc_x, acc_y, acc_z};

    if(!imu_gyro_frame_is_valid(gyro_raw_frame)) {
        imu_bad_frame_count++;
        return;
    }

    if(imu_acc_frame_is_valid(acc_raw_frame))
    {
        imu_record_valid_frame(gyro_raw_frame, acc_raw_frame);
    }
    else
    {
        /* Acceleration can be unreliable during a manoeuvre while the gyro
         * still provides a usable attitude propagation signal.  Do not freeze
         * the attitude loop or turn that transient into an IMU-loss failsafe. */
        imu_bad_frame_count++;
        acc_x = imu_last_acc_valid[0];
        acc_y = imu_last_acc_valid[1];
        acc_z = imu_last_acc_valid[2];
    }
    imu_last_valid_sample_us = current_time_us;

    imu_log_gyro_unfiltered[0] = gyro_x;
    imu_log_gyro_unfiltered[1] = gyro_y;
    imu_log_gyro_unfiltered[2] = gyro_z;
    imu_data.gyro_unfiltered[0] = gyro_x;
    imu_data.gyro_unfiltered[1] = gyro_y;
    imu_data.gyro_unfiltered[2] = gyro_z;
    imu_log_acc_unfiltered[0] = acc_x;
    imu_log_acc_unfiltered[1] = acc_y;
    imu_log_acc_unfiltered[2] = acc_z;

   
    imu_data.gyro_actual[0]=iir2_low_pass_filter_update(&imu_data.gyro_iir2_filter[0], gyro_x);
    imu_data.gyro_actual[1]=iir2_low_pass_filter_update(&imu_data.gyro_iir2_filter[1], gyro_y);
    imu_data.gyro_actual[2]=iir2_low_pass_filter_update(&imu_data.gyro_iir2_filter[2], gyro_z);

    // 计算当前的角速度模长 (deg/s) 和加速度模长 (g) 
    // 采用滤波后的角速度进行判断，过滤高频电噪声尖峰，防止ZUPT被误打断
    float gyro_norm = sqrt(imu_data.gyro_actual[0]*imu_data.gyro_actual[0] + imu_data.gyro_actual[1]*imu_data.gyro_actual[1] + imu_data.gyro_actual[2]*imu_data.gyro_actual[2]);
    float acc_norm_raw = sqrt(acc_x*acc_x + acc_y*acc_y + acc_z*acc_z);
    uint8 imu_static_candidate = (gyro_norm < 0.5f && ABS(acc_norm_raw - 1.0f) < 0.05f);
    
    // ZUPT静止检测：进一步放宽门限防止因轻微震动漏识别；同时大幅提高收敛速度
    if(imu_static_candidate) { 
        zupt_stable_cnt++;
        if(zupt_stable_cnt > 100) 
        { // 连续稳定超过1秒，更新零偏
            imu_data.gyro_offset_actual[0] += gyro_x * 0.002f;
            imu_data.gyro_offset_actual[1] += gyro_y * 0.002f;
            imu_data.gyro_offset_actual[2] += gyro_z * 0.002f;
            zupt_stable_cnt = 100; // 防止溢出
        }
    } else {
        zupt_stable_cnt = 0; // 一旦飞机有任何转动或震动，立刻停止更新零偏！
    }

    // 应用滤波器
    imu_data.acc_actual[2]=iir2_low_pass_filter_update(&imu_data.acc_iir2_filter[2], acc_z);
    imu_data.acc_actual[0]=iir2_low_pass_filter_update(&imu_data.acc_iir2_filter[0], acc_x);
    imu_data.acc_actual[1]=iir2_low_pass_filter_update(&imu_data.acc_iir2_filter[1], acc_y);
  

    // 创建临时局部变量转成 rad/s 供积分运算使用，绝不污染全局的 gyro_actual (deg/s)
    float gyro_rad[3];
    gyro_rad[0] = imu_data.gyro_actual[0] * (3.1415926f / 180.0f);
    gyro_rad[1] = imu_data.gyro_actual[1] * (3.1415926f / 180.0f);
    gyro_rad[2] = imu_data.gyro_actual[2] * (3.1415926f / 180.0f);

    if(gyro_norm < IMU_STATIC_GYRO_DPS && ABS(acc_norm_raw - 1.0f) < IMU_STATIC_ACC_ERR_G) {
        gyro_rad[0] = 0.0f;
        gyro_rad[1] = 0.0f;
        gyro_rad[2] = 0.0f;
    } else if(imu_static_candidate && zupt_stable_cnt > 20) 
    {
        // 六轴 IMU 没有 yaw 绝对参考，静止后必须切断 Z 轴残余零偏继续积分。
        gyro_rad[2] = 0.0f;
    }

    // 姿态解算会在内部归一化加速度；必须传副本，不能污染高度 EKF 使用的真实物理加速度幅值。
    float acc_for_att[3] = {
        imu_data.acc_actual[0],
        imu_data.acc_actual[1],
        imu_data.acc_actual[2]
    };
    imu_mahony_filter(gyro_rad, acc_for_att, dt);

    // === 将解算结果输入到全局飞控状态结构体 ===
    vehicle_state.current_roll  = imu_data.roll;
    vehicle_state.current_pitch = imu_data.pitch;
    vehicle_state.current_yaw   = imu_data.yaw;
    
    // 提前计算好三角函数，供光流和TOF做倾角补偿使用 (统一转换为弧度)
    vehicle_state.roll_sin  = sinf(imu_data.roll  * 3.1415926f / 180.0f);
    vehicle_state.roll_cos  = cosf(imu_data.roll  * 3.1415926f / 180.0f);
    vehicle_state.pitch_sin = sinf(imu_data.pitch * 3.1415926f / 180.0f);
    vehicle_state.pitch_cos = cosf(imu_data.pitch * 3.1415926f / 180.0f);
    vehicle_state.yaw_sin   = sinf(imu_data.yaw   * 3.1415926f / 180.0f);
    vehicle_state.yaw_cos   = cosf(imu_data.yaw   * 3.1415926f / 180.0f);

    attitude_history_record(current_time_us,
                            imu_data.roll,
                            imu_data.pitch,
                            imu_data.yaw);
    vision_attitude_shared_publish(current_time_us,
                                   imu_data.roll,
                                   imu_data.pitch,
                                   imu_data.yaw,
                                   vehicle_state.current_height);

}

// 初始化函数
void imu_data_init(void)
{
    attitude_history_reset();
    // 初始化滤波器
    init_filters();
    
    // 初始化方向余弦矩阵
    init_att_matrix();
    
    // 初始化四元数为单位四元数
    imu_data.quaternion[0] = 1.0f;
    imu_data.quaternion[1] = 0.0f;
    imu_data.quaternion[2] = 0.0f;
    imu_data.quaternion[3] = 0.0f;
    
    // 初始化欧拉角
    imu_data.roll = 0.0f;
    imu_data.pitch = 0.0f;
    imu_data.yaw = 0.0f;
    
    // 初始化积分项
    imu_data.ex_int = 0.0f;
    imu_data.ey_int = 0.0f;
    imu_data.ez_int = 0.0f;
    
    // 初始化状态标志
    imu_data.cal_enable = 0;
    
    // 初始化偏移量
    for(int i = 0; i < 3; i++) {
        imu_data.gyro_offset[i] = 0.0f;
        imu_data.gyro_offset_actual[i] = 0.0f;
        imu_data.acc_offset[i] = 0.0f;
    
        imu_data.gyro_actual[i] = 0.0f;
        imu_data.gyro_unfiltered[i] = 0.0f;
        imu_data.acc_actual[i] = 0.0f;
 
    }
    
    // 初始化转角
    imu_data.turn_angle = 0.0f;

    imu_raw_frame_ready = 0;
    imu_last_valid_sample_us = 0;
    imu_bad_frame_count = 0;
    imu_last_gyro_valid[0] = 0.0f;
    imu_last_gyro_valid[1] = 0.0f;
    imu_last_gyro_valid[2] = 0.0f;
    imu_last_acc_valid[0] = 0.0f;
    imu_last_acc_valid[1] = 0.0f;
    imu_last_acc_valid[2] = 1.0f;
}

uint32 imu_get_bad_frame_count(void)
{
    return imu_bad_frame_count;
}

uint8 imu_attitude_is_valid(void)
{
    float quat_norm;
    uint32 now_us;

    if(imu_raw_frame_ready == 0u)
    {
        return 0u;
    }

    now_us = system_time_us();
    if((imu_last_valid_sample_us == 0u) ||
       ((now_us - imu_last_valid_sample_us) > 100000u))
    {
        return 0u;
    }

    quat_norm = imu_data.quaternion[0] * imu_data.quaternion[0] +
                imu_data.quaternion[1] * imu_data.quaternion[1] +
                imu_data.quaternion[2] * imu_data.quaternion[2] +
                imu_data.quaternion[3] * imu_data.quaternion[3];
    if((quat_norm < 0.8f) || (quat_norm > 1.2f))
    {
        return 0u;
    }

    return 1u;
}

uint8 imu_is_valid(void)
{
    float acc_norm;

    if(imu_attitude_is_valid() == 0u)
    {
        return 0u;
    }

    /* Keep the stricter acceleration sanity check for preflight only. */
    acc_norm = sqrt(imu_data.acc_actual[0] * imu_data.acc_actual[0] +
                    imu_data.acc_actual[1] * imu_data.acc_actual[1] +
                    imu_data.acc_actual[2] * imu_data.acc_actual[2]);
    return ((acc_norm >= 0.5f) && (acc_norm <= 2.0f)) ? 1u : 0u;
}


void imu_reset_turn_angle(void)
{
     imu_data.turn_angle=0.0f;
}

// 获取姿态角
void imu_get_attitude(float *roll, float *pitch, float *yaw)
{
    *roll = imu_data.roll;   // 已经是度
    *pitch = imu_data.pitch;
    *yaw = imu_data.yaw;
}

// 获取四元数
void imu_get_quaternion(float q[4])
{
    q[0] = imu_data.quaternion[0];
    q[1] = imu_data.quaternion[1];
    q[2] = imu_data.quaternion[2];
    q[3] = imu_data.quaternion[3];
}

// 获取方向余弦矩阵
void imu_get_direction_cosine_matrix(float matrix[3][3])
{
    for(int i = 0; i < 3; i++) {
        for(int j = 0; j < 3; j++) {
            matrix[i][j] = imu_data.att_matrix[i][j];
        }
    }
}

// 进行坐标转换
void imu_transform_vector_body_to_world(float body_vec[3], float world_vec[3])
{
    body_to_world_trans(body_vec, world_vec);
}

void imu_transform_vector_world_to_body(float world_vec[3], float body_vec[3])
{
    world_to_body_trans(world_vec, body_vec);
}


// 获取载体坐标系下的重力向量
void imu_get_gravity_vector(float gravity[3])
{
    // 从方向余弦矩阵获取载体坐标系下的重力向量
    for(int i = 0; i < 3; i++) {
        gravity[i] = imu_data.att_matrix[2][i]; // Z轴向量（重力方向）
    }
}

// 获取载体坐标系下的线性加速度
void imu_get_linear_acceleration(float linear_acc[3])
{
    float gravity[3];
    imu_get_gravity_vector(gravity);
    
    // 计算线性纯运动加速度
    for(int i = 0; i < 3; i++) {
        // 恢复：由于目前统一使用了原始Z轴朝上，静止时测量值 A = 1g，此时 gravity 也等于 1g。
        // 只有两者相减，才能抵消重力得到纯运动的 0g。
        linear_acc[i] = (imu_data.acc_actual[i] - gravity[i]) * 9.81f; 
    }

    imu_debug_acc_actual_z = imu_data.acc_actual[2];
    imu_debug_gravity_z = gravity[2];
    imu_debug_linear_acc_body_x = linear_acc[0];
    imu_debug_linear_acc_body_y = linear_acc[1];
    imu_debug_linear_acc_body_z = linear_acc[2];
    imu_debug_att_m20 = imu_data.att_matrix[2][0];
    imu_debug_att_m21 = imu_data.att_matrix[2][1];
    imu_debug_att_m22 = imu_data.att_matrix[2][2];
}

// 获取完整姿态信息
void imu_get_full_attitude(float *roll, float *pitch, float *yaw, float *quat)
{
    if(roll) *roll = imu_data.roll;
    if(pitch) *pitch = imu_data.pitch;
    if(yaw) *yaw = imu_data.yaw;
    if(quat) {
        quat[0] = imu_data.quaternion[0]; // w
        quat[1] = imu_data.quaternion[1]; // x
        quat[2] = imu_data.quaternion[2]; // y
        quat[3] = imu_data.quaternion[3]; // z
    }
}

// 设置姿态解算参数
void imu_set_parameters(float kp, float ki, float mag_kp)
{
    imu_data.gkp = kp;
    imu_data.gki = ki;

}

// 启动快速复位
void imu_start_quick_reset(uint8_t g_reset, uint8_t m_reset)
{
    imu_data.g_reset = g_reset;

}

// 系统毫秒时间
// timer_get(TIMER_US) divides the 32-bit 8 MHz hardware counter by 8 after
// reading it.  Therefore the value returned here wraps every 2^29 us, not
// every 2^32 us.  Extend that raw value locally so control-loop timestamps
// remain monotonic (with the normal uint32 wrap behaviour at 2^32 us).
#define SYSTEM_TIME_RAW_WRAP_US   (1u << 29)

static uint8  system_time_timer_ready = 0;
static uint32 system_time_last_raw_us = 0;
static uint32 system_time_epoch_us    = 0;
uint32 system_time_us(void)
{
    uint32 raw_time_us;
    uint32 primask;

    if(system_time_timer_ready == 0)
    {
        timer_init(TC_TIME2_CH0, TIMER_US);
        timer_clear(TC_TIME2_CH0);
        timer_start(TC_TIME2_CH0);
        system_time_timer_ready = 1;

        system_time_last_raw_us = timer_get(TC_TIME2_CH0);
        system_time_epoch_us    = 0;
        return system_time_last_raw_us;
    }

    // system_time_us() is used by both periodic control work and logging.
    // Keep the wrap detection/update as one transaction if one caller is
    // interrupted by another near the raw-counter wrap boundary.
    primask = __get_PRIMASK();
    __disable_irq();

    raw_time_us = timer_get(TC_TIME2_CH0);
    if(raw_time_us < system_time_last_raw_us)
    {
        system_time_epoch_us += SYSTEM_TIME_RAW_WRAP_US;
    }
    system_time_last_raw_us = raw_time_us;

    raw_time_us += system_time_epoch_us;
    __set_PRIMASK(primask);

    return raw_time_us;
}
