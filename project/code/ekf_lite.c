#include "zf_common_headfile.h"

// EKF（轻量级扩展卡尔曼滤波）状态量实例
ekf_lite_state_t ekf_lite_state = {0};
// EKF 健康状态与标志位实例
ekf_lite_health_t ekf_lite_health = {0};

// Z轴(高度和垂直速度)的协方差矩阵 P (2x2矩阵)
float ekf_lite_p_z[2][2] = {{100.0f, 0.0f}, {0.0f, 100.0f}};
// XY轴(水平位置和速度)的协方差矩阵 P 的对角线元素 (x, y, vx, vy)
float ekf_lite_p_xy[4] = {200.0f, 200.0f, 400.0f, 400.0f};

// 求浮点数的绝对值
static float ekf_lite_absf(float value)
{
    return (value < 0.0f) ? -value : value;
}

// 浮点数限幅函数，限制在 [min_value, max_value] 范围内
static float ekf_lite_clampf(float value, float min_value, float max_value)
{
    if(value < min_value) return min_value;
    if(value > max_value) return max_value;
    return value;
}

// 将 EKF 状态强制重置为飞行器当前状态（通常在初始化或异常恢复时调用）
void ekf_lite_reset_to_vehicle_state(void)
{
    // 重置状态量（位置和速度）
    ekf_lite_state.x = vehicle_state.current_pos_x;
    ekf_lite_state.y = vehicle_state.current_pos_y;
    ekf_lite_state.z = vehicle_state.current_height;
    ekf_lite_state.vx = vehicle_state.current_vel_x;
    ekf_lite_state.vy = vehicle_state.current_vel_y;
    ekf_lite_state.vz = vehicle_state.current_vel_z;
    ekf_lite_state.tof_bias = 0.0f;
    ekf_lite_state.flow_scale = 1.0f;

    // 恢复 Z 轴协方差初始值（表示初始不确定度较大）
    ekf_lite_p_z[0][0] = 100.0f;
    ekf_lite_p_z[0][1] = 0.0f;
    ekf_lite_p_z[1][0] = 0.0f;
    ekf_lite_p_z[1][1] = 100.0f;

    // 恢复 XY 轴协方差初始值
    ekf_lite_p_xy[0] = 200.0f;
    ekf_lite_p_xy[1] = 200.0f;
    ekf_lite_p_xy[2] = 400.0f;
    ekf_lite_p_xy[3] = 400.0f;

    // 重置健康状态标志位
    ekf_lite_health.inited = 1u;
    ekf_lite_health.tof_used = 0u;
    ekf_lite_health.flow_used = 0u;
    ekf_lite_health.healthy = 1u;
    ekf_lite_health.tof_innov = 0.0f;
    ekf_lite_health.flow_innov_x = 0.0f;
    ekf_lite_health.flow_innov_y = 0.0f;
}

void ekf_lite_init(void)
{
    ekf_lite_reset_to_vehicle_state();
}
/*
static void ekf_lite_predict_height(float dT_s)
{h
    float linear_acc_body[3];
    float linear_acc_world[3];
    float acc_z_cm_s2;

    imu_get_linear_acceleration(linear_acc_body);
    imu_transform_vector_body_to_world(linear_acc_body, linear_acc_world);
    acc_z_cm_s2 = -linear_acc_world[2] * 100.0f;
    acc_z_cm_s2 = ekf_lite_clampf(acc_z_cm_s2, -600.0f, 600.0f);

    ekf_lite_state.z += ekf_lite_state.vz * dT_s + 0.5f * acc_z_cm_s2 * dT_s * dT_s;
    ekf_lite_state.vz += acc_z_cm_s2 * dT_s;

    ekf_lite_p_z[0][0] += ekf_lite_p_z[1][1] * dT_s * dT_s + 6.0f;
    ekf_lite_p_z[0][1] += ekf_lite_p_z[1][1] * dT_s;
    ekf_lite_p_z[1][0] = ekf_lite_p_z[0][1];
    ekf_lite_p_z[1][1] += 80.0f * dT_s;
}

static void ekf_lite_update_height(void)
{
    const float r_tof = 25.0f;
    float z_meas = vehicle_state.current_height - ekf_lite_state.tof_bias;
    float innov = z_meas - ekf_lite_state.z;
    float s;
    float k0;
    float k1;
    float p00;
    float p01;
    float p10;
    float p11;

    ekf_lite_health.tof_innov = innov;
    ekf_lite_health.tof_used = 0u;

    // 如果高度处于不合理范围，拒绝使用本次TOF数据更新
    if(vehicle_state.current_height < 3.0f || vehicle_state.current_height > 180.0f)
    {
        return;
    }

    // 新息门控 (Gate)：如果误差过大(>35cm)，说明可能是地形突变或干扰，拒绝更新
    if(ekf_lite_absf(innov) > EKF_LITE_TOF_GATE_CM)
    {
        return;
    }

    // 计算新息协方差 S = H * P * H^T + R
    s = ekf_lite_p_z[0][0] + r_tof;
    if(s < 0.001f)
    {
        return;
    }

    p00 = ekf_lite_p_z[0][0];
    p01 = ekf_lite_p_z[0][1];
    p10 = ekf_lite_p_z[1][0];
    p11 = ekf_lite_p_z[1][1];

    // 计算卡尔曼增益 K = P * H^T * S^-1
    k0 = p00 / s;
    k1 = p10 / s;

    // 状态更新：X = X + K * innov
    ekf_lite_state.z += k0 * innov;
    ekf_lite_state.vz += k1 * innov;

    // 协方差更新：P = (I - K * H) * P
    ekf_lite_p_z[0][0] = p00 - k0 * p00;
    ekf_lite_p_z[0][1] = p01 - k0 * p01;
    ekf_lite_p_z[1][0] = p10 - k1 * p00;
    ekf_lite_p_z[1][1] = p11 - k1 * p01;

    // 对最终状态进行限幅，防止滤波发散
    ekf_lite_state.z = ekf_lite_clampf(ekf_lite_state.z, 0.0f, 220.0f);
    ekf_lite_state.vz = ekf_lite_clampf(ekf_lite_state.vz, -250.0f, 250.0f);
    ekf_lite_health.tof_used = 1u;
}

// EKF 预测步：XY水平位置与速度 (简单的匀速运动模型)
static void ekf_lite_predict_xy(float dT_s)
{
    // 位置 = 速度 * dt
    ekf_lite_state.x += ekf_lite_state.vx * dT_s;
    ekf_lite_state.y += ekf_lite_state.vy * dT_s;

    // 更新位置和速度的协方差，并加入过程噪声 Q
    ekf_lite_p_xy[0] += ekf_lite_p_xy[2] * dT_s * dT_s + 5.0f;
    ekf_lite_p_xy[1] += ekf_lite_p_xy[3] * dT_s * dT_s + 5.0f;
    ekf_lite_p_xy[2] += 120.0f * dT_s;
    ekf_lite_p_xy[3] += 120.0f * dT_s;
}

// EKF 更新步：XY水平速度 (利用光流传感器数据校正)
static void ekf_lite_update_flow(void)
{
    const float r_flow = 900.0f; // 光流测量的噪声协方差 R
    float innov_x = vehicle_state.current_vel_x - ekf_lite_state.vx; // X轴新息
    float innov_y = vehicle_state.current_vel_y - ekf_lite_state.vy; // Y轴新息
    float kx;
    float ky;

    ekf_lite_health.flow_innov_x = innov_x;
    ekf_lite_health.flow_innov_y = innov_y;
    ekf_lite_health.flow_used = 0u;

    // 检查光流数据是否有效
    if(vehicle_state.flow_valid == 0u)
    {
        return;
    }

    // 如果飞行器倾角过大(>25度)，光流容易失真或丢失，拒绝更新
    if(ekf_lite_absf(imu_data.roll) > 25.0f || ekf_lite_absf(imu_data.pitch) > 25.0f)
    {
        return;
    }

    // 剧烈偏航(Yaw)旋转时也会导致光流估算偏差大，拒绝更新
    if(ekf_lite_absf(imu_data.gyro_actual[2]) > 120.0f)
    {
        return;
    }

    // 新息门控：如果速度误差过大(>180cm/s)，拒绝更新以防干扰
    if(ekf_lite_absf(innov_x) > EKF_LITE_FLOW_GATE_CMS || ekf_lite_absf(innov_y) > EKF_LITE_FLOW_GATE_CMS)
    {
        return;
    }

    // 计算卡尔曼增益 K (这里仅简化为标量计算，因为 X Y 是解耦的)
    kx = ekf_lite_p_xy[2] / (ekf_lite_p_xy[2] + r_flow);
    ky = ekf_lite_p_xy[3] / (ekf_lite_p_xy[3] + r_flow);

    // 状态更新与协方差更新
    ekf_lite_state.vx += kx * innov_x;
    ekf_lite_state.vy += ky * innov_y;
    ekf_lite_p_xy[2] *= (1.0f - kx);
    ekf_lite_p_xy[3] *= (1.0f - ky);

    ekf_lite_state.vx = ekf_lite_clampf(ekf_lite_state.vx, -350.0f, 350.0f);
    ekf_lite_state.vy = ekf_lite_clampf(ekf_lite_state.vy, -350.0f, 350.0f);
    ekf_lite_health.flow_used = 1u;
}

// EKF 核心主函数 (需周期性调用)
void ekf_lite_update(float dT_s)
{
#if EKF_LITE_ENABLE
    // 限制最大时间步长，防止过度积分导致发散
    if(dT_s <= 0.0f || dT_s > EKF_LITE_MAX_DT_S)
    {
        dT_s = 0.02f;
    }

    // 若 EKF 尚未初始化，则执行初始化
    if(ekf_lite_health.inited == 0u)
    {
        ekf_lite_reset_to_vehicle_state();
    }

    // 执行卡尔曼滤波的预测和更新
    ekf_lite_predict_height(dT_s);
    ekf_lite_predict_xy(dT_s);
    ekf_lite_update_height();
    ekf_lite_update_flow();

    // 根据高度状态判断 EKF 系统是否健康
    ekf_lite_health.healthy = (ekf_lite_state.z >= 0.0f && ekf_lite_state.z <= 220.0f) ? 1u : 0u;

#if EKF_LITE_PUBLISH_ENABLE
    // 将滤波后的最优估计值发布给主控 vehicle_state（影子模式不开启）
    if(ekf_lite_health.healthy != 0u)
    {
        vehicle_state.current_height = ekf_lite_state.z;
        vehicle_state.current_vel_z = ekf_lite_state.vz;
        vehicle_state.current_pos_x = ekf_lite_state.x;
        vehicle_state.current_pos_y = ekf_lite_state.y;
        vehicle_state.current_vel_x = ekf_lite_state.vx;
        vehicle_state.current_vel_y = ekf_lite_state.vy;
    }
#endif

#if EKF_LITE_DEBUG_ENABLE
    // Debug 分频打印
    static uint16_t debug_div = 0u;
    if(debug_div++ >= EKF_LITE_DEBUG_DIV)
    {
        debug_div = 0u;
        printf("[EKF_LITE] z=%.2f vz=%.2f x=%.2f y=%.2f vx=%.2f vy=%.2f tof=%d flow=%d iz=%.2f iv=(%.2f,%.2f)\r\n",
               ekf_lite_state.z,
               ekf_lite_state.vz,
               ekf_lite_state.x,
               ekf_lite_state.y,
               ekf_lite_state.vx,
               ekf_lite_state.vy,
               ekf_lite_health.tof_used,
               ekf_lite_health.flow_used,
               ekf_lite_health.tof_innov,
               ekf_lite_health.flow_innov_x,
               ekf_lite_health.flow_innov_y);
    }
#endif
#else
    (void)dT_s;
#endif
}
*/