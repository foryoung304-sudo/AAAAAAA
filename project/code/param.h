#ifndef __PARAM_H__
#define __PARAM_H__ 

#include "zf_common_headfile.h"
#include "fisheye_lut.h"

#define JOYSTICK_MAX_VALUE    2048
#define MAX_VEL_XYZ            200.0f   //最大xy轴速度(cm/s)
#define MAX_YAW_RATE            20.0f   //最大yaw轴速度(度/s)
#define MAX_HEIGHT              150.0f   //最大高度(cm)
#define MIN_HEIGHT              16.0f   // 重心离地的落地高度：原始ToF约6 cm + 10 cm安装偏置

#define CAMERA_FOCAL_LENGTH_PIXEL     60.0f   //相机焦距(像素) 广角镜头通常在50~80之间
#define IMAGE_CENTER_X 108.00f
#define IMAGE_CENTER_Y 60.00f

#define BEACON_SCALE_X 0.0198f  // 前后方向
#define BEACON_SCALE_Y 0.0127f  // 左右方向

// 相机位于机体中心后方10cm，机体系X前正、Y右正
#define CAMERA_OFFSET_BODY_X_CM (-10.0f)
#define CAMERA_OFFSET_BODY_Y_CM 0.0f

#define MAX_ROLL_PITCH          10.0f   //最大横滚俯仰角(度)


#define ATT_THROTTLE_HOLD_DEADZONE_CM_S  5.0f
#define ALT_TARGET_VEL_MAX_CM_S          3.5f
#define ALT_TARGET_VEL_DEADZONE_CM_S      0.3f
#define ALT_TARGET_VEL_SLEW_CM_S2         5.0f

#define AUTO_LAND_TRIGGER_HEIGHT_CM      22.0f
#define AUTO_LAND_TARGET_HEIGHT_CM       MIN_HEIGHT
/* In automatic landing, reaching this measured height disarms immediately. */
#define AUTO_LAND_DISARM_HEIGHT_CM       19.0f
#define AUTO_LAND_DISARM_VEL_CM_S        5.0f

/* Normal landing first captures one horizontal point and brakes at the
 * current height.  Descent starts only after the aircraft is genuinely
 * settled, with a bounded timeout so a cable load cannot hold it aloft
 * forever.  Sensor/Flow faults bypass the settling gate. */
#define AUTO_LAND_STATE_INACTIVE          0u
#define AUTO_LAND_STATE_SETTLING          1u
#define AUTO_LAND_STATE_DESCENDING        2u
#define AUTO_LAND_SETTLE_HORIZ_SPEED_CM_S 5.0f
#define AUTO_LAND_SETTLE_POS_ERR_CM        8.0f
#define AUTO_LAND_SETTLE_VZ_CM_S           5.0f
#define AUTO_LAND_SETTLE_STABLE_TIME_S     0.40f
#define AUTO_LAND_SETTLE_TIMEOUT_S         2.00f

#define PREFLIGHT_ERR_IMU                (1u << 0)
#define PREFLIGHT_ERR_TOF                (1u << 1)
#define PREFLIGHT_ERR_FLOW_NO_BYTES      (1u << 2)
#define PREFLIGHT_ERR_FLOW_NO_FRAME      (1u << 3)
#define PREFLIGHT_ERR_FLOW_QUALITY       (1u << 4)
#define PREFLIGHT_ERR_FLOW_NOT_READY     (1u << 5)


#define LOC_TEST_TARGET_HEIGHT_CM  150.0f
#define LOC_TEST_TARGET_POS_X_CM     0.0f
#define LOC_TEST_TARGET_POS_Y_CM     0.0f
#define MISSION_CRUISE_HEIGHT_ERR_CM 25.0f
#define MISSION_CRUISE_VZ_MAX_CM_S    8.0f
#define MISSION_CRUISE_STABLE_TIME_S  0.50f
#define MISSION_CRUISE_DROP_PAUSE_HEIGHT_CM 110.0f
#define MISSION_HEIGHT_RECOVERY_RELATIVE_SPEED_LIMIT_CM_S 25.0f


/* Competition flight always depends on optical-flow position control.  Do
 * not accept a single lucky frame: require a short continuous healthy window
 * before arming. */
#define PREFLIGHT_REQUIRE_FLOW           1u
#define PREFLIGHT_FLOW_READY_TIME_S      0.30f

/* In-flight faults are debounced separately.  Competition lighting and edge
 * geometry can cause short ToF/Flow dropouts, so give those sensors time to
 * recover before requesting the existing controlled auto-land path. */
#define FLIGHT_FAILSAFE_IMU_CONFIRM_S    0.30f
#define FLIGHT_FAILSAFE_TOF_CONFIRM_S    2.00f
#define FLIGHT_FAILSAFE_FLOW_CONFIRM_S   4.00f

/* Competition flight is autonomous after arming.  A radio dropout must not
 * become a landing or disarm command; set to 1 only for supervised builds
 * that intentionally use radio-loss auto-land. */
#define RADIO_LOSS_AUTOLAND_ENABLE       0

// 限幅宏
#define LIMIT(x, min, max) ((x) < (min) ? (min) : ((x) > (max) ? (max) : (x)))
#define ABS(x) ((x) < 0 ? -(x) : (x))

//---------------------运行周期--------------------//
#define SYS_BASE_FREQ           500.0f   //系统基础频率(Hz)
#define SYS_BASE_DT             (1.0f/SYS_BASE_FREQ)   //系统基础时间间隔(s)

//姿态周期
#define ATT_INNER_DIVIDER           1   //内环角速度环分频
#define ATT_INNER_DT                SYS_BASE_DT      //姿态控制时间间隔(s)

#define ATT_OUTER_DIVIDER           5   //外环角度环分频
#define ATT_OUTER_DT                (SYS_BASE_DT*ATT_OUTER_DIVIDER)   //姿态控制时间间隔(s)



// 数据周期
#define IMU_SAMPLE_TIME     0.002f   
#define FLOW_SAMPLE_TIME    0.02f
#define TOF_SAMPLE_TIME     0.02f
#define REMOTE_SAMPLE_TIME  0.02f

#define TEST_MODE_ENABLE     1
#define ATT_ONLY_MODE_ENABLE 0
#define LOC_ONLY_MODE_ENABLE 0
#define ALT_ONLY_MODE_ENABLE 0
#define VISION_MISSION_MODE_ENABLE 1
#define BEACON_APPROACH_TEST_ENABLE 0

#define FLY_MODE_ENABLE       0

#define TRIPOD_MODE_ENABLE    0

#if TEST_MODE_ENABLE
#if ((ATT_ONLY_MODE_ENABLE + LOC_ONLY_MODE_ENABLE + ALT_ONLY_MODE_ENABLE + \
      VISION_MISSION_MODE_ENABLE + BEACON_APPROACH_TEST_ENABLE + \
      FLY_MODE_ENABLE) > 1)
#error "Only one test mode can be enabled at a time."
#endif
#endif
extern volatile uint32_t imu_update_cnt;
extern volatile uint32_t att_ctrl_cnt;
extern volatile uint32_t motor_mix_cnt;
typedef struct
{

	//acc
	float acc_zero_offset[3];             //加速度计零偏
	float acc_sensitivity_ref[3];//1G
	//gyro
	float gyr_zero_offset[3];             //陀螺仪零偏
	//
	float body_central_pos_cm[3];   //重心相对传感器位置偏移量
	//
	float 	pid_att_1level[3][4]; //姿态控制角速度环PID参数
	float 	pid_att_2level[3][4]; //姿态控制角度环PID参数
	float 	pid_alt_1level[4];          //高度控制高度速度环PID参数
	float 	pid_alt_2level[4];           //高度控制高度环PID参数
	float 	pid_loc_1level[4];          //位置控制位置速度环PID参数
	float 	pid_loc_2level[4];           //位置控制位置环PID参数

	float   warn_power_voltage;
	int	    bat_cell;
	float   lowest_power_voltage;
	
	float	auto_take_off_height;
	float auto_take_off_speed;
	float auto_landing_speed;
	float idle_speed_pwm;
	
	//ins
	uint8	acc_calibrated;
	uint8	reserve1;
	uint8	reserve2;
	//
	uint8 frist_init;	//飞控第一次初始化，需要做一些特殊工作，比如清空flash	

}param_t;


typedef enum
{
    FLY_AUTOTAKEOFF=0,//自动起飞
    FLY_AUTOLANDING,//自动降落
    FLY_AUTOFLY,//自动飞行
    FLY_POS_HOLD,//定点模式
    FLY_HEIGHT_HOLD,//定高模式
}flight_mode_t;

// ==========================================
// 3. 动态期望值结构体
// ==========================================
typedef struct {
    // 位置与速度期望 (给 loc_ctrl 位置环用)
    float target_pos_x;           // 期望X轴位置 (cm)
    float target_pos_y;           // 期望Y轴位置 (cm)
    float target_vel_x;           // 期望X轴平移速度 (cm/s)
    float target_vel_y;           // 期望Y轴平移速度 (cm/s)
    
    // 高度与爬升期望 (给 alt_ctrl 高度环用)
    float target_height;          // 期望高度 (cm)
    float target_vel_z;           // 期望垂直爬升速度 (cm/s)
    float target_throttle;        // 期望油门 (0-100%)
    
    // 姿态角度期望 (给 att_ctrl 姿态环用)
    float target_roll;            // 期望横滚角 (度)
    float target_pitch;           // 期望俯仰角 (度)
    float target_yaw_rate;        // 期望偏航角速度 (度/秒)

} vehicle_setpoint_t;


// ==========================================
// 4. 动态物理状态结构体 (飞机当前的真实情况)
// ==========================================
typedef struct {
    uint8_t       armed;          // 电机是否解锁 (0:锁定, 1:解锁)
    uint8_t       flow_valid;     // 光流是否有效 (0:无效, 1:有效)
    flight_mode_t flight_mode;    // 当前处于什么飞行模式
    
    // 飞机当前的真实物理反馈
    float current_height;         // TOF 测出的真实高度 (cm)
    float current_vel_x;          // 光流测出的真实X轴速度 (cm/s)
    float current_vel_y;          // 光流测出的真实Y轴速度 (cm/s)
    float current_vel_z;          // 光流测出的真实Z轴速度 (cm/s)

    float current_pos_x;          
    float current_pos_y;          

    float current_roll;           // IMU 真实横滚角
    float current_pitch;          // IMU 真实俯仰角
    float current_yaw;            // IMU 真实偏航角
    float roll_sin;
    float roll_cos;
    float pitch_sin;
    float pitch_cos;
    float yaw_sin;
    float yaw_cos;
    float battery_voltage;
    float battery_voltage_filtered;
    float hover_throttle_base;
} vehicle_state_t;

typedef struct
{
        uint8_t *data;
        int width;
        int height;
        int step;
}image_t;
typedef struct
{
        float *data;
        int width;
        int height;
        int step;
}fimage_t;

typedef struct
{
    int16_t *data;
    int width;
    int height;
    int step;
}i16image_t;




// ==========================================
// 遥控器手动输入结构体 (解析后的干净数据)
// ==========================================
typedef struct 
{
    float roll;       // 期望横滚角 (度)
    float pitch;      // 期望俯仰角 (度)
    float yaw_rate;   // 期望偏航角速度 (度/秒)
    float climb_rate; // 期望爬升速度 (cm/s)
    int16 test_duty;  // 电机测试直接占空比 (0~10000)
} manual_input_t;

extern param_t param;
extern flight_mode_t flight_mode;
extern vehicle_setpoint_t vehicle_setpoint; 
extern vehicle_state_t vehicle_state;
extern uint8_t locked_flag;
extern uint8_t auto_landing_request;
extern uint8_t auto_landing_active;
extern volatile uint8_t preflight_error_flags;
extern volatile uint8_t flight_sensor_failsafe_flags;
extern uint8_t mission_height_recovery_active;
extern uint8_t mission_task_requested;
extern uint8_t mission_cruise_ready;
extern manual_input_t manual_input;

extern uint8_t base_image[MT9V03X_H][MT9V03X_W];
extern uint8_t binary_image[MT9V03X_H][MT9V03X_W];
extern uint8_t undistorted_image[MT9V03X_H][MT9V03X_W]; // 声明去畸变后的图像缓冲区
extern uint8_t decoupled_image[MT9V03X_H][MT9V03X_W];   // 声明姿态解耦后的图像缓冲区

extern image_t base_img;
extern image_t binary_img;
extern image_t undistorted_img; 
extern image_t decoupled_img;
extern image_t mapx_img;
extern image_t mapy_img; 

void fisheye_lut_apply(fisheye_lut_id_t id);

uint8_t preflight_check(void);

void param_init(void);
void param_update(float dT_s);  
void debug_capture_states_20ms(void);
void debug_print_states(void);


#endif
