# 无人机项目背景

MCU:
- CYT4BB7

IMU:
- Mahony
- 500Hz (2ms)
- pitch:上+下-   roll：左-右+   yaw：顺时针+逆时针-


机体坐标
- X前正
- Y右正
- Z上正

光流:
- LC302

- 光流方向:
    - 前移 X+
    - 后移 X-
    - 左移 Y-
    - 右移 Y+
    - 调试/标定时优先看 `body_state:true_pos_x / true_pos_y`
    - `earth_state:pos_x / pos_y` 已经经过 yaw 旋转，不能直接拿来标定光流比例

高度:
- TOF

- 图像坐标系:
    - X右+
    - Y下+
    



定时器：
- imu 2ms 
- flow姿态补偿角度获取 2ms
- 更新tof 更新flow 20ms



当前参数:

- LC302光流:
  - `LC302_SWAP_XY = 1`
  - `LC302_FLOW_SCALE_X = 1.90f`
  - `LC302_FLOW_SCALE_Y = 1.60f`
  - `FLOW_OFFSET_X_CM = 9.3f`
  - `FLOW_OFFSET_Y_CM = 2.3f`
  - `FLOW_YAW_OFFSET_ENABLE = 1`
  - `FLOW_YAW_VISION_ENABLE = 1`
  - `FLOW_YAW_VISION_GAIN_X = -0.26f`
  - `FLOW_YAW_VISION_GAIN_Y = 0.45f`
  - `FLOW_GYRO_COMP_GAIN_X = 1.85f`
  - `FLOW_GYRO_COMP_GAIN_Y = 1.50f`
  - `FLOW_RAW_DEADZONE_CM = 0.12f`
  - `FLOW_GYRO_DEADZONE_CM = 0.03f`


光流阶段结论:

- PMW3901在随机纹理/桌边等高纹理表面可用
- 贴随机噪声后，矩形闭合测试约:
  - body闭合误差约 5.2cm
  - earth闭合误差约 6.8cm
- 屋子中间瓷砖/低纹理地面识别差，主要是地面纹理/光照问题，不优先怀疑比例参数
- PMW3901不再继续深挖极限，后续作为备份方案保留


- LC302:
  - 虽然还是对于瓷砖地面不耐受，但精度比pmw3901好很多
  - 测试已完成：姿态误差<5cm,闭合测试由于地面纹理以及手推姿态误差较大
  - 不同地面纹理，scale不同，先暂时不动，等到下赛道测试
  - 当前LC302使用外部TOF高度 `vehicle_state.current_height` 做换算，不使用LC302包内 `height=999`
  - 当前方向映射:
    - 机体前推: LC302 raw_y为负
    - 机体左移: LC302 raw_x为正
    - 经过swap和符号处理后，body坐标保持 X前正、Y左正
  - 当前优先看 `body_state:true_pos_x / true_pos_y` 判断光流本体；`earth_state:pos_x / pos_y` 会受到yaw角和坐标旋转累计影响

LC302最近测试记录:

- 原地yaw测试:
  - 单纯yaw offset只能补探头偏心绕重心转动，不能完全消除LC302视场旋转误判
  - 已加入经验项 `yaw_vision_disp_x/y = gain * height * dtheta_z`
  - `dtheta_z` 改用姿态yaw差分并做正负180度跳变处理，比只用gyro积分更接近实际转角
  - 当前yaw补偿后，正反方向原地yaw残差已降到几厘米级，暂时不继续深挖

- Roll/Pitch姿态补偿测试:
  - `flow_gyro_int_x/y/z` 单位为rad，gyro积分本身单位正确
  - roll/pitch补偿改用姿态角差分作为 `d_theta_x/y`
  - 当前最优增益约:
    - roll/Y轴: 1.48，当前使用 `FLOW_GYRO_COMP_GAIN_Y = 1.50f`
    - pitch/X轴: 1.84，当前使用 `FLOW_GYRO_COMP_GAIN_X = 1.85f`
  - 大角度测试中，raw_abs约380~430cm，补偿后true_abs约165~176cm，说明补偿有效
  - 当roll/pitch接近20~30度时，残差明显增大，后续应通过tilt gate降低光流权重，不建议继续强行调gain

- 原路返回闭环测试:
  - 测试路线: 先前推，再右移，然后原路返回
  - 该测试比普通矩形更能减少不同地面纹理带来的scale变化
  - 最近一次结果:
    - `body_state:true_pos_x = -1.43cm`
    - `body_state:true_pos_y = -3.14cm`
    - `earth_state:pos_x = 1.51cm`
    - `earth_state:pos_y = -9.42cm`
  - 结论:
    - body坐标闭环已经比较好，LC302本体、方向和scale暂时可用
    - earth坐标误差主要来自yaw角参与坐标旋转后的累计，不优先怀疑LC302原始位移
    - 后续标定scale时继续优先用body坐标，不要直接用earth坐标调比例
  
高度测试：
- dl1b：姿态补偿有效，误差3cm内
- vl53l8：误差同上


姿态环测试：
角度环：
Roll   2.5 / 0 / 0
Pitch  2.5 / 0 / 0
Yaw    0.85 / 0 / 0

角速度环：
Roll   0.15 / 0.027  / 0
Pitch  0.15 / 0.04  / 0
Yaw    0.35 / 0.025 / 0

高度环：
- tof_ekf_lite:
    - ACC_GAIN=0.20
    - ACC_LPF=0.18
    - Q_Z=25
    - Q_VZ=400
    - R_Z=20
    - TOF_VEL_ALPHA=0.55
    - R_VZ=200  








待解决:
- LC302 scale确定

- 坐标系最终确认:
  - body 坐标用于光流标定
  - earth 坐标用于导航/控制
  - 注意不要把两者混着判断比例

当前不要优先怀疑:

- 各个模块坐标系不同，先不要统一，等全部测试完

- 位置环流程！！！
对。位置环也应走同样流程：

```text
采集真实飞行数据
→ 离线生成参考轨迹
→ 回放位置/速度估计器
→ 参数扫描
→ 独立日志验证
→ 再调控制器
```

记录：

```text
timestamp
光流原始 dx/dy
投影/旋转补偿
IMU姿态与角速度
估计 x/y、vx/vy
目标位置/速度
roll/pitch指令
电机输出
高度
```

顺序必须是：

```text
先保证 vx/vy 快、准、不漂
→ 再调速度环
→ 再调位置环
```

但光流有额外难点：

- 比例随高度变化
- 横滚/俯仰旋转产生假位移
- ToF高度误差直接污染比例
- 地面纹理/亮度改变质量
- 缺少绝对位置观测，长期位置必漂

所以位置环不能只靠“优化 EKF 参数”根治长期漂移。比赛场景最终需要信标、视觉地标或其他绝对位置修正。