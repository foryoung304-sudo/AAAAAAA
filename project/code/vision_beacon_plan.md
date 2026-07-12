# 视觉信标轻量方案

## 目标

摄像头先只作为低频信标辅助，不作为飞控核心传感器。

当前主线仍然是：

- TOF / height-lite 负责高度悬停。
- 光流 / flow-lite 负责水平速度和短时间位置保持。
- 摄像头只负责识别地面信标灯，并给出相对偏差。
- 状态机负责判断当前应该追踪第几个信标。

第一版不要让摄像头直接控制电机，也不要让它替代光流。

## 信标 ID 思路

图像里暂时不识别信标 ID。

信标 ID 由任务顺序决定：

```text
beacon[0] = 左下 / 第一个信标
beacon[1] = 下一个信标
beacon[2] = 下一个信标
...
```

摄像头看到一个信标时，默认它就是当前任务里的 `next_beacon_id`。

飞到信标上方后灯会灭，状态机认为当前信标完成，然后切到下一个信标。

## 摄像头输出

第一版摄像头只需要输出这些量：

```c
typedef struct
{
    uint8_t valid;
    float rel_x_cm;
    float rel_y_cm;
    float confidence;
} vision_beacon_t;
```

含义：

- `valid`：当前是否识别到信标。
- `rel_x_cm`：信标相对飞机/相机的地面 X 偏差。
- `rel_y_cm`：信标相对飞机/相机的地面 Y 偏差。
- `confidence`：识别置信度，第一版可选。

如果地面投影还没做好，可以先用图像归一化偏差：

```c
float offset_x;  // -1.0 到 1.0，图像左右偏差
float offset_y;  // -1.0 到 1.0，图像上下偏差
```

## 几何条件

目前已知条件：

- 飞行高度约 140cm 到 150cm。
- 镜头约 170 度。
- 相机已经标定并去畸变。
- 信标在地面平面上。

推荐转换链路：

```text
去畸变后的像素点
    -> 相机归一化射线
    -> 根据相机安装角和飞机姿态旋转
    -> 与地面平面求交
    -> 得到 rel_x_cm / rel_y_cm
```

注意姿态误差会被高度放大：

```text
145cm * tan(2deg) 约等于 5cm
```

所以 roll / pitch 有 1 到 2 度误差时，地面投影会有几厘米误差。

## 状态机

围绕信标追踪做一个小状态机：

```text
SEARCH:
    搜索当前 next_beacon_id 对应的信标。

TRACK:
    已经看到信标，用 rel_x / rel_y 或图像偏差进行对准。

LOST:
    刚才看到过信标，现在短暂丢失。

ARRIVED:
    认为已经到达信标上方。

NEXT:
    next_beacon_id++，切换到下一个信标，然后回到 SEARCH。
```

不要把每一次“看不到信标”都当成识别失败。因为飞到信标上方后，灯可能会灭。

## 到达判定

不要只靠灯灭判断到达，要用组合条件：

```text
最近一段时间看到过信标
最后一次 rel_x_cm / rel_y_cm 足够小
vision_valid 从 1 变成 0
丢失持续约 100ms 到 300ms
```

满足这些条件时，认为：

```text
当前信标已到达
next_beacon_id++
```

建议初始阈值：

```c
#define VISION_ARRIVE_X_CM        10.0f
#define VISION_ARRIVE_Y_CM        10.0f
#define VISION_RECENT_MS          300u
#define VISION_LIGHT_OFF_MS       150u
```

这些值后面根据日志调。

## 接入控制

第一版建议让视觉作为位置环辅助，不直接进 EKF-lite。

推荐链路：

```text
vision rel_x / rel_y
    -> 额外位置误差 或 目标位置微调
    -> 现有位置环
    -> flow-lite 速度反馈
```

方案 A：作为额外位置误差

```c
pos_error_x += vision_kx * vision_rel_x_cm;
pos_error_y += vision_ky * vision_rel_y_cm;
```

方案 B：缓慢移动目标位置

```c
target_pos_x += vision_kx * vision_rel_x_cm;
target_pos_y += vision_ky * vision_rel_y_cm;
```

短期推荐方案 A，因为接入快。必须加小增益和限幅。

建议限幅：

```c
#define VISION_POS_ERR_LIMIT_CM   20.0f
#define VISION_LOST_HOLD_MS       300u
```

## 后续再进 EKF-lite

如果已知每个信标的地图坐标：

```text
beacon_world_x/y = beacon_map[next_beacon_id]
vision_rel_x/y = 摄像头测到的信标相对位置
vision_drone_x/y = beacon_world_x/y - 转到世界系后的 vision_rel_x/y
```

此时可以构造 EKF-lite 的位置观测：

```text
innov_x = vision_drone_x - ekf_lite_state.x
innov_y = vision_drone_y - ekf_lite_state.y
```

但要等 `rel_x_cm / rel_y_cm` 足够稳定后再做。

第一阶段不要直接修改 `ekf_lite_state.x/y`。

## 明天实现顺序

1. 定义 `vision_beacon_t` 和视觉健康状态。
2. 写 `SEARCH / TRACK / LOST / ARRIVED / NEXT` 状态机。
3. 主循环打印这些量，不要在 ISR 里打印：

```text
vision_valid
state
next_beacon_id
rel_x_cm / rel_y_cm 或 offset_x / offset_y
seen_recently
arrived
```

4. 手持飞机或相机经过信标，先验证状态机。
5. 验证灯灭到达逻辑：

```text
可见 -> 已居中 -> 灯灭/丢失 100ms 到 300ms -> arrived
```

6. 日志稳定后，再用小增益接入位置环。

## 第一阶段成功标准

- `valid == 0` 时，视觉不影响飞控。
- 只有看到信标时才进入 `TRACK`。
- 只有“最近看到过 + 足够居中 + 随后灯灭/丢失”才进入 `ARRIVED`。
- `next_beacon_id` 只增加一次，不重复跳。
- 不需要 ISR 打印。

## 核心原则

视觉只是辅助，不是主稳定器。

主稳定器仍然是：

```text
TOF 高度 + flow-lite 水平 + 现有姿态环
```

视觉只回答两个问题：

```text
目标信标相对我在哪里？
我是否已经到达它上方？
```

