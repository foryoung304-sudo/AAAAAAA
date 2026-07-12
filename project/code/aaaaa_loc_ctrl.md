# alt_ctrl profile 封装 → 通用 1D 轨迹模块

## 为什么要封装

alt_ctrl 的 profile 逻辑散落在 lt_2level_ctrl() 约 200 行里，
和高度环的 PID、phase、takeoff boost 等紧耦合。
loc 要用同样的轨迹生成器，不应该复制粘贴。

核心思路：抽出一个 **纯 1D 轨迹 profile 模块**，
只关心「给定目标和当前位置，生成一条平滑的位置/速度/加速度轨迹」。

alt_ctrl 完全不动，新写的 profile.c 在底层复用相同的工具函数
(ctrl_brake_speed, ctrl_slew_limit, ctrl_smoothstep01)。

---

## 三个新文件

| 文件 | 职责 |
|---|---|
| profile.h | 公开接口：配置结构体、状态结构体、API 声明 |
| profile.c | 实现：profile init / reset / update / get_desired_vel |
| （alt_ctrl.c 不变） | 将来可改为调用 profile.h，但不急 |

---

## profile.h
```c
