#include "zf_common_headfile.h"

// 信标信息结构
BeaconInfo beacon;

// 误差相关变量
float erro;              // 当前误差（信标中心与图像中心的X偏移）
float error_back;        // 上一次误差
float erro_yawan;        // 偏航误差
float erro_yawan_back;   // 上一次偏航误差
float last_erro = 94;   // 最后一次误差（初始值94）

// ###########################################################################
// 函数名称: detect_beacon
// 函数功能: 从图像中检测信标（红外光源）
// 参数说明: 
//           img  - 摄像头图像指针
//           info - 信标信息输出结构指针
// 返回说明: 无
// ###########################################################################
void detect_beacon(image_t *img, BeaconInfo *info)
{
    int16_t sumvalue = 0;   // 当前区域像素总和
    int16_t value = 0;      // 临时变量
    int temp = 0;          // 交换用临时变量

    // ======================= 初始化信标信息 =======================
    info->status = BEACON_NOT_FOUND;   // 默认未找到
    info->centerX = 0;                // 信标中心X坐标
    info->centerY = 0;                // 信标中心Y坐标
    info->length = 0;                 // 当前长度（距图像中心距离）
    info->length_last = 0;           // 上一次长度
    info->area = 0;                   // 面积
    info->last_centerX = 0;          // 上一次中心X
    info->last_centerY = 0;          // 上一次中心Y
    info->sum_last = 0;               // 上一次像素总和
    info->sum = 0;                   // 当前像素总和
    info->quanzhong = 0;              // 权重（用于筛选最佳候选点）
    info->last_quanzhong = 0;         // 上一次权重

    // ======================= 扫描图像寻找信标 =======================
    // 扫描范围：Y从39到图像高度-10，X从10到图像宽度-10
    // 跳过边缘区域，减少误检测
    for(int y = 39; y < MT9V03X_H - 10; y++)
    {
        for(int x = 10; x < MT9V03X_W - 10; x++)
        {
            // 判断当前像素是否超过红外阈值（可能是信标）
            if(AT_IMAGE(img, x, y) > IR_THRESHOLD)
            {
                // 在10x10邻域内求和（检测亮点区域）
                for(int i = 0; i < 10; i++)
                    for(int j = 0; j < 10; j++)
                    {
                        value += AT_IMAGE(img, x - j, y - j);
                    }

                // ======================= 寻找最亮区域 =======================
                if(value > sumvalue)
                {
                    // 如果新点比当前最佳点更亮，且与上次检测点有足够距离
                    if(func_abs(y - info->centerY) > 20)
                    {
                        // 保存当前最佳点作为"上一次最佳点"
                        info->sum_last = sumvalue;
                        info->last_centerX = info->centerX;
                        info->last_centerY = info->centerY;
                    }

                    // 更新最佳点
                    sumvalue = value;
                    info->sum = sumvalue;
                    info->centerX = x;
                    info->centerY = y;
                    value = 0;
                }
                // 备选点：较亮但不是最亮的点
                else if(value > info->sum_last && ABS(y - info->last_centerY) > 20)
                {
                    info->sum_last = value;
                    info->last_centerX = x;
                    info->last_centerY = y;
                }
                else 
                {
                    value = 0;  // 重置
                }
            }
        }
    } // <-- 必须在这里先结束 Y 轴的循环！等全图扫描完了再进行一次性的权重比较

    // ======================= 扫描结束后，计算权重并挑选最优解 =======================
    if(info->centerX != 0)
    {
        info->status = BEACON_FOUND;
        
        // 如果找到了备用候选点，比较主备两点的实际综合权重
        if(info->last_centerX != 0)
        {
            // 权重计算：优先选择下方(离飞机近)且靠近画面中心，同时参考总亮度
            float weight1 = 2.0f * info->sum      + 6.5f * func_abs(MT9V03X_H - info->centerY)      - 1.5f * func_abs(MT9V03X_W / 2 - info->centerX);
            float weight2 = 2.0f * info->sum_last + 6.5f * func_abs(MT9V03X_H - info->last_centerY) - 1.5f * func_abs(MT9V03X_W / 2 - info->last_centerX);

            // 如果备用点的权重更高，执行"篡位"交换，让备用点成为最终输出
            if(weight2 > weight1)
            {
                temp = info->sum;      info->sum = info->sum_last;           info->sum_last = temp;
                temp = info->centerX;  info->centerX = info->last_centerX;   info->last_centerX = temp;
                temp = info->centerY;  info->centerY = info->last_centerY;   info->last_centerY = temp;
            }
        }
    }

    // ======================= 计算最终误差 =======================
    last_erro = erro;
    erro = 1.0f * (MT9V03X_W / 2 - info->centerX);  // X方向误差

    if(info->status == BEACON_NOT_FOUND)
    {
        if(last_erro < 30)
        {
            // 如果刚跟丢，保持转动趋势(防丢失盲转打角)
            erro = (last_erro >= 0 ? 1 : -1) * 65.0f;  
        }
        else
        {
            erro = last_erro;  // 保持上次误差
        }
    }

    erro_yawan = erro;  // 同步偏航误差
}

bool visited[MT9V03X_H][MT9V03X_W] = {false};

// 从二值化图像中寻找近似圆形的中心点
// 原理：找到上下左右距离近似相等的点（近似圆形）
uint16_t find_centers(const image_t *img, CenterPoint centers[MAX_CENTERS]) 
{
    // 初始化
    memset(visited, 0, sizeof(visited));               // 访问标记
    memset(centers, 0, sizeof(centers));             // 中心点数组
    uint16_t center_count = 0;                       // 找到的数量
    const uint8 max_error = 10;                        // 容许误差

    // 遍历图像
    for(uint16_t y = 0; y < MT9V03X_H; y++) {
        for(uint16_t x = 0; x < MT9V03X_W; x++) {
            // 找到白色像素且未访问
            if(AT_IMAGE(img, x, y) == 255 && !visited[y][x]) 
            {

                // 向下找最底端
                int bottom_y = y;
                while(bottom_y + 1 < MT9V03X_H && 
                      AT_IMAGE(img, x, bottom_y + 1) == 255 && 
                      !visited[bottom_y + 1][x])
                {
                    bottom_y++;
                }

                // 中点
                int middle_y = (y + bottom_y) / 2;
                int middle_x = x;

                // 向左找最左端
                int left_x = middle_x;
                while(left_x > 0 && 
                      AT_IMAGE(img, left_x - 1, middle_y) == 255 && 
                      !visited[middle_y][left_x - 1]) 
                {
                    left_x--;
                }

                // 向右找最右端
                int right_x = middle_x;
                while(right_x + 1 < MT9V03X_W && 
                      AT_IMAGE(img, right_x + 1, middle_y) == 255 && 
                      !visited[middle_y][right_x + 1]) 
                {
                    right_x++;
                }

                // 计算四个方向距离

                int up_distance = middle_y - y;
                int down_distance = bottom_y - middle_y;
                int left_distance = middle_x - left_x;
                int right_distance = right_x - middle_x;

                // 取最大最小值
                int max_dist = up_distance;
                int min_dist = up_distance;

                if(down_distance > max_dist) max_dist = down_distance;
                if(left_distance > max_dist) max_dist = left_distance;
                if(right_distance > max_dist) max_dist = right_distance;

                if(down_distance < min_dist) min_dist = down_distance;
                if(left_distance < min_dist) min_dist = left_distance;
                if(right_distance < min_dist) min_dist = right_distance;

                // 判断是否近似圆形（上下左右距离差值在容许范围内）

                if(max_dist - min_dist <= max_error) 
                {
                    // 记录中心点
                    if(center_count < MAX_CENTERS) 
                    {
                        centers[center_count].x = middle_x;
                        centers[center_count].y = middle_y;
                        centers[center_count].distance = max_dist;
                        // 权重：优先下方、靠近图像中心、半径大的点
                        centers[center_count].quanzhong = 
                            (1 - (float)middle_x / 94) * 0.4f +    // X居中
                            (float)middle_y / 120 * 0.3f +        // 下方优先
                            (float)max_dist * 0.3f;               // 半径大优先
                        center_count++;
                    }

                    // 标记该区域为已访问（避免重复检测）
 
                    int half_size = max_dist + 10;
                    int y_min = (middle_y > half_size) ? (middle_y - half_size) : 0;
                    int y_max = (middle_y + half_size < MT9V03X_H) ? 
                                (middle_y + half_size) : (MT9V03X_H - 1);
                    int x_min = (middle_x > half_size) ? (middle_x - half_size) : 0;
                    int x_max = (middle_x + half_size < MT9V03X_W) ? 
                                (middle_x + half_size) : (MT9V03X_W - 1);

                    for(int r = y_min; r <= y_max; r++) 
                    {
                        for(int c = x_min; c <= x_max; c++) 
                        {
                            if(AT_IMAGE(img, c, r) == 255) {
                                visited[r][c] = true;
                            }
                        }
                    }
                }
            }
        }
    }

    return center_count;
}

// --- 核心姿态解耦函数 (Attitude Compensation) ---
// 将倾斜画面中的像素，映射回一个"完美的虚拟水平相机"画面上。
// 输入：
//   - u_raw, v_raw: 图像处理算法识别到的、歪斜画面里的原始信标中心像素坐标。
//   - drone_roll: IMU / AHRS 算出的飞机当前横滚角 (单位：弧度！绝对不准用角度！)。
//   - drone_pitch: IMU / AHRS 算出的飞机当前俯仰角 (单位：弧度！绝对不准用角度！)。
// 输出：
//   - u_corr, v_corr: 指针，返回矫正后的、无姿态耦合的"上帝视角"坐标。
void vision_attitude_compensation(float u_raw, float v_raw, float *u_corr, float *v_corr)
{
    // ==========================================
    // 步骤一：预计算 sin 和 cos 
    // ==========================================
    // 将角度转换为弧度的方法：(角度 * 3.14159f / 180.0f)
    // 确保 Roll > 0 代表右翼下倾；Pitch > 0 代表机头抬起。
    float cr = vehicle_state.roll_cos;
    float sr = vehicle_state.roll_sin;
    float cp = vehicle_state.pitch_cos;
    float sp = vehicle_state.pitch_sin;

    // ==========================================
    // 步骤二：图像坐标系 -> 相机3D射线向量 (Pc)
    // ==========================================
    // 这是一个以相机焦点为原点的三维坐标系：X_c右，Y_c下，Z_c正前(焦距)
    float Xc = u_raw - IMAGE_CENTER_X;
    float Yc = v_raw - IMAGE_CENTER_Y;
    float Zc = CAMERA_FOCAL_LENGTH_PIXEL;

    // ==========================================
    // 步骤三：3D旋转矩阵乘法（掰直 Pc 到 Pg）
    // ==========================================
    // Pg = R * Pc
    // 这里采用最通用的旋转矩阵（ Yaw-Pitch-Roll 顺序的逆变换），消除倾斜。    
    //  Xg: 转换后射线在水平面上的左右偏差
    float Xg = Zc * sp - Yc * cp * sr + Xc * cp * cr;

    //  Yg: 转换后射线在水平面上的前后偏差
    float Yg = Yc * cr + Xc * sr;

    // Zg: 转换后的视线深度 
    float Zg = Zc * cp + Yc * sp * sr - Xc * sp * cr;

    // ==========================================
    // 步骤四：重新投影回"虚拟水平图像平面" (深度除法)
    // ==========================================
    // 利用相似三角形原理，把 3D 的 P_g 投影回 2D 的 (u_corr, v_corr) 平面上。

    // 保护：如果飞机倾斜太夸张，导致 Zg 小于等于 0，说明目标在飞机后方或水平线上，不可见。
    if (Zg < 0.001f) {
        *u_corr = IMAGE_CENTER_X; // 丢弃该点，回到中心
        *v_corr = IMAGE_CENTER_Y;
        return;
    }
    // 最终解耦后的坐标输出
    *u_corr = IMAGE_CENTER_X + CAMERA_FOCAL_LENGTH_PIXEL * (Xg / Zg);
    *v_corr = IMAGE_CENTER_Y + CAMERA_FOCAL_LENGTH_PIXEL * (Yg / Zg);
}

// --- 整图姿态解耦函数 (全图逆映射) ---
// 将整个带倾斜的图像，重新投影为"虚拟水平相机"所看到的视角
// 仅供前期测试使用，因为全像素的浮点运算非常耗时！
void image_attitude_compensation(const image_t *img_src, image_t *img_dst)
{
    float cr = vehicle_state.roll_cos;
    float sr = vehicle_state.roll_sin;
    float cp = vehicle_state.pitch_cos;
    float sp = vehicle_state.pitch_sin;

    // 1. 清空目标图像，防止边缘出现上一次的残影
    memset(img_dst->data, 0, img_dst->height * img_dst->step);

    // 2. 逆映射遍历：遍历目标图像（虚拟水平相机）的每一个像素
    for (int y = 0; y < img_dst->height; y++)
    {
        for (int x = 0; x < img_dst->width; x++)
        {
            // 将目标图像像素坐标转化为 3D 射线 (Pg)
            float Xg = (float)x - IMAGE_CENTER_X;
            float Yg = (float)y - IMAGE_CENTER_Y;
            float Zg = CAMERA_FOCAL_LENGTH_PIXEL;

            // 乘以逆旋转矩阵 R^T，反推原始倾斜相机下的 3D 射线 (Pc)
            float Xc =  Xg * cp * cr + Yg * sr - Zg * sp * cr;
            float Yc = -Xg * cp * sr + Yg * cr + Zg * sp * sr;
            float Zc =  Xg * sp      + 0.0f    + Zg * cp;

            if (Zc < 0.001f) {
                continue; // 射线在相机后方，不可见
            }

            // 投影回原始倾斜相机的 2D 像素平面
            int u_src = (int)(IMAGE_CENTER_X + CAMERA_FOCAL_LENGTH_PIXEL * (Xc / Zc));
            int v_src = (int)(IMAGE_CENTER_Y + CAMERA_FOCAL_LENGTH_PIXEL * (Yc / Zc));

            // 如果反推出来的像素在原图范围内，则进行色彩赋值
            if (u_src >= 0 && u_src < img_src->width && v_src >= 0 && v_src < img_src->height)
            {
                AT_IMAGE(img_dst, x, y) = AT_IMAGE(img_src, u_src, v_src);
            }
        }
    }
}

uint8_t image_process()
{
    memcpy(base_image, mt9v03x_image, MT9V03X_IMAGE_SIZE); // 将采集到的图像数据复制到 base_image 中
    uint8_t threshold = fast_threshold(base_image[0], MT9V03X_W, MT9V03X_H); // 计算 OTSU 阈值
    threshold_fixed(&base_img, &binary_img, threshold, 0, 255); // 二值化处理
    image_erode3(&binary_img, &decoupled_img); // 腐蚀去噪
    image_dilate3(&decoupled_img, &binary_img); // 膨胀恢复目标大小
    image_remap8(&base_img, &undistorted_img, &mapx_img, &mapy_img); // 去畸变重映射
    detect_beacon(&undistorted_img, &beacon); // 寻找信标

        // 3. 如果找到了信标点，进行解耦和标记
        if (beacon.status == BEACON_FOUND)
        {
            int u_raw = beacon.centerX;
            int v_raw = beacon.centerY;
            float u_corrected, v_corrected;
            
            // 核心：调用姿态解耦函数，传入当前飞机的Roll/Pitch，得到纠正后的虚拟坐标
            vision_attitude_compensation((float)u_raw, (float)v_raw, &u_corrected, &v_corrected);

            // 周期性通过串口打印，避免刷屏太快看不清
            static uint32_t last_vision_print_time = 0;
            if(system_time_us() - last_vision_print_time > 100000) {
                last_vision_print_time = system_time_us();
                printf("Raw: (%3d, %3d) | Att(R,P): (%6.1f, %6.1f) | Corr: (%6.1f, %6.1f)\n",
                       u_raw, v_raw, vehicle_state.current_roll, vehicle_state.current_pitch, u_corrected, v_corrected);
            }
            // 在屏幕画面上画上标记，方便肉眼确认抓没抓对点
            draw_x(&undistorted_img, u_raw, v_raw, 5, 255);                       // 原始坐标画个白色的 'X'
            draw_o(&undistorted_img, (int)u_corrected, (int)v_corrected, 6, 128); // 解耦坐标画个灰色的 'O'
        }

    return 1;
}
