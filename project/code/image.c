/*
 * image.c
 *
 *  Created on: 2025年9月27日
 *      Author: cow
 */
/*
//------------------------------------------------------------------------------------------------//
         mt9v03x_finish_flag;                   // 一场图像采集完成标志位
         mt9v03x_image[MT9V03X_H][MT9V03X_W];   //摄像头采集原始图像
         img0                                   //原始图像
         img1                                   //处理后图像
//------------------------------------------------------------------------------------------------//
*/
#include "zf_common_headfile.h"
//------------------------------------------------------------------------------------------------//


// Define clip function to limit values within a range
static inline int clip(int value, int min_val, int max_val) {
    if (value < min_val) return min_val;
    if (value > max_val) return max_val;
    return value;
}



void image_clone(image_t *img0,image_t *img1)
{
    if(mt9v03x_finish_flag)
    {
        if(img0->width==img0->step && img1->width==img1->step)
         {
            memcpy(img0->data,img1->data,MT9V03X_IMAGE_SIZE);
         }
        else
        {
            for(int y=0;y<img0->height;y++)
            {
                memcpy(&AT_IMAGE(img1,0,y),&AT_IMAGE(img0,0,y),img0->width);
            }
        }
         mt9v03x_finish_flag=0;

    }

}

void image_clear(image_t *img)
{
    if(img->width==img->step)
    {
        memset(img->data,0,MT9V03X_IMAGE_SIZE);
    }
    else
    {
        for(int y=0;y<img->height;y++)
        {
            memset(&AT_IMAGE(img,0,y),0,img->width);
        }
    }
}

// ------------------------------二值化----------------------------------------------------------------//

//固定阈值二值化
void threshold_fixed(image_t *img0,image_t *img1,uint8_t thres,uint8_t low,uint8_t high)
{
    if(img0->width==img1->width && img0->height==img1->height)
    {
        for(int y=0;y<img0->height;y++)
        {
            for(int x=0;x<img0->width;x++)
            {
                AT_IMAGE(img1,x,y)=AT_IMAGE(img0,x,y)<thres?low:high;
            }
        }
    }

}

//自适应阈值二值化
void threshole_adaptive(image_t *img0,image_t *img1,int block_size,int down,int low,int high)
{
    if(block_size>1 && block_size%2==1)
    {
        int half=block_size/2;
        for(int y=0;y<img0->height;y++)
        {
            for(int x=0;x<img0->width;x++)
            {
                int thres=0;
                for(int dy=-half;dy<=half;dy++)
                {
                    for(int dx=-half;dx<=half;dx++)
                    {
                        thres+=AT_IMAGE_CLIP(img0,x+dx,y+dy);
                    }
                }
                thres/=block_size*block_size;
                thres-=down;
                AT_IMAGE(img1,x,y)=AT_IMAGE(img0,x,y)<thres?low:high;
            }
        }
    }
}


//得到大津阈值
uint16_t Getthreshold_OSTU(image_t *img,uint8_t x0,uint8_t y0,uint8_t x1,uint8_t y1)
{
    uint16_t histogram[256]={0};
    uint32_t min_value,max_value;
    uint32_t pix_amount=0;
    uint32_t pix_integral=0;
    uint32_t pix_back_amount=0;
    uint32_t pix_back_integral=0;
    int32_t pix_fore_amount=0;
    int32_t pix_fore_integral=0;

    float omega_back,omega_fore,micro_back,micro_fore,sigma_beta,sigma;

    uint16_t thres=0;
    for(uint8_t y=y0;y<y1;y+=2)
    {
        for(uint8_t x=x0;x<x1;x+=2)
        {
            ++histogram[AT_IMAGE(img,x,y)];
        }
    }
    for (min_value = 0; min_value < 256 && histogram[min_value] == 0; min_value++)
    {
            ; // 获取最小灰度的值
    }
    for (max_value = 255; max_value > min_value && histogram[max_value] == 0; max_value--)
    {
            ; // 获取最大灰度的值
    }
    if (max_value == min_value)
     {
        thres=(uint8_t)(max_value); // 图像中只有一个颜色
     }
     if (min_value + 1 == max_value)
     {
         thres=(uint8_t)(min_value); // 图像中只有二个颜色
     }
     for(uint16_t j=(uint16_t)min_value;j<=max_value;j++)
     {
         pix_amount+=histogram[j];

     }
     for(uint16_t j=(uint16_t)min_value;j<=max_value;j++)
     {
         pix_integral+=histogram[j]*j;

     }
     sigma_beta=-1;
     for (uint16_t j = (uint16)min_value; j < max_value; j++)
     {
         pix_back_amount=pix_back_amount+histogram[j];          //灰度值<=j的像素总数
         pix_fore_amount=pix_amount-pix_back_amount;            //大于j的总数
         omega_back=(float)pix_back_amount/pix_amount;          //平均值
         omega_fore=(float)pix_fore_amount/pix_amount;
         pix_back_integral+=histogram[j]*j;                     //背景灰度值
         pix_fore_integral=pix_integral-pix_back_integral;      //前景灰度值
         micro_back=(float)pix_back_integral/pix_back_amount;   //背景灰度百分比
         micro_fore=(float)pix_fore_integral/pix_fore_amount;   //前景灰度百分比
         sigma=omega_back*omega_fore*(micro_back-micro_fore)*(micro_back-micro_fore);//类间方差
         if(sigma>sigma_beta)                                   //寻找最大的类间方差，类间方差越大，二值化效果越好
         {
             sigma_beta=sigma;
             thres=(uint8_t)j;                                  //返回类间方差最大时的最佳阈值
         }

     }
     return thres;
}
/*-------------------------------------------------------------------------------------------------------------------
  @brief     快速大津求阈值，来自山威
  @param     image       图像数组
             col         列 ，宽度
             row         行，长度
  @return    null
  Sample     threshold=fast_threshold(mt9v03x_image[0],MT9V03X_W, MT9V03X_H);//山威快速大津
  @note      据说比传统大津法快一点，使用效果差不多
-------------------------------------------------------------------------------------------------------------------*/
int fast_threshold(uint8 *image, uint16 col, uint16 row)   //注意计算阈值的一定要是原图像
{
    #define GrayScale 256
    uint16 width = col;
    uint16 height = row;
    int pixelCount[GrayScale];
    float pixelPro[GrayScale];
    int i, j;
    int pixelSum = width * height/4;
    int threshold = 0;
    uint8* data = image;  //指向像素数据的指针
    for (i = 0; i < GrayScale; i++)
    {
        pixelCount[i] = 0;
        pixelPro[i] = 0;
    }
    uint32 gray_sum=0;
    //统计灰度级中每个像素在整幅图像中的个数
    for (i = 0; i < height; i+=2)
    {
        for (j = 0; j < width; j+=2)
        {
            pixelCount[(int)data[i * width + j]]++;  //将当前的点的像素值作为计数数组的下标
            gray_sum+=(int)data[i * width + j];       //灰度值总和
        }
    }
    //计算每个像素值的点在整幅图像中的比例
    for (i = 0; i < GrayScale; i++)
    {
        pixelPro[i] = (float)pixelCount[i] / pixelSum;
    }
    //遍历灰度级[0,255]
    float w0, w1, u0tmp, u1tmp, u0, u1, u, deltaTmp, deltaMax = 0;
    w0 = w1 = u0tmp = u1tmp = u0 = u1 = u = deltaTmp = 0;
    for (j = 0; j < GrayScale; j++)
    {
        w0 += pixelPro[j];  //背景部分每个灰度值的像素点所占比例之和   即背景部分的比例
        u0tmp += j * pixelPro[j];  //背景部分 每个灰度值的点的比例 *灰度值
        w1=1-w0;
        u1tmp=gray_sum/pixelSum-u0tmp;
        u0 = u0tmp / w0;              //背景平均灰度
        u1 = u1tmp / w1;              //前景平均灰度
        u = u0tmp + u1tmp;            //全局平均灰度
        deltaTmp = w0 * pow((u0 - u), 2) + w1 * pow((u1 - u), 2);
        if (deltaTmp > deltaMax)
        {
            deltaMax = deltaTmp;
            threshold = j;
        }
        if (deltaTmp < deltaMax)
        {
            break;
        }
    }
    return threshold;
}
//大津阈值二值化
void threshold_OSTU(image_t *img0,image_t *img1,uint8_t low,uint8_t high)
{
    uint8_t thres=(uint8_t)Getthreshold_OSTU(img0, 0, 0, img1->width, img1->height);
    threshold_fixed(img0,img1,thres,low,high);
}

// ------------------------------二值化-----------------------------------------------------------------------------------------------//


//-------------------------------图像逻辑----------------------------------------------------------------------------------------------//

//图像逻辑与
void image_and(image_t *img0,image_t *img1,image_t *img2)
{
if(img0->width == img1->width && img0->height == img1->height && img0->width == img2->width && img0->height == img2->height)
    {
        for(int y=0;y<img0->height;y++)
        {
            for(int x=0;x<img0->width;x++)
            {
                AT_IMAGE(img2,x,y)=(AT_IMAGE(img0,x,y)==0 && AT_IMAGE(img1,x,y)==0)?0:255;
            }
        }
    }
}

//图像逻辑或
void image_or(image_t *img0,image_t *img1,image_t *img2)
{
    if(img0->width == img1->width && img0->height == img1->height && img0->width == img2->width && img0->height == img2->height)
    {
        for(int y=0;y<img0->height;y++)
        {
            for(int x=0;x<img0->width;x++)
            {
                AT_IMAGE(img2,x,y)=(AT_IMAGE(img0,x,y)==0 || AT_IMAGE(img1,x,y)==0)?0:255;
            }
        }
    }
}


//-------------------------------图像逻辑----------------------------------------------------------------------------------------------//



//-------------------------------图像处理-----------------------------------------------------------------//


//2x2最小池化降采样
void image_minpool2(image_t *img0,image_t *img1)
{
    if(img0->width / 2 == img1->width && img0->height / 2 == img1->height)
    {
        uint8_t min_value;
        for(int y=1;y<img0->height;y+=2)
        {
            for(int x=1;x<img0->width;x+=2)
            {
                min_value=255;
                if(AT_IMAGE(img0,x,y)<min_value)min_value=AT_IMAGE(img0,x,y);
                if(AT_IMAGE(img0,x-1,y)<min_value)min_value=AT_IMAGE(img0,x-1,y);
                if(AT_IMAGE(img0,x,y-1)<min_value)min_value=AT_IMAGE(img0,x,y-1);
                if(AT_IMAGE(img0,x-1,y-1)<min_value)min_value=AT_IMAGE(img0,x-1,y-1);
                AT_IMAGE(img1,x/2,y/2)=min_value;   //用最暗点替代2x2区域，并缩小图像至1/2
            }
        }
    }
}


/*图像滤波降噪
*             [1 2 1]
              [2 4 2] / 16  特定卷积核进行图像平滑处理
              [1 2 1]
*/
void image_blur(image_t *img0,image_t *img1,uint32_t kernel)
{
    if(img0->width==img1->width && img0->height==img1->height)
    {
        for(int y=1;y<img0->height-1;y++)
        {
            for(int x=1;x<img0->width-1;x++)
            {
                AT_IMAGE(img1,x,y)=(1 * AT_IMAGE(img0, x - 1, y - 1) + 2 * AT_IMAGE(img0, x, y - 1) + 1 * AT_IMAGE(img0, x + 1, y - 1) +
                        2 * AT_IMAGE(img0, x - 1, y) + 4 * AT_IMAGE(img0, x, y) + 2 * AT_IMAGE(img0, x + 1, y) +
                        1 * AT_IMAGE(img0, x - 1, y + 1) + 2 * AT_IMAGE(img0, x, y + 1) + 1 * AT_IMAGE(img0, x + 1, y + 1)) / 16;


            }
        }
    }
}


//3x3sobel边缘提取
/*卷积核进行偏导数计算，计算梯度检测剧烈变化
                     *  --------------------------
                     *  [-1  0  1]
                        [-2  0  2]  →  检测垂直边缘
                        [-1  0  1]
                     * ---------------------------
                     * [ 1  2  1]
                       [ 0  0  0]  →  检测水平边缘
                       [-1 -2 -1]
                     *----------------------------
 */
void image_sobel3(image_t *img0,image_t *img1)
{
    if(img0->width==img1->width && img0->height==img1->height)
    {
        int gx,gy;
        for (int y = 1; y < img0->height - 1; y++) {
                for (int x = 1; x < img0->width - 1; x++)
                {
                    gx = (-1 * AT_IMAGE(img0, x - 1, y - 1) + 1 * AT_IMAGE(img0, x + 1, y - 1) +
                          -2 * AT_IMAGE(img0, x - 1, y) + 2 * AT_IMAGE(img0, x + 1, y) +
                          -1 * AT_IMAGE(img0, x - 1, y + 1) + 1 * AT_IMAGE(img0, x + 1, y + 1)) / 4;
                    gy = (1 * AT_IMAGE(img0, x - 1, y - 1) + 2 * AT_IMAGE(img0, x, y - 1) + 1 * AT_IMAGE(img0, x + 1, y - 1) +
                          -1 * AT_IMAGE(img0, x - 1, y + 1) - 2 * AT_IMAGE(img0, x, y + 1) - 1 * AT_IMAGE(img0, x + 1, y + 1)) / 4;
                    AT_IMAGE(img1, x, y) = (abs(gx) + abs(gy)) / 2;

                }
            }
    }

}


//3x3腐蚀
/*3x3最小池化
 * 对于输入图像中的每个像素(x,y):
        [ P(x-1,y-1)  P(x,y-1)  P(x+1,y-1) ]
        [ P(x-1,y)    P(x,y)    P(x+1,y)   ]  →  min(P[3x3]) → 输出到img1(x,y)
        [ P(x-1,y+1)  P(x,y+1)  P(x+1,y+1) ]
 */
void image_erode3(image_t *img0,image_t *img1)
{
    if(img0->width==img1->width && img0->height==img1->height)
    {
        uint8_t min_value;
        for (int y = 1; y < img0->height - 1; y++)
        {
                for (int x = 1; x < img0->width - 1; x++)
                {
                    min_value = 255;
                    for (int dy = -1; dy <= 1; dy++)
                    {
                        for (int dx = -1; dx <= 1; dx++)
                        {
                            if (AT_IMAGE(img0, x + dx, y + dy) < min_value) min_value = AT_IMAGE(img0, x + dx, y + dy);
                        }
                    }
                    AT_IMAGE(img1, x, y) = min_value;
                }
        }
    }
}


//3x3膨胀
/*
 * 对于输入图像中的每个像素(x,y):
        [ P(x-1,y-1)  P(x,y-1)  P(x+1,y-1) ]
        [ P(x-1,y)    P(x,y)    P(x+1,y)   ]  →  max(P[3x3]) → 输出到img1(x,y)
        [ P(x-1,y+1)  P(x,y+1)  P(x+1,y+1) ]
 */
void image_dilate3(image_t *img0,image_t *img1)
{
    if(img0->width==img1->width && img0->height==img1->height)
    {
        uint8_t max_value;
        for (int y = 1; y < img0->height - 1; y++)
        {
                for (int x = 1; x < img0->width - 1; x++)
                {
                    max_value = 0;
                            for (int dy = -1; dy <= 1; dy++)
                    {
                                for (int dx = -1; dx <= 1; dx++)
                        {
                            if (AT_IMAGE(img0, x + dx, y + dy) > max_value) max_value = AT_IMAGE(img0, x + dx, y + dy);
                        }
                    }
                    AT_IMAGE(img1, x, y) = max_value;
                }
            }
    }
}

//重映射
/*
 * 查找映射表
 */
void image_remap(image_t *img0, image_t *img1, image_t *mapx, image_t *mapy)
{
    if (!(img0 && img0->data)) return;
    if (!(img1 && img1->data)) return;
    if (!(mapx && mapx->data)) return;
    if (!(mapy && mapy->data)) return;
    if (!(img0 != img1 && img0->data != img1->data)) return;
    if (!(img0->width == img1->width && img0->height == img1->height)) return;
    if (!(mapx->width == mapy->width && mapx->height == mapy->height)) return;
    if (!(img0->width == mapx->width && img0->height == mapx->height)) return;

    for (int y = 1; y < img0->height - 1; y++)
    {
            for (int x = 1; x < img0->width - 1; x++)
            {
                AT_IMAGE(img1, x, y) = AT_IMAGE(img0, (int) (AT_IMAGE(mapx, x, y) + 0.5), (int) (AT_IMAGE(mapy, x, y) + 0.5));
            }
        }

}
void image_remap8(image_t *img0, image_t *img1, const image_t *mapx, const image_t *mapy)
{
    if (!(img0 && img0->data)) return;
    if (!(img1 && img1->data)) return;
    if (!(mapx && mapx->data)) return;
    if (!(mapy && mapy->data)) return;
    if (!(img0 != img1 && img0->data != img1->data)) return;
    if (!(img0->width == img1->width && img0->height == img1->height)) return;
    if (!(mapx->width == mapy->width && mapx->height == mapy->height)) return;
    if (!(img0->width == mapx->width && img0->height == mapx->height)) return;

    uint32_t height=img0->height;
    uint32_t width=img0->width;
    uint32_t total_pixels=height*width;

    uint8_t *dst_ptr=img1->data;
    const uint8_t *mapx_ptr=mapx->data;
    const uint8_t *mapy_ptr=mapy->data;

    for(uint32_t i=0;i<total_pixels;i++)
    {
        uint32_t src_x=*mapx_ptr++;
        uint32_t src_y=*mapy_ptr++;

        *dst_ptr=AT_IMAGE(img0,src_x,src_y);
  
        dst_ptr++;
    }
}

//-------------------------------图像处理-----------------------------------------------------------------//


//---------------------------------数据处理------------------------------------------------//

/*折线段近似
 *
 */
void approx_lines(int pts[][2],int pts_num,float epsilon,int lines[][2],int *lines_num)
{
    if(!pts) return;
    if(!(epsilon>0)) return;
    int dx=pts[pts_num-1][0]-pts[0][0];
    int dy=pts[pts_num-1][1]-pts[0][1];

    float dn=sqrtf(dx*dx+dy*dy);
    float nx=-dy/dn; //单位法向量
    float ny=dx/dn;
    float max_dist=0;
    float dist;
    int idx=-1;
    for(int i=1;i<pts_num-1;i++)
    {
        dist=fabs((pts[i][0]-pts[0][0])*nx+(pts[i][1]-pts[0][1])*ny);
        if(dist>max_dist)
        {
            max_dist=dist;      //距离最远的点
            idx=i;
        }
    }
    if(max_dist>=epsilon)       //距离大于阈值，则分为两段进行递归调用，直到可近似为直线
    {
        int num1=*lines_num;
        approx_lines(pts,idx+1,epsilon,lines,&num1);
        int num2=*lines_num-num1-1;
        approx_lines(pts+idx,pts_num-idx,epsilon,lines+num1-1,&num2);
        *lines_num=num1+num2-1;
    }
    else                       //距离小于阈值，则可直接近似为首尾相连的线段
    {
        lines[0][0] = pts[0][0];
        lines[0][1] = pts[0][1];
        lines[1][0] = pts[pts_num - 1][0];
        lines[1][1] = pts[pts_num - 1][1];
        *lines_num = 2;
    }
}


static uint8_t approx_depth = 0;
/**
 * @brief 道格拉斯-普克算法（浮点型）：折线近似拟合点集
 * @param pts        输入点集 [][0]=X, [][1]=Y（中线点集）
 * @param pts_num    输入点集的有效数量（真实有效中线点数量，非理论值）
 * @param epsilon    近似阈值（像素，值越小拟合越精细）
 * @param lines      输出折线端点集 [][0]=X, [][1]=Y（连续线段的端点）
 * @param lines_num  输出：折线端点的数量（传入时需初始化，递归后返回最终数量）
 * @note 1. 无效点（X超出屏幕范围）会被过滤，不参与计算；
 * @note 2. 算法核心：最远点距离≥阈值则递归分割，否则保留首尾点；
 * @note 3. 输入点集需是连续的有效中线点（从下往上的顺序）
 */
void approx_lines_float(float pts[][2], int pts_num, float epsilon, float lines[][2], int *lines_num)
{
    // 1. 入参合法性校验
    if (pts == NULL || lines == NULL || lines_num == NULL) return;
    if (pts_num < 2 || epsilon <= 0) { // 少于2个点无法构成线段
        *lines_num = 0;
        return;
    }
    approx_depth++;
     if(approx_depth > 8) { // 限制最大递归深度8层
         approx_depth--;
         return;
     }
    // 2. 过滤无效点（仅保留屏幕内的有效中线点）
    float valid_pts[MT9V03X_H][2]; // 临时存储有效点
    int valid_num = 0;
    for (int i = 0; i < pts_num; i++) {
        // 有效点判定：X在0~MT9V03X_W-1范围内（避免延用值/空值干扰）
        if (pts[i][0] >= 0 && pts[i][0] < MT9V03X_W) {
            valid_pts[valid_num][0] = pts[i][0];
            valid_pts[valid_num][1] = pts[i][1];
            valid_num++;
        }
    }
    if (valid_num < 2) { // 过滤后有效点不足2个，直接返回
        *lines_num = 0;
        return;
    }

    // 3. 计算首尾点的向量（改为float，避免精度丢失）
    float dx = valid_pts[valid_num-1][0] - valid_pts[0][0];
    float dy = valid_pts[valid_num-1][1] - valid_pts[0][1];
    float dn = sqrtf(dx*dx + dy*dy);
    if (dn < 1e-6) { // 首尾点重合，无需分割
        lines[0][0] = valid_pts[0][0];
        lines[0][1] = valid_pts[0][1];
        *lines_num = 1;
        return;
    }

    // 4. 计算单位法向量
    float nx = -dy / dn;
    float ny = dx / dn;

    // 5. 找距离首尾连线最远的点
    float max_dist = 0.0f;
    int max_idx = -1;
    for (int i = 1; i < valid_num - 1; i++) {
        // 点到直线的法向距离（公式：(x-x0)*nx + (y-y0)*ny）
        float dist = fabsf((valid_pts[i][0] - valid_pts[0][0])*nx + (valid_pts[i][1] - valid_pts[0][1])*ny);
        if (dist > max_dist) {
            max_dist = dist;
            max_idx = i;
        }
    }

    // 6. 递归分割/直接拟合
    if (max_dist >= epsilon && max_idx != -1) { // 距离≥阈值，递归分割
        float lines1[MT9V03X_H][2], lines2[MT9V03X_H][2];
        int num1 = 0, num2 = 0;

        // 递归处理前半段：首点 → 最远点
        approx_lines_float(valid_pts, max_idx + 1, epsilon, lines1, &num1);
        // 递归处理后半段：最远点 → 尾点
        approx_lines_float(valid_pts + max_idx, valid_num - max_idx, epsilon, lines2, &num2);

        // 合并两段结果（去掉重复的最远点）
        *lines_num = 0;
        // 复制前半段结果
        for (int i = 0; i < num1; i++) {
            lines[*lines_num][0] = lines1[i][0];
            lines[*lines_num][1] = lines1[i][1];
            (*lines_num)++;
        }
        // 复制后半段结果（跳过第一个点，与前半段最后一个点重复）
        for (int i = 1; i < num2; i++) {
            lines[*lines_num][0] = lines2[i][0];
            lines[*lines_num][1] = lines2[i][1];
            (*lines_num)++;
        }
    } else { // 距离<阈值，直接保留首尾点作为折线端点
        lines[0][0] = valid_pts[0][0];
        lines[0][1] = valid_pts[0][1];
        lines[1][0] = valid_pts[valid_num - 1][0];
        lines[1][1] = valid_pts[valid_num - 1][1];
        *lines_num = 2;
    }
    approx_depth--; // 递归返回时深度减1
}

/*点集三角滤波
 * 对二维点集进行三角滤波
 */

void blur_points(float pts_in[][2],int num,float pts_out[][2],int kernel)
{
    if(!(kernel%2==1)) return;
    int half=kernel/2;
    for(int i=0;i<num;i++)
    {
        pts_out[i][0]=pts_out[i][1]=0;
        for(int j=-half;j<=half;j++)//三角核进行加权平均
        {
            pts_out[i][0]+=pts_in[clip(i+j,0,num-1)][0]*(half+1-abs(j));//j=0，权重最大，向外依次递减
            pts_out[i][1]+=pts_in[clip(i+j,0,num-1)][1]*(half+1-abs(j));

        }
        //归一化
        pts_out[i][0]/=(2*half+2)*(half+1)/2;
        pts_out[i][1]/=(2*half+2)*(half+1)/2;
    }
}


/*点集等距采样
 *
 */

void resample_points(float pts_in[][2],int num1,float pts_out[][2],int *num2,float dist)
{
    float remain=0.f;
    int len=0;
    //遍历每一段折线
    for(int i=0;i<num1-1 && len<*num2;i++)
    {
        float x0=pts_in[i][0];
        float y0=pts_in[i][1];
        float x1=pts_in[i+1][0];
        float y1=pts_in[i+1][1];
        float dx=x1-x0;
        float dy=y1-y0;
        float dn=sqrtf(dx*dx+dy*dy);
        if(dn==0)continue;
        float ux=dx/dn; //单位方向向量
        float uy=dy/dn;


        //当前线段放置采样点
        //remain：与上一个采样点间距离
        while(remain<dn && len<*num2)
        {

            pts_out[len][0]=x0+ux*remain;

            pts_out[len][1]=y0+uy*remain;

            len++;
            dn-=remain;//更新当前线段未处理长度
            remain=dist;
        }
        remain-=dn;
    }
    *num2=len;
}

/*点集局部角度变化率
 *
 */
// 优化版角度函数：无sqrt/atan2，整数运算，TC264无压力
void local_angle_points(float pts_in[][2], int num, float angle_out[], int dist) {
    for(int i=0; i<num; i++) {
        angle_out[i] = 0;
        // 只处理前20行，且避免越界
        if(i < dist || i > 20 || i > num-1-dist) continue;

        int idx1 = i - dist;
        int idx2 = i + dist;
        // 整数运算：计算向量叉乘（替代角度，叉乘值=|A×B|，反映夹角大小）
        int dx1 = pts_in[i][0] - pts_in[idx1][0];
        int dy1 = pts_in[i][1] - pts_in[idx1][1];
        int dx2 = pts_in[idx2][0] - pts_in[i][0];
        int dy2 = pts_in[idx2][1] - pts_in[i][1];

        // 叉乘绝对值：越大，夹角越大（就是要找的角点）
        angle_out[i] = abs(dx1*dy2 - dx2*dy1);
    }
}




/*角度变化率非极大抑制
 * 保留局部极大值，将其他非极大值置0
 */

void nms_angle(float angle_in[],int num,float angle_out[],int kernel)
{
    if(!(kernel%2==1))return;
    int half=kernel/2;
    for(int i =0;i<num;i++)
    {
        angle_out[i]=angle_in[i];
        for(int j=-half;j<=half;j++)
        {
            if(fabs(angle_in[clip(i+j,0,num-1)])>fabs(angle_out[i]))    //领域内比较
            {
                angle_out[i]=0;
                break;
            }
        }
    }
}




//-----------------------------------------数据处理--------------------------------------------------//



//----------------------------------------绘制------------------------------------------------------//
/*
 * 画x
 */
void draw_x(image_t *img, int x, int y, int len, uint8_t value)
{
    for (int i = -len; i <= len; i++)
    {  
                AT_IMAGE(img, clip(x + i, 0, img->width - 1), clip(y + i, 0, img->height - 1)) = value;
                AT_IMAGE(img, clip(x - i, 0, img->width - 1), clip(y + i, 0, img->height - 1)) = value;
    }
}

/*画o
 *
 */
void draw_o(image_t *img, int x, int y, int radius, uint8_t value)
{
    for (float i = -PI; i <= PI; i += PI / 10)
    {
        AT_IMAGE(img, clip(x + radius * cosf(i), 0, img->width - 1), clip(y + radius * sinf(i), 0, img->height - 1)) = value;
    }
}



/*绘制直线
 *

void draw_line(image_t *img,float pt0[2], pt1[2],uint8_t value)
{
    int dx=pt1[0]-pt0[0];
    int dy=pt1[1]-pt0[1];
    if(abs(dx)>abs(dy))
    {
        for(int x=pt0[0];x!=pt1[0];x+=(dx>0?1:-1))
        {
            int y=pt0[1]+(x-pt0[0]*dy/dx);
            AT_IMAGE(img,clip(x,0,img->width-1),clip(y,0,img->height-1))=value;
        }
    }
    else
    {
        for(int y=pt0[1];y!=pt1[1];y+=(dy>0?1:-1))
        {
            int x=pt0[0]+(y-pt0[1])*dx/dy;
            AT_IMAGE(img,clip(x,0,img->width-1),clip(y,0,img->height-1))=value;
        }
    }
}
*/

/*绘制直线
 * 适配float类型的点坐标，修复类型不匹配和计算错误
 */
void draw_line(image_t *img, float x1, float y1, float x2, float y2, uint8_t value)
{
    // 1. 使用float计算增量
    float dx = x2 - x1;  // 注意：应该是x2-x1，而不是x1-x2
    float dy = y2 - y1;  // 注意：应该是y2-y1，而不是y1-y2

    // 2. 处理特殊情况：起点终点相同
    if (dx == 0 && dy == 0) {
        int x = clip((int)round(x1), 0, img->width - 1);
        int y = clip((int)round(y1), 0, img->height - 1);
        AT_IMAGE(img, x, y) = value;
        return;
    }

    // 3. 基于x或y方向的增量判断迭代方向
    if (fabs(dx) > fabs(dy)) {
        // x方向为主方向
        float step = dx > 0 ? 1.0f : -1.0f;
        int steps = (int)fabs(dx);

        for (int i = 0; i <= steps; i++) {
            float x = x1 + step * i;
            float y = y1 + (x - x1) * dy / dx;  // y = y1 + (x-x1) * slope

            // 转换为整数像素坐标并裁剪
            int px = clip((int)round(x), 0, img->width - 1);
            int py = clip((int)round(y), 0, img->height - 1);
            AT_IMAGE(img, px, py) = value;
        }
    } else {
        // y方向为主方向
        float step = dy > 0 ? 1.0f : -1.0f;
        int steps = (int)fabs(dy);

        for (int i = 0; i <= steps; i++) {
            float y = y1 + step * i;
            float x = x1 + (y - y1) * dx / dy;  // 修正：x1 + (y-y1) * slope

            int px = clip((int)round(x), 0, img->width - 1);
            int py = clip((int)round(y), 0, img->height - 1);
            AT_IMAGE(img, px, py) = value;
        }
    }
}
// 建议画3像素宽的黑线：
void draw_thick_line(image_t *img, float x1, float y1, float x2, float y2, uint8_t value)
{
    draw_line(img, x1, y1, x2, y2, value);
    draw_line(img, x1-1, y1, x2-1, y2, value);  // 左边
    draw_line(img, x1+1, y1, x2+1, y2, value);  // 右边
}
