/*
 * image.h
 *
 *  Created on: 2025��9��27��
 *      Author: cow
 */

#ifndef CODE_IMAGE_H_
#define CODE_IMAGE_H_

#include "zf_common_headfile.h"





#define AT_IMAGE(img, x, y)          ((img)->data[(y)*(img)->step+(x)])
#define AT_IMAGE_CLIP(img, x, y)     AT_IMAGE(img, clip(x, 0, (img)->width-1), clip(y, 0, (img)->height-1))



void image_clone(image_t *img0,image_t *img1);
void image_clear(image_t *img);


void threshold_fixed(image_t *img0,image_t *img1,uint8_t thres,uint8_t low,uint8_t high);
void threshole_adaptive(image_t *img0,image_t *img1,int block_size,int down,int low,int high);
uint16_t Getthreshold_OSTU(image_t *img,uint8_t x0,uint8_t y0,uint8_t x1,uint8_t y1);
int fast_threshold(uint8 *image, uint16 col, uint16 row);
void threshold_OSTU(image_t *img0,image_t *img1,uint8_t low,uint8_t high);


void image_and(image_t *img0,image_t *img1,image_t *img2);
void image_or(image_t *img0,image_t *img1,image_t *img2);
void image_minpool2(image_t *img0,image_t *img1);
void image_blur(image_t *img0,image_t *img1,uint32_t kernel);
void image_sobel3(image_t *img0,image_t *img1);
void image_erode3(image_t *img0,image_t *img1);
void image_dilate3(image_t *img0,image_t *img1);
void image_remap(image_t *img0, image_t *img1, image_t *mapx, image_t *mapy);
void image_remap8(image_t *img0, image_t *img1, const image_t *mapx, const image_t *mapy);

//void findleftline_eight(image_t *img,int x,int y,float pts[][2],int *num);
//void findrightline_eight(image_t *img,int x,int y,int pts[][2],int *num);
//void findline_left_adaptive(image_t *img, int block_size, int clip_value, int x, int y, int pts[][2], int *num);
//void findline_righthand_adaptive(image_t *img,int block_size,int clip_value,int x,int y,int pts[][2],int *num);
void approx_lines(int pts[][2],int pts_num,float epsilon,int lines[][2],int *lines_num);
void approx_lines_float(float pts[][2],int pts_num,float epsilon,float lines[][2],int *lines_num);
void blur_points(float pts_in[][2],int num,float pts_out[][2],int kernel);
void resample_points(float pts_in[][2],int num1,float pts_out[][2],int *num2,float dist);
void local_angle_points(float pts_in[][2],int num,float angle_out[],int dist);
void nms_angle(float angle_in[],int num,float angle_out[],int kernel);

void draw_x(image_t *img, int x, int y, int len, uint8_t value) ;
void draw_o(image_t *img, int x, int y, int radius, uint8_t value);
void draw_line(image_t *img, float x1, float y1, float x2, float y2, uint8_t value);
void draw_thick_line(image_t *img, float x1, float y1, float x2, float y2, uint8_t value);

#endif /* CODE_IMAGE_H_ */
