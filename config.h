#pragma once
#include <iostream>
#include <sstream>
#include <string>
#include <time.h>
#include <stdio.h>
#include <math.h>
#include <opencv2/core.hpp>
#include <opencv2/core/utility.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/calib3d.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/videoio.hpp>
#include <opencv2/highgui.hpp>
#include<opencv/cv.h>
using namespace std;
using namespace cv;
#define PI acos(-1)

extern Size imageSize;//标定图像的分辨率
extern Mat cameraMatrix1;//左相机内参矩阵
extern Mat distCoeffs1;//左相机畸变系数矩阵
extern Mat cameraMatrix2;//右相机内参矩阵
extern Mat distCoeffs2;//右相机畸变系数矩阵
extern Mat R, T; //R 旋转矢量 T平移矢量
extern Mat R1, R2, P1, P2, Q; //校正旋转矩阵R，投影矩阵P 重投影矩阵Q 
extern Mat mapx1, mapy1, mapx2, mapy2; //映射表  
extern Rect validROI1, validROI2; //图像校正之后，会对图像进行裁剪，这里的validROI就是指裁剪之后的区域  
extern Mat imgLeft;//校正后的左图像
extern Mat imgRight;//校正后的右图像
#define cannythreshold 80

//标记点检测做了canny后的图像存储的位置
extern string savePath;
extern string savePath2;

//相机配置文件
extern string filename;

//左右相机拍摄的用于平面重建的图像
extern string path1;
extern string path2;

//是否展示立体校正后的结果图像
extern bool is_show_rectify_performance;


//y x    左右图像标记点匹配时，在y允许的匹配误差，在imgLeft.cols / x上允许的误差
extern int y;
extern int x;

//是否展示左右图像的标记点匹配的情况图片
extern bool is_show_stereo_match;


//ransac算法得到平面算法
extern int max_iter; //迭代次数
extern double ransac_threshold;

//平面方程参数或者直线方程参数
struct vec4d {
	double A;
	double B;
	double C;
	double D;
};


//三维向量或者三维点
struct vec3d {
	double x;
	double y;
	double z;
};