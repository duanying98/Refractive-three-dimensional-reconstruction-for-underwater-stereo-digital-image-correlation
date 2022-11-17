#include "config.h"



/*******************************************************************************************************/
/*******************************************************************************************************/
/*******************************************构建折射界面相关参数****************************************/
Size imageSize;//标定图像的分辨率
Mat cameraMatrix1;//左相机内参矩阵
Mat distCoeffs1;//左相机畸变系数矩阵
Mat cameraMatrix2;//右相机内参矩阵
Mat distCoeffs2;//右相机畸变系数矩阵
Mat R, T; //R 旋转矢量 T平移矢量
Mat R1, R2, P1, P2, Q; //校正旋转矩阵R，投影矩阵P 重投影矩阵Q 
Mat mapx1, mapy1, mapx2, mapy2; //映射表  
Rect validROI1, validROI2; //图像校正之后，会对图像进行裁剪，这里的validROI就是指裁剪之后的区域  
Mat imgLeft;//校正后的左图像
Mat imgRight;//校正后的右图像
#define cannythreshold 80

//标记点检测做了canny后的图像存储的位置
string savePath = "E://bandzipDocuments//Circle//data_1_Circle//imgL_Circle.jpg";
string savePath2 = "E://bandzipDocuments//Circle//data_1_Circle//imgR_Circle.jpg";

//相机配置文件
string filename = "D://Projects//c++Projects//circleCoding//Calibration_Result.xml";

//左右相机拍摄的用于平面重建的图像
string path1 = "E://bandzipDocuments//Circle//data_1//imgL.jpg";
string path2 = "E://bandzipDocuments//Circle//data_1//imgR.jpg";

//是否展示立体校正后的结果图像
bool is_show_rectify_performance = false;

//y x    左右图像标记点匹配时，在y允许的匹配误差，在imgLeft.cols / x上允许的误差
int y = 10;
int x = 23;

//是否展示左右图像的标记点匹配的情况图片
bool is_show_stereo_match = false;

//ransac算法得到平面算法
int max_iter = 1000000; //迭代次数
double ransac_threshold = 0.000005;