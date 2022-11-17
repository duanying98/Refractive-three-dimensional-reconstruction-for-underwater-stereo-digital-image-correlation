#pragma once
#include "config.h"

//读取相机参数
bool readFile(string filename);

//输入左右图像,获得立体校正后的imgLeft，imgRight
void stereo_rectify(Mat grayimgLeft, Mat grayimgRight);

//展示
void show_rectify_performance();

//输入立体校正后的灰度图，以及检测到的圆的保存路径， 返回找到的标记点的外接矩形的质心
vector<RotatedRect> findellipse(Mat LLL, string savePath);

//Match_boxL Match_boxR 左右图像匹配的标记点的坐标
//boxL boxR  左右图像找到的标记点坐标
//y x    在y允许的匹配误差，在imgLeft.cols / x上允许的误差
//isShow   是否显示匹配结果
void stereo_match(vector<RotatedRect> &Match_boxL, vector<RotatedRect> &Match_boxR, vector<RotatedRect> boxL, vector<RotatedRect> boxR, int y, int x, bool isShow);

//匹配到的左右图像标记点的坐标，返回三维坐标
vector<Point3d> getWord3d(vector<RotatedRect> Match_boxL, vector<RotatedRect> Match_boxR);

//基于RANSAC算法合成平面
vec4d ransac(vector<Point3d> &pts_3d, int max_iter, double threshold);

//获取平面
vec4d plane_reconstruction();