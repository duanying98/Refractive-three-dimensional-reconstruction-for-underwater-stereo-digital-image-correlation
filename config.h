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

extern Size imageSize;//�궨ͼ��ķֱ���
extern Mat cameraMatrix1;//������ڲξ���
extern Mat distCoeffs1;//���������ϵ������
extern Mat cameraMatrix2;//������ڲξ���
extern Mat distCoeffs2;//���������ϵ������
extern Mat R, T; //R ��תʸ�� Tƽ��ʸ��
extern Mat R1, R2, P1, P2, Q; //У����ת����R��ͶӰ����P ��ͶӰ����Q 
extern Mat mapx1, mapy1, mapx2, mapy2; //ӳ���  
extern Rect validROI1, validROI2; //ͼ��У��֮�󣬻��ͼ����вü��������validROI����ָ�ü�֮�������  
extern Mat imgLeft;//У�������ͼ��
extern Mat imgRight;//У�������ͼ��
#define cannythreshold 80

//��ǵ�������canny���ͼ��洢��λ��
extern string savePath;
extern string savePath2;

//��������ļ�
extern string filename;

//����������������ƽ���ؽ���ͼ��
extern string path1;
extern string path2;

//�Ƿ�չʾ����У����Ľ��ͼ��
extern bool is_show_rectify_performance;


//y x    ����ͼ���ǵ�ƥ��ʱ����y�����ƥ������imgLeft.cols / x����������
extern int y;
extern int x;

//�Ƿ�չʾ����ͼ��ı�ǵ�ƥ������ͼƬ
extern bool is_show_stereo_match;


//ransac�㷨�õ�ƽ���㷨
extern int max_iter; //��������
extern double ransac_threshold;

//ƽ�淽�̲�������ֱ�߷��̲���
struct vec4d {
	double A;
	double B;
	double C;
	double D;
};


//��ά����������ά��
struct vec3d {
	double x;
	double y;
	double z;
};