#include "config.h"



/*******************************************************************************************************/
/*******************************************************************************************************/
/*******************************************�������������ز���****************************************/
Size imageSize;//�궨ͼ��ķֱ���
Mat cameraMatrix1;//������ڲξ���
Mat distCoeffs1;//���������ϵ������
Mat cameraMatrix2;//������ڲξ���
Mat distCoeffs2;//���������ϵ������
Mat R, T; //R ��תʸ�� Tƽ��ʸ��
Mat R1, R2, P1, P2, Q; //У����ת����R��ͶӰ����P ��ͶӰ����Q 
Mat mapx1, mapy1, mapx2, mapy2; //ӳ���  
Rect validROI1, validROI2; //ͼ��У��֮�󣬻��ͼ����вü��������validROI����ָ�ü�֮�������  
Mat imgLeft;//У�������ͼ��
Mat imgRight;//У�������ͼ��
#define cannythreshold 80

//��ǵ�������canny���ͼ��洢��λ��
string savePath = "E://bandzipDocuments//Circle//data_1_Circle//imgL_Circle.jpg";
string savePath2 = "E://bandzipDocuments//Circle//data_1_Circle//imgR_Circle.jpg";

//��������ļ�
string filename = "D://Projects//c++Projects//circleCoding//Calibration_Result.xml";

//����������������ƽ���ؽ���ͼ��
string path1 = "E://bandzipDocuments//Circle//data_1//imgL.jpg";
string path2 = "E://bandzipDocuments//Circle//data_1//imgR.jpg";

//�Ƿ�չʾ����У����Ľ��ͼ��
bool is_show_rectify_performance = false;

//y x    ����ͼ���ǵ�ƥ��ʱ����y�����ƥ������imgLeft.cols / x����������
int y = 10;
int x = 23;

//�Ƿ�չʾ����ͼ��ı�ǵ�ƥ������ͼƬ
bool is_show_stereo_match = false;

//ransac�㷨�õ�ƽ���㷨
int max_iter = 1000000; //��������
double ransac_threshold = 0.000005;