#pragma once
#include "config.h"

//��ȡ�������
bool readFile(string filename);

//��������ͼ��,�������У�����imgLeft��imgRight
void stereo_rectify(Mat grayimgLeft, Mat grayimgRight);

//չʾ
void show_rectify_performance();

//��������У����ĻҶ�ͼ���Լ���⵽��Բ�ı���·���� �����ҵ��ı�ǵ����Ӿ��ε�����
vector<RotatedRect> findellipse(Mat LLL, string savePath);

//Match_boxL Match_boxR ����ͼ��ƥ��ı�ǵ������
//boxL boxR  ����ͼ���ҵ��ı�ǵ�����
//y x    ��y�����ƥ������imgLeft.cols / x����������
//isShow   �Ƿ���ʾƥ����
void stereo_match(vector<RotatedRect> &Match_boxL, vector<RotatedRect> &Match_boxR, vector<RotatedRect> boxL, vector<RotatedRect> boxR, int y, int x, bool isShow);

//ƥ�䵽������ͼ���ǵ�����꣬������ά����
vector<Point3d> getWord3d(vector<RotatedRect> Match_boxL, vector<RotatedRect> Match_boxR);

//����RANSAC�㷨�ϳ�ƽ��
vec4d ransac(vector<Point3d> &pts_3d, int max_iter, double threshold);

//��ȡƽ��
vec4d plane_reconstruction();