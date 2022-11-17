#pragma once
#include "config.h"

//����������ά�����Ĳ��
//n = u(x1, y1, z1) x v(x2, y2, z2)
//= (y1z2 - y2z1, x2z1 - z2x1, x1y2 - x2y1)
vec3d _Cross(vec3d left, vec3d right);

//������ά�����ĵ�λ����
vec3d _Normalize(vec3d T);

//����ֱ����ƽ��Ľ���
//����plane��ʾƽ�淽�̣�ӦΪAx+By+Cz+D=0�ĸ�ʽ�����У�A,B,C��Ϊ�䷨����
//point��ʾ�߶ε��յ�
//normal��ʾ�߶εĵ�λ����
vec3d _Intersection(vec3d point, vec3d normal, vec4d plane);

//����������ά�����ļн�
double _Vector_angle(vec3d vec1, vec3d vec2);

//Compute the unit direction of the refracted ray.     ����������ߵĵ�λ����  
// <param name="'vec1'">Incident vector   ������ߵ�λ����</param>
// <param name="'N'">Normal of the interface   �������ķ�����</param>
// <param name="'n1'">Refractive index of incident medium   ����ǵ�������</param>
// <param name="'n2'">Refractive index of emergent medium   ����ǵ�������</param>
// <param name="'theta1'">Incident angle   �����</param>
// <param name="'theta2'">Emergent angle   �����</param>
// <returns>Unit direction vector</returns>
vec3d _Get_refracted_vector(vec3d vec1, vec3d N, double n1, double n2, double theta1, double theta2);

//Compute the intersection of two lines.          ���������ߵĽ���
/// <param name="'point1'">End point of line 1;   �߶�1�Ķ˵� </param>
/// <param name="'vec1'">Direction of line 1;     �߶�1�ķ�������</param>
/// <param name="'point2'">End point of line 2;   �߶�2�Ķ˵�</param>
/// <param name="'vec2'">Direction of line 2.     �߶�2�ķ�������</param>
/// <returns>Coordinates of the intersection      �����߶εĽ�������</returns>
vec3d _Intersection(vec3d point1, vec3d vec1, vec3d point2, vec3d vec2);

/********************************************************************************************
 * \brief Driver for refractive 3D reconstruction.          ������ά�ؽ�
 * \param 'raw_point' distorted object point P.             'raw_point'����Ť���ĵ�P��
 * \param 'translation' translation vector of the right camera relative to the frame O-XYZ.  translation�������������ڲο�����ϵO-XYZ��ƽ��ʸ����
 * \param 'interf1' refracting interface Ax + By + Cz +D = 0, with N = (A. B. C).
 * \param n1 n2 n3 thickness  �������Լ��������
 * \returns The true 3D coordinates of the object point.    ������ʵ����ά����
 ********************************************************************************************/
vec3d refractive_3d_reconstruction(vec3d raw_point, vec3d translation, vec4d interf1, double n1, double n2, double n3, double thickness);