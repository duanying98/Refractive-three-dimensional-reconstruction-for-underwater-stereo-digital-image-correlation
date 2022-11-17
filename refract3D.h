#pragma once
#include "config.h"

//计算两个三维向量的叉积
//n = u(x1, y1, z1) x v(x2, y2, z2)
//= (y1z2 - y2z1, x2z1 - z2x1, x1y2 - x2y1)
vec3d _Cross(vec3d left, vec3d right);

//计算三维向量的单位向量
vec3d _Normalize(vec3d T);

//计算直线与平面的交点
//其中plane表示平面方程，应为Ax+By+Cz+D=0的格式，其中（A,B,C）为其法向量
//point表示线段的终点
//normal表示线段的单位向量
vec3d _Intersection(vec3d point, vec3d normal, vec4d plane);

//计算两个三维向量的夹角
double _Vector_angle(vec3d vec1, vec3d vec2);

//Compute the unit direction of the refracted ray.     计算折射光线的单位向量  
// <param name="'vec1'">Incident vector   入射光线单位向量</param>
// <param name="'N'">Normal of the interface   折射界面的法向量</param>
// <param name="'n1'">Refractive index of incident medium   入射角的折射率</param>
// <param name="'n2'">Refractive index of emergent medium   折射角的折射率</param>
// <param name="'theta1'">Incident angle   入射角</param>
// <param name="'theta2'">Emergent angle   折射角</param>
// <returns>Unit direction vector</returns>
vec3d _Get_refracted_vector(vec3d vec1, vec3d N, double n1, double n2, double theta1, double theta2);

//Compute the intersection of two lines.          计算两条线的交点
/// <param name="'point1'">End point of line 1;   线段1的端点 </param>
/// <param name="'vec1'">Direction of line 1;     线段1的方向向量</param>
/// <param name="'point2'">End point of line 2;   线段2的端点</param>
/// <param name="'vec2'">Direction of line 2.     线段2的方向向量</param>
/// <returns>Coordinates of the intersection      两条线段的交点坐标</returns>
vec3d _Intersection(vec3d point1, vec3d vec1, vec3d point2, vec3d vec2);

/********************************************************************************************
 * \brief Driver for refractive 3D reconstruction.          折射三维重建
 * \param 'raw_point' distorted object point P.             'raw_point'代表扭曲的点P。
 * \param 'translation' translation vector of the right camera relative to the frame O-XYZ.  translation代表右相机相对于参考坐标系O-XYZ的平移矢量。
 * \param 'interf1' refracting interface Ax + By + Cz +D = 0, with N = (A. B. C).
 * \param n1 n2 n3 thickness  折射率以及玻璃厚度
 * \returns The true 3D coordinates of the object point.    返回真实的三维坐标
 ********************************************************************************************/
vec3d refractive_3d_reconstruction(vec3d raw_point, vec3d translation, vec4d interf1, double n1, double n2, double n3, double thickness);