#include "plane.h"
#include "refract3D.h"


int main() {
	//测试折射界面重建
	/*vec4d plane;
	plane = plane_reconstruction();
	cout << plane.A << " " << plane.B << " " << plane.C << " " << plane.D << endl;*/

	//测试两个向量的叉积
	/*vec3d left;
	left.x = 2;
	left.y = 4;
	left.z = 8;

	vec3d right;
	right.x = 4;
	right.y = 5;
	right.z = 6;

	vec3d cross;
	cross = _Cross(left, right);
	cout << cross.x << endl;
	cout << cross.y << endl;
	cout << cross.z << endl;*/

	//测试向量单位化
	/*vec3d T;
	T.x = 1;
	T.y = 1;
	T.z = 1;
	vec3d A;
	A = _Normalize(T);

	cout << A.x << endl;
	cout << A.y << endl;
	cout << A.z << endl;*/

	//测试计算直线与平面的交点
	/*vec3d point;
	point.x = 9;
	point.y = 0;
	point.z = 0;

	vec3d normal;
	normal.x = 1;
	normal.y = 0;
	normal.z = 0;

	vec4d plane;
	plane.A = 1;
	plane.B = 0;
	plane.C = 0;
	plane.D = -1;

	vec3d A;
	A = _Intersection(point,normal,plane);

	cout << A.x << endl;
	cout << A.y << endl;
	cout << A.z << endl; */

	//测试两个向量夹角
	/*vec3d T;
	T.x = 1;
	T.y = 0;
	T.z = 0;
	vec3d B;
	B.x = 0;
	B.y = 0;
	B.z = 1;
	cout << _Vector_angle(T, B) << endl;
	cout << cos(PI/2) << endl;
	cout << acos(0) << endl;
	cout << PI / 2 << endl;*/

	//测试计算折射光线的向量
	/*vec3d vec1;
	vec1.x = 2 * 1.732;
	vec1.y = 0;
	vec1.z = 2;
	vec1 = _Normalize(vec1);

	vec3d N;
	N.x = 0;
	N.y = 0;
	N.z = 1;

	double n1 = 1;
	double n2 = 1.732;

	double theta1 = PI / 3;
	double theta2 = PI / 6;

	vec3d refracted_vector;
	refracted_vector = _Get_refracted_vector(vec1, N, n1, n2, theta1, theta2);
	cout << refracted_vector.x << endl;
	cout << refracted_vector.y << endl;
	cout << refracted_vector.z << endl;

	vec3d refracted_vector2;
	refracted_vector2.x = 2;
	refracted_vector2.y = 0;
	refracted_vector2.z = 2 * 1.732;
	refracted_vector2 = _Normalize(refracted_vector2);
	cout << refracted_vector2.x << endl;
	cout << refracted_vector2.y << endl;
	cout << refracted_vector2.z << endl;*/

	//计算两条线的交点测试
	/*vec3d point1;
	point1.x = 2;
	point1.y = 0;
	point1.z = 0;

	vec3d vec1;
	vec1.x = 1;
	vec1.y = 0;
	vec1.z = 0;

	vec3d point2;
	point2.x = 1;
	point2.y = 0;
	point2.z = 2;

	vec3d vec2;
	vec2.x = 0;
	vec2.y = 0;
	vec2.z = 1;

	vec3d intersection;
	intersection = _Intersection(point1, vec1, point2, vec2);
	cout << intersection.x << endl;
	cout << intersection.y << endl;
	cout << intersection.z << endl;*/

	//测试折射界面2的重建
	/*vec4d interf1;
	interf1.A = 0;
	interf1.B = 0;
	interf1.C = 1;
	interf1.D = 0;
	cout << interf1.A << endl;
	cout << interf1.B << endl;
	cout << interf1.C << endl;
	cout << interf1.D << endl;

	double delta = 9;
	double near_d = interf1.D;

	// Build the interface 2
	double far_d = near_d - delta * sqrt(pow(interf1.A, 2) + pow(interf1.B, 2) + pow(interf1.C, 2));

	vec4d interf2 = { interf1.A, interf1.B, interf1.C, far_d };
	cout << interf2.A << endl;
	cout << interf2.B << endl;
	cout << interf2.C << endl;
	cout << interf2.D << endl;*/

	//折射重建测试
	/*vec3d raw_point;
	raw_point.x = 9;
	raw_point.y = 0;
	raw_point.z = 3 * 1.732;

	vec3d translation;
	translation.x = -18;
	translation.y = 0;
	translation.z = 0;

	vec4d interf1;
	interf1.A = 0;
	interf1.B = 0;
	interf1.C = 1;
	interf1.D = -2;

	double n1 = 1;
	double n2 = 1.732;
	double n3 = 1;

	double thickness = 1;

	vec3d res;
	res = refractive_3d_reconstruction(raw_point, translation, interf1, n1, n2, n3, thickness);
	cout << res.x << endl;
	cout << res.y << endl;
	cout << res.z << endl;

	cout << 3 * 1.732 << endl;*/


}