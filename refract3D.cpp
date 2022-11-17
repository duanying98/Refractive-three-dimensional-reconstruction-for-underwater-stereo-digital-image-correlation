#include "refract3D.h"

vec3d _Cross(vec3d left, vec3d right) {
	return {
		left.y*right.z - right.y*left.z,
		right.x*left.z - right.z*left.x,
		left.x*right.y - right.x*left.y
	};
}

vec3d _Normalize(vec3d T) {
	double a = sqrt(pow(T.x, 2) + pow(T.y, 2) + pow(T.z, 2));
	return { T.x / a,T.y / a,T.z / a };
}

vec3d _Intersection(vec3d point, vec3d normal, vec4d plane) {
	double x = -((plane.D * normal.x - plane.B * normal.y * point.x - plane.C * normal.z * point.x + plane.B * normal.x * point.y + plane.C * normal.x * point.z) / (plane.A * normal.x + plane.B * normal.y + plane.C * normal.z));
	double y = -((plane.D * normal.y + plane.A * normal.y * point.x - plane.A * normal.x * point.y - plane.C * normal.z * point.y + plane.C * normal.y * point.z) / (plane.A * normal.x + plane.B * normal.y + plane.C * normal.z));
	double z = -((plane.D * normal.z + plane.A * normal.z * point.x + plane.B * normal.z * point.y - plane.A * normal.x * point.z - plane.B * normal.y * point.z) / (plane.A * normal.x + plane.B * normal.y + plane.C * normal.z));
	if (x == -0) {
		x = 0;
	}
	if (y == -0) {
		y = 0;
	}
	if (z == -0) {
		z = 0;
	}
	return { x,y,z };
}

double _Vector_angle(vec3d vec1, vec3d vec2) {
	vec3d vec11 = _Normalize(vec1);
	vec3d vec22 = _Normalize(vec2);

	return acos(
		(vec11.x*vec22.x + vec11.y*vec22.y + vec11.z*vec22.z)
		/
		(sqrt(pow(vec11.x, 2) + pow(vec11.y, 2) + pow(vec11.z, 2))*sqrt(pow(vec22.x, 2) + pow(vec22.y, 2) + pow(vec22.z, 2)))
	);
}

vec3d _Get_refracted_vector(vec3d vec1, vec3d N, double n1, double n2, double theta1, double theta2) {
	vec3d _Prev = { (n1 / n2) * vec1.x,
					(n1 / n2) * vec1.y,
					(n1 / n2) * vec1.z };

	vec3d _Tail = { (n1 / n2 * cos(theta1) - cos(theta2)) * N.x,
					(n1 / n2 * cos(theta1) - cos(theta2)) * N.y,
					(n1 / n2 * cos(theta1) - cos(theta2)) * N.z };

	return { _Prev.x - _Tail.x, _Prev.y - _Tail.y, _Prev.z - _Tail.z };
}

vec3d _Intersection(vec3d point1, vec3d vec1, vec3d point2, vec3d vec2) {
	vec3d vec3 = _Cross(vec1, vec2);
	double x1 = -((point1.z * vec1.x * vec2.y * vec3.x - point2.z * vec1.x * vec2.y * vec3.x -
		point1.x * vec1.z * vec2.y * vec3.x - point1.y * vec1.x * vec2.z * vec3.x +
		point2.y * vec1.x * vec2.z * vec3.x + point1.x * vec1.y * vec2.z * vec3.x -
		point1.z * vec1.x * vec2.x * vec3.y + point2.z * vec1.x * vec2.x * vec3.y +
		point1.x * vec1.z * vec2.x * vec3.y - point2.x * vec1.x * vec2.z * vec3.y +
		point1.y * vec1.x * vec2.x * vec3.z - point2.y * vec1.x * vec2.x * vec3.z -
		point1.x * vec1.y * vec2.x * vec3.z + point2.x * vec1.x * vec2.y * vec3.z) /
		(vec1.z * vec2.y * vec3.x - vec1.y * vec2.z * vec3.x - vec1.z * vec2.x * vec3.y +
			vec1.x * vec2.z * vec3.y + vec1.y * vec2.x * vec3.z - vec1.x * vec2.y * vec3.z));
	double y1 = -((-point1.z * vec1.y * vec2.y * vec3.x + point2.z * vec1.y * vec2.y * vec3.x +
		point1.y * vec1.z * vec2.y * vec3.x - point2.y * vec1.y * vec2.z * vec3.x +
		point1.z * vec1.y * vec2.x * vec3.y - point2.z * vec1.y * vec2.x * vec3.y -
		point1.y * vec1.z * vec2.x * vec3.y + point1.y * vec1.x * vec2.z * vec3.y -
		point1.x * vec1.y * vec2.z * vec3.y + point2.x * vec1.y * vec2.z * vec3.y +
		point2.y * vec1.y * vec2.x * vec3.z - point1.y * vec1.x * vec2.y * vec3.z +
		point1.x * vec1.y * vec2.y * vec3.z - point2.x * vec1.y * vec2.y * vec3.z) /
		(-vec1.z * vec2.y * vec3.x + vec1.y * vec2.z * vec3.x + vec1.z * vec2.x * vec3.y -
			vec1.x * vec2.z * vec3.y - vec1.y * vec2.x * vec3.z + vec1.x * vec2.y * vec3.z));
	double z1 = -((point2.z * vec1.z * vec2.y * vec3.x - point1.z * vec1.y * vec2.z * vec3.x +
		point1.y * vec1.z * vec2.z * vec3.x - point2.y * vec1.z * vec2.z * vec3.x -
		point2.z * vec1.z * vec2.x * vec3.y + point1.z * vec1.x * vec2.z * vec3.y -
		point1.x * vec1.z * vec2.z * vec3.y + point2.x * vec1.z * vec2.z * vec3.y +
		point1.z * vec1.y * vec2.x * vec3.z - point1.y * vec1.z * vec2.x * vec3.z +
		point2.y * vec1.z * vec2.x * vec3.z - point1.z * vec1.x * vec2.y * vec3.z +
		point1.x * vec1.z * vec2.y * vec3.z - point2.x * vec1.z * vec2.y * vec3.z) /
		(-vec1.z * vec2.y * vec3.x + vec1.y * vec2.z * vec3.x + vec1.z * vec2.x * vec3.y -
			vec1.x * vec2.z * vec3.y - vec1.y * vec2.x * vec3.z + vec1.x * vec2.y * vec3.z));

	double x2 = -((-point1.z * vec1.y * vec2.x * vec3.x + point2.z * vec1.y * vec2.x * vec3.x +
		point1.y * vec1.z * vec2.x * vec3.x - point2.y * vec1.z * vec2.x * vec3.x +
		point2.x * vec1.z * vec2.y * vec3.x - point2.x * vec1.y * vec2.z * vec3.x +
		point1.z * vec1.x * vec2.x * vec3.y - point2.z * vec1.x * vec2.x * vec3.y -
		point1.x * vec1.z * vec2.x * vec3.y + point2.x * vec1.x * vec2.z * vec3.y -
		point1.y * vec1.x * vec2.x * vec3.z + point2.y * vec1.x * vec2.x * vec3.z +
		point1.x * vec1.y * vec2.x * vec3.z - point2.x * vec1.x * vec2.y * vec3.z) /
		(-vec1.z * vec2.y * vec3.x + vec1.y * vec2.z * vec3.x + vec1.z * vec2.x * vec3.y -
			vec1.x * vec2.z * vec3.y - vec1.y * vec2.x * vec3.z + vec1.x * vec2.y * vec3.z));
	double y2 = -((-point1.z * vec1.y * vec2.y * vec3.x + point2.z * vec1.y * vec2.y * vec3.x +
		point1.y * vec1.z * vec2.y * vec3.x - point2.y * vec1.y * vec2.z * vec3.x -
		point2.y * vec1.z * vec2.x * vec3.y + point1.z * vec1.x * vec2.y * vec3.y -
		point2.z * vec1.x * vec2.y * vec3.y - point1.x * vec1.z * vec2.y * vec3.y +
		point2.x * vec1.z * vec2.y * vec3.y + point2.y * vec1.x * vec2.z * vec3.y +
		point2.y * vec1.y * vec2.x * vec3.z - point1.y * vec1.x * vec2.y * vec3.z +
		point1.x * vec1.y * vec2.y * vec3.z - point2.x * vec1.y * vec2.y * vec3.z) /
		(-vec1.z * vec2.y * vec3.x + vec1.y * vec2.z * vec3.x +
			vec1.z * vec2.x * vec3.y - vec1.x * vec2.z * vec3.y -
			vec1.y * vec2.x * vec3.z + vec1.x * vec2.y * vec3.z));
	double z2 = -((point2.z * vec1.z * vec2.y * vec3.x - point1.z * vec1.y * vec2.z * vec3.x +
		point1.y * vec1.z * vec2.z * vec3.x - point2.y * vec1.z * vec2.z * vec3.x -
		point2.z * vec1.z * vec2.x * vec3.y + point1.z * vec1.x * vec2.z * vec3.y -
		point1.x * vec1.z * vec2.z * vec3.y + point2.x * vec1.z * vec2.z * vec3.y +
		point2.z * vec1.y * vec2.x * vec3.z - point2.z * vec1.x * vec2.y * vec3.z -
		point1.y * vec1.x * vec2.z * vec3.z + point2.y * vec1.x * vec2.z * vec3.z +
		point1.x * vec1.y * vec2.z * vec3.z - point2.x * vec1.y * vec2.z * vec3.z) /
		(-vec1.z * vec2.y * vec3.x + vec1.y * vec2.z * vec3.x +
			vec1.z * vec2.x * vec3.y - vec1.x * vec2.z * vec3.y -
			vec1.y * vec2.x * vec3.z + vec1.x * vec2.y * vec3.z));


	double x = (x1 + x2) / 2;
	double y = (y1 + y2) / 2;
	double z = (z1 + z2) / 2;
	if (x == -0) {
		x = 0;
	}
	if (y == -0) {
		y = 0;
	}
	if (z == -0) {
		z = 0;
	}


	return { x,	y, z };
}

vec3d refractive_3d_reconstruction(vec3d raw_point, vec3d translation, vec4d interf1, double n1, double n2, double n3, double thickness) {

	double delta = thickness;
	vec3d t = translation;
	double near_d = interf1.D;

	// Build the interface 2
	double far_d = near_d - delta * sqrt(pow(interf1.A, 2) + pow(interf1.B, 2) + pow(interf1.C, 2));

	vec4d interf2 = { interf1.A, interf1.B, interf1.C, far_d };

	// Compute the normalized normal of the interfaces 1 and 2
	vec3d N1 = { interf1.A, interf1.B, interf1.C };
	vec3d N2 = { interf2.A, interf2.B, interf2.C };
	N1 = _Normalize(N1);
	N2 = _Normalize(N2);

	// Direction vectors of the ray L1 and L1'
	vec3d nlineL1 = _Normalize(raw_point);
	vec3d nlineR1 = _Normalize({ raw_point.x + t.x,raw_point.y + t.y,raw_point.z + t.z });

	// Incident points P1 and P1' of L1 and L1' at the interface 1
	vec3d pointL1 = _Intersection({ 0, 0, 0 }, nlineL1, interf1);
	vec3d pointR1 = _Intersection({ -t.x, -t.y, -t.z }, nlineR1, interf1);

	// Incident angles of rays L1 and L1'
	double angleL1 = _Vector_angle(nlineL1, N1);
	double angleR1 = _Vector_angle(nlineR1, N1);

	// Refracted angles of rays L1 and L1' 
	double angleL2 = asin(n1 * sin(angleL1) / n2);
	double angleR2 = asin(n1 * sin(angleR1) / n2);

	// Direction vectors of the ray L2 and L2'
	vec3d nlineL2 = _Get_refracted_vector(nlineL1, N1, n1, n2, angleL1, angleL2);
	vec3d nlineR2 = _Get_refracted_vector(nlineR1, N1, n1, n2, angleR1, angleR2);

	// Incident points P2 and P2' of L2 and L2' at the interface 2
	vec3d pointL2 = _Intersection(pointL1, nlineL2, interf2);
	vec3d pointR2 = _Intersection(pointR1, nlineR2, interf2);

	// Incident angles of rays L2 and L2'
	double angleL3 = angleL2;
	double angleR3 = angleR2;

	// Refracted angles of rays L2 and L2'
	double angleL4 = asin(n2 * sin(angleL3) / n3);
	double angleR4 = asin(n2 * sin(angleR3) / n3);

	// Direction vectors of the ray L3 and L3'
	vec3d nlineL3 = _Get_refracted_vector(nlineL2, N2, n2, n3, angleL3, angleL4);
	vec3d nlineR3 = _Get_refracted_vector(nlineR2, N2, n2, n3, angleR3, angleR4);

	//auto nCommonVerticalLine3 = detail::_Cross(nlineL3, nlineR3);

	// Compute the true object point Q
	return _Intersection(pointL2, nlineL3, pointR2, nlineR3);
}