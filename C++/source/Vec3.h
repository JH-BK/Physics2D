#pragma once
#include <math.h>

// 3-dimensional vector
class Vec3 {
public:
	double x;
	double y;
	double z;

public:
	Vec3();
	Vec3(double X, double Y, double Z);
	double X();
	double Y();
	double Z();
	double length();				// get length
	double distance(Vec3 vec);		// get length between two vectors
									//double CrossProduct(Vec3& vec);

	Vec3 operator+(Vec3 & ref) {
		return Vec3(x + ref.X(), y + ref.Y(), z + ref.Z());
	}
	Vec3 operator-(Vec3 & ref) {
		return Vec3(x - ref.X(), y - ref.Y(), z - ref.Z());
	}
	double operator*(Vec3 & ref) {
		return (x * ref.X()) + (y * ref.Y()) + (z * ref.Z());
	}
	Vec3 operator*(double a) {
		return Vec3(a * x, a * y, a * z);
	}
	Vec3 operator/(double a) {
		return Vec3(x / a, y / a, z / a);
	}
	Vec3& operator+=(Vec3 & ref) {
		x += ref.X();
		y += ref.Y();
		z += ref.Z();
		return *this;
	}
	Vec3& operator-=(Vec3 & ref) {
		x -= ref.X();
		y -= ref.Y();
		z -= ref.Z();
		return *this;
	}
	Vec3& operator *=(double a) {
		x *= a;
		y *= a;
		z *= a;
		return *this;
	}
	bool operator==(Vec3 & ref) {
		if (x == ref.X() && y == ref.Y() && z == ref.Z())
			return 1;
		else return 0;
	}
	bool operator!=(Vec3 & ref) {
		if (x != ref.X() || y != ref.Y() || z != ref.Z())
			return 1;
		else return 0;
	}

	~Vec3() {}
};
double CrossProduct(Vec3 a, Vec3 b);
Vec3 normalize(Vec3 a);

// Rotation matrix
class Mat22 {
public:
	double m00;
	double m01;
	double m10;
	double m11;
	//Vec2 xCol;
	//Vec2 yCol;

	Mat22(Vec2 x, Vec2 y) {
		m00 = x.X();
		m01 = x.Y();
		m10 = y.X();
		m11 = y.Y();
		//xCol = x;
		//yCol = y;
	}
	Mat22(double radian) {
		double c = cos(radian);
		double s = sin(radian);
		m00 = c; m01 = -s;
		m10 = s; m11 = c;
	}
	Vec2 operator* (Vec2 & rhs) const {
		return Vec2(m00 * rhs.X() + m01 * rhs.Y(), m10 * rhs.X() + m11 * rhs.Y());
	}
};

