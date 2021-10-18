#pragma once
#include <math.h>

// 2-dimensional vector
class Vec2 {
public:
	double x;
	double y;

public:
	Vec2();
	Vec2(double X, double Y);
	double X();
	double Y();
	double length();				// get length
	double distance(Vec2 vec);		// get length between two vectors
	//double CrossProduct(Vec2& vec);

	Vec2 operator+(Vec2 & ref) {
		return Vec2(x + ref.X(), y + ref.Y());
	}
	Vec2 operator-(Vec2 & ref) {
		return Vec2(x - ref.X(), y - ref.Y());
	}
	double operator*(Vec2 & ref) {
		return (x * ref.X()) + (y * ref.Y());
	}
	Vec2 operator*(double a) {
		return Vec2(a * x, a * y);
	}
	Vec2 operator/(double a) {
		return Vec2(x / a, y / a);
	}
	Vec2& operator+=(Vec2 & ref) {
		x += ref.X();
		y += ref.Y();
		return *this;
	}
	Vec2& operator-=(Vec2 & ref) {
		x -= ref.X();
		y -= ref.Y();
		return *this;
	}
	Vec2& operator *=(double a) {
		x *= a;
		y *= a;
		return *this;
	}
	bool operator==(Vec2 & ref) {
		if (x == ref.X() && y == ref.Y())
			return 1;
		else return 0;
	}
	bool operator!=(Vec2 & ref) {
		if (x != ref.X() || y != ref.Y())
			return 1;
		else return 0;
	}

	~Vec2() {}
};
double CrossProduct(Vec2 a, Vec2 b);
Vec2 CrossProduct(Vec2 a, double s);
Vec2 CrossProduct(double s, Vec2 a);
Vec2 normalize(Vec2 a);

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

