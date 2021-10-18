#include "stdafx.h"
#include <cmath>
#include "Vec2.h"
using namespace std;

Vec2::Vec2() : x(0), y(0) {}
Vec2::Vec2(double X, double Y) : x(X), y(Y) {}

double Vec2::X() {
	return x;
}
double Vec2::Y() {
	return y;
}
double Vec2::length() {
	return sqrt(pow(x, 2) + pow(y, 2));
}
double Vec2::distance(Vec2 vec) {
	return sqrt(pow(x - vec.X(), 2) + pow(y - vec.Y(), 2));
}
/*double Vec2::CrossProduct(Vec2& vec) {
	return x * vec.Y() - y * vec.X();
}*/

double CrossProduct(Vec2 a, Vec2 b) {
	return a.X()*b.Y() - a.Y()*b.X();
}
Vec2 CrossProduct(Vec2 a, double s) {
	return Vec2(s*a.Y(), -s * a.X());
}
Vec2 CrossProduct(double s, Vec2 a) {
	return Vec2(-s * a.Y(), s * a.X());
}
Vec2 normalize(Vec2 a) {
	return a / a.length();
}