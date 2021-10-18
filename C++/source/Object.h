#pragma once
#include "Vec2.h"
#include <float.h>

// Axis Aligned Bounding Box property
struct AABB {
	Vec2 min;			// box's vertex
	Vec2 max;			// another vertex (opposite side)
	int width;			// half width
	int height;			// half height
	AABB() {
		min = Vec2(0, 0);
		max = Vec2(0, 0);
		width = 0;
		height = 0;
	}
};

// Circle property
struct Circle {
	double radius;
	//Vec2 position; 
	Circle() {
		radius = 0;
	}
};

struct Object {
	struct AABB rect;	// Object contains every polygon properties (for now)
	struct Circle cir;
	UINT objectTYPE;	// 0: NULL 1: AABB 2: Circle
	double restitution; // Collision restitution (0~1)
	double mass;
	double inv_mass;	// Inverse mass
	// Linear
	Vec2 impulse;		// Impulse applied from every collision point
	Vec2 acc;			// acceleration
	Vec2 velocity;
	Vec2 position;
	// Angular
	double inertia;			// moment of inertia
	double inv_inertia;
	float orientation;		// radian
	float angularVelocity;
	float torqueImpulse;	// Rotational impulse applied from every collision point
	// Polygon Properties
	Vec2* normals;			// Normal vector from every line
	Vec2* vertices;			// Polygon vertices
	int verticeNum;			// number of vertices
	// Impulse
	bool Colliding;
	double kineticEnergy;
	bool Sleep;

	Object(Vec2 min, Vec2 max, double res, double ms, Vec2 vel) {
		rect.max = max;
		rect.min = min;
		objectTYPE = 1;
		rect.width = int((rect.max.X() - rect.min.X()) / 2);
		rect.height = int((rect.max.Y() - rect.min.Y()) / 2);
		cir.radius = 0;
		//cir.position = Vec2(0, 0);
		restitution = res;
		mass = ms;
		if (mass == 0) inv_mass = 0;
		else
			inv_mass = 1 / mass;

		inertia = (pow(rect.width * 2, 2) + pow(rect.height * 2, 2)) * ms / 12;
		if (inertia == 0) inv_inertia = 0;
		else
			inv_inertia = 1 / inertia;
		impulse = Vec2();
		acc = Vec2();
		velocity = vel;
		position = (max + min) / 2;
		orientation = 0;
		angularVelocity = 0;
		torqueImpulse = 0;

		Colliding = 0;
		kineticEnergy = 1000;
		Sleep = 0;

		verticeNum = 4;
		normals = new Vec2[verticeNum];
		vertices = new Vec2[verticeNum];
		normals[0] = Vec2(1, 0);
		normals[1] = Vec2(0, 1);
		normals[2] = Vec2(-1, 0);
		normals[3] = Vec2(0, -1);
		vertices[0] = Vec2(rect.width, -rect.height);
		vertices[1] = Vec2(rect.width, rect.height);
		vertices[2] = Vec2(-rect.width, rect.height);
		vertices[3] = Vec2(-rect.width, -rect.height);
	}
	Object(double radius, Vec2 pos, double res, double ms, Vec2 vel) {
		rect.max = Vec2();
		rect.min = Vec2();
		cir.radius = radius;
		objectTYPE = 2;
		//cir.position = pos;
		restitution = res;
		mass = ms;
		if (mass == 0) inv_mass = 0;
		else
			inv_mass = 1 / mass;
		inertia = 0.5 * ms * pow(radius, 2);
		if (inertia == 0) inv_inertia = 0;
		else
			inv_inertia = 1 / inertia;
		impulse = Vec2();
		acc = Vec2();
		velocity = vel;
		position = pos;
		// Angular
		orientation = 0;
		angularVelocity = 0;
		torqueImpulse = 0;

		Colliding = 0;
		kineticEnergy = 1000;
		Sleep = 0;

		normals = NULL;
		vertices = NULL;
		verticeNum = 0;
	}
	Object() {
		objectTYPE = 0;
		restitution = 0;
		mass = 0;
		inv_mass = 0;
		inertia = 0;
		inv_inertia = 0;
		impulse = Vec2();
		acc = Vec2();
		velocity = Vec2();
		position = Vec2();
		// Angular
		orientation = 0;
		angularVelocity = 0;
		torqueImpulse = 0;

		Colliding = 0;
		kineticEnergy = 1000;
		Sleep = 0;

		normals = NULL;
		vertices = NULL;
		verticeNum = 0;
	}
	Vec2 GetSupport(Vec2& dir) {			// Get support point from polygon for given direction
		double bestProjection = -FLT_MAX;
		Vec2 bestVertex;
		Mat22 mat = Mat22(orientation);		// rotation conversion matrix

		for (UINT32 i = 0; i < verticeNum; i++) {
			Vec2 v = mat * vertices[i];
			double projection = v * dir;	// projecting to given direction
			if (projection > bestProjection) { // get greatest projection
				bestVertex = v;
				bestProjection = projection;
			}
		}
		return bestVertex;
	}
};

class Manifold {			// Collision status, properties
public:
	Object* A;				// Object 1
	Object* B;				// Object 2
	double penetration;		// penetration when colliding
	Vec2 normal;			// Collision normal
	double friction;		// friction between two object
	Vec2 colPoint;			// Collision point
	//double temp;			// not using now
	bool isJoint;
	Vec2 anchorA, anchorB;

public:
	Manifold() {
		A = NULL;
		B = NULL;
		penetration = 0;
		normal = Vec2();
		friction = 0;
		colPoint = Vec2();
		isJoint = FALSE;
		anchorA = Vec2();
		anchorB = Vec2();
	}
	Manifold(Object* a, Object* b) {
		A = a;
		B = b;
		penetration = 0;
		normal = Vec2();
		friction = 0.8; // Object간 마찰 함수 필요
		colPoint = Vec2();
		isJoint = FALSE;
	}
	int CollisionType();
	void ResolveCollision();
	void Collision();
	void attachJoint(Vec2 anchora, Vec2 anchorb);

	friend bool AABBvsAABB(Manifold *m);		// for easy coding....
	friend bool CirclevsCircle(Manifold *m);	// should be modified
	friend bool AABBvsAABB(Manifold *m);
};

struct Joint {
	Object* A;
	Object* B;
	Vec2 anchorA, anchorB;

	Joint(Object* a, Object* b, Vec2 ancA, Vec2 ancB) {
		A = a;
		B = b;
		anchorA = ancA;
		anchorB = ancB;
	}

	Joint() {
		A = NULL;
		B = NULL;
		anchorA = Vec2();
		anchorB = Vec2();
	}

	void ResolveJoint();
};

bool AABBvsAABB(Manifold *m);									// Box vs Box collision processing
bool CirclevsCircle(Manifold *m);								// Circle vs Circle collision processing
bool AABBvsCircle(Manifold *m);									// Box vs Circle collision processing
void PositionalCorrelation(Manifold *m);						// Large penetration resolving
void ArtificialGrav(Object *A, double gravity, int x, int y);	// Artificial point gravity
void VerticalGrav(Object *A, double gravity);					// Vertical gravity (object version)
void VerticalGrav(Manifold *man, double gravity);				// Vertical gravity (manifold version)
//void ArtificialOrbitalGrav(Object *A, double solarmass, double gravityConst, int x, int y, int tick);
double FindAxisLeastPenetration(UINT32* faceIndex, Object* A, Object* B);	// Finding axis with least penetration
void PenetrationFiltering(Manifold *m);