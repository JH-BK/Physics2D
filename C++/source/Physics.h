#pragma once
#include "resource.h"
#include "Object.h"

// Window variables
HDC hdc;
HDC hMemDC;
HWND hWndMain;
HBITMAP OldBitmap;
HBITMAP hBitmap;

// Window size
static int left = 50;
static int top = 80;                                                                       
static int right = 1150;
static int bottom = 600;

// Object & Manifold initialization
static int objnum = 9;
static int oldobjnum = objnum;
static int index = objnum - 1;
static const int maxobj = 120;
static Object cir1 = Object(20, Vec2(600, 420), 0.8, 0, Vec2());
static Object cir2 = Object(20, Vec2(650, 421), 0.8, 0, Vec2());
static Object cir3 = Object(20, Vec2(700, 422), 0.8, 0, Vec2());
static Object rect1 = Object(Vec2(250, 440), Vec2(510, 450), 0.4, 0, Vec2());
static Object rect2 = Object(Vec2(left, bottom - 10), Vec2(right, bottom), 0.4, 0, Vec2());
static Object rect3 = Object(Vec2(400, 200), Vec2(450, 250), 0.4, 80, Vec2(0, -100));
static Object rect4 = Object(Vec2(left, top + 1), Vec2(left + 10, bottom - 10), 0.4, 0, Vec2());
static Object rect5 = Object(Vec2(right - 10, top + 1), Vec2(right, bottom - 10), 0.4, 0, Vec2());
static Object cir4 = Object(30, Vec2(500, 600), 0.8, 10, Vec2(300, 0));
 
static Object* objList[maxobj] = { &rect1, &rect2, &rect3, &rect4, &rect5, &cir1, &cir2, &cir3, &cir4 };
int mannum = objnum * (objnum - 1) / 2;
int oldmannum = 0;
Manifold* manifoldList;

// System variables
static int nTime = 0;
static char szTime[128];
static char szBuf[128];

// Time control
static int fps = 200;
static int tick = 1000 / fps;
static bool toggle = 1;

// Physical properties
static const double gravity = 9.81 * 100;
static const double restitution = 0.4;
static const double friction = 0.9;
static const double ballfriction = 0.9;
static const double squarefriction = 0.9;

void makeManifold(Object** objList, Manifold* manifoldList) {
	int count = 0;
	for (int i = 0; i < objnum; i++) {
		for (int j = i; j < objnum; j++) {
			if (i != j) {
				manifoldList[count] = Manifold(objList[i], objList[j]);
				count++;
			}
		}
	}
}
void manifoldCollision(Manifold* List) {
	for (int i = 0; i < mannum; i++) {
		//VerticalGrav(&List[i], gravity);
		/*if (List[i].A == &rect3 && List[i].B == &cir4 && List[i].isJoint == 0) List[i].attachJoint(Vec2(50, 0), Vec2(0, 50)); // Temporary Code /////////////////////////// */
		List[i].Collision();
	}
}
// not using now... easy wall collision (early version)
void wallref(Object * obj, int left, int top, int right, int bottom) {

	if (obj->rect.max != obj->rect.min) {
		int A1 = int((obj->rect.max.X() - obj->rect.min.X()) / 2);
		int A2 = int((obj->rect.max.Y() - obj->rect.min.Y()) / 2);

		if (obj->position.Y() + A2 > bottom) {
			obj->position.y = bottom - A2;
			obj->impulse += Vec2(0, -obj->velocity.y * (1 + restitution)) * obj->mass;
			obj->impulse += Vec2(-obj->velocity.x * (1 - squarefriction), 0) * obj->mass;
		}

		if (obj->position.Y() - A2 < top) {
			obj->position.y = top + A2;
			obj->impulse += Vec2(0, -obj->velocity.y * (1 + restitution)) * obj->mass;
			obj->impulse += Vec2(-obj->velocity.x * (1 - squarefriction), 0) * obj->mass;
		}

		if (obj->position.X() + A1 > right) {
			obj->position.x = right - A1;
			obj->impulse += Vec2(-obj->velocity.x * (1 + restitution), 0) * obj->mass;
			obj->impulse += Vec2(0, -obj->velocity.y * (1 - squarefriction)) * obj->mass;
		}

		if (obj->position.X() - A1 < left) {
			obj->position.x = left + A1;
			obj->impulse += Vec2(-obj->velocity.x * (1 + restitution), 0) * obj->mass;
			obj->impulse += Vec2(0, -obj->velocity.y * (1 - squarefriction)) * obj->mass;
		}
	}
	else {
		int A1 = int(obj->cir.radius);
		int A2 = A1;
		double vel = 0;
		double impulse = 0;
		if (obj->position.Y() + A2 > bottom) {
			obj->position.y = bottom - A2;
			obj->velocity.y = -obj->velocity.y * restitution;
			vel = obj->velocity.x - (obj->angularVelocity * obj->cir.radius);
			impulse = vel * (1 - ballfriction) * obj->mass;
			obj->velocity.x -= impulse / obj->mass;
			obj->torqueImpulse += obj->cir.radius * impulse;
		}

		if (obj->position.Y() - A2 < top) {
			obj->position.y = top + A2;
			obj->velocity.y = -obj->velocity.y * restitution;
			vel = obj->velocity.x + (obj->angularVelocity * obj->cir.radius);
			impulse = vel * (1 - ballfriction) * obj->mass;
			obj->velocity.x -= impulse / obj->mass;
			obj->torqueImpulse -= obj->cir.radius * impulse;
		}
		if (obj->position.X() + A1 > right) {
			obj->position.x = right - A1;
			obj->velocity.x = -obj->velocity.x * restitution;
			vel = obj->velocity.y + (obj->angularVelocity * obj->cir.radius);
			impulse = vel * (1 - ballfriction) * obj->mass;
			obj->velocity.y -= impulse / obj->mass;
			obj->torqueImpulse -= obj->cir.radius * impulse;
		}

		if (obj->position.X() - A1 < left) {
			obj->position.x = left + A1;
			obj->velocity.x = -obj->velocity.x * restitution;
			vel = obj->velocity.y - (obj->angularVelocity * obj->cir.radius);
			impulse = vel * (1 - ballfriction) * obj->mass;
			obj->velocity.y -= impulse / obj->mass;
			obj->torqueImpulse += obj->cir.radius * impulse;
		}
	}
};

void makeShape(HDC hdc, Object* obj) {
	if (obj->rect.max == obj->rect.min) {
		Ellipse(hdc, int(obj->position.x - obj->cir.radius), int(obj->position.y - obj->cir.radius), int(obj->position.x + obj->cir.radius), int(obj->position.y + obj->cir.radius));
		Vec2 line = Vec2(obj->position);
		MoveToEx(hdc, line.X(), line.Y(), NULL);
		line = line + Vec2(obj->cir.radius * sin(obj->orientation), obj->cir.radius * cos(obj->orientation) * (-1));
		LineTo(hdc, line.X(), line.Y());
	}
	else if (obj->cir.radius == 0)
		if (obj->orientation == 0)
			Rectangle(hdc, int(obj->position.x - obj->rect.width), int(obj->position.y - obj->rect.height), int(obj->position.x + obj->rect.width), int(obj->position.y + obj->rect.height));
		else {
			Mat22 mat = Mat22(obj->orientation);
			Vec2 points[4];
			Vec2 center = obj->position;
			double x = (obj->rect.max.X() - obj->rect.min.X());
			double y = (obj->rect.max.Y() - obj->rect.min.Y());
			points[0] = mat * (obj->rect.max - obj->rect.min) / 2;
			points[3] = points[0] * (-1);
			points[1] = points[3] + mat * Vec2(x, 0);
			points[2] = points[3] + mat * Vec2(0, y);
			for (int i = 0; i < 4; i++) points[i] += center;
			MoveToEx(hdc, points[0].X(), points[0].Y(), NULL);
			UINT traceNo[4] = { 2, 4, 3, 1 };
			for (int i = 0; i < 4; i++) LineTo(hdc, points[traceNo[i]-1].X(), points[traceNo[i] - 1].Y());;
		}
};

void positionUpdate(Object* obj) {
	obj->velocity += obj->impulse * obj->inv_mass;
	obj->position += obj->velocity * 5 / 1000;
	obj->impulse = Vec2();
};

void angleUpdate(Object* obj) {
	obj->angularVelocity += obj->torqueImpulse * obj->inv_inertia;
	obj->orientation += obj->angularVelocity * 5 / 1000;
	obj->torqueImpulse = 0;
};

void applyImpulse(Object* obj) {
	obj->velocity += obj->impulse * obj->inv_mass;
	if (obj->velocity.length() > 10000)
		obj->velocity = obj->velocity / obj->velocity.length() * 10000;

	obj->angularVelocity += obj->torqueImpulse * obj->inv_inertia;
	if (obj->angularVelocity > 30)
		obj->angularVelocity = 30;
	else if (obj->angularVelocity < -30)
		obj->angularVelocity = -30;

	double temp = pow(obj->velocity.length(), 2) + pow(obj->angularVelocity, 2);
	obj->kineticEnergy = obj->kineticEnergy * 0.95 + temp * 0.05;
	if (obj->kineticEnergy < 1000 / 5 * 0.25) {
		if (obj->Colliding != 0) {
			if (obj->mass != 0) {
				obj->velocity = Vec2();
				obj->angularVelocity = 0;
				obj->Sleep = 1;
			}
			else obj->Sleep = 1;
		}
	}
	else
		obj->Sleep = 0;

	obj->position += obj->velocity * 5 / 1000;
	obj->orientation += obj->angularVelocity * 5 / 1000;
	obj->Colliding = 0;
	obj->impulse = Vec2();
	obj->torqueImpulse = 0;
};

void ArtificialGravityON(UINT gravity, int posX, int posY) {
	for (int i = 0; i < objnum; i++) {
		ArtificialGrav(objList[i], gravity, posX, posY);
	}
}

void ArtificialGravityOnMouse(UINT gravity, HWND hWnd) {
	for (int i = 0; i < objnum; i++) {
		POINT pt;
		GetCursorPos(&pt);
		ScreenToClient(hWnd, &pt);
		ArtificialGrav(objList[i], gravity, pt.x, pt.y);
	}
}

void VerticalGravityON(UINT gravity) {
	for (int i = 0; i < objnum; i++) {
		VerticalGrav(objList[i], gravity);
	}
}