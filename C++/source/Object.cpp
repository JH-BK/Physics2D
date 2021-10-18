#include "stdafx.h"
#include "Object.h"

// Return type of collision
int Manifold::CollisionType() {
	if (isJoint == 1) return 4; // Joint attached manifold
	else {
		if (A->objectTYPE == 1) {
			if (B->objectTYPE == 1)
				return 0; // AABB vs AABB : 0
			else if (B->objectTYPE == 2)
				return 1; // AABB vs Circle : 1
			else return 3;
		}
		else if (A->objectTYPE == 2) {
			if (B->objectTYPE == 2)
				return 2; // Circle vs Circle : 2
			else if (B->objectTYPE = 1)
				return 1; // Circle vs AABB : 1
			else return 3;
		}
		else return 3;
	}
}
// Processes collision effect
void Manifold::ResolveCollision() {
	Vec2 rv = B->velocity - A->velocity; // get relative velocity
	if (colPoint != Vec2()) {
		rv -= CrossProduct(A->angularVelocity, colPoint - A->position);
		rv += CrossProduct(B->angularVelocity, colPoint - B->position); // apply rotational effect
	}
	double velAlongNormal = rv * normal; // get relative velocity in normal direction

	Vec2 buff = normal * velAlongNormal;
	if (velAlongNormal > 0) { // if distance going further, break
		if (!isJoint)
			return;
	}

	double e;
	if (isJoint) { e = 0; }
	else { e = min(A->restitution, B->restitution); } // get minimum restitution

	Vec2 rA, rB;
	double j = -(1 + e) * velAlongNormal;   // get impulse for normal direction
	if (colPoint != Vec2()) {				// if collision point detected
		rA = colPoint - A->position;
		rB = colPoint - B->position;
		j /= ((A->inv_mass + B->inv_mass) + (CrossProduct(CrossProduct(rA, normal), rA) * A->inv_inertia
			+ CrossProduct(CrossProduct(rB, normal), rB) * B->inv_inertia) * normal);
	}	// apply rotational effect
	else {
		j /= (A->inv_mass + B->inv_mass);
	}


	Vec2 impulse = normal * j; // making impulse vector

	A->impulse -= impulse; // applying
	B->impulse += impulse;

	buff = normal * velAlongNormal;
	Vec2 tangent = rv - buff;				// get tangentional relative velocity

	double jt = tangent.length();
	if (tangent.length() == 0)
		tangent = Vec2();
	else
		tangent = tangent / tangent.length(); // normalize

	if (colPoint != Vec2()) {	// get temporal friction impulse
		jt = jt * (-1) / ((A->inv_mass + A->inv_mass)
			+ (CrossProduct(CrossProduct(rA, tangent), rA) * A->inv_inertia
				+ CrossProduct(CrossProduct(rB, tangent), rB) * B->inv_inertia) * tangent);
	}
	else
		jt = jt * (-1) / ((A->inv_mass + B->inv_mass));

	Vec2 frictionImpulse = Vec2();
	if (-jt < j * friction) // mu
		// Static friction
		frictionImpulse = tangent * jt;
	else
		// Dynamic friction
		frictionImpulse = tangent * j * (friction - 0.1) * (-1); // friction - 0.1 -> dynamic friction constant..


	A->impulse -= frictionImpulse;
	B->impulse += frictionImpulse;
	if (colPoint != Vec2()) {
		// apply rotational effect
		A->torqueImpulse -= CrossProduct(rA, frictionImpulse) * 0.9;
		B->torqueImpulse += CrossProduct(rB, frictionImpulse) * 0.9;
		A->torqueImpulse -= CrossProduct(rA, impulse) * 0.9;
		B->torqueImpulse += CrossProduct(rB, impulse) * 0.9;
	}
}
// Collision detection & Resolving
void Manifold::Collision() {
	int type = CollisionType();
	switch (type) {
	case 0: // AABB AABB
		if (AABBvsAABB(this)) {
			A->Colliding++;
			B->Colliding++;
			ResolveCollision();
			PositionalCorrelation(this);
		}
		else
			normal = Vec2(0, 0);
		break;
	case 1: // AABB Circle
		if (AABBvsCircle(this)) {
			A->Colliding++;
			B->Colliding++;
			ResolveCollision();
			PositionalCorrelation(this);
		}
		else
			normal = Vec2(0, 0);
		break;
	case 2: // Circle Circle
		if (CirclevsCircle(this)) {
			A->Colliding++;
			B->Colliding++;
			ResolveCollision();
			PositionalCorrelation(this);
		}
		else
			normal = Vec2(0, 0);
		break;
	case 4: // Joint manifold
		A->Colliding++;
		B->Colliding++;
		colPoint = (A->position + Mat22(A->orientation) * anchorA + B->position + Mat22(B->orientation) * anchorB)/2;
		normal = normalize(B->velocity + CrossProduct(B->angularVelocity, anchorB) - A->velocity - CrossProduct(A->angularVelocity, anchorA));
		//penetration = (A->position - B->position + Mat22(A->orientation) * anchorA - Mat22(B->orientation) * anchorB).length();
		penetration = 0;
		B->position = A->position + Mat22(A->orientation) * anchorA - Mat22(B->orientation) * anchorB;
		ResolveCollision();
		//PositionalCorrelation(this);
	default:
		this->penetration = 0;
		normal = Vec2();
		break;
	}
}
bool AABBvsAABB(Manifold *m) {
	Object *A = m->A;
	Object *B = m->B;
	Vec2 n = B->position - A->position;
	// if too far, no further calculation
	if ((A->vertices[0] - A->position).length() + (B->vertices[0] - B->position).length() + 10 < n.length() && !m->isJoint) {
		m->colPoint = Vec2();
		return false;
	}
	// Box collision with no orientation (not using now)
	/*if (A->orientation == 0 && B->orientation == 0) {
		AABB abox = A->rect;
		AABB bbox = B->rect;

		double a_extent = (abox.max.X() - abox.min.X()) / 2;
		double b_extent = (bbox.max.X() - bbox.min.X()) / 2;

		double x_overlap = a_extent + b_extent - abs(int(n.X()));

		if (x_overlap > 0) {
			double a_extent = (abox.max.Y() - abox.min.Y()) / 2;
			double b_extent = (bbox.max.Y() - bbox.min.Y()) / 2;
			double y_overlap = a_extent + b_extent - abs(int(n.Y()));

			if (y_overlap > 0) {
				if (x_overlap < y_overlap) {
					if (n.X() > 0)
						m->normal = Vec2(1, 0);
					else
						m->normal = Vec2(-1, 0);
					m->penetration = x_overlap;
					m->colPoint = Vec2();
				}
				else {
					if (n.Y() > 0)
						m->normal = Vec2(0, 1);
					else
						m->normal = Vec2(0, -1);
					m->penetration = y_overlap;
					m->colPoint = Vec2();
				}
			}
			else {
				m->colPoint = Vec2();
				return false;
			}
		}
		else {
			m->colPoint = Vec2();
			return false;
		}
		return true;
	}*/
	// Box collision with rotation
	else {
		// Used separation axis theorem
		UINT32 vertexIndex1, vertexIndex2;
		double penetration1, penetration2;
		penetration1 = FindAxisLeastPenetration(&vertexIndex1, m->A, m->B);
		penetration2 = FindAxisLeastPenetration(&vertexIndex2, m->B, m->A); // get penetration data from every normals
		Mat22 mat = Mat22(0);

		if (penetration1 > 0 || penetration2 > 0) { // if every penetration is positive, no collision
			m->colPoint = Vec2();
			return false;
		}
		else {
			/*UINT32 angleOffset = abs(int((A->orientation - B->orientation) / 3.14 * 180));
			double location = atan2(n.Y(), n.X());*/
			if (penetration1 < penetration2) {
				mat = Mat22(m->B->orientation);
				m->penetration = -penetration2;
				m->normal = mat * m->B->normals[vertexIndex2] * (-1);
				Vec2 temp = A->GetSupport(m->normal) + A->position;
				if (m->colPoint != Vec2() && m->colPoint != temp) {
					m->colPoint = (temp * 6 + m->colPoint) / 7;		// LPF for collision point variance
				}
				else
					m->colPoint = temp;
			}
			else {
				mat = Mat22(m->A->orientation);
				m->penetration = -penetration1;
				m->normal = mat * m->A->normals[vertexIndex1];
				Vec2 temp = B->GetSupport(m->normal) * (-1) + B->position;
				if (m->colPoint != Vec2()) {
					m->colPoint = (temp * 6 + m->colPoint) / 7;		// LPF for collision point variance
				}
				else
					m->colPoint = temp;
			}
		}
		if (!m->isJoint) PenetrationFiltering(m);
		return true;
	}
}
bool CirclevsCircle(Manifold *m) {
	Object *A = m->A;
	Object *B = m->B;
	Vec2 n = B->position - A->position;
	// if too far, no further calculation
	if (A->cir.radius + B->cir.radius < n.length() && !m->isJoint) {
		m->colPoint = Vec2();
		return false;
	}

	Circle acir = A->cir;
	Circle bcir = B->cir;

	// calculating overlapping distance
	double overlap = acir.radius + bcir.radius - 1 - abs(int(n.length()));

	if (overlap > 0) {
		if (n.length() == 0)
			m->normal = Vec2();
		else
			m->normal = n / n.length();
		m->penetration = overlap;
		m->colPoint = (A->position + m->normal * acir.radius + B->position - m->normal * bcir.radius) / 2;
		if (!m->isJoint) PenetrationFiltering(m);
	}
	else {
		m->colPoint = Vec2();
		return false;
	}
	return true;
}
bool AABBvsCircle(Manifold *m) {
	Object *A;
	Object *B;
	if (m->A->rect.max != m->A->rect.min) {
		A = m->A;
		B = m->B;
	}
	else {
		A = m->B;
		B = m->A;
	}
	Vec2 n = B->position - A->position;
	if (((A->rect.max - A->rect.min) / 2).length() + B->cir.radius + 5 < n.length() && !m->isJoint) {
		m->colPoint = Vec2();
		return false;
	}

	float x_extent = A->rect.width;
	float y_extent = A->rect.height;

	Mat22 mat = Mat22(-(A->orientation));
	n = mat * n;
	Vec2 closest = n;

	// closest point clamping
	if (closest.x > x_extent)
		closest.x = x_extent;
	if (closest.x < -x_extent)
		closest.x = -x_extent;
	if (closest.y > y_extent)
		closest.y = y_extent;
	if (closest.y < -y_extent)
		closest.y = -y_extent;

	bool inside = false;
	// Circle is inside the AABB, so we need to clamp the circle's center
	// to the closest edge
	if (n == closest) {
		inside = true;
		// Find closest axis
		if (abs(int(x_extent) - int(n.X())) < abs(int(y_extent) - int(n.Y()))) {
			// Clamp to closest extent
			if (closest.X() > 0)
				closest = Vec2(x_extent, closest.Y());
			else
				closest = Vec2(-x_extent, closest.Y());
		}
		// y axis is shorter
		else {
			// Clamp to closest extent
			if (closest.Y() > 0)
				closest = Vec2(closest.X(), y_extent);
			else
				closest = Vec2(closest.X(), -y_extent);
		}
	}
	Vec2 normal = n - closest;
	double d = pow(normal.length(), 2);
	double r = B->cir.radius;

	if (d > r*r && !inside) {
		m->colPoint = Vec2();
		return false;
	}

	d = normal.length();
	if (normal.length() == 0)
		normal = Vec2();
	else
		normal = normal / normal.length(); // normalization

	if (A->orientation != 0) {
		mat = Mat22(A->orientation);
		normal = mat * normal;
		closest = mat * closest;
	}
	if (m->A->rect.max == m->A->rect.min) {
		normal = normal * (-1);
	}
	if (inside) {
		Vec2 temp = closest * (1.1) / (A->inv_mass + B->inv_mass);
		A->position -= temp * A->inv_mass;
		B->position += temp * B->inv_mass;
		m->normal = normal * (-1);
		m->penetration = r - d;
		m->colPoint = A->position + closest;
	}
	else {
		m->normal = normal;
		m->penetration = r - d;
		m->colPoint = A->position + closest;
	}
	if (!m->isJoint) PenetrationFiltering(m);
	return true;
}
void Manifold::attachJoint(Vec2 anchora, Vec2 anchorb) {
	isJoint = TRUE;
	anchorA = anchora;
	anchorB = anchorb;
}
void PositionalCorrelation(Manifold *m) {
	const double percent = 0.2;
	const double slop = 1;
	Vec2 correlation = m->normal * max(m->penetration - slop, 0.0f) / (m->A->inv_mass + m->B->inv_mass) * percent;
	/*if (m->A->objectTYPE == 1 && m->B->objectTYPE == 1) {
		if (m->penetration > 5) m->penetration = 5;
		//correlation *= pow(m->penetration - slop, 1);
	}*/
	m->A->position -= correlation * m->A->inv_mass;
	m->B->position += correlation * m->B->inv_mass;
	m->A->impulse -= correlation * 1000 / 5;
	m->B->impulse += correlation * 1000 / 5;
}
void ArtificialGrav(Object *A, double gravity, int x, int y) {
	Vec2 center = Vec2(x, y);
	center = center - A->position;
	double r = (center.length()*center.length()*center.length());
	if (r < 1000)
		r = 1000;
	center = center * 10 * gravity / r;
	center = center * 5 / 1000;
	A->impulse += center * A->mass;
}
void VerticalGrav(Object *A, double gravity) {
	A->impulse += (Vec2(0, 1) * gravity * 5 / 1000) * A->mass;
}
void VerticalGrav(Manifold *man, double gravity) {
	Object* A = man->A;
	Object* B = man->B;
	A->impulse += (Vec2(0, 1) * gravity * 5 / 1000) * A->mass;
	B->impulse += (Vec2(0, 1) * gravity * 5 / 1000) * B->mass;
}
double FindAxisLeastPenetration(UINT32* faceIndex, Object* A, Object* B) {
	double bestDistance = -FLT_MAX;
	UINT32 bestIndex = 0;

	for (UINT32 i = 0; i < A->verticeNum; i++) {
		Mat22 mat = Mat22(A->orientation);
		Vec2 n = mat * A->normals[i];
		Vec2 s = B->GetSupport(n * (-1)) + B->position;
		Vec2 v = mat * A->vertices[i] + A->position;

		double d = n * (s - v);

		if (d > bestDistance) {
			bestDistance = d;
			bestIndex = i;
		}
	}
	*faceIndex = bestIndex;
	return bestDistance;
}
void Joint::ResolveJoint() {
	Mat22 matA = Mat22(A->orientation);
	Mat22 matB = Mat22(B->orientation);
	Vec2 ancA = matA * anchorA;
	Vec2 ancB = matB * anchorB;

	Vec2 rp = (B->position + ancB) - (A->position + ancA);
	rp = rp / (B->inv_mass + A->inv_mass);
	Vec2 rv = (B->velocity - A->velocity);
	rv = rv / (B->inv_mass + A->inv_mass);

	B->impulse -= (rp + rv) * B->inv_mass * B->mass;
	B->torqueImpulse += CrossProduct((rp + rv), anchorB);
	A->impulse += (rp + rv) * A->inv_mass * A->mass;
	A->torqueImpulse -= CrossProduct((rp + rv), anchorA);
}

void PenetrationFiltering(Manifold *m) {
	if (m->penetration < 0) {
		m->penetration = 0;
		m->colPoint = Vec2();
	}
	else {
		m->A->Colliding++;
		m->B->Colliding++;
	}
}