#ifndef UTILS_H
#define UTILS_H

#include <iostream>
#include "Parabola.h"

struct Utils 
{
	static bool inHalfPlane(const Vector2d& v1, const Vector2d& v2) 
	{
		double cross2d = v1.x()*v2.y() - v2.x()*v1.y();

		return cross2d > 0.0;
	}

	// checks whether two edges share a vertex
	static bool shareVertex(const Edge& e1, const Edge& e2) 
	{
		if (e1.vertex2 == e2.vertex1 || e1.vertex1 == e2.vertex2) {
			return true;
		}

		return false;
	}

	// checks if two rays intersect
	static Vector2d lineIntersection(const Vector2d& p1, const Vector2d& l1, 
									 const Vector2d& p2, const Vector2d& l2) 
	{
		double u = (p1.y()*l2.x() + l2.y()*p2.x() - p2.y()*l2.x()  - l2.y()*p1.x()) / 
			   (l1.x()*l2.y() - l2.x()*l1.y());
		double v = (p1.x() + l1.x()*u - p2.x()) / l2.x();

		if (u >= 0) {
			return p1 + l1*u;
		}

		return Vector2d();
	}

	// checks if line intersects with parabola
	static Vector2d parabolaIntersection(const Parabola& parabola, 
										 const Vector2d& p, const Vector2d& l) 
	{
		// TODO: using a brute force method! 
		// Analytic approach: y_parabola = y_line => ax^2 + bx + c = mx + c => solve for x
		Vector2d u = p + l;
		double dx = u.x() - p.x();
		if (dx == 0) {
			return Vector2d(p.x(), parabola.getY(p.x()));
		}

		double m = (u.y() - p.y()) / dx;
		for (double x = p.x(); x < 500; x = x + 0.01) {
			
			double y = m*(x - p.x()) + p.y(); 
			if (std::abs(parabola.getY(x) - y) < 0.01) {
				return Vector2d(x, y);
			}
		}

		return Vector2d();
	}

	// finds closest point on an edge from another point not on the edge
	static Vector2d closestPointOnEdge(const Vector2d& p, const Edge& e)
	{
		double dx = e.vertex2.x() - e.vertex1.x();
		double dy = e.vertex2.y() - e.vertex1.y();
		Vector2d dv = e.vertex2 - e.vertex1;

		double u = ((p.x() - e.vertex1.x()) * dv.x() + 
				    (p.y() - e.vertex1.y()) * dv.y()) / 
					(dv.x()*dv.x() + dv.y()*dv.y());

		if (u <= 0) {
			return e.vertex1;

		} else if (u >= 1) {
			return e.vertex2;
		}

		Vector2d m = e.vertex1 + u*dv;
		return m;
	}

	// returns smallest distance from point to edge 
	static double distToEdge(const Vector2d& p, const Edge& e)
	{
		Vector2d m = closestPointOnEdge(p, e);
		return (p - m).norm();
	}
};

#endif