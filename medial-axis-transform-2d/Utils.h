#ifndef UTILS_H
#define UTILS_H

#include <iostream>
#include "Parabola.h"

struct Utils 
{
    static double cross2d(const Vector2d& v1, const Vector2d& v2)
    {
        return v1.x()*v2.y() - v2.x()*v1.y();
    }
    
	static bool inHalfPlane(const Vector2d& v1, const Vector2d& v2) 
	{
		return cross2d(v1, v2) > 0.0;
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

		if (u >= 0 || v >= 0) {
			return p1 + l1*u;
		}

		return Vector2d::Zero();
	}
    
	// checks if line intersects with parabola
	static Vector2d parabolaIntersection(const Parabola& parabola,
										 const Vector2d& p, const Vector2d& l)
	{
		// FIX: brute force method not accurate!
		// Analytic approach: y_parabola = y_line => ax^2 + bx + c = mx + c => solve for x
		Vector2d u = p + l;
		double dx = u.x() - p.x();
		if (dx == 0) {
            std::pair<double, double> ys = parabola.getY(p.x());
            return Vector2d(p.x(), ys.first);
        }

		double m = (u.y() - p.y()) / dx;
		for (double x = 100; x < 500; x = x + 0.0001) {
			
			double y = m*(x - p.x()) + p.y();
            std::pair<double, double> ys = parabola.getY(x);
            
            if (std::abs(ys.first - y) < 0.001 || std::abs(ys.second - y) < 0.001) {
				return Vector2d(x, y);
			}
		}

		return Vector2d::Zero();
	}
    
	// finds closest point on an edge from another point not on the edge
	static Vector2d closestPointOnEdge(const Vector2d& p, const Edge& e)
	{
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
    
    static double findCircle(const Edge& e1, const Edge& e2, const Edge& e3, Vector2d& c)
    {
        Vector2d tangent1 = e1.tangent();
        Vector2d tangent2 = e2.tangent();
        Vector2d tangent3 = e3.tangent();
        
        Vector2d int1 = lineIntersection(e1.vertex2, tangent1, e2.vertex1, -tangent2);
        Vector2d bisector1 = (-tangent1 + tangent2) / 2;
        bisector1.normalize();
        
        Vector2d int2 = lineIntersection(e2.vertex2, tangent2, e3.vertex1, -tangent3);
        Vector2d bisector2 = (-tangent2 + tangent3) / 2;
        bisector2.normalize();
        
        c = lineIntersection(int1, bisector1, int2, bisector2);
        
        return distToEdge(c, e1);
    }
    
    // returns center of circle given edge, edge, point
    static double findCircle(const Edge& e1, const Edge& e2, const Vector2d& p, Vector2d& c)
    {
        // FIX: brute force approach!
        Vector2d tangent1 = e1.tangent();
        Vector2d tangent2 = e2.tangent();
        
        Vector2d intersection = lineIntersection(e1.vertex2, tangent1, e2.vertex1, -tangent2);
        Vector2d bisector1 = (-tangent1 + tangent2) / 2;
        bisector1.normalize();
    
        double r1 = rand();
        double r2 = rand();
        double r3 = rand();
        
        double t = 0;
        while (!(fabs(r1 - r2) < 0.001 && fabs(r2 - r3) < 0.001)) {
            c = intersection + t*bisector1;
            r1 = distToEdge(c, e1);
            r2 = distToEdge(c, e2);
            r3 = (c - p).norm();
            
            t += 0.001;
        }
        
        return r1;
    }
    
    // returns center of circle given edge, point, point
    static double findCircle(const Edge& e, const Vector2d& p1, const Vector2d& p2, Vector2d& c)
    {
        // FIX: brute force approach!
        Vector2d mid = (p1 + p2) / 2;
        Edge ep(p1, p2);
        Vector2d n = ep.normal();
        
        double r1 = rand();
        double r2 = rand();
        double r3 = rand();
        
        double t = 0;
        while (!(fabs(r1 - r2) < 0.001 && fabs(r2 - r3) < 0.001)) {
            c = mid + t*n;
            r1 = distToEdge(c, e);
            r2 = (c - p1).norm();
            r3 = (c - p2).norm();
            
            t += 0.001;
        }
        
        return r1;
    }

    // computes equation of a circle given 3 points on the circle
    // assumes points are not collinear
    static double findCircle(const Vector2d& p1, const Vector2d& p2, const Vector2d& p3, Vector2d& c)
    {
        double dy1 = p2.y() - p1.y();
        double dx1 = p2.x() - p1.x();
        double dy2 = p3.y() - p2.y();
        double dx2 = p3.x() - p2.x();
        
        double m1 = dy1 / dx1;
        double m2 = dy2 / dx2;
        
        Vector2d mid1 = (p1 + p2) / 2;
        Vector2d mid2 = (p3 + p2) / 2;
  
        if (dy1 == 0) { // m1 = 0
            c.x() = mid1.x();
            if (dx2 == 0) { // m2 = inf
                c.y() = mid2.y();
                
            } else {
                c.y() = mid2.y() + (mid2.x() - c.x()) / m2;
            }
            
        } else if(dy2 == 0) { // m2 = 0
            c.x() = mid2.x();
            if (dx1 == 0) { // m1 = inf
                c.y() = mid1.y();
                
            } else {
                c.y() = mid1.y() + (mid1.x() - c.x()) / m1;
            }
            
        } else if (dx1 == 0) { // m1 = inf
            c.y() = mid1.y();
            c.x() = m2*(mid2.y() - c.y()) + mid2.x();
            
        } else if (dx2 == 0) { // m2 = inf
            c.y() = mid2.y();
            c.x() = m1*(mid1.y() - c.y()) + mid1.x();
            
        } else {
            c.x() = (m1*m2*(mid1.y() - mid2.y()) - m1*mid2.x() + m2*mid1.x()) / (m2 - m1);
            c.y() = mid1.y() - (c.x() - mid1.x()) / m1;
        }
        
        return (c - p1).norm();
    }
};

#endif