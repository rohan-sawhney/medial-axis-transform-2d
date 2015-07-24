#ifndef PARABOLA_H
#define PARABOLA_H

#include <math.h>
#include <utility>
#include "KeyPoint.h"
#include "Edge.h"
#include <iostream>

struct Parabola 
{
	// constructors
    Parabola(): set(0) {}
	Parabola(const Vector2d& focus, const Edge& directrix)
	{
		h = focus.x();
		k = focus.y();
        
        a = directrix.vertex2.y() - directrix.vertex1.y();
        b = -(directrix.vertex2.x() - directrix.vertex1.x());
        c = directrix.vertex2.x()*directrix.vertex1.y() - directrix.vertex1.x()*directrix.vertex2.y();
        
        set = 1;
	}
	Parabola(const Parabola& p): a(p.a), b(p.b), c(p.c), h(p.h), k(p.k), set(p.set) {}

	// returns the y value of a point on the parabola given x
    std::pair<double, double> getY(const double& x) const
	{
		// (ax + by + c)^2 / (a^2 + b^2) = (x-h)^2 + (y-k)^2
		// solve for y in terms of x - use wolfram!

        double y1 = 0;
        double y2 = 0;
        double a2 = a*a;
        double b2 = b*b;
        double ah = a*h;
        double bk = b*k;
        double bh = b*h;
        
        if (a == 0) {
            y1 = (bh*bh - 2*b2*h*x + b2*k*k + b2*x*x - c*c) / (2*b*(bk + c));
            y2 = y1;
            
        } else {
            
            double exp1 = a2*k + a*b*x + b2*k + b*c;
            double exp2 = a2 + b2;
            double exp3 = ah + bk + c;
            double exp4 = -ah + 2*a*x + bk + c;
            
            y1 = (exp1 - sqrt(exp2*exp3*exp4)) / a2;
            y2 = (exp1 + sqrt(exp2*exp3*exp4)) / a2;
        }
        
        std::pair<double, double> ys;
        ys.first = y1;
        ys.second = y2;
        
		return ys;
	}

	// member variables
	double a;
	double b;
	double c;
	double h;
	double k;
	int set;
};

#endif