#ifndef PARABOLA_H
#define PARABOLA_H

#include <math.h>
#include "KeyPoint.h"
#include "Edge.h"

struct Parabola 
{
	// constructors
	Parabola() {}
	Parabola(const Vector2d& focus, const Edge& directrix)
	{
		h = focus.x();
		k = focus.y();

		double m = directrix.slope();
		a = m;
		b = -1;
		c = directrix.vertex1.y() - m * directrix.vertex1.x();

		set = true;
	}
	Parabola(const Parabola& p): a(p.a), b(p.b), c(p.c), h(p.h), k(p.k), set(p.set) {}

	// assignment operator
	Parabola& operator=(const Parabola& p)
	{
		a = p.a;
		b = p.b;
		c = p.c;
		h = p.h;
		k = p.k;
		set = p.set;

		return *this;
	}

	// returns the y value of a point on the parabola given x
	double getY(const double& x) const
	{
		// (ax + by + c)^2 / (a^2 + b^2) = (x-h)^2 + (y-k)^2
		// solve for y in terms of x - use wolfram!

		double a2 = a*a;
		double b2 = b*b;
		double ah = a*h;
		double bk = b*k;
		double exp1 = a2*k + a*b*x + b2*k + b*c;
		double exp2 = a2 + b2;
		double exp3 = ah + bk + c;
		double exp4 = -ah + 2*a*x + bk + c;

		double y = (exp1 - sqrt(exp2*exp3*exp4)) / a2;

		return y;
	}

	// member variables
	double a;
	double b;
	double c;
	double h;
	double k;
	bool set;
};

#endif