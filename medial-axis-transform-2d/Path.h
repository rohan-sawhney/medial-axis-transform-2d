#ifndef PATH_H
#define PATH_H

#include "BoundaryElement.h"
#include "Parabola.h"

struct Path
{
	// constructors
	Path() {}
	Path(const KeyPoint& kp1, const KeyPoint& kp2, const BoundaryElement& g1, const BoundaryElement& g2):
		 keyPoint1(kp1), keyPoint2(kp2), gov1(g1), gov2(g2) {}
	Path(const KeyPoint& kp1, const BoundaryElement& g1, const BoundaryElement& g2):
		 keyPoint1(kp1), gov1(g1), gov2(g2) {}
	Path(const Path& p):
		 keyPoint1(p.keyPoint1), keyPoint2(p.keyPoint2), gov1(p.gov1), gov2(p.gov2), parabola(p.parabola) {}

	// member variables
	KeyPoint keyPoint1;
	KeyPoint keyPoint2;
	BoundaryElement gov1;
	BoundaryElement gov2;
	Parabola parabola;	
};

#endif