#ifndef BOUNDARY_ELEMENT_H
#define BOUNDARY_ELEMENT_H

#include <string>
#include "Edge.h"

struct BoundaryElement: public Vector2d, public Edge
{
	// constructors
	BoundaryElement(): index(-1), transForward(-1), transBack(-1), isGov(false),
                       shouldTransitionForward(false), shouldTransitionBack(false) {}
	BoundaryElement(const Edge& e, const Vector2d& h1, const Vector2d& h2, const int id):
					Edge(e), type("Edge"), halfLine1(h1), halfLine2(h2), 
					index(id), transForward(-1), transBack(-1), isGov(false),
                    shouldTransitionForward(false), shouldTransitionBack(false) {}
	BoundaryElement(const Vector2d& v, const Vector2d& h1, const Vector2d& h2, const int id):
					Vector2d(v), type("Vertex"), halfLine1(h1), halfLine2(h2), 
					index(id), transForward(-1), transBack(-1), isGov(false),
                    shouldTransitionForward(false), shouldTransitionBack(false) {}
         
	// member variables
	std::string type; // vertex or edge
	Vector2d halfLine1;
	Vector2d halfLine2;
	int index;
	int transForward;
	int transBack;
    bool shouldTransitionForward;
    bool shouldTransitionBack;
    bool isGov;
};

#endif 