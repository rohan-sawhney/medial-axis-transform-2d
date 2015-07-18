#ifndef BOUNDARY_GENERATOR_H
#define BOUNDARY_GENERATOR_H

#include <vector>
#include "BoundaryElement.h"

class BoundaryGenerator 
{
public:
	// returns the boundaryElements vector
	std::vector<BoundaryElement> getBoundaryElements(unsigned char shape);

private:
	// populates the boundaryElements vector with edges and concave vertices
	void generateShape(const Vector2d *shape, int vertexCount);

	// member variable
	std::vector<BoundaryElement> boundaryElements;
};

#endif