#ifndef MEDIAL_AXIS_TRANSFORM_H
#define MEDIAL_AXIS_TRANSFORM_H

#include <vector>
#include "Path.h"
#include "Utils.h"

class MedialAxisTransform 
{
public:
	// sets the boundary elements for medial axis transform computation
	void setBoundaryElements(const std::vector<BoundaryElement>& be);

	// runs the medial axis transform algorithm
	std::vector<Path> run();

private:
	// initializes the first path by finding the first convex vertex in the boundary
	void initializeFirstPath(Path& firstPath) const;
    
    // finds boundary elements intersecting with the medial ball
    void findIntersections(const Path& path);

	// check path validity
	void checkValidity(Path& path);

	// helper functions for tracing paths
	void traceEdgeEdgePath(Path& path);
	void traceEdgeVertexPath(Path& path, const int order);
	void traceVertexVertexPath(Path& path);

	// adds new valid path to meidal paths
	void tracePath(Path& path, std::vector<Path>& medialPaths);

	// handle transitions
	void handleTransitions();

	// initializes new paths based on previous key point and governors
	void initializeNewPaths(const Path& path, std::vector<Path>& newPathList);

	// member variable
	std::vector<BoundaryElement> boundaryElements;
    std::vector<BoundaryElement> validIntersections;
};

#endif 