#ifndef MEDIAL_AXIS_TRANSFORM_H
#define MEDIAL_AXIS_TRANSFORM_H

#include "Path.h"
#include "Utils.h"
#include <vector>

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

	// adds new valid path to meidal paths
	void tracePath(Path& path, std::vector<Path>& medialPaths) const;

	// helper functions for tracing paths
	void traceEdgeEdgePath(Path& path) const;
	void traceEdgeVertexPath(Path& path, const int order) const;
	void traceVertexVertexPath(Path& path) const;

	// handle transitions
	void handleTransitions(std::vector<BoundaryElement>& validIntersections);

	// initializes new paths based on previous key point and governors
	void initializeNewPaths(const Path& path, std::vector<Path>& newPathList);

	// member variable
	std::vector<BoundaryElement> boundaryElements;
};

#endif 