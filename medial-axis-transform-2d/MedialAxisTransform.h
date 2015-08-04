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
    
    // returns closest intersection point within or on medial ball
    int closestIntersectionPoint(const KeyPoint& kp, const double radius,
                                 const Path& path) const;
    
    // sets new keypoint if candidate is not vaild
    void setNewKeypoint(Path& path, const int traceType) const;
    
	// check path validity
	void checkValidity(Path& path, const int traceType) const;

	// helper functions for tracing paths
	void traceEdgeEdgePath(Path& path);
	void traceEdgeVertexPath(Path& path, const int order);
	void traceVertexVertexPath(Path& path);

	// adds new valid path to meidal paths
	void tracePath(Path& path, std::vector<Path>& medialPaths);
    
    // finds boundary elements intersecting with the medial ball
    void findIntersections(Path& path,
                           std::vector<BoundaryElement>& intersections);

	// handle transitions
	void handleTransitions(std::vector<BoundaryElement>& intersections);

	// initializes new paths based on previous key point and governors
	void initializeNewPaths(Path& path, std::vector<Path>& newPathList);

	// member variables
	std::vector<BoundaryElement> boundaryElements;
};

#endif 