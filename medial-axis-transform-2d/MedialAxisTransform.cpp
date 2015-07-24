#include <stack>
#include <utility>
#include "MedialAxisTransform.h"

#define EPSILON 1e-6

// remove edges sharing a concave vertex. If a vertex is a governor, then the edges proceed
// or precede the vertex in intersectionElements depending on which of the two governors the
// vertex is. Otherwise, the vertex lies in between the edges
void removeConcaveEdges(std::vector<std::pair<BoundaryElement, bool> >& intersectionElements)
{
    size_t size = intersectionElements.size();

    // check if gov1 is a concave vertex
    if (intersectionElements[0].first.type == "Vertex") {
        // the first element could be edge 1 or edge 2
        if (intersectionElements[1].first.vertex2 == intersectionElements[0].first ||
            intersectionElements[1].first.vertex1 == intersectionElements[0].first) {
            intersectionElements[1].second = false;
        }
        
        // check for edge 2
        if (intersectionElements[2].first.vertex1 == intersectionElements[0].first) {
            intersectionElements[2].second = false;
        }
    }
    
    // check if gov2 is a concave vertex
    if (intersectionElements[size-1].first.type == "Vertex") {
        // the second last element could be edge 1 or edge 2
        if (intersectionElements[size-2].first.vertex2 == intersectionElements[size-1].first ||
            intersectionElements[size-2].first.vertex1 == intersectionElements[size-1].first) {
            intersectionElements[size-2].second = false;
        }
        
        // check for edge 1
        if (intersectionElements[size-3].first.vertex2 == intersectionElements[size-1].first) {
            intersectionElements[size-3].second = false;
        }
    }
    
    // check if any other intersection element is a concave vertex
    for (size_t i = 1; i < size-1; i++) {
        if (intersectionElements[i].first.type == "Vertex") {
            // check for edge 1
            if (intersectionElements[i-1].first.vertex2 == intersectionElements[i].first) {
                intersectionElements[i-1].second = false;
            }
            
            // check for edge 2
            if (intersectionElements[i+1].first.vertex1 == intersectionElements[i].first) {
                intersectionElements[i+1].second = false;
            }
        }
    }
}

void MedialAxisTransform::setBoundaryElements(const std::vector<BoundaryElement>& be)
{
	boundaryElements = be;
}

void MedialAxisTransform::initializeFirstPath(Path& firstPath) const 
{
	size_t size = boundaryElements.size();

	for (int i = 0; i < size; i++) {
		int next = (i+1) % size;
		// find first convex vertex to initialize as key point 
		if (boundaryElements[i].type == "Edge" && 
			boundaryElements[next].type == "Edge" &&
			!Utils::inHalfPlane(boundaryElements[i].tangent(), boundaryElements[next].tangent())) {
				
			firstPath.keyPoint1 = KeyPoint(boundaryElements[i].vertex2, 0);
			firstPath.gov1 = boundaryElements[i];
			firstPath.gov2 = boundaryElements[next];

			break;
		}
	}
}

void MedialAxisTransform::checkValidity(Path& path, const int traceType) const
{
    // check if there is any boundary element inside the medial ball (has to be concave vertex)
    bool isValid = true;
    for (size_t i = 0; i < boundaryElements.size(); i++) {
        if (boundaryElements[i].type == "Vertex" &&
            boundaryElements[i].index != path.gov1.index &&
            boundaryElements[i].index != path.gov2.index) {
            
            if (!(path.gov1.type == "Edge" && path.gov1.vertex1 == boundaryElements[i]) &&
                !(path.gov2.type == "Edge" && path.gov2.vertex2 == boundaryElements[i])) {
                
                double d = (path.keyPoint2 - boundaryElements[i]).norm();
                if (fabs(d - path.keyPoint2.radius) > 0.01 && d < path.keyPoint2.radius) {
                    isValid = false;
                }
            }
        }
    }
    
    // if there is, determine new center and radius for medial ball
    if (!isValid) {
        // find the closest boundary element to the first keypoint
        int index = -1;
        double minD = 999;
        
        size_t size = boundaryElements.size();
        int startIndex = path.gov2.index+1;
        if (startIndex == size) startIndex = 0;
        
        int endIndex = path.gov1.index;
        if (endIndex < path.gov2.index) endIndex += size;
        
        for (size_t i = startIndex; i < endIndex; i++) {
            size_t j = i;
            if (j >= size) j -= size;
        
            if (boundaryElements[j].type == "Vertex") {
                double d = (path.keyPoint1 - boundaryElements[j]).norm();
                if (fabs(d - minD) > 0.01 && d < minD) {
                    minD = d;
                    index = boundaryElements[j].index;
                }
            }
        }
        
        double radius;
        Vector2d center;
        
        if (traceType == 0) {
            radius = Utils::findCircle((Edge)path.gov1, (Edge)path.gov2,
                                       (Vector2d)boundaryElements[index], center);
            
        } else if (traceType == 1) {
            if (path.gov2.type == "Vertex") {
                radius = Utils::findCircle((Edge)path.gov1, (Vector2d)path.gov2,
                                           (Vector2d)boundaryElements[index], center);
                
            } else {
                radius = Utils::findCircle((Edge)path.gov2, (Vector2d)boundaryElements[index],
                                           (Vector2d)path.gov1, center);
            }
            
        } else {
            radius = Utils::findCircle((Vector2d)path.gov1, (Vector2d)path.gov2,
                                       (Vector2d)boundaryElements[index], center);
        }

        path.keyPoint2 = KeyPoint(center, radius);
    }
}

void MedialAxisTransform::traceEdgeEdgePath(Path& path) const
{
	if (path.gov1.halfLine1 == path.gov2.halfLine2) {
		// key point is an end point 
		path.keyPoint2 = KeyPoint(path.gov2.vertex2, 0);
 		
	} else {
		Vector2d bisector = (-path.gov1.tangent() + path.gov2.tangent())/2;

		// compute candidate points 
		Vector2d candPoint1 = Utils::lineIntersection(path.keyPoint1, bisector, 
													  path.gov1.vertex1, path.gov1.halfLine1);

		Vector2d candPoint2 = Utils::lineIntersection(path.keyPoint1, bisector,
													  path.gov2.vertex2, path.gov2.halfLine2);

		// find the closer candidate key point 
		Vector2d candPoint = (path.keyPoint1 - candPoint1).squaredNorm() < 
							 (path.keyPoint1 - candPoint2).squaredNorm() ? 
							 candPoint1 : candPoint2;

		// find distance to closest governor 
		double radius = Utils::distToEdge(candPoint, path.gov1);

		path.keyPoint2 = KeyPoint(candPoint, radius);
        
		checkValidity(path, 0);
	}
}

void MedialAxisTransform::traceEdgeVertexPath(Path& path, const int order) const
{
	Vector2d focus;
	Vector2d candPoint1;
	Vector2d candPoint2;

	if (order == 0) {
		focus = path.gov1;
		Edge directrix = path.gov2;
		path.parabola = Parabola(focus, directrix);

		// compute candidate points
		candPoint1 = Utils::parabolaIntersection(path.parabola,
												 (Vector2d)path.gov1, path.gov1.halfLine1);

		candPoint2 = Utils::parabolaIntersection(path.parabola,
												 path.gov2.vertex2, path.gov2.halfLine2);

	} else {
		focus = path.gov2;
		Edge directrix = path.gov1;
		path.parabola = Parabola(focus, directrix);

		// compute candidate points 
		candPoint1 = Utils::parabolaIntersection(path.parabola,
												 path.gov1.vertex1, path.gov1.halfLine1);

		candPoint2 = Utils::parabolaIntersection(path.parabola,
												 (Vector2d)path.gov2, path.gov2.halfLine2);
	}

	// find the closer candidate key point
	Vector2d candPoint = (path.keyPoint1 - candPoint1).squaredNorm() < 
						 (path.keyPoint1 - candPoint2).squaredNorm() ? 
						 candPoint1 : candPoint2;

	// find distance to closest governor 
	double radius = (candPoint - focus).norm();

	path.keyPoint2 = KeyPoint(candPoint, radius);
    
    // FIX: Works, but don't know why?
    if (path.keyPoint1.y() <= focus.y()) {
        path.parabola.set = 1;
    } else {
        path.parabola.set = 2;
    }
    
	checkValidity(path, 1);
}

void MedialAxisTransform::traceVertexVertexPath(Path& path) const
{
	Edge edge(path.gov1, path.gov2);
	Vector2d normal = edge.normal();
	Vector2d mid = (path.gov1 + path.gov2) / 2;
    
	// compute candidate points
	Vector2d candPoint1 = Utils::lineIntersection(mid, normal, 
												  (Vector2d)path.gov1, path.gov1.halfLine1);

	Vector2d candPoint2 = Utils::lineIntersection(mid, normal,
												  (Vector2d)path.gov2, path.gov2.halfLine2);

	Vector2d zero = Vector2d::Zero();
	if (candPoint1 != zero || candPoint2 != zero) {
		// find the closer candidate key point
        
		Vector2d candPoint;
		if (candPoint1 == zero) candPoint = candPoint2;
		else if (candPoint2 == zero) candPoint = candPoint1;
		else candPoint = (path.keyPoint1 - candPoint1).squaredNorm() < 
					     (path.keyPoint1 - candPoint2).squaredNorm() ? 
						 candPoint1 : candPoint2;

		// find distance to closest governor 
		double radius = (candPoint - path.gov1).norm();
		path.keyPoint2 = KeyPoint(candPoint, radius);
        
        checkValidity(path, 2);
        
    } else {
        // find boundary element closest to first keypoint
         int index = -1;
         double minD = 999;
         
         size_t size = boundaryElements.size();
         int startIndex = path.gov2.index+1;
         if (startIndex == size) startIndex = 0;
         
         int endIndex = path.gov1.index;
         if (endIndex < path.gov2.index) endIndex += size;
         
         for (size_t i = startIndex; i < endIndex; i++) {
            size_t j = i;
            if (j >= size) j -= size;
         
            double d = (path.keyPoint1 - boundaryElements[j]).norm();
            if (fabs(d - minD) > 0.01 && d < minD) {
                minD = d;
                index = boundaryElements[j].index;
            }
         }
        
        // remove concave edges
        BoundaryElement be = boundaryElements[index];
        if (be.type == "Edge") {
            int prev = index - 1;
            if (prev < 0) prev = (int)boundaryElements.size()-1;
            
            int next = index + 1;
            if (next == (int)boundaryElements.size()) next = 0;
                
            if (boundaryElements[prev].type == "Vertex" &&
                (path.keyPoint1 - boundaryElements[prev]).norm() < 0.01) { // FIX: Should be less that epsilon
                be = boundaryElements[prev];
                
            } else if (boundaryElements[next].type == "Vertex" &&
                       (path.keyPoint1 - boundaryElements[next]).norm() < 0.01) { // FIX: Should be less that epsilon 
                be = boundaryElements[next];
            }
        }
        
        // if there is, determine new center and radius for medial ball
        double radius;
        Vector2d center;
         
        if (be.type == "Edge") {
            radius = Utils::findCircle((Edge)be, (Vector2d)path.gov2, (Vector2d)path.gov1, center);
            
        } else {
            radius = Utils::findCircle((Vector2d)path.gov1, (Vector2d)path.gov2,
                                       (Vector2d)be, center);
        }
        
        path.keyPoint2 = KeyPoint(center, radius);
    }
}

void MedialAxisTransform::tracePath(Path& path, std::vector<Path>& medialPaths) const
{
	// trace paths case by case
	if (path.gov1.type == "Edge" && path.gov2.type == "Edge") {
		traceEdgeEdgePath(path);

	} else if (path.gov1.type == "Vertex" && path.gov2.type == "Edge") {
		traceEdgeVertexPath(path, 0);

	} else if (path.gov1.type == "Edge" && path.gov2.type == "Vertex") {
		traceEdgeVertexPath(path, 1);

	} else if(path.gov1.type == "Vertex" && path.gov2.type == "Vertex") {
		traceVertexVertexPath(path);
	}

	medialPaths.push_back(path);
}

void MedialAxisTransform::findIntersections(const Path& path,
                                            std::vector<BoundaryElement>& intersections) const
{
    std::vector<std::pair<BoundaryElement, bool> > intersectionElements;
    
    // insert intersecting boundary element around junction point in sorted order
    // implementation detail: gov2 has smaller index than gov1
    size_t size = boundaryElements.size();
    int endIndex = path.gov1.index;
    if (endIndex < path.gov2.index) endIndex += size;
    
    for (size_t i = path.gov2.index; i <= endIndex; i++) {
        size_t index = i;
        if (index >= size) index -= size;
        
        double d;
        if (boundaryElements[index].type == "Edge") {
            d = Utils::distToEdge(path.keyPoint2, boundaryElements[index]);
            
        } else if (boundaryElements[index].type == "Vertex") {
            d = (path.keyPoint2 - boundaryElements[index]).norm();
        }
        
        if (fabs(path.keyPoint2.radius - d) < 0.01) { // FIX: Should be less that epsilon
            std::cout << "index: " << index << std::endl;
            std::pair<BoundaryElement, bool> boundaryElementPair(boundaryElements[index], true);
            intersectionElements.push_back(boundaryElementPair);
        }
    }
    
    // remove concave edges
    removeConcaveEdges(intersectionElements);
    
    // fill valid intersecting boundary elements
    for (size_t i = 0; i < intersectionElements.size(); i++) {
        if (intersectionElements[i].second) {
            intersections.push_back(intersectionElements[i].first);
        }
    }
}

void MedialAxisTransform::handleTransitions(std::vector<BoundaryElement>& intersections)
{
    // detect forward transitions
    for (size_t i = 0; i < intersections.size()-1; i++) {
        if (intersections[i].type == "Vertex") {
            if (boundaryElements[intersections[i].index].transForward == intersections[i+1].index) {
                boundaryElements[intersections[i].index].shouldTransitionForward = true;
                
            } else {
                boundaryElements[intersections[i].index].shouldTransitionForward = false;
                boundaryElements[intersections[i].index].transForward = intersections[i+1].index;
            }
        }
    }
    
    // detect backward transitions
    // vertex should not have a forward and backward transition in the same governor pair
    for (size_t i = intersections.size()-1; i > 0; i--) {
        if (intersections[i].type == "Vertex") {
            if (boundaryElements[intersections[i].index].transBack == intersections[i-1].index) {
                boundaryElements[intersections[i].index].shouldTransitionBack = true;
                
            } else {
                boundaryElements[intersections[i].index].shouldTransitionBack = false;
                boundaryElements[intersections[i].index].transBack = intersections[i-1].index;
            }
        }
    }
    
    // transition forward
    for (size_t i = 0; i < intersections.size()-1; i++) {
        if (boundaryElements[intersections[i].index].shouldTransitionForward) {
            int nextIndex = intersections[i].index+1;
            if (nextIndex == boundaryElements.size()) nextIndex = 0;
            intersections[i] = boundaryElements[nextIndex];
        }
    }
    
    // transition backward
    for (size_t i = intersections.size()-1; i > 0; i--) {
        if (boundaryElements[intersections[i].index].shouldTransitionBack) {
            int nextIndex = intersections[i].index-1;
            if (nextIndex < 0) nextIndex = (int)boundaryElements.size()-1;
            intersections[i] = boundaryElements[nextIndex];
        }
    }
}

void MedialAxisTransform::initializeNewPaths(Path& path, std::vector<Path>& newPathList)
{
    // get intersecting boundary elements
    std::vector<BoundaryElement> intersections;
    findIntersections(path, intersections);

    // handle transitions
	handleTransitions(intersections);
    
	// populate path list
	newPathList.clear();
	for (size_t i = 0; i < intersections.size()-1; i++) {
        std::cout << "idx1: " << intersections[i+1].index << " idx2: "
                  << intersections[i].index << std::endl;
		Path newPath(path.keyPoint2, intersections[i+1], intersections[i]);
		newPathList.push_back(newPath);
	}
    
     std::cout << std::endl;
}

std::vector<Path> MedialAxisTransform::run() 
{
	// source: Joan-Arinyo et al. Computing the Medial Axis Transform of Polygonal Domains by Tracing Paths
	std::vector<Path> medialPaths;
	std::vector<Path> newPathList;
	std::stack<Path> pathStack;

	Path firstPath;
	initializeFirstPath(firstPath);
	pathStack.push(firstPath);
	
 	while (!pathStack.empty()) {
		Path path = pathStack.top();
		pathStack.pop();

		tracePath(path, medialPaths);
		if (path.keyPoint2.radius != 0.0) {
			initializeNewPaths(path, newPathList);
            if (newPathList.size() == 1) medialPaths[medialPaths.size()-1].keyPoint2.isTransition = true;
            
			for (int i = 0; i < newPathList.size(); i++) {
				pathStack.push(newPathList[i]);
			}
		}
	}
	
	return medialPaths;
}