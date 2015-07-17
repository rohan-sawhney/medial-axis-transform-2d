 #include "MedialAxisTransform.h"
#include <stack>
#include <utility>

#define EPSILON 1e-6

void MedialAxisTransform::setBoundaryElements(const std::vector<BoundaryElement>& be)
{
	boundaryElements = be;
}

void MedialAxisTransform::initializeFirstPath(Path& firstPath) const 
{
	int size = boundaryElements.size();

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

		//TODO: check validity 
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

	//TODO: check validity 
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

	Vector2d invalid;
	if (candPoint1 != invalid || candPoint2 != invalid) {
		// find the closer candidate key point 
		Vector2d candPoint;
		if (candPoint1 == invalid) candPoint = candPoint2;
		else if (candPoint2 == invalid) candPoint = candPoint1;
		else candPoint = (path.keyPoint1 - candPoint1).squaredNorm() < 
					     (path.keyPoint1 - candPoint2).squaredNorm() ? 
						 candPoint1 : candPoint2;

		// find distance to closest governor 
		double radius = (candPoint - path.gov1).norm();
		path.keyPoint2 = KeyPoint(candPoint, radius);
	} 

	//TODO: check validity 
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

// remove edges sharing a convex vertex. If a vertex is a governor, then the edges proceed 
// or precede the vertex in intersectionElements depending on which of the two governors the 
// vertex is. Otherwise, the vertex lies in between the edges 
void removeConvexEdges(std::vector<std::pair<BoundaryElement, bool> >& intersectionElements)
{
	size_t size = intersectionElements.size();

	// check if gov1 is a convex vertex
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

	// check if gov2 is a convex vertex
	if (intersectionElements[size-1].first.type == "Vertex") {
		// the second last element could be edge 1 or edge 2
		if (intersectionElements[size-2].first.vertex2 == intersectionElements[size-1].first ||
			intersectionElements[size-2].first.vertex1 == intersectionElements[size-1].first) {
			intersectionElements[size-2].second = false;
		}
		
		// check for edge 2
		if (intersectionElements[size-3].first.vertex1 == intersectionElements[size-1].first) {
			intersectionElements[size-3].second = false;
		}
	}

	// check if any other intersection element is a convex vertex
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

void MedialAxisTransform::handleTransitions(std::vector<BoundaryElement>& validIntersections)
{
	// forward transition
	for (size_t i = 0; i < validIntersections.size()-1; i++) {
		if (validIntersections[i].type == "Vertex") {
			if (boundaryElements[validIntersections[i].index].transForward == validIntersections[i+1].index) {

				int nextIndex = validIntersections[i].index+1;
				if (nextIndex == boundaryElements.size()) nextIndex = 0;
				validIntersections[i] = boundaryElements[nextIndex];

			} else {
				boundaryElements[validIntersections[i].index].transForward = validIntersections[i+1].index;
			}
		}
	}

	// backward transition
	// vertex should not have a forward and backward transition in the same goveror pair
	for (size_t i = validIntersections.size()-1; i > 0; i--) {
		if (validIntersections[i].type == "Vertex") {
			if (boundaryElements[validIntersections[i].index].transBack == validIntersections[i-1].index) {
				
				int nextIndex = validIntersections[i].index-1;
				if (nextIndex < 0) nextIndex = boundaryElements.size()-1;
				validIntersections[i] = boundaryElements[nextIndex];

			} else {
				boundaryElements[validIntersections[i].index].transBack = validIntersections[i-1].index;
			}
		}
	}
}

void MedialAxisTransform::initializeNewPaths(const Path& path, std::vector<Path>& newPathList)
{
	std::vector<std::pair<BoundaryElement, bool> > intersectionElements;
	std::vector<BoundaryElement> validIntersections;
	
	// insert intersecting boundary element around junction point in sorted order
	// implementation detail: gov2 has smaller index than gov1
	size_t size = boundaryElements.size();
	int endIndex = path.gov1.index;
	if (endIndex < path.gov2.index) endIndex += size;

	for (size_t i = path.gov2.index; i <= endIndex; i++) {
		int index = i;
		if (index >= size) index -= size;

		double d;
		if (boundaryElements[index].type == "Edge") {
			d = Utils::distToEdge(path.keyPoint2, boundaryElements[index]);
			
		} else if (boundaryElements[index].type == "Vertex") {
			d = (path.keyPoint2 - boundaryElements[index]).norm();
		}

		if (std::abs(d - path.keyPoint2.radius) < 0.1) { // FIX: Should be less that epsilon
			std::cout << "index: " << index << std::endl;
			std::pair<BoundaryElement, bool> boundaryElementPair(boundaryElements[index], true);
			intersectionElements.push_back(boundaryElementPair);
		}
	}

	// remove convex edges
	removeConvexEdges(intersectionElements);

	// fill valid intersecting boundary elements
	for (size_t i = 0; i < intersectionElements.size(); i++) {
		if (intersectionElements[i].second) {
			validIntersections.push_back(intersectionElements[i].first);
		}
	}

	// handle transitions
	handleTransitions(validIntersections);

	// populate path list
	newPathList.clear();
	for (size_t i = 0; i < validIntersections.size()-1; i++) {
		Path newPath(path.keyPoint2, validIntersections[i+1], validIntersections[i]);
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
			for (int i = 0; i < newPathList.size(); i++) {
				pathStack.push(newPathList[i]);
			}
		}
	}
	
	return medialPaths;
}