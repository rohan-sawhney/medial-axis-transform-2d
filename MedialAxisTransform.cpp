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

		if (std::abs(d - path.keyPoint2.radius) < 0.1) {
			std::cout << "index: " << index << std::endl;
			std::pair<BoundaryElement, bool> boundaryElementPair(boundaryElements[index], true);
			intersectionElements.push_back(boundaryElementPair);
		}
	}

	// remove edges sharing a convex vertex 
	for (int i = 0; i < (int)intersectionElements.size(); i++) {
		if (intersectionElements[i].first.type == "Vertex") {
			// edge 1
			if (i-1 >= 0 && intersectionElements[i-1].first.vertex2 == intersectionElements[i].first) {
				// vertex is between both governors
				intersectionElements[i-1].second = false;

			} else if (i+1 < intersectionElements.size() && 
					   intersectionElements[i+1].first.vertex2 == intersectionElements[i].first) {
				// vertex was a governor and precedes both edges
				intersectionElements[i+1].second = false;
			}	

			// edge 2
			if (i+1 < intersectionElements.size() && 
				intersectionElements[i+1].first.vertex1 == intersectionElements[i].first) {
				// vertex is between both governors
				intersectionElements[i+1].second = false;

			} else if (i+2 < intersectionElements.size() && 
					   intersectionElements[i+2].first.vertex1 == intersectionElements[i].first) {
				// vertex was a governor and precedes both edges
				intersectionElements[i+2].second = false;
			}		
		}
	}

	// fill valid intersecting boundary elements
	for (size_t i = 0; i < intersectionElements.size(); i++) {
		if (intersectionElements[i].second) {
			validIntersections.push_back(intersectionElements[i].first);
		}
	}

	// populate path list
	newPathList.clear();
	for (size_t i = 0; i < validIntersections.size()-1; i++) {
		// handle transitions
		if (validIntersections[i].type == "Vertex") {
			if (boundaryElements[validIntersections[i].index].transIndex == validIntersections[i+1].index) {
				// determine next index based on orientation
				int nextIndex;
				if (validIntersections[i].index < validIntersections[i+1].index) {
					// counter oriented
					nextIndex = validIntersections[i].index+1;
					if (nextIndex == boundaryElements.size()) nextIndex = 0;

				} else {
					// iso oriented
					nextIndex = validIntersections[i].index-1;
					if (nextIndex < 0) nextIndex = boundaryElements.size()-1;
				}
				validIntersections[i] = boundaryElements[nextIndex];
				
			} else {
				boundaryElements[validIntersections[i].index].transIndex = validIntersections[i+1].index;
			}
		}

		std::cout << "idx1: " << validIntersections[i+1].index 
				  << " idx2: " << validIntersections[i].index << std::endl;

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
	
//	int j = 0;
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

//		j++;
//		if (j == 6) break;
	}
	
	return medialPaths;
}