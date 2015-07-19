#ifndef EDGE_H
#define EDGE_H

#include <Eigen/Core>
using Eigen::Vector2d;

struct Edge
{
	// constructors
	Edge() {}
	Edge(const Vector2d& v1, const Vector2d& v2): vertex1(v1), vertex2(v2) {}
	Edge(const Edge& e): vertex1(e.vertex1), vertex2(e.vertex2) {}
        
	// returns normalized tangent 
	Vector2d tangent() const {
		Vector2d t(vertex2 - vertex1);
		t.normalize();

		return t;
	}

	// returns normalized normal
	Vector2d normal() const {
		Vector2d t = tangent();
		Vector2d n(t.y(),-t.x());

		return n;
	}

	// returns slope
	double slope() const {
		double m = (vertex2.y() - vertex1.y()) / (vertex2.x() - vertex1.x());

		return m;
	}
    
	// member variables 
	Vector2d vertex1;
	Vector2d vertex2;
};

#endif 