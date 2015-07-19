#ifndef KEYPOINT_H
#define KEYPOINT_H

#include <Eigen/Core>
using Eigen::Vector2d;

struct KeyPoint: public Vector2d
{
	// constructors
	KeyPoint(): Vector2d(), radius(-1), isTransition(false) {}
	KeyPoint(const double x, const double y, const double r): Vector2d(x, y), radius(r), isTransition(false) {}
	KeyPoint(const Vector2d& p, const double r): Vector2d(p), radius(r), isTransition(false) {}
	KeyPoint(const KeyPoint& kp): Vector2d(kp.x(), kp.y()), radius(kp.radius), isTransition(kp.isTransition) {}

	// member variable
	double radius;
    bool isTransition;
};

#endif