#ifndef POSE_H
#define POSE_H

#include <iostream>

/** This structure represents pose in 2d space. */
struct Pose
{

  double x;
  double y;
  double theta;

  Pose() : x(0), y(0), theta(0) { }

  Pose(double x, double y, double theta) : x(x), y(y), theta(theta) { }

  Pose(const Pose& other) : x(other.x), y(other.y), theta(other.theta) { }

  const Pose& operator=(const Pose& other)
  {
    x = other.x;
    y = other.y;
    theta = other.theta;
    return *this;
  }

  friend std::ostream& operator<<(std::ostream& out, const Pose& p)
  {
    return out << "[" << p.x << ", " << p.y << ", " << p.theta << "]";
  }

};

#endif /* POSE_H */

