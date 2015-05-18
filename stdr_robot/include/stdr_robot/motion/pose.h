/*
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>
 * 
 * Authors:
 * 
 * Seguey Alexandrov, sergeyalexandrov@mail.com
 * 
 * About this code:
 * provides with a data structure for using poses in 2D 
 * 
 * (pose = position + orientation)
 * 
 */

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

