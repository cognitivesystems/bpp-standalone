/*
 * FittingPoint.cpp
 *
 *  Created on: August, 2015
 *      Author: Cai
 */

#include "FittingPoint.h"

namespace bpa
{
FittingPoint::FittingPoint(double xi, double yi, double zi, int q)
{
  coordinates << xi, yi, zi;
  quadrant = q;
  score = 0;
  temp_a_helt = 0;

  switch (q)
  {
    case 1:
      direction_x << 1, 0, 0;
      direction_y << 0, 1, 0;
      direction_z << 0, 0, 1;
      direction_box_pos << 0, 0, 0;
      break;
    case 2:
      direction_x << 1, 0, 0;
      direction_y << 0, -1, 0;
      direction_z << 0, 0, 1;
      direction_box_pos << 0, -1, 0;
      break;
    case 3:
      direction_x << -1, 0, 0;
      direction_y << 0, -1, 0;
      direction_z << 0, 0, 1;
      direction_box_pos << -1, -1, 0;
      break;
    case 4:
      direction_x << -1, 0, 0;
      direction_y << 0, 1, 0;
      direction_z << 0, 0, 1;
      direction_box_pos << -1, 0, 0;
      break;
    default:
      break;
  }
}

FittingPoint::FittingPoint(Eigen::Vector3d point, int q)
{
  coordinates = point;
  quadrant = q;
  score = 0;
  temp_a_helt = 0;

  switch (q)
  {
    case 1:
      direction_x << 1, 0, 0;
      direction_y << 0, 1, 0;
      direction_z << 0, 0, 1;
      direction_box_pos << 0, 0, 0;
      break;
    case 2:
      direction_x << 1, 0, 0;
      direction_y << 0, -1, 0;
      direction_z << 0, 0, 1;
      direction_box_pos << 0, -1, 0;
      break;
    case 3:
      direction_x << -1, 0, 0;
      direction_y << 0, -1, 0;
      direction_z << 0, 0, 1;
      direction_box_pos << -1, -1, 0;
      break;
    case 4:
      direction_x << -1, 0, 0;
      direction_y << 0, 1, 0;
      direction_z << 0, 0, 1;
      direction_box_pos << -1, 0, 0;
      break;
    default:
      break;
  }
}

//????
FittingPoint::FittingPoint()
{
  coordinates << 0.0, 0.0, 0.0;
  quadrant = 1;
  score = 0;
  temp_a_helt = 0;

  direction_x << 1, 0, 0;
  direction_y << 0, 1, 0;
  direction_z << 0, 0, 1;
  direction_box_pos << 0.0, 0.0, 0.0;
}

// FittingPoint::FittingPoint(const FittingPoint &fp)
//{
//    coordinates.pos_x = fp.coordinates.pos_x;
//    coordinates.pos_y = fp.coordinates.pos_y;
//    coordinates.pos_z = fp.coordinates.pos_z;
//    quadrant = fp.quadrant;
//    score = fp.score;
//    temp_a_helt = fp.temp_a_helt;
//}

FittingPoint::~FittingPoint()
{
}

bool FittingPoint::equals(FittingPoint& fp)
{
  return (floatEqual(this->coordinates(0), fp.coordinates(0)) && floatEqual(this->coordinates(1), fp.coordinates(1)) &&
          floatEqual(this->coordinates(2), fp.coordinates(2)) && (this->quadrant == fp.quadrant));
}

bool FittingPoint::operator==(const FittingPoint& other)
{
  return (floatEqual(this->coordinates(0), other.coordinates(0)) &&
          floatEqual(this->coordinates(1), other.coordinates(1)) &&
          floatEqual(this->coordinates(2), other.coordinates(2)) && (this->quadrant == other.quadrant));
}

bool FittingPoint::hasLowerScoreThan(const FittingPoint& other)
{
  return this->score < other.score;
}
}
