/*
 * Copyright 2018-2019 Autoware Foundation. All rights reserved.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#include <amathutils_lib/amathutils.hpp>
#include <algorithm>

namespace amathutils
{
geometry_msgs::Point getNearPtOnLine(const geometry_msgs::Point &_p, const geometry_msgs::Point &_a,
                                     const geometry_msgs::Point &_b)
{
  double len = find_distance(_a, _b);

  geometry_msgs::Point vnab;
  geometry_msgs::Point vap;
  geometry_msgs::Point ret;

  vnab.x = (_b.x - _a.x) / len;
  vnab.y = (_b.y - _a.y) / len;
  vnab.z = (_b.z - _a.z) / len;

  vap.x = _p.x - _a.x;
  vap.y = _p.y - _a.y;
  vap.z = _p.z - _a.z;

  double dist_ax = vnab.x * vap.x + vnab.y * vap.y + vnab.z * vap.z;

  ret.x = _a.x + (vnab.x * dist_ax);
  ret.y = _a.y + (vnab.y * dist_ax);
  ret.z = _a.z + (vnab.z * dist_ax);

  return ret;
}
double find_distance(const geometry_msgs::Point &_from, const geometry_msgs::Point &_to)
{
  return std::hypot(std::hypot(_from.x - _to.x, _from.y - _to.y), _from.z - _to.z);
}
double find_distance(const geometry_msgs::Pose &_from, const geometry_msgs::Pose &_to)
{
  return find_distance(_from.position, _to.position);
}

double find_angle(const geometry_msgs::Point &_from, const geometry_msgs::Point &_to)
{
  double _angle = std::atan2(_to.y - _from.y, _to.x - _from.x);
  if (_angle < 0.0)
    _angle = _angle + 2 * M_PI;

  return _angle * 360 / (2 * M_PI);
}

int findOrientation(const geometry_msgs::Point &p, const geometry_msgs::Point &q, const geometry_msgs::Point &r)
{
  // To find orientation of ordered triplet (p, q, r).
  // The function returns following values
  // 0 --> p, q and r are collinear
  // 1 --> Clockwise
  // 2 --> Counterclockwise
  // See https://www.geeksforgeeks.org/orientation-3-ordered-points/
  // for details of below formula.
  double val = (q.y - p.y) * (r.x - q.x) - (q.x - p.x) * (r.y - q.y);

  if (std::fabs(val) <= eps)  // collinear
  {
    return 0;
  }
  else if (val > 0)  // clockwise
  {
    return 1;
  }
  else  // counterclockwise
  {
    return 2;
  }
}

bool isPointOnLine(const geometry_msgs::Point &_target, const geometry_msgs::Point &_line_p1,
  const geometry_msgs::Point &_line_p2)
{
  // Given three collinear points _line_p1, _target, _line_p2, the function checks if
  // point _target lies on line segment '_line_p1::_line_p2'
  double x_max = std::max(_line_p1.x, _line_p2.x);
  double x_min = std::min(_line_p1.x, _line_p2.x);
  double y_max = std::max(_line_p1.y, _line_p2.y);
  double y_min = std::min(_line_p1.y, _line_p2.y);
  if ((_target.x < x_max || std::fabs(x_max - _target.x) < eps) &&  // _target.x <= x_max
      (_target.x > x_min || std::fabs(x_min - _target.x) < eps) &&  // _target.x >= x_min
      (_target.y < y_max || std::fabs(y_max - _target.y) < eps) &&  // _target.y <= y_max
      (_target.y > y_min || std::fabs(y_min - _target.y) < eps))    // _target.y >= y_min
    return true;
  else
    return false;
}

bool isIntersectLine(const geometry_msgs::Point &_l1_p1, const geometry_msgs::Point &_l1_p2,
                     const geometry_msgs::Point &_l2_p1, const geometry_msgs::Point &_l2_p2)
{
  // See https://www.geeksforgeeks.org/check-if-two-given-line-segments-intersect/
  // for details of below algorithm

  int o1 = findOrientation(_l1_p1, _l1_p2, _l2_p1);
  int o2 = findOrientation(_l1_p1, _l1_p2, _l2_p2);
  int o3 = findOrientation(_l2_p1, _l2_p2, _l1_p1);
  int o4 = findOrientation(_l2_p1, _l2_p2, _l1_p2);

  // General Case:
  // (_l1_p1, _l1_p2, _l2_p1) and (_l1_p1, _l1_p2, _l2_p2) have different orientations
  // && (_l2_p1, _l2_p2, _l1_p1) and _l2_p1, _l2_p2, _l1_p2) have different orientations
  if (o1 != o2 && o3 != o4)
      return true;

  // Special Cases:
  // _l1_p1, _l1_p2 and _l2_p1 are colinear and _l2_p1 lies on segment _l1_p1::_l1_p2
  if (o1 == 0 && isPointOnLine(_l2_p1, _l1_p1, _l1_p2))
    return true;
  // _l1_p1, _l1_p2 and _l2_p2 are colinear and _l2_p2 lies on segment _l1_p1::_l1_p2
  if (o2 == 0 && isPointOnLine(_l2_p2, _l1_p1, _l1_p2))
    return true;
  // _l2_p1, _l2_p2 and _l1_p1 are colinear and _l1_p1 lies on segment _l2_p1::_l2_p2
  if (o3 == 0 && isPointOnLine(_l1_p1, _l2_p1, _l2_p2))
    return true;
  // _l2_p1, _l2_p2 and _l1_p2 are colinear and _l1_p2 lies on segment _l2_p1::_l2_p2
  if (o4 == 0 && isPointOnLine(_l1_p2, _l2_p1, _l2_p2))
    return true;

  // Otherwise, the lines do not intersect
  return false;
}

/*
 *        |
 *     .　|       =>LEFT = 1
 *     　 |   .   =>RIGHT = -1
 *      　|　
 */

int isPointLeftFromLine(const geometry_msgs::Point &_target, const geometry_msgs::Point &_line_p1,
                        const geometry_msgs::Point &_line_p2)
{
  const int LEFT = 1;
  const int RIGHT = -1;
  const int ONLINE = 0;
  const int ret = findOrientation(_line_p1, _target, _line_p2);
  if (ret == 1)
  {
    return LEFT;
  }
  else if (ret == 2)
  {
    return RIGHT;
  }
  else
  {
    return ONLINE;
  }
}

// Following implementation comes from the website below:
// Author: Dan Sunday
// site: http://geomalgorithms.com/a02-_lines.html
// viewed: 2019/5/20
double distanceFromSegment(
  const geometry_msgs::Point &_l1, const geometry_msgs::Point &_l2, const geometry_msgs::Point &_p)
{
  geometry_msgs::Point v, w;
  v.x = _l2.x - _l1.x;
  v.y = _l2.y - _l1.y;
  v.z = _l2.z - _l1.z;

  w.x = _p.x - _l1.x;
  w.y = _p.y - _l1.y;
  w.z = _p.z - _l1.z;

  double dot_vw = v.x * w.x + v.y * w.y + v.z * w.z;
  double dot_vv = v.x * v.x + v.y * v.y + v.z * v.z;

  if (dot_vw <= 0)
  {
    return find_distance(_p, _l1);
  }

  if (dot_vv <= dot_vw)
  {
    return find_distance(_p, _l2);
  }

  double b = dot_vw / dot_vv;
  geometry_msgs::Point pb;
  pb.x = _l1.x + b * v.x;
  pb.y = _l1.y + b * v.y;
  pb.z = _l1.z + b * v.z;

  return find_distance(_p, pb);
}

double getPoseYawAngle(const geometry_msgs::Pose &_pose)
{
  double r, p, y;

  // get Quaternion
  tf::Quaternion quat(_pose.orientation.x, _pose.orientation.y, _pose.orientation.z, _pose.orientation.w);
  // converted to RPY[-pi : pi]
  tf::Matrix3x3(quat).getRPY(r, p, y);
  return y;
}

geometry_msgs::Quaternion getQuaternionFromYaw(const double &_yaw)
{
  tf2::Quaternion q;
  q.setRPY(0, 0, _yaw);
  return tf2::toMsg(q);
}

double calcPosesAngleDiffRaw(const geometry_msgs::Pose &p_from, const geometry_msgs::Pose &_p_to)
{
  return getPoseYawAngle(p_from) - getPoseYawAngle(_p_to);
}

double normalizeRadian(const double _angle)
{
  double n_angle = std::fmod(_angle, 2 * M_PI);
  n_angle = n_angle > M_PI ? n_angle - 2 * M_PI : n_angle < -M_PI ? 2 * M_PI + n_angle : n_angle;

  // another way
  // Math.atan2(Math.sin(_angle), Math.cos(_angle));
  return n_angle;
}

double calcPosesAngleDiffDeg(const geometry_msgs::Pose &_p_from, const geometry_msgs::Pose &_p_to)
{
  // convert to [-pi : pi]
  return rad2deg(normalizeRadian(calcPosesAngleDiffRaw(_p_from, _p_to)));
}

double calcPosesAngleDiffRad(const geometry_msgs::Pose &_p_from, const geometry_msgs::Pose &_p_to)
{
  // convert to [-pi : pi]
  return normalizeRadian(calcPosesAngleDiffRaw(_p_from, _p_to));
}

bool getIntersect(geometry_msgs::Point p1, geometry_msgs::Point p2, geometry_msgs::Point p3,
  geometry_msgs::Point p4, geometry_msgs::Point* intersect)
{
  return getIntersect(p1.x, p1.y, p2.x, p2.y, p3.x, p3.y, p4.x, p4.y, &intersect->x, &intersect->y);
}

bool getIntersect(double x1, double y1, double x2, double y2, double x3, double y3, double x4,
  double y4, double* intersect_x, double* intersect_y )
{
    // let p1(x1, y1), p2(x2, y2), p3(x3, y3), p4(x4,y4)
    // intersect of line segment p1 to p2 and p3 to p4 satisfies
    // p1 + r(p2 - p1) = p3 + s(p4 - p3)
    // 0 <= r <= 1
    // 0 <= s <= 1
    double denominator = (y4 - y3) * (x2 - x1) - (x4 - x3) * (y2 - y1);

    if (denominator == 0)
    {
      // line is parallel
      return false;
    }

    double r = ( (y4 - y3) * (x3 - x1) - (x4 - x3) * (y3 - y1) ) / denominator;
    double s = ( (y2 - y1) * (x3 - x1) - (x2 - x1) * (y3 - y1) ) / denominator;

    if (r >= 0 && r <= 1 && s >= 0 && s <= 1)
    {
      *intersect_x = x1 + r * (x2 - x1);
      *intersect_y = y1 + r * (y2 - y1);
      return true;
    }
    else
    {
      return false;
    }
}

}  // namespace amathutils
