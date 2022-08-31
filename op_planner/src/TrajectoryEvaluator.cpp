/// \file TrajectoryEvaluator.cpp
/// \brief Calculate collision costs for roll out trajectory for free trajectory evaluation for OpenPlanner local
/// planner version 1.5+ (Dynamic Obstacles Prediction) \author Hatem Darweesh \date Jan 3, 2019

#include "op_planner/TrajectoryEvaluator.h"
#include "op_planner/MatrixOperations.h"
#include <cfloat>

namespace PlannerHNS
{
double constexpr g_lateral_skip_value = 20.0;
double constexpr g_longitudinal_safe_overtake_distance = 0.5;
bool constexpr g_enable_debug = false;

TrajectoryEvaluator::TrajectoryEvaluator()
{
}

TrajectoryEvaluator::~TrajectoryEvaluator()
{
}

void TrajectoryEvaluator::SetEvalParams(const EvaluationParams& eval_param)
{
  eval_params_ = eval_param;
}

void TrajectoryEvaluator::SetPlanningParams(const PlanningParams& planning_param)
{
  planning_params_ = planning_param;
}

TrajectoryCost TrajectoryEvaluator::doOneStep(const std::vector<std::vector<WayPoint> >& m_GlobalPaths,
                                              const std::vector<std::vector<WayPoint> >& roll_outs,
                                              const std::vector<WayPoint>& total_paths, const WayPoint& curr_state,
                                              const PlanningParams& original_params, const CAR_BASIC_INFO& car_info,
                                              const VehicleState& vehicle_state,
                                              const std::vector<DetectedObject>& obj_list,
                                              const RoadNetwork &map,
                                              const bool& b_static_only,
                                              const int& prev_curr_index,
                                              const bool& b_keep_curr)
{
  m_Map = map;
  PlanningParams params = original_params;

  if (roll_outs.size() == 1)
  {
    params.rollOutNumber = 0;
  }

  double critical_lateral_distance = car_info.width / 2.0 + params.horizontalSafetyDistancel;
  double critical_long_front_distance =
      car_info.wheel_base / 2.0 + car_info.length / 2.0 + params.verticalSafetyDistance;
  double critical_long_back_distance = car_info.length / 2.0 - car_info.wheel_base / 2.0;

  int curr_index = -1;
  if (prev_curr_index >= 0 && prev_curr_index < roll_outs.size())
    curr_index = prev_curr_index;
  else
    curr_index = getCurrentRollOutIndex(total_paths, curr_state, params);

  initializeLocalRollOuts(curr_state, car_info, params, critical_long_back_distance, roll_outs, local_roll_outs_);

  initializeSafetyPolygon(curr_state, car_info, vehicle_state, critical_lateral_distance, critical_long_front_distance,
                          critical_long_back_distance, safety_border_);

  initializeCosts(roll_outs, params, trajectory_costs_);

  calculateTransitionCosts(trajectory_costs_, curr_index, params);

  // Populates all_contour_points with the contours of all detected obstacles.
  // Also, if enablePrediction param is set to true then populates all predicted trajectory points of all detected 
  // obstacles into all_trajectories_points.
  //
  // Note: those trajectory points which are inside ego's contour or another obstacle's contour are not added
  // into all_trajectories_points_
  collectContoursAndTrajectories(obj_list, safety_border_, all_contour_points_, all_trajectories_points_, all_obs_wrt_trajectories_points_,
                                b_static_only);

  collision_points_.clear();
  CalcCostsAndObsOnRollouts(m_GlobalPaths, params, critical_lateral_distance, critical_long_front_distance, local_roll_outs_,
                            all_contour_points_, all_trajectories_points_, all_obs_wrt_trajectories_points_, trajectory_costs_, curr_state,
                            collision_points_);

  normalizeCosts(eval_params_, trajectory_costs_);

  TrajectoryCost best_trajectory = findBestTrajectory(params, prev_curr_index, b_keep_curr, trajectory_costs_);

  return best_trajectory;
}

void TrajectoryEvaluator::initializeLocalRollOuts(const WayPoint& curr_state, const CAR_BASIC_INFO& car_info,
                                                  const PlanningParams& params, const double& c_long_back_d,
                                                  const std::vector<std::vector<WayPoint> >& original_roll_outs,
                                                  std::vector<std::vector<WayPoint> >& local_roll_outs)
{
  local_roll_outs = original_roll_outs;
  int center_index = params.rollOutNumber / 2;

  for (auto& local_roll_out : local_roll_outs)
  {
    if (!local_roll_out.empty())
    {
      WayPoint center_back_point = local_roll_out.at(0);
      center_back_point.pos.x -= c_long_back_d * cos(curr_state.pos.a);
      center_back_point.pos.y -= c_long_back_d * sin(curr_state.pos.a);
      local_roll_out.insert(local_roll_out.begin(), center_back_point);
      PlannerHNS::PlanningHelpers::CalcAngleAndCost(local_roll_out);
    }
  }

  for (unsigned int i = 0; i < local_roll_outs.size(); i++)
  {
    for (unsigned int j = 0; j < local_roll_outs.at(i).size(); j++)
    {
      if (local_roll_outs.at(center_index).size() > j)
        local_roll_outs.at(i).at(j).width =
            hypot(local_roll_outs.at(center_index).at(j).pos.y - local_roll_outs.at(i).at(j).pos.y,
                  local_roll_outs.at(center_index).at(j).pos.x - local_roll_outs.at(i).at(j).pos.x);
      else
        std::cout << "Error .. RollOuts are not synchronized , check TrajectoryEvaluator, initializeLocalRollOuts "
                     "(center_size, j) ("
                  << local_roll_outs.at(center_index).size() << ", " << j << ")" << std::endl;
    }
  }
}

void TrajectoryEvaluator::collectContoursAndTrajectories(const std::vector<PlannerHNS::DetectedObject>& obj_list,
                                                         PolygonShape& ego_car_border,
                                                         std::vector<WayPoint>& contour_points,
                                                         std::vector<WayPoint>& trajectory_points,
                                                         std::vector<WayPoint>& obs_wrt_trajectory_points,
                                                         const bool& b_static_only)
{
  PlannerHNS::WayPoint p, obs;
  double d = 0;
  contour_points.clear();
  trajectory_points.clear();
  obs_wrt_trajectory_points.clear();

  std::vector<PlannerHNS::WayPoint> processed_traj_points;

  filtered_obs_traj_pair_.clear();
  filtered_obs_.clear();
  objects_attention.clear();


  for (unsigned int i = 0; i < obj_list.size(); i++)
  {
    double w = obj_list.at(i).w / 2.0;


    // Take contour points of all detected objects and put them in one big waypoint vector 
    for (unsigned int i_con = 0; i_con < obj_list.at(i).contour.size(); i_con++)
    {
      p.pos = obj_list.at(i).contour.at(i_con);
      p.pos.a = obj_list.at(i).center.pos.a;
      p.v = obj_list.at(i).center.v;
      p.id = static_cast<int>(i);
      p.width = 0;
      contour_points.push_back(p);
    }

    if (b_static_only)
    {
      continue;
    }


    // Iterate over all predicted trajectories of current object
    for (unsigned int i_trj = 0; i_trj < obj_list.at(i).predTrajectories.size(); i_trj++)
    {
      // Iterate over all waypoints of current predicted trajectory
      for (unsigned int i_p = 0; i_p < obj_list.at(i).predTrajectories.at(i_trj).size(); i_p++)
      {
        p = obj_list.at(i).predTrajectories.at(i_trj).at(i_p);
        p.v = obj_list.at(i).center.v;
        p.id = static_cast<int>(i);
        p.width = w;

        bool b_blocking = false;

        // Check if b_blocking is true or current trajectory waypoint is inside ego car's contour then break 
        if (b_blocking || PlanningHelpers::PointInsidePolygon(ego_car_border.points, p.pos) == 1)
        {
          break;
        }

        // bool b_found_point = false;

        // By default when trajectory_points is empty it will push back the p waypoint of predicted trajectories 
        // into trajectory points

        trajectory_points.push_back(p);
        processed_traj_points.push_back(p);

        obs.pos = obj_list.at(i).center.pos;
        obs.pos.a = obj_list.at(i).center.pos.a;
        obs.v = obj_list.at(i).center.v;
        obs.id = obj_list.at(i).id;
        obs.laneId = obj_list.at(i).predTrajectories.at(0).at(0).laneId;
        obs.width = p.width;
        obs_wrt_trajectory_points.push_back(obs);
        
      }
    }
    

    if (processed_traj_points.size() > 0)
    {
      filtered_obs_traj_pair_.push_back(processed_traj_points);
      filtered_obs_.push_back(obj_list.at(i));
      // objects_attention.push_back(obj_list.at(i));
      
    }
    processed_traj_points.clear();
  }

}

void TrajectoryEvaluator::normalizeCosts(const EvaluationParams& eval_param,
                                         std::vector<TrajectoryCost>& trajectory_costs)
{
  double total_priorities_costs = 0;
  double total_lane_change_costs = 0;
  double total_transition_costs = 0;
  double total_lon_costs = 0;
  double total_lat_costs = 0;
  double max_lon_cost = std::numeric_limits<double>::min();
  double min_lon_cost = std::numeric_limits<double>::max();
  double max_lat_cost = std::numeric_limits<double>::min();
  double min_lat_cost = std::numeric_limits<double>::max();
  double lon_diff = 0;
  double lat_diff = 0;
  double epsilon = 0.0001;

  for (auto& trajectory_cost : trajectory_costs)
  {
    if (trajectory_cost.lateral_cost > max_lat_cost)
      max_lat_cost = trajectory_cost.lateral_cost;

    if (trajectory_cost.lateral_cost < min_lat_cost)
      min_lat_cost = trajectory_cost.lateral_cost;

    if (trajectory_cost.longitudinal_cost > max_lon_cost)
      max_lon_cost = trajectory_cost.longitudinal_cost;

    if (trajectory_cost.longitudinal_cost < min_lon_cost)
      min_lon_cost = trajectory_cost.longitudinal_cost;
  }

  lon_diff = max_lon_cost - min_lon_cost;
  lat_diff = max_lat_cost - min_lat_cost;

  for (auto& trajectory_cost : trajectory_costs)
  {
    if (lat_diff > epsilon)
      trajectory_cost.lateral_cost = (trajectory_cost.lateral_cost - min_lat_cost) / lat_diff;
    else
      trajectory_cost.lateral_cost = 0;

    if (lon_diff > epsilon)
      trajectory_cost.longitudinal_cost = (trajectory_cost.longitudinal_cost - min_lon_cost) / lon_diff;
    else
      trajectory_cost.longitudinal_cost = 0;
  }

  for (auto& trajectory_cost : trajectory_costs)
  {
    total_priorities_costs += trajectory_cost.priority_cost;
    total_transition_costs += trajectory_cost.transition_cost;
    total_lon_costs += trajectory_cost.longitudinal_cost;
    total_lat_costs += trajectory_cost.lateral_cost;
    total_lane_change_costs += trajectory_cost.lane_change_cost;
  }

  for (auto& trajectory_cost : trajectory_costs)
  {
    if (total_priorities_costs != 0)
      trajectory_cost.priority_cost = trajectory_cost.priority_cost / total_priorities_costs;
    else
      trajectory_cost.priority_cost = 0;

    if (total_transition_costs != 0)
      trajectory_cost.transition_cost = trajectory_cost.transition_cost / total_transition_costs;
    else
      trajectory_cost.transition_cost = 0;

    if (total_lat_costs != 0)
      trajectory_cost.lateral_cost = trajectory_cost.lateral_cost / total_lat_costs;
    else
      trajectory_cost.lateral_cost = 0;

    if (total_lon_costs != 0)
      trajectory_cost.longitudinal_cost = trajectory_cost.longitudinal_cost / total_lon_costs;
    else
      trajectory_cost.longitudinal_cost = 0;

    if (total_lane_change_costs != 0)
      trajectory_cost.lane_change_cost = trajectory_cost.lane_change_cost / total_lane_change_costs;
    else
      trajectory_cost.lane_change_cost = 0;

    trajectory_cost.cost = (eval_param.priority_weight_ * trajectory_cost.priority_cost +
                            eval_param.transition_weight_ * trajectory_cost.transition_cost +
                            eval_param.lateral_weight_ * trajectory_cost.lateral_cost +
                            eval_param.longitudinal_weight_ * trajectory_cost.longitudinal_cost +
                            eval_param.lane_change_weight_ * trajectory_cost.lane_change_cost);
  }
}

void TrajectoryEvaluator::calculateTransitionCosts(std::vector<TrajectoryCost>& trajectory_costs, const int& curr_index,
                                                   const PlanningParams& params)
{
  for (int ic = 0; ic < trajectory_costs.size(); ic++)
  {
    trajectory_costs.at(ic).transition_cost = fabs(params.rollOutDensity * (ic - curr_index));
  }
}

int TrajectoryEvaluator::getCurrentRollOutIndex(const std::vector<WayPoint>& total_path, const WayPoint& curr_state,
                                                const PlanningParams& params)
{
  RelativeInfo obj_info;
  PlanningHelpers::GetRelativeInfo(total_path, curr_state, obj_info);
  double relative_index = obj_info.perp_distance / params.rollOutDensity;

  if (relative_index > 0)
    relative_index = floor(relative_index);
  else
    relative_index = ceil(relative_index);

  int curr_index = static_cast<int>(params.rollOutNumber / 2. + relative_index);

  if (curr_index < 0)
    curr_index = 0;
  else if (curr_index > params.rollOutNumber)
    curr_index = params.rollOutNumber;

  return curr_index;
}

void TrajectoryEvaluator::initializeCosts(const std::vector<std::vector<WayPoint> >& roll_outs,
                                          const PlanningParams& params, std::vector<TrajectoryCost>& trajectory_costs)
{
  trajectory_costs.clear();
  if (!roll_outs.empty())
  {
    TrajectoryCost tc;
    int center_index = params.rollOutNumber / 2;
    tc.lane_index = 0;
    for (unsigned int it = 0; it < roll_outs.size(); it++)
    {
      tc.index = static_cast<int>(it);
      tc.relative_index = static_cast<int>(it) - center_index;
      tc.distance_from_center = params.rollOutDensity * tc.relative_index;
      tc.priority_cost = fabs(tc.distance_from_center);
      tc.closest_obj_distance = params.horizonDistance;
      if (!roll_outs.at(it).empty())
      {
        tc.lane_change_cost = roll_outs.at(it).at(0).laneChangeCost;
      }
      trajectory_costs.push_back(tc);
    }
  }
}

void TrajectoryEvaluator::initializeSafetyPolygon(const WayPoint& curr_state, const CAR_BASIC_INFO& car_info,
                                                  const VehicleState& vehicle_state, const double& c_lateral_d,
                                                  const double& c_long_front_d, const double& c_long_back_d,
                                                  PolygonShape& car_border)
{
  PlannerHNS::Mat3 inv_rotation_mat(curr_state.pos.a - M_PI_2);
  PlannerHNS::Mat3 inv_translation_mat(curr_state.pos.x, curr_state.pos.y);

  double corner_slide_distance = c_lateral_d / 2.0;
  double ratio_to_angle = corner_slide_distance / car_info.max_wheel_angle;
  double slide_distance = vehicle_state.steer * ratio_to_angle;

  GPSPoint bottom_left(-c_lateral_d, -c_long_back_d, curr_state.pos.z, 0);
  GPSPoint bottom_right(c_lateral_d, -c_long_back_d, curr_state.pos.z, 0);

  GPSPoint top_right_car(c_lateral_d, car_info.wheel_base / 3.0 + car_info.length / 3.0, curr_state.pos.z, 0);
  GPSPoint top_left_car(-c_lateral_d, car_info.wheel_base / 3.0 + car_info.length / 3.0, curr_state.pos.z, 0);

  GPSPoint top_right(c_lateral_d - slide_distance, c_long_front_d, curr_state.pos.z, 0);
  GPSPoint top_left(-c_lateral_d - slide_distance, c_long_front_d, curr_state.pos.z, 0);

  bottom_left = inv_rotation_mat * bottom_left;
  bottom_left = inv_translation_mat * bottom_left;

  top_right = inv_rotation_mat * top_right;
  top_right = inv_translation_mat * top_right;

  bottom_right = inv_rotation_mat * bottom_right;
  bottom_right = inv_translation_mat * bottom_right;

  top_left = inv_rotation_mat * top_left;
  top_left = inv_translation_mat * top_left;

  top_right_car = inv_rotation_mat * top_right_car;
  top_right_car = inv_translation_mat * top_right_car;

  top_left_car = inv_rotation_mat * top_left_car;
  top_left_car = inv_translation_mat * top_left_car;

  car_border.points.clear();
  car_border.points.push_back(bottom_left);
  car_border.points.push_back(bottom_right);
  car_border.points.push_back(top_right_car);
  car_border.points.push_back(top_right);
  car_border.points.push_back(top_left);
  car_border.points.push_back(top_left_car);
}

TrajectoryCost TrajectoryEvaluator::findBestTrajectory(const PlanningParams& params, const int& prev_curr_index,
                                                       const bool& b_keep_curr,
                                                       std::vector<TrajectoryCost> trajectory_costs)
{
  TrajectoryCost best_trajectory;
  best_trajectory.bBlocked = true;
  best_trajectory.closest_obj_distance = params.horizonDistance;
  best_trajectory.closest_obj_velocity = 0;
  best_trajectory.index = params.rollOutNumber / 2;
  best_trajectory.lane_index = 0;
  //  double all_closest_obj_distance = params.horizonDistance;
  //  double all_closest_obj_velocity = 0;

  // because the default best trajectory is the center one,
  // I assign distance and velocity from the center to the best, in case all blocked, this will be the best trajectory
  // I assume that it is blocker by default, for safety reasons

  if (best_trajectory.index >= 0 && best_trajectory.index < trajectory_costs.size())
  {
    best_trajectory = trajectory_costs.at(best_trajectory.index);
    best_trajectory.bBlocked = true;
  }

  std::sort(trajectory_costs.begin(), trajectory_costs.end(), sortCosts);

  // new approach, in b_keep_curr branch
  // 1. remove blocked trajectories, also remove all trajectory in the same side after the blocked trajectory,
  // if center is blocked one side is selected according to general drive direction
  // 2. find average diff between the non blocked trajectories
  // 3. find diff between previous trajectory cost and the current best cost
  // 4. change best if only diff > avg_diff
  if (prev_curr_index >= 0 && prev_curr_index < trajectory_costs.size() && b_keep_curr)
  {
    double avg_diff = 0;
    // find average diff before remove blocked trajectories
    for (int ic = 0; ic < trajectory_costs.size() - 1; ic++)
    {
      avg_diff += (trajectory_costs.at(ic + 1).cost - trajectory_costs.at(ic).cost);
    }

    avg_diff = avg_diff / (double)trajectory_costs.size();

    // Remove blocked
    for (int i = 0; i < trajectory_costs.size(); i++)
    {
      if (trajectory_costs.at(i).bBlocked)
      {
        trajectory_costs.erase(trajectory_costs.begin() + i);
        i--;
      }
    }

    if (!trajectory_costs.empty())
    {
      double closest_obj_distance = params.horizonDistance;
      double closest_obj_velocity = 0;

      // find closest object to vehicle and that object's velocity
      for (auto& trajectory_cost : trajectory_costs)
      {
        if (trajectory_cost.closest_obj_distance < closest_obj_distance)
        {
          closest_obj_distance = trajectory_cost.closest_obj_distance;
          closest_obj_velocity = trajectory_cost.closest_obj_velocity;
        }
      }

      // we consider the first one with smallest cost is the best trajectory. trajectory_costs.at(0) , so
      // now let's find the previous one.

      int best_traj_index = 0;
      int relativ_prev_curr_index = prev_curr_index - params.rollOutNumber / 2;
      for (unsigned int ic = 0; ic < trajectory_costs.size(); ic++)
      {
        // keep the best as the previous if: 1. exists , 2. cost different between previous best and current bes is less
        // than the average cost diff
        if (trajectory_costs.at(ic).relative_index == relativ_prev_curr_index &&
            fabs(trajectory_costs.at(ic).cost - trajectory_costs.at(0).cost) < avg_diff)
        {
          best_traj_index = static_cast<int>(ic);
          break;
        }
      }
      // if we can't find the previous one, then most properly it is blocked now, so , we have to change trajectory to
      // best, trajectory_costs.at(0)
      best_trajectory = trajectory_costs.at(best_traj_index);
    }
  }
  else
  {
    // Assign closest distance and velocity of the best trajectory , only as additional information for the decision
    // maker
    //	  best_trajectory.closest_obj_distance = all_closest_obj_distance;
    //	  best_trajectory.closest_obj_velocity = all_closest_obj_velocity;

    // Find Best not blocked rollout
    for (auto& trajectory_cost : trajectory_costs)
    {
      if (!trajectory_cost.bBlocked)
      {
        // trajectory_costs.at(ic).closest_obj_distance = best_trajectory.closest_obj_distance;
        best_trajectory = trajectory_cost;
        break;
      }
    }
  }

  return best_trajectory;
}

/**
 *
 * @param params Planning params
 * @param critical_lateral_distance Lateral safety distance
 * @param critical_long_front_distance Longitudinal safety distance
 * @param rollOuts roll_outs
 * @param obs_points List of obstacle points
 * @param trajectory_points List of all the waypoints from all rollouts
 * @param obs_wrt_trajectory_points Respective obstacle of each trajectory point.
  * @param trajectory_costs Output, trajectories costs
 * @param curr_pos Current vehicle position
 * @param collision_points Points where a colision will happen.
 */
void TrajectoryEvaluator::CalcCostsAndObsOnRollouts(
    const std::vector<std::vector<WayPoint> >& m_TotalOriginalPaths,
    const PlanningParams& params, 
    const double& critical_lateral_distance, 
    const double& critical_long_front_distance,
    const std::vector<std::vector<WayPoint> >& roll_outs, 
    const std::vector<WayPoint>& obs_points,
    const std::vector<WayPoint>& trajectory_points, 
    const std::vector<WayPoint>& obs_wrt_trajectory_points, 
    std::vector<TrajectoryCost>& trajectory_costs,
    const WayPoint& curr_pos, 
    std::vector<WayPoint>& collision_points)
{
  // Relative object info
  RelativeInfo obsInfoRollout, obsInfoGlobal, goalInfoRollout, carInfoRollout, carInfoGlobal;

  // Obstacle IDs to skip
  std::vector<int> ObsToSkip;

  // Longitudinal distance of the obstacle on the rollout.
  double longitudinalDistOnRollout, longitudinalDistOnGlobal, safeLongitudinalDistOnRollout,
      safeLongitudinalDistOnGlobal, distance_to_goal;

  size_t center_index = params.rollOutNumber / 2;

  // Relative info of car on center rollout.
  PlanningHelpers::GetRelativeInfo(roll_outs[center_index], curr_pos, carInfoGlobal);

  // Relative info of the goal point on center rollout.
  PlanningHelpers::GetRelativeInfo(roll_outs[center_index], roll_outs[center_index].end()[-1], goalInfoRollout);

  // Distance to goal
  distance_to_goal = PlanningHelpers::GetDistanceFromPoseToEnd(curr_pos, roll_outs[center_index]) -
                     params.additionalBrakingDistance + params.verticalSafetyDistance;

  // Extract partial global paths from global paths with lanes & map information
  m_TotalPaths.clear();
  if (m_prev_index.size() != m_TotalOriginalPaths.size()) {
    m_prev_index.clear();
    for (unsigned int i = 0; i < m_TotalOriginalPaths.size(); i++) {
      m_prev_index.push_back(0);
    }
  }

  for (unsigned int i = 0; i < m_TotalOriginalPaths.size(); i++) {
    t_centerTrajectorySmoothed.clear();
    m_prev_index.at(i) = PlannerHNS::PlanningHelpers::ExtractPartFromPointToDistanceDirectionFast(
        m_TotalOriginalPaths.at(i), 
        curr_pos, 
        params.horizonDistance,
        params.pathDensity, 
        t_centerTrajectorySmoothed,
        m_prev_index.at(i));

    if (m_prev_index.at(i) > 0) {
      m_prev_index.at(i) = m_prev_index.at(i) - 1;
    }

    m_TotalPaths.push_back(t_centerTrajectorySmoothed);
  }

  std::vector<std::vector<std::vector<PlannerHNS::WayPoint> > > zero_obs_m_RollOuts;
  // Generate zero obstacle rollouts for predictive evaluations
  GetZeroObstacleRollouts(m_TotalPaths, curr_pos, params, zero_obs_m_RollOuts);

  int rollout_index = 0;

  for (const auto& roll_out : roll_outs)
  {
    // Relative info of car on rollout.
    PlanningHelpers::GetRelativeInfo(roll_out, curr_pos, carInfoRollout);

    for (const auto& obs_point : obs_points)
    {
      // If in the skip list
      if (std::find(ObsToSkip.begin(), ObsToSkip.end(), obs_point.id) != ObsToSkip.end())
      {
        continue;
      }

      // First let's filter as many obstacles as possible.
      // Getting necessary data.
      PlanningHelpers::GetRelativeInfo(roll_out, obs_point, obsInfoRollout);
      PlanningHelpers::GetRelativeInfo(roll_outs[center_index], obs_point, obsInfoGlobal);

      // If obstacle too far laterally (Twice the skip distance) from the global path.
      if (obsInfoGlobal.perp_distance > g_lateral_skip_value)
      {
        ObsToSkip.push_back(obs_point.id);
        continue;
      }

      // Get the longitudinal distance on the rollout of the point.
      longitudinalDistOnRollout =
          PlanningHelpers::GetExactDistanceOnTrajectory(roll_out, carInfoRollout, obsInfoRollout);
      longitudinalDistOnGlobal =
          PlanningHelpers::GetExactDistanceOnTrajectory(roll_outs[center_index], carInfoGlobal, obsInfoGlobal);

      safeLongitudinalDistOnRollout = longitudinalDistOnRollout - critical_long_front_distance;
      safeLongitudinalDistOnGlobal = longitudinalDistOnGlobal - critical_long_front_distance;

      // If obstacle is behind us we make the distances negative.
      if (obsInfoGlobal.iFront == 0 && longitudinalDistOnGlobal > 0)
      {
        longitudinalDistOnGlobal = -longitudinalDistOnGlobal;
      }

      if (distance_to_goal > params.minFollowingDistance &&
          longitudinalDistOnGlobal > distance_to_goal) {
        continue;
      }

      // If obstacle too far behind us (minFollowingDistance) we mark it to skip it.
      // If obs behind us but not far enough we skip the point but not the whole obstacle.
      if (longitudinalDistOnGlobal < -params.minFollowingDistance)
      {
        ObsToSkip.push_back(obs_point.id);
        continue;
      }
      else if (longitudinalDistOnGlobal < 0)
      {
        continue;
      }

      // If obstacle too far longitudinally (Twice the follow distance) from the car we mark it to skip it.
      // If between follow distance and 2 * follow distance we just ignore the point
      if (safeLongitudinalDistOnGlobal > 2 * params.minFollowingDistance)
      {
        ObsToSkip.push_back(obs_point.id);
        continue;
      }
      else if (safeLongitudinalDistOnGlobal > params.minFollowingDistance)
      {
        continue;
      }

      // Obj in safety box and no obstacle detected closer
      // OR Perpendicular distance between obs and rollout below safety distance
      if (((safety_border_.PointInsidePolygon(safety_border_, obs_point.pos)) &&
           (safeLongitudinalDistOnRollout < trajectory_costs[rollout_index].closest_obj_distance)) ||
          ((abs(obsInfoRollout.perp_distance) < critical_lateral_distance) &&
           (safeLongitudinalDistOnRollout < trajectory_costs[rollout_index].closest_obj_distance)))
      {
        trajectory_costs[rollout_index].bBlocked = true;
        trajectory_costs[rollout_index].closest_obj_distance = safeLongitudinalDistOnRollout;
        trajectory_costs[rollout_index].closest_obj_velocity = obs_point.v;

        collision_points.push_back(obsInfoRollout.perp_point);
        trajectory_costs[rollout_index].lateral_cost += 2.0;
      }
    }

  /**
   * =====================================< Rollout eval for yielding >=================================================
   */

    // If no predicted trajectories then skip further processing
    if (filtered_obs_traj_pair_.size() == 0)
    {
      // std::cout << "Trajectory size is zero ... " << std::endl;
      continue;
    }

    int stopLineID = -1;
    int stopSignID = -1;
    double distanceToClosestStopLine;
    RelativeInfo stopLineInfoGlobal;
    TRAFFIC_SIGN_TYPE signType;
    WayPoint stop_wp;

    // Get distance to closest yielding stop line
    distanceToClosestStopLine = PlanningHelpers::GetDistanceToClosestStopLine(m_Map,
      m_TotalOriginalPaths.at(0), curr_pos,
      params.giveUpDistance, 
      params.horizonDistance,
      params.enableStopSignBehavior,
      stopLineID, stopSignID, stop_wp, signType) - params.verticalSafetyDistance;

    // Get global relative info of stop line waypoint on center rollout.
    PlanningHelpers::GetRelativeInfo(zero_obs_m_RollOuts.at(0).at(center_index), stop_wp, stopLineInfoGlobal);

    if (attention_rois.size() == 0)
      GetYieldROIs(stop_wp, stopLineInfoGlobal, signType);


    // If Ego is approaching yielding line then evaluate rollout for yielding
    if (distanceToClosestStopLine < 500 && stopSignID > 0)
    {
      EvaluateRolloutForPredictiveYielding(
        curr_pos,
        params, 
        trajectory_costs, 
        trajectory_points,
        obs_wrt_trajectory_points,
        zero_obs_m_RollOuts.at(0).at(rollout_index), 
        rollout_index,
        critical_lateral_distance, 
        collision_points, 
        carInfoRollout, 
        carInfoGlobal,
        stopLineInfoGlobal,
        stop_wp,
        distanceToClosestStopLine);
      }

    /**
     * ======================================================================================
     */
      rollout_index++;
  }
}

// 
/**
 * @brief 
 * Generate a zero obstacle rollout with constant speed profile for evaluating future trajectory blocking even if the car is stationary.
 * 
 * @param TotalPaths Global extracted paths
 * @param curr_state Current vehicle position
 * @param params Planning params
 * @param m_LanesRollOutsToUse Generated rollout for evaluation
 */
void TrajectoryEvaluator::GetZeroObstacleRollouts(
  const std::vector<std::vector<WayPoint> >& TotalPaths,
  const WayPoint& curr_state, 
  const PlanningParams& params,
  std::vector<std::vector<std::vector<PlannerHNS::WayPoint> > >& zero_obs_m_RollOuts)
{
  PlannerHNS::WayPoint start_point = curr_state;
  start_point.v = params.maxSpeed;

  std::vector<PlannerHNS::WayPoint> sampledPoints_debug;
  m_Planner.GenerateRunoffTrajectory(TotalPaths, start_point,
            params.enableLaneChange,
            params.maxSpeed,
            params.microPlanDistance,
            params.maxSpeed,
            params.minSpeed,
            params.carTipMargin,
            params.rollInMargin,
            params.rollInSpeedFactor,
            params.pathDensity,
            params.rollOutDensity,
            params.rollOutNumber,
            params.smoothingDataWeight,
            params.smoothingSmoothWeight,
            params.smoothingToleranceError,
            params.speedProfileFactor,
            params.enableHeadingSmoothing,
            -1 , -1,
            zero_obs_m_RollOuts, sampledPoints_debug);

  for(unsigned int i=0; i < zero_obs_m_RollOuts.size(); i++)
  {
    for(unsigned int j=0; j < zero_obs_m_RollOuts.at(i).size(); j++)
    {
      PlannerHNS::PlanningHelpers::PredictConstantTimeCostForTrajectory(zero_obs_m_RollOuts.at(i).at(j), start_point, params.minSpeed, params.microPlanDistance);
    }
  }
}


void TrajectoryEvaluator::computeCostsAndPredictColisionsOnRollout(
    const PlanningParams& params, std::vector<TrajectoryCost>& trajectory_costs,
    const std::vector<WayPoint>& trajectory_points, const std::vector<WayPoint>& roll_out, const int rollout_index,
    const double& critical_lateral_distance, std::vector<WayPoint>& collision_points)
{
  for (const auto& trajectory_point : trajectory_points)  // for predictive collision estimation, using the estimated
                                                          // trajectories for other moving objects
  {
    RelativeInfo info;
    int prev_index = 0;
    PlanningHelpers::GetRelativeInfoLimited(roll_out, trajectory_point, info, prev_index);

    double actual_lateral_distance = fabs(info.perp_distance) - 0.05;  // add small distance so this never become zero
    double actual_longitudinal_distance =
        info.from_back_distance + roll_out.at(info.iBack).cost - 0.05;  // add small distance so this never become zero
    double t_diff = fabs(info.perp_point.timeCost - trajectory_point.timeCost);

    if (actual_longitudinal_distance > params.pathDensity &&
        actual_longitudinal_distance < params.minFollowingDistance && actual_lateral_distance < g_lateral_skip_value &&
        !info.bAfter && !info.bBefore && t_diff < eval_params_.collision_time_)
    {
      trajectory_costs[rollout_index].longitudinal_cost += 1.0 / actual_longitudinal_distance;

      // collision point
      if (actual_lateral_distance < critical_lateral_distance && t_diff < eval_params_.collision_time_)
      {
        trajectory_costs[rollout_index].lateral_cost +=
            2.0;  // use half meter fixed critical distance as contact cost for all collision points in the range
        collision_points.push_back(info.perp_point);
        if (actual_longitudinal_distance < params.minFollowingDistance)
        {
          trajectory_costs[rollout_index].bBlocked = true;
        }

        if (trajectory_costs[rollout_index].closest_obj_distance > actual_longitudinal_distance)
        {
          trajectory_costs[rollout_index].closest_obj_distance = actual_longitudinal_distance;
          trajectory_costs[rollout_index].closest_obj_velocity = trajectory_point.v;
        }
      }
      else
      {
        trajectory_costs[rollout_index].lateral_cost += 1.0 / actual_lateral_distance;
      }
    }
  }
}

void TrajectoryEvaluator::GetYieldROIs(const WayPoint &stopWp, const RelativeInfo& stopLineInfoGlobal, TRAFFIC_SIGN_TYPE& sign_type)
{

    std::vector<double> x_origin_offsets;
    std::vector<double> y_origin_offsets;
    std::vector<double> x_offsets;
    std::vector<double> y_offsets;
    std::vector<double> upper_bound_angle;
    std::vector<double> lower_bound_angle;

    switch(sign_type)
      {
      case 3:
        // RIGHT
        x_origin_offsets.push_back(0);
        y_origin_offsets.push_back(5);
        x_offsets.push_back(15);
        y_offsets.push_back(5);
        upper_bound_angle.push_back(-60);
        lower_bound_angle.push_back(-150);
        break;
      case 4:
        // LEFT
        x_origin_offsets.push_back(0);
        y_origin_offsets.push_back(-5);
        x_offsets.push_back(15);
        y_offsets.push_back(-5);
        upper_bound_angle.push_back(60);
        lower_bound_angle.push_back(150);
        break;
      case 5:
        // RIGHT LEFT
        x_origin_offsets.push_back(0);
        y_origin_offsets.push_back(5);
        x_offsets.push_back(15);
        y_offsets.push_back(5);
        upper_bound_angle.push_back(-60);
        lower_bound_angle.push_back(-150);
        // -----------------------------------
        x_origin_offsets.push_back(0);
        y_origin_offsets.push_back(-5);
        x_offsets.push_back(15);
        y_offsets.push_back(-5);
        upper_bound_angle.push_back(60);
        lower_bound_angle.push_back(150);
        break;
      case 6:
        // FORWARD
        x_origin_offsets.push_back(0);
        y_origin_offsets.push_back(0);
        x_offsets.push_back(0);
        y_offsets.push_back(15);
        upper_bound_angle.push_back(30);
        lower_bound_angle.push_back(-30);
        break;
      case 7:
        // FORWARD LEFT
        x_origin_offsets.push_back(0);
        y_origin_offsets.push_back(0);
        x_offsets.push_back(0);
        y_offsets.push_back(15);
        upper_bound_angle.push_back(60);
        lower_bound_angle.push_back(-30);
        //------------------------------------
        x_origin_offsets.push_back(0);
        y_origin_offsets.push_back(-5);
        x_offsets.push_back(15);
        y_offsets.push_back(-5);
        upper_bound_angle.push_back(60);
        lower_bound_angle.push_back(150);
        break;
      case 8:
        // FORWARD RIGHT
        x_origin_offsets.push_back(0);
        y_origin_offsets.push_back(0);
        x_offsets.push_back(0);
        y_offsets.push_back(15);
        upper_bound_angle.push_back(30);
        lower_bound_angle.push_back(-60);
        //------------------------------------
        x_origin_offsets.push_back(0);
        y_origin_offsets.push_back(5);
        x_offsets.push_back(15);
        y_offsets.push_back(5);
        upper_bound_angle.push_back(-60);
        lower_bound_angle.push_back(-150);
        break;
      default:
        return;
      }


    for (int i = 0; i<x_offsets.size(); i++)
    {
      std::vector<WayPoint> curr_roi;
      WayPoint p1, p2, p3, p4;
      Mat3 invRotationMat, invTranslationMat;

      invRotationMat = Mat3(stopWp.pos.a);
      invTranslationMat = Mat3(stopWp.pos.x, stopWp.pos.y);
      
      p1.pos.x += x_origin_offsets.at(i);
      p1.pos.y += y_origin_offsets.at(i);
      p1.pos = invRotationMat * p1.pos;
      p1.pos = invTranslationMat* p1.pos;

      p2.pos.x = x_offsets.at(i);
      p2.pos.y = y_offsets.at(i);
      p2.pos = invRotationMat * p2.pos;
      p2.pos = invTranslationMat* p2.pos;

      p3.pos.x = 200 * cos(DEG2RAD*upper_bound_angle.at(i));
      p3.pos.y = 200 * sin(DEG2RAD*upper_bound_angle.at(i));
      p3.pos = invRotationMat * p3.pos;
      p3.pos = invTranslationMat * p3.pos;

      p4.pos.x = 200 * cos(DEG2RAD*lower_bound_angle.at(i));
      p4.pos.y = 200 * sin(DEG2RAD*lower_bound_angle.at(i));
      p4.pos = invRotationMat * p4.pos;
      p4.pos = invTranslationMat * p4.pos;

      curr_roi.push_back(p1);
      curr_roi.push_back(p2);
      curr_roi.push_back(p3);
      curr_roi.push_back(p4);
      curr_roi.push_back(p1);

      attention_rois.push_back(curr_roi);
      curr_roi.clear();
    }

}

// 
/**
 * @brief 
 * Evaluate rollout for predictive yielding
 * 
 * @param curr_state Current vehicle position
 * @param params Planning params
 * @param trajectory_costs Output, trajectories costs
 * @param trajectory_points List of all the predicted trajectory waypoints from all rollouts
 * @param obs_wrt_trajectory_points Respective obstacle of each trajectory point.
 * @param rollOut roll_out to be evaluated
 * @param rollout_index index of current rollout in roll_outs list
 * @param critical_lateral_distance Lateral safety distance
 * @param collision_points Points where a colision will happen.
 * @param critical_long_front_distance Longitudinal safety distance
 * @param carInfoRollout Ego's relative info w.r.t rollout
 * @param carInfoGlobal Ego's relative info w.r.t center rollout
 * @param stopLineInfoGlobal Closest stop line relative info w.r.t center rollout
 * @param stopWp Closest stop line WayPoint
 * @param distanceToClosestStopLine Distance to closest stopline
 */
void TrajectoryEvaluator::EvaluateRolloutForPredictiveYielding(
    const WayPoint& curr_state,
    const PlanningParams& params, 
    std::vector<TrajectoryCost>& trajectory_costs, 
    const std::vector<WayPoint>& trajectory_points,
    const std::vector<WayPoint>& obs_wrt_trajectory_points,
    const std::vector<WayPoint>& roll_out, 
    const int rollout_index, 
    const double& critical_lateral_distance, 
    std::vector<WayPoint>& collision_points, 
    const RelativeInfo& carInfoRollout, 
    const RelativeInfo& carInfoGlobal,
    const RelativeInfo& stopLineInfoGlobal,
    const WayPoint &stopWp,
    const double& distanceToClosestStopLine)
{

  double obstacleLongitudinalDistWithEgo;
  double obstacleLongitudinalDistWithStopLine;
  double trajectoryPointLongitudinalDistWithEgo;
  double trajectoryPointLongitudinalDistStopLine;
  double nearest_collision_point = DBL_MAX;
  bool skip_this_obstacle = false;

  for (int j = 0; j<filtered_obs_traj_pair_.size(); j++)
  {
      int prev_index = 0;
      skip_this_obstacle = false;
      bool mockup_trajectories = false;
      RelativeInfo obstacle_info;
      WayPoint obs;

      obs.pos = filtered_obs_.at(j).center.pos;
      obs.pos.a = filtered_obs_.at(j).center.pos.a;
      obs.v = filtered_obs_.at(j).center.v;
      obs.id = filtered_obs_.at(j).id;
      obs.laneId = filtered_obs_.at(j).predTrajectories.at(0).at(0).laneId;
      obs.width = filtered_obs_traj_pair_.at(j).at(0).width;

      PlanningHelpers::GetRelativeInfoLimited(roll_out, obs, obstacle_info, prev_index);

      obstacleLongitudinalDistWithEgo =
          PlanningHelpers::GetExactDistanceOnTrajectory(roll_out, carInfoGlobal, obstacle_info);
      obstacleLongitudinalDistWithStopLine =
          PlanningHelpers::GetExactDistanceOnTrajectory(roll_out, stopLineInfoGlobal, obstacle_info);

      // If ROIs are available then check if obstacle is in any ROI
      if (attention_rois.size()>0)
      {
        int in_roi = 0;
        for (int k = 0; k < attention_rois.size(); k++)
        {
          in_roi += PlanningHelpers::PointInsidePolygon(attention_rois.at(k), obs);    
        }
        if (in_roi == 0)
        {
          // Skip obstacle is outside ROI
          continue;
        }
      }
        

      // Check if any obstacle is ahead of us and behind the yielding stop line
      // and is following the same rollout as we are.
      if(obstacleLongitudinalDistWithEgo > 0 
        && obstacleLongitudinalDistWithStopLine <=0 
        && fabs(obstacle_info.angle_diff) < 15 
        && fabs(obstacle_info.perp_distance) < 3)
      {
        // Disable predictive block as we are are gonna follow the vehicle ahead of us
        // trajectory_costs[rollout_index].predictive_blocked= false;
        // TODO: VERIFY THIS
        // break;

        // std::cout<<"Skipped OBJECT AHEAD of us, BEHIND stopline, SAME LANE"<< obstacle_info.perp_distance<<std::endl;
        
        continue;
      }
      // Check if obstacle is ahead of us and the yielding stop line
      // and is following the same rollout as we are.
      else if (obstacleLongitudinalDistWithEgo > 0 
        && obstacleLongitudinalDistWithStopLine > 0 
        && fabs(obstacle_info.angle_diff < 15) 
        && fabs(obstacle_info.perp_distance) < 3)
      {
        // Skip processing trajectory point of this obstacle
        // std::cout<<"Skipped OBJECT AHEAD of us, AHEAD stopline, SAME LANE"<< obstacle_info.perp_distance<<std::endl;
        continue;
      }
      // Check if obstacle is behind us 
      // and is following the same rollout as we are.
      else if (obstacleLongitudinalDistWithEgo < 0 
        && fabs(obstacle_info.angle_diff < 15) 
        && fabs(obstacle_info.perp_distance) < 3)
      {
        // std::cout<<"Skipped OBJECT BEHIND us, SAME LANE"<< obstacle_info.perp_distance<<std::endl;
        // Skip processing trajectory point of this obstacle
        continue;
      }

      double lastLongitudinalCost = 0;
      double lastLateralCost = 0;
      double lastNearestCollisionPoint = 0;
      bool predictiveBlocked = false;

      // Check if obstacle has near zero velocity. Maybe there is a traffic conjestion.
      // We generate zero obstacle rollouts for this car and do collision estimation with them.

      // std::cout<<"obs.v "<<obs.v<<std::endl;
      if (obs.v < 5)
      {
        if(obs.v < 1)
        {
          mockup_trajectories = true;
        }
        
        PlannerH planner;
        // Lane speed limit goes in max speed
        double max_speed = 25;
        double min_prediction_distance = 2;
        double d = (21 * max_speed) + filtered_obs_.at(j).l;

        if(d > 2)
        {
          min_prediction_distance = d;
        }

        obs.v = max_speed;
        filtered_obs_.at(j).center.v = max_speed;
        filtered_obs_traj_pair_.at(j).clear();
        std::vector<WayPoint*> pClosestWaypoints = MappingHelpers::GetClosestWaypointsListFromMap(filtered_obs_.at(j).center, m_Map, 2, true);

        filtered_obs_.at(j).predTrajectories.clear();
        planner.PredictTrajectoriesUsingDP(filtered_obs_.at(j).center, pClosestWaypoints, 60, filtered_obs_.at(j).predTrajectories, false, filtered_obs_.at(j).bDirection, 0.5);

        for(unsigned int t = 0; t < filtered_obs_.at(j).predTrajectories.size(); t ++)
        {
          if(filtered_obs_.at(j).predTrajectories.at(t).size() > 0)
          {
            filtered_obs_.at(j).predTrajectories.at(t).at(0).collisionCost = 0;
          }
        }
        
        for(unsigned int k=0; k < filtered_obs_.at(j).predTrajectories.size(); k++)
        {
          PlannerHNS::PlanningHelpers::PredictConstantTimeCostForTrajectory(filtered_obs_.at(j).predTrajectories.at(k), filtered_obs_.at(j).center, -1, min_prediction_distance);
          for(auto& updated_point: filtered_obs_.at(j).predTrajectories.at(k))
          {
            filtered_obs_traj_pair_.at(j).push_back(updated_point);
          }
        }
      }

  
      for (int i = 0; i < filtered_obs_traj_pair_.at(j).size() - 1; i++)
      {
        int prev_index = 0;
        RelativeInfo trajectoryPoint_info;

        PlanningHelpers::GetRelativeInfoLimited(roll_out, filtered_obs_traj_pair_.at(j).at(i), trajectoryPoint_info, prev_index);

        double actual_lateral_distance = fabs(trajectoryPoint_info.perp_distance) - 0.05;  // add small distance so this never become zero
        double actual_longitudinal_distance = trajectoryPoint_info.from_back_distance + roll_out.at(trajectoryPoint_info.iBack).cost - 0.05;  // add small distance so this never become zero

        // Get the longitudinal distances on the rollout
        trajectoryPointLongitudinalDistWithEgo =
            PlanningHelpers::GetExactDistanceOnTrajectory(roll_out, carInfoGlobal, trajectoryPoint_info);

        trajectoryPointLongitudinalDistStopLine =
          PlanningHelpers::GetExactDistanceOnTrajectory(roll_out, stopLineInfoGlobal, trajectoryPoint_info);

        // Check if trajectory point is behind the yielding stop line 
        // and is following the same rollout as we are.
        if (trajectoryPointLongitudinalDistStopLine <= 0.5 
          && fabs(trajectoryPoint_info.angle_diff < 15) 
          && fabs(trajectoryPoint_info.perp_distance) < 3)
        {
        // std::cout<<"Skipped TRAJECTORY POINT BEHIND stopLine, SAME LANE"<< obstacle_info.perp_distance<<std::endl;
        // std::cout<<"-----------------------------------------"<<std::endl;
          skip_this_obstacle = true;
          break;
        }


        // Time to reach difference between nearest rollout point and current predicted trajectory point
        double t_diff = fabs(trajectoryPoint_info.perp_point.timeCost - filtered_obs_traj_pair_.at(j).at(i).timeCost);
        
        if (actual_lateral_distance < g_lateral_skip_value 
            && !trajectoryPoint_info.bAfter 
            && !trajectoryPoint_info.bBefore)
        {
          // Increase rollout cost if there is intersection of rollout and predicted trajectories
          // trajectory_costs[rollout_index].longitudinal_cost += 1.0 / actual_longitudinal_distance;

          lastLongitudinalCost += 1.0 / actual_longitudinal_distance;

          // If trajectory point distance is less than critcal lateral distance and TTC (time to collision) is less than collision time buffer
          // then mark the current rollout as predictive blocked
          if (actual_lateral_distance < critical_lateral_distance && t_diff < eval_params_.collision_time_ )
          {
            // trajectory_costs[rollout_index].lateral_cost +=2.0;  // use half meter fixed critical distance as contact cost for all collision points in the range
            // trajectory_costs[rollout_index].predictive_blocked= true;

            if(mockup_trajectories && !trajectory_costs[rollout_index].bBlocked)
            {
              // Skip mockups 
               continue;
            }

            lastLateralCost += 2.0;
            predictiveBlocked = true;

            if (lastNearestCollisionPoint < trajectoryPointLongitudinalDistWithEgo)
            {
              // nearest_collision_point = trajectoryPointLongitudinalDistWithEgo;
              lastNearestCollisionPoint = trajectoryPointLongitudinalDistWithEgo;
            }
          }
          else
          {
            // trajectory_costs[rollout_index].lateral_cost += 1.0 / actual_lateral_distance;
            lastLateralCost += actual_lateral_distance;
          }
        }
      }

      if(mockup_trajectories)
      {
        mockup_trajectories = false;
      }

      if (skip_this_obstacle)
      {
        skip_this_obstacle = false;
        continue;
      }
      else
      {
       
        trajectory_costs[rollout_index].longitudinal_cost = lastLongitudinalCost;
        trajectory_costs[rollout_index].lateral_cost = lastLateralCost;
        if (predictiveBlocked)
        {
          objects_attention.push_back(filtered_obs_.at(j));
          trajectory_costs[rollout_index].predictive_blocked = predictiveBlocked;
          nearest_collision_point = lastNearestCollisionPoint;
        }
      }
      predictiveBlocked = false;
    }
}

}  // namespace PlannerHNS
