/// \file TrajectoryEvaluator.h
/// \brief Calculate collision costs for roll out trajectory for free trajectory evaluation for OpenPlanner local planner version 1.5+ (Dynamic Obstacles Prediction)
/// \author Hatem Darweesh
/// \date Jan 3, 2019

#ifndef TRAJECTORY_EVALUATOR_H_
#define TRAJECTORY_EVALUATOR_H_

#include "PlanningHelpers.h"
#include "MappingHelpers.h"
#include "PlannerCommonDef.h"
#include "op_planner/PlannerH.h"

namespace PlannerHNS
{

class EvaluationParams
{

public:
  double priority_weight_;
  double transition_weight_;
  double longitudinal_weight_;
  double lateral_weight_;
  double lane_change_weight_;
  double collision_time_;
  double min_prediction_time_;
  double min_prediction_distance_;

  EvaluationParams()
  {
    priority_weight_ = 0.3;
    transition_weight_ = 0.2;
    longitudinal_weight_ = 0.2;
    lateral_weight_ = 0.3;
    lane_change_weight_ = 0;
    collision_time_ = 3;
    min_prediction_time_ = 11;
    min_prediction_distance_ = 2;
  }

  EvaluationParams(double periority_w, double transition_w, double logitudinal_w, double lateral_w,
                    double lane_change_w, double collision_t)
  {
    priority_weight_ = periority_w;
    transition_weight_ = transition_w;
    longitudinal_weight_ = logitudinal_w;
    lateral_weight_ = lateral_w;
    lane_change_weight_ = lane_change_w;
    collision_time_ = collision_t;
  }
};

class TrajectoryEvaluator
{
public:
  TrajectoryEvaluator();
  virtual ~TrajectoryEvaluator();

  TrajectoryCost doOneStep(const int g_index,
                           const std::vector<std::vector<WayPoint> >& m_GlobalPaths,
                           const std::vector<std::vector<WayPoint> >& roll_outs,
                           const std::vector<std::vector<WayPoint> >& total_paths, const WayPoint& curr_state,
                           const PlanningParams& params, const CAR_BASIC_INFO& car_info,
                           const VehicleState& vehicle_state, std::vector<DetectedObject>& obj_list,
                           const RoadNetwork &map,
                           const bool& b_static_only = false,
                           const int& prev_curr_index = -1,
						   const bool& b_keep_curr = false);

  void SetEvalParams(const EvaluationParams& eval_param);
  void SetPlanningParams(const PlanningParams& planning_param);
  void normalizeCosts(const EvaluationParams& eval_param, std::vector<TrajectoryCost>& trajectory_costs);
  static bool sortCosts(const TrajectoryCost& c1, const TrajectoryCost& c2)
  {
    return c1.cost < c2.cost;
  }

public:
  std::vector<WayPoint> all_contour_points_;
  std::vector<DetectedObject> m_obj_list_;
  std::vector<DetectedObject> objects_attention;
  std::vector<std::vector<WayPoint>> attention_rois;
  std::vector<WayPoint> collision_points_;
  PolygonShape safety_border_;
  std::vector<std::vector<WayPoint> > local_roll_outs_;
  std::vector<TrajectoryCost> trajectory_costs_;
  RoadNetwork m_Map;
  PlannerHNS::PlannerH m_Planner;

private:
  EvaluationParams eval_params_;
  PlanningParams planning_params_;

private:

  void calculateTransitionCosts(std::vector<TrajectoryCost>& trajectory_costs, const int& curr_index,
                                const PlanningParams& params);

  void collectContoursAndTrajectories(const std::vector<PlannerHNS::DetectedObject>& obj_list, PolygonShape& ego_car_border,
                                      std::vector<WayPoint>& contour_points,
                                      const bool& b_static_only = false);

  int getCurrentRollOutIndex(const std::vector<WayPoint>& total_path, const WayPoint& curr_state,
                             const PlanningParams& params);

  void initializeCosts(const std::vector<std::vector<WayPoint> >& roll_outs, const PlanningParams& params,
                       std::vector<TrajectoryCost>& trajectory_costs);

  void initializeSafetyPolygon(const WayPoint& curr_state, const CAR_BASIC_INFO& car_info,
                               const VehicleState& vehicle_state, const double& c_lateral_d,
                               const double& c_long_front_d, const double& c_long_back_d, PolygonShape& car_border);

  void initializeLocalRollOuts(const WayPoint& curr_state, const CAR_BASIC_INFO& car_info, const PlanningParams& params, const double& c_long_back_d, const std::vector<std::vector<WayPoint> >& original_roll_outs, std::vector<std::vector<WayPoint> >& local_roll_outs);

  void CalcCostsAndObsOnRollouts(
    const std::vector<std::vector<WayPoint> >& m_GlobalPaths,
    const std::vector<std::vector<WayPoint> >& total_paths,
    const PlanningParams& params, 
    const double& critical_lateral_distance, 
    const double& critical_long_front_distance, 
    const std::vector<std::vector<WayPoint> >& rollOuts, 
    const std::vector<WayPoint>& obsPoints, 
    std::vector<TrajectoryCost>& trajectoryCosts, 
    const WayPoint& currPosition, 
    std::vector<WayPoint>& collision_points,
    std::vector<DetectedObject>& obj_list);

  void computeCostsAndPredictColisionsOnRollout(const PlanningParams& params, std::vector<TrajectoryCost>& trajectory_costs, const std::vector<WayPoint>& trajectory_points, const std::vector<WayPoint>& roll_out, const int rolloutIndex, const double& critical_lateral_distance, std::vector<WayPoint>& collision_points);

  void GetYieldROIs(const WayPoint &stopWp, const RelativeInfo& stopLineInfoGlobal, TRAFFIC_SIGN_TYPE& sign_type);
  
  void GetZeroObstacleRollouts(
    const std::vector<std::vector<WayPoint> >& TotalPaths,
    const WayPoint& curr_state, 
    const PlanningParams& params,
    std::vector<std::vector<std::vector<PlannerHNS::WayPoint> > >& zeroObstacleRollouts);

  
  void EvaluateRolloutForPredictiveYielding(
    const RelativeInfo& carInfoGlobal,
    const RelativeInfo& stopLineInfoGlobal,
    const std::vector<WayPoint>& roll_out,
    const PlanningParams& params,
    const WayPoint& curr_state,
    const WayPoint& stopWp,
    const double& critical_lateral_distance,
    const double& distanceToClosestStopLine,
    const size_t center_index,
    std::vector<TrajectoryCost>& trajectory_costs,
    std::vector<DetectedObject>& obj_list);

  TrajectoryCost findBestTrajectory(const PlanningParams& params, const int& prev_curr_index, const bool& b_keep_curr, std::vector<TrajectoryCost> trajectory_costs);

};

}

#endif /* TRAJECTORY_EVALUATOR_H_ */
