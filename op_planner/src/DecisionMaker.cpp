
/// \file DecisionMaker.cpp
/// \brief Initialize behaviors state machine, and calculate required parameters for the state machine transition
/// conditions \author Hatem Darweesh \date Dec 14, 2016

#include "op_planner/DecisionMaker.h"
#include "op_planner/PlanningHelpers.h"
#include "op_planner/MappingHelpers.h"
#include "op_planner/MatrixOperations.h"

namespace PlannerHNS
{
DecisionMaker::DecisionMaker()
{
  m_CurrGlobalId = -1;
  m_iCurrentTotalPathId = 0;
  // pLane = nullptr;
  m_pCurrentBehaviorState = nullptr;
  m_pGoToGoalState = nullptr;
  m_pWaitState = nullptr;
  m_pMissionCompleteState = nullptr;
  m_pAvoidObstacleState = nullptr;
  m_pTrafficLightStopState = nullptr;
  m_pTrafficLightWaitState = nullptr;
  m_pStopSignStopState = nullptr;
  m_pStopSignWaitState = nullptr;
  m_pFollowState = nullptr;
  m_pMissionCompleteState = nullptr;
  m_pGoalState = nullptr;
  m_pGoToGoalState = nullptr;
  m_pWaitState = nullptr;
  m_pInitState = nullptr;
  m_pFollowState = nullptr;
  m_pAvoidObstacleState = nullptr;
  m_pStopState = nullptr;
  m_bRequestNewGlobalPlan = false;
  m_bUseInternalACC = false;
}

DecisionMaker::~DecisionMaker()
{
  delete m_pStopState;
  delete m_pMissionCompleteState;
  delete m_pGoalState;
  delete m_pGoToGoalState;
  delete m_pWaitState;
  delete m_pInitState;
  delete m_pFollowState;
  delete m_pAvoidObstacleState;
  delete m_pTrafficLightStopState;
  delete m_pTrafficLightWaitState;
  delete m_pStopSignWaitState;
  delete m_pStopSignStopState;
}

void DecisionMaker::Init(const ControllerParams& ctrlParams, const PlannerHNS::PlanningParams& params,
                         const CAR_BASIC_INFO& carInfo)
{
  m_CarInfo = carInfo;
  m_ControlParams = ctrlParams;
  m_params = params;
  m_original_params = params;

  m_VelocityController.Init(m_ControlParams, m_CarInfo, true);

  m_pidVelocity.Init(0.01, 0.004, 0.01);
  m_pidVelocity.Setlimit(m_params.maxSpeed, 0);

  m_pidStopping.Init(0.05, 0.05, 0.1);
  //  m_pidStopping.Setlimit(m_params.horizonDistance, 0);

  m_pidFollowing.Init(0.05, 0.05, 0.01);
  m_pidFollowing.Setlimit(m_params.minFollowingDistance, 0);

  nh.getParam("/ssc_interface/acceleration_limit", m_acceleration_limit);
  nh.getParam("/ssc_interface/deceleration_limit", m_deceleration_limit);

  setAcceleration(1.0);
  setDeceleration(1.0);

  InitBehaviorStates();

  if (m_pCurrentBehaviorState) {
    m_bWindowReached = false;
    m_pCurrentBehaviorState->SetBehaviorsParams(&m_params);
    PreCalculatedConditions* pValues = m_pCurrentBehaviorState->GetCalcParams();
    pValues->minStoppingDistance = m_params.horizonDistance;
    pValues->iCentralTrajectory = m_params.rollOutNumber / 2;
    pValues->iPrevSafeTrajectory = pValues->iCentralTrajectory;
    pValues->iCurrSafeLane = 0;
    pValues->stoppingDistances.clear();
    pValues->stoppingDistances.push_back(m_params.horizonDistance);
    pValues->currentVelocity = 0;
    pValues->bTrafficIsRed = false;
    pValues->currentTrafficLightID = -1;
    pValues->currentStopSignID = -1;
    pValues->bFullyBlock = false;
    pValues->bFinalLocalTrajectory = false;
    pValues->distanceToNext = m_params.horizonDistance;
    pValues->velocityOfNext = 0;
    pValues->distanceToGoal = m_params.horizonDistance;
    pValues->currentGoalID = -1;
    pValues->prevGoalID = -1;
  }
}

void DecisionMaker::UpdateAvoidanceParams(bool enable_swerve, int roll_out_numbers)
{
  if (!enable_swerve && enable_swerve != m_params.enableSwerving) {
    m_pCurrentBehaviorState->GetCalcParams()->bRePlan = true;
  }

  m_params.enableSwerving = enable_swerve;
  m_params.rollOutNumber = roll_out_numbers;
}

void DecisionMaker::InitBehaviorStates()
{
  m_pStopState = new StopStateII(&m_params, nullptr, nullptr);
  m_pMissionCompleteState = new MissionAccomplishedStateII(m_pStopState->m_pParams, m_pStopState->GetCalcParams(), 0);
  m_pGoalState = new GoalStateII(m_pStopState->m_pParams, m_pStopState->GetCalcParams(), m_pMissionCompleteState);
  m_pGoToGoalState = new ForwardStateII(m_pStopState->m_pParams, m_pStopState->GetCalcParams(), m_pGoalState);
  m_pInitState = new InitStateII(m_pStopState->m_pParams, m_pStopState->GetCalcParams(), m_pGoToGoalState);

  m_pFollowState = new FollowStateII(m_pStopState->m_pParams, m_pStopState->GetCalcParams(), m_pGoToGoalState);
  m_pAvoidObstacleState = new SwerveStateII(m_pStopState->m_pParams, m_pStopState->GetCalcParams(), m_pGoToGoalState);
  m_pStopSignWaitState =
      new StopSignWaitStateII(m_pStopState->m_pParams, m_pStopState->GetCalcParams(), m_pGoToGoalState);
  m_pStopSignStopState =
      new StopSignStopStateII(m_pStopState->m_pParams, m_pStopState->GetCalcParams(), m_pStopSignWaitState);

  m_pTrafficLightWaitState =
      new TrafficLightWaitStateII(m_pStopState->m_pParams, m_pStopState->GetCalcParams(), m_pGoToGoalState);
  m_pTrafficLightStopState =
      new TrafficLightStopStateII(m_pStopState->m_pParams, m_pStopState->GetCalcParams(), m_pGoToGoalState);

  m_pStopState->InsertNextState(m_pGoToGoalState);
  m_pStopState->InsertNextState(m_pGoalState);
  m_pStopState->InsertNextState(m_pFollowState);
  m_pStopState->decisionMakingCount = 0;

  m_pGoToGoalState->InsertNextState(m_pAvoidObstacleState);
  m_pGoToGoalState->InsertNextState(m_pStopSignStopState);
  m_pGoToGoalState->InsertNextState(m_pTrafficLightStopState);
  m_pGoToGoalState->InsertNextState(m_pFollowState);
  m_pGoToGoalState->InsertNextState(m_pStopState);
  m_pGoToGoalState->decisionMakingCount = 0;  // m_params.nReliableCount;

  m_pGoalState->InsertNextState(m_pGoToGoalState);

  m_pGoalState->InsertNextState(m_pMissionCompleteState);
  m_pMissionCompleteState->InsertNextState(m_pFollowState);

  m_pStopSignStopState->InsertNextState(m_pStopSignWaitState);
  m_pStopSignStopState->InsertNextState(m_pFollowState);

  m_pStopSignWaitState->decisionMakingTime = m_params.stopSignStopTime;
  m_pStopSignWaitState->InsertNextState(m_pStopSignStopState);
  m_pStopSignWaitState->InsertNextState(m_pGoalState);

  m_pTrafficLightStopState->InsertNextState(m_pTrafficLightWaitState);
  m_pTrafficLightStopState->InsertNextState(m_pFollowState);

  m_pTrafficLightWaitState->InsertNextState(m_pTrafficLightStopState);
  m_pTrafficLightWaitState->InsertNextState(m_pGoalState);

  m_pFollowState->InsertNextState(m_pAvoidObstacleState);
  m_pFollowState->InsertNextState(m_pStopSignStopState);
  m_pFollowState->InsertNextState(m_pTrafficLightStopState);
  m_pFollowState->InsertNextState(m_pGoalState);
  m_pFollowState->InsertNextState(m_pStopState);
  m_pFollowState->decisionMakingCount = 0;  // m_params.nReliableCount;

  m_pInitState->decisionMakingCount = 0;  // m_params.nReliableCount;

  m_pCurrentBehaviorState = m_pInitState;

  m_bWindowReached = false;
}

bool DecisionMaker::GetNextTrafficLight(const int& prevTrafficLightId,
                                        const std::vector<PlannerHNS::TrafficLight>& trafficLights,
                                        PlannerHNS::TrafficLight& trafficL)
{
  for (const auto& trafficLight : trafficLights) {
    double d = hypot(trafficLight.pose.pos.y - state.pos.y, trafficLight.pose.pos.x - state.pos.x);
    if (d <= trafficLight.stoppingDistance) {
      double a_diff = UtilityHNS::UtilityH::AngleBetweenTwoAnglesPositive(
          UtilityHNS::UtilityH::FixNegativeAngle(trafficLight.pose.pos.a),
          UtilityHNS::UtilityH::FixNegativeAngle(state.pos.a));

      if (a_diff < M_PI_2 && trafficLight.id != prevTrafficLightId) {
        // std::cout << "Detected Light, ID = " << trafficLights.at(i).id << ", Distance = " << d << ", Angle = " <<
        // trafficLights.at(i).pos.a*RAD2DEG << ", Car Heading = " << state.pos.a*RAD2DEG << ", Diff = " <<
        // a_diff*RAD2DEG << std::endl;
        trafficL = trafficLight;
        return true;
      }
    }
  }

  return false;
}

void DecisionMaker::CalculateImportantParameterForDecisionMaking(const PlannerHNS::VehicleState& car_state,
                                                                 const bool& bEmergencyStop,
                                                                 const std::vector<TrafficLight>& detectedLights,
                                                                 const TrajectoryCost& bestTrajectory)
{
  if (m_TotalPaths.empty()) {
    return;
  }

  PreCalculatedConditions* pValues = m_pCurrentBehaviorState->GetCalcParams();

  if (m_CarInfo.max_deceleration != 0) {
    pValues->minStoppingDistance = -pow(car_state.speed, 2)/(m_CarInfo.max_deceleration * 2.0);
  } else {
    pValues->minStoppingDistance = m_params.horizonDistance;
  }

  // Stopline list, containing all the potential stopping points ahead of us.
  pValues->stoppingDistances.clear();
  pValues->stoppingDistances.push_back(m_params.horizonDistance);

  pValues->currentVelocity = car_state.speed;
  pValues->bTrafficIsRed = false;
  pValues->currentTrafficLightID = -1;
  pValues->bFullyBlock = false;
  pValues->bFinalLocalTrajectory = false;

  pValues->distanceToNext = bestTrajectory.closest_obj_distance;
  pValues->velocityOfNext = bestTrajectory.closest_obj_velocity;

  /**
   * Global Lanes section, set global path index and ID
   */
  // if(bestTrajectory.lane_index >= 0 && m_bRequestNewGlobalPlan == false)
  if (bestTrajectory.lane_index >= 0 && bestTrajectory.lane_index < m_TotalPaths.size()) {
    pValues->iCurrSafeLane = bestTrajectory.lane_index;
  } else {
    PlannerHNS::RelativeInfo info;
    PlannerHNS::PlanningHelpers::GetRelativeInfoRange(
        m_TotalPaths, state, m_params.rollOutDensity * m_params.rollOutNumber / 2.0 + 0.1, info);
    pValues->iCurrSafeLane = info.iGlobalPath;
  }

  m_iCurrentTotalPathId = pValues->iCurrSafeLane;

  for (unsigned int ig = 0; ig < m_TotalPaths.size(); ig++) {
    if (ig == m_iCurrentTotalPathId && !m_TotalPaths.at(ig).empty()) {
      m_CurrGlobalId = m_TotalPaths.at(ig).at(0).gid;
    }
  }

  /**
   * ---------------------------------------------------------------------
   */

  /**
   * Local Trajectory section, set local trajectory index
   */

  if (m_LanesRollOuts.at(m_iCurrentTotalPathId).size() <= 1) {
    m_params.rollOutNumber = 0;
    m_params.enableSwerving = false;
  } else {
    m_params.rollOutNumber = static_cast<int>(m_LanesRollOuts.at(m_iCurrentTotalPathId).size() - 1);
    m_params.enableSwerving = m_original_params.enableSwerving;
  }

  pValues->iCentralTrajectory = m_pCurrentBehaviorState->m_pParams->rollOutNumber / 2;

  if (pValues->iPrevSafeTrajectory < 0) {
    pValues->iPrevSafeTrajectory = pValues->iCentralTrajectory;
  }

  if (bestTrajectory.index >= 0 && bestTrajectory.index < (int)m_LanesRollOuts.at(m_iCurrentTotalPathId).size()) {
    pValues->iCurrSafeTrajectory = bestTrajectory.index;
  } else {
    pValues->iCurrSafeTrajectory = pValues->iCentralTrajectory;
  }
  /**
   * --------------------------------------------------------------------
   */

  pValues->bFullyBlock = bestTrajectory.bBlocked;

  double critical_long_front_distance = m_params.additionalBrakingDistance + m_params.verticalSafetyDistance;

  pValues->distanceToGoal =
      PlannerHNS::PlanningHelpers::GetDistanceFromPoseToEnd(state, m_TotalPaths.at(pValues->iCurrSafeLane)) - critical_long_front_distance;

  if ((pValues->distanceToGoal < -m_params.goalDiscoveryDistance) ||
      (pValues->distanceToGoal > m_params.horizonDistance)) {
    pValues->distanceToGoal = m_params.horizonDistance;
  }

  // We add the goal to the stopline list.
  pValues->stoppingDistances.push_back(pValues->distanceToGoal);

  if (pValues->distanceToGoal < m_params.goalDiscoveryDistance) {
    pValues->currentGoalID = -1;
  } else {
    pValues->currentGoalID = 1;
    pValues->prevGoalID = 1;
  }

  int stopLineID = -1;
  int stopSignID = -1;
  int trafficLightID = -1;
  double distanceToClosestStopLine;
  bool bGreenTrafficLight = true;

  distanceToClosestStopLine = PlanningHelpers::GetDistanceToClosestStopLineAndCheck(
                                  m_TotalPaths.at(pValues->iCurrSafeLane), state, m_params.giveUpDistance,
                                  stopLineID,stopSignID, trafficLightID) - m_params.verticalSafetyDistance;

  // Stopline within horizon
  if (stopLineID != -1) {
    pValues->stoppingDistances.push_back(distanceToClosestStopLine);
  }

  // We take the closest stop line as stopping point.
  double distance_to_stop = *std::min_element(pValues->stoppingDistances.begin(), pValues->stoppingDistances.end());

  // Calculate ego car velocity for STOPPING state
  if (distance_to_stop < 0) {
    pValues->egoStoppingVelocity = 0;
  }
  else {
    pValues->egoStoppingVelocity = sqrt(2 * abs(m_params.stopping_deceleration) * std::max(distance_to_stop - critical_long_front_distance, 0.));
  }

  // clip ego STOPPING velocity between 0 and maxSpeed
  pValues->egoStoppingVelocity = std::min(std::max(pValues->egoStoppingVelocity, 0.0), m_params.maxSpeed);

  // Calculate ego car velocity for FOLLOW state
  if (!pValues->bFullyBlock)
  {
    pValues->egoFollowingVelocity = m_params.maxSpeed;
  }
  else
  {
    // clip small speeds of object
    double objectVelocity = 0.0;
    if (pValues->velocityOfNext > 2)
      objectVelocity = pValues->velocityOfNext;

    // object speed dependent - in higher speeds keeps bigger distance
    double normalDistance =
        pValues->distanceToNext - m_params.follow_reaction_time * objectVelocity - m_params.additionalBrakingDistance;

    double under_sqrt = objectVelocity * objectVelocity - 2 * m_params.follow_deceleration * normalDistance;
    if (under_sqrt > 0)
      pValues->egoFollowingVelocity = sqrt(under_sqrt);
    else
      pValues->egoFollowingVelocity = 0.0;
  }
  // clip ego FOLLOW velocity between 0 and maxSpeed
  pValues->egoFollowingVelocity = std::min(std::max(pValues->egoFollowingVelocity, 0.0), m_params.maxSpeed);

  if(distanceToClosestStopLine > m_params.giveUpDistance && pValues->currentVelocity > pValues->egoStoppingVelocity){
    m_bWindowReached = true;
  }

  if (distanceToClosestStopLine <= m_params.giveUpDistance) {
    m_bWindowReached = false;
  }

  if (m_bWindowReached) {
    if (m_pCurrentBehaviorState->m_pParams->enableTrafficLightBehavior) {
      pValues->currentTrafficLightID = trafficLightID;
      for (const auto& detectedLight : detectedLights) {
        if (detectedLight.id == trafficLightID) {
          bGreenTrafficLight = (detectedLight.lightType == GREEN_LIGHT);
        }
      }
    }

//    std::cout << "enableStopSignBehavior : " << m_pCurrentBehaviorState->m_pParams->enableStopSignBehavior;
//    std::cout << ", enableTrafficLightBehavior : " << m_pCurrentBehaviorState->m_pParams->enableTrafficLightBehavior;


    if (m_pCurrentBehaviorState->m_pParams->enableStopSignBehavior ||
        m_pCurrentBehaviorState->m_pParams->enableTrafficLightBehavior) {
      pValues->currentStopSignID = stopSignID;
      pValues->stoppingDistances.push_back(distanceToClosestStopLine);

      // std::cout << "LP => D: " << pValues->distanceToStop() << ", PrevSignID: " << pValues->prevTrafficLightID << ",
      // CurrSignID: " << pValues->currentTrafficLightID << ", Green: " << bGreenTrafficLight << std::endl;
    }
  }

  // std::cout << "Distance To Closest: " << distanceToClosestStopLine << ", Stop LineID: " << stopLineID << ", Stop
  // SignID: " << stopSignID << ", TFID: " << trafficLightID << std::endl;

  pValues->bTrafficIsRed = !bGreenTrafficLight;

  if (bEmergencyStop) {
    pValues->bFullyBlock = true;
    pValues->distanceToNext = 1;
    pValues->velocityOfNext = 0;
  }

  if (!m_Path.empty() && !m_TotalOriginalPaths.empty()) {
    double d_between_ends = hypot(m_TotalOriginalPaths.at(m_iCurrentTotalPathId).back().pos.y - m_Path.back().pos.y,
                                  m_TotalOriginalPaths.at(m_iCurrentTotalPathId).back().pos.x - m_Path.back().pos.x);
    if (d_between_ends < m_params.pathDensity) {
      pValues->bFinalLocalTrajectory = true;
    }
  }

  if (m_TotalPaths.size() > 1) {
    for (unsigned int i = 0; i < m_TotalPaths.size(); i++) {
      RelativeInfo curr_total_path_inf;
      int dummy_index = 0;
      PlanningHelpers::GetRelativeInfo(m_TotalPaths.at(i), state, curr_total_path_inf, dummy_index);
      pValues->distanceToChangeLane = m_TotalPaths.at(i).back().cost - curr_total_path_inf.perp_point.cost;
      if ((pValues->distanceToChangeLane < m_params.microPlanDistance * 0.75) ||
          (fabs(curr_total_path_inf.perp_distance) < 1.0 && m_iCurrentTotalPathId == i)) {
        m_bRequestNewGlobalPlan = true;
      }
    }
  }

  // 	if(m_RollOuts.size() > 2)
  // 	{
  // 		std::cout << "From Decision Maker, RollIndex: " << bestTrajectory.index << ", SafeTraj: " <<
  // pValues->iCurrSafeTrajectory << ", PrevTraj: " <<pValues->iPrevSafeTrajectory << ", Blocked: " <<
  // bestTrajectory.bBlocked
  // 		<< ", dtoNext:" <<  pValues->distanceToNext << ", dtoAvoid: " << m_params.minDistanceToAvoid << std::endl;
  // 	}
}

bool DecisionMaker::ReachEndOfGlobalPath(const double& min_distance, const int& iGlobalPathIndex)
{
  if (m_TotalPaths.empty()) {
    return false;
  }

  PlannerHNS::RelativeInfo info;
  PlanningHelpers::GetRelativeInfo(m_TotalPaths.at(iGlobalPathIndex), state, info);

  double d = 0;
  for (unsigned int i = info.iFront; i < m_TotalPaths.at(iGlobalPathIndex).size() - 1; i++) {
    d += hypot(m_TotalPaths.at(iGlobalPathIndex).at(i + 1).pos.y - m_TotalPaths.at(iGlobalPathIndex).at(i).pos.y,
               m_TotalPaths.at(iGlobalPathIndex).at(i + 1).pos.x - m_TotalPaths.at(iGlobalPathIndex).at(i).pos.x);
    if (d > min_distance) {
      return false;
    }
  }

  return true;
}

void DecisionMaker::SetNewGlobalPath(const std::vector<std::vector<WayPoint> >& globalPath)
{
  if (m_pCurrentBehaviorState) {
    m_pCurrentBehaviorState->GetCalcParams()->bNewGlobalPath = true;
    m_bRequestNewGlobalPlan = false;
    m_TotalOriginalPaths = globalPath;
    m_prev_index.clear();
    for (unsigned int i = 0; i < globalPath.size(); i++) {
      m_prev_index.push_back(0);
    }
  }
}

bool DecisionMaker::SelectSafeTrajectory()
{
  bool bNewTrajectory = false;
  PlannerHNS::PreCalculatedConditions* preCalcPrams = m_pCurrentBehaviorState->GetCalcParams();

  if (!preCalcPrams || m_LanesRollOuts.at(m_iCurrentTotalPathId).empty()) {
    return bNewTrajectory;
  }

  int currIndex = PlannerHNS::PlanningHelpers::GetClosestNextPointIndexFast(m_Path, state);
  int index_limit = static_cast<int>(m_Path.size() / 2. + 1.);

  if ((currIndex > index_limit || preCalcPrams->bRePlan || preCalcPrams->bNewGlobalPath) &&
      !preCalcPrams->bFinalLocalTrajectory) {
    // std::cout << "New Local Plan !! " << currIndex << ", "<< preCalcPrams->bRePlan << ", " <<
    // preCalcPrams->bNewGlobalPath  << ", " <<  m_TotalPath.at(0).size() << ", PrevLocal: " << m_Path.size();

    m_Path = m_LanesRollOuts.at(m_iCurrentTotalPathId).at(preCalcPrams->iCurrSafeTrajectory);
    // std::cout << ", NewLocal: " << m_Path.size() << std::endl;

    preCalcPrams->bNewGlobalPath = false;
    preCalcPrams->bRePlan = false;
    bNewTrajectory = true;
  }

  return bNewTrajectory;
}

double DecisionMaker::UpdateVelocityDirectlyToTrajectorySmooth(BehaviorState& beh, const VehicleState& CurrStatus,
                                                               const double& dt)
{
  PlannerHNS::PreCalculatedConditions* preCalcPrams = m_pCurrentBehaviorState->GetCalcParams();

  if (!preCalcPrams || m_TotalPaths.empty()) {
    return 0;
  }

  BehaviorState beh_with_max = beh;

  RelativeInfo total_info;
  PlanningHelpers::GetRelativeInfo(m_TotalPaths.at(m_iCurrentTotalPathId), state, total_info);
  beh_with_max.maxVelocity = PlannerHNS::PlanningHelpers::GetVelocityAhead(
      m_TotalPaths.at(m_iCurrentTotalPathId), total_info, total_info.iBack,
      preCalcPrams->minStoppingDistance * m_ControlParams.curveSlowDownRatio);
  if (beh_with_max.maxVelocity > m_params.maxSpeed) {
    beh_with_max.maxVelocity = m_params.maxSpeed;
  }

  VehicleState desired_state = m_VelocityController.DoOneStep(dt, beh_with_max, CurrStatus);

  for (WayPoint& wayPoint : m_Path) {
    wayPoint.v = desired_state.speed;
  }

  return desired_state.speed;
}

double DecisionMaker::UpdateVelocityDirectlyToTrajectory(const BehaviorState& beh, const VehicleState& CurrStatus,
                                                         const double& dt)
{
  PlannerHNS::PreCalculatedConditions* preCalcPrams = m_pCurrentBehaviorState->GetCalcParams();

  if (!preCalcPrams || m_TotalPaths.empty()) {
    return 0;
  }

  RelativeInfo info, total_info;
  PlanningHelpers::GetRelativeInfo(m_TotalPaths.at(m_iCurrentTotalPathId), state, total_info);
  PlanningHelpers::GetRelativeInfo(m_Path, state, info);

  std::stringstream log_stream;
  double speed_change_distance = -pow(CurrStatus.speed,2) / (2 * m_params.speed_deceleration);
  double desired_velocity = 0.;
  double max_velocity = std::min(PlannerHNS::PlanningHelpers::GetVelocityAheadLinear(
      m_TotalPaths.at(m_iCurrentTotalPathId), total_info, total_info.iBack, speed_change_distance, CurrStatus.speed,
      m_params.speed_deceleration), m_params.maxSpeed);

  if (beh.state == STOPPING_STATE || beh.state == TRAFFIC_LIGHT_STOP_STATE || beh.state == STOP_SIGN_STOP_STATE) {
    double min_dist = pow(CurrStatus.speed, 2) / (4.0 * 2);

    if (beh.stopDistance <= min_dist && m_params.enableQuickStop) {
      setDeceleration(0.0);
      desired_velocity = 0.0;
    } else {
      setDeceleration(5.0);
      desired_velocity = beh.egoStoppingVelocity;
    }
    return clipTargetVelocityAndWriteToPath(desired_velocity, max_velocity);

  } else if (beh.state == FOLLOW_STATE) {
    double min_dist = pow(CurrStatus.speed - beh.followVelocity, 2)/(4.0 * 2);
    if (beh.followDistance <= min_dist && m_params.enableQuickStop) {
      setDeceleration(0.0);
      desired_velocity = 0.0;
    }
    // HACK
    // First cycles (4-7) when object is detected its velocity is 0.0. It means a standing obstacle and will result
    // in quite hard braking. Actual standing obstacle has very small speeds, so will not fall here.
    // So this else if is to bypass sudden braking when entering follow mode.
    else if (beh.followVelocity == 0.0) {
      setDeceleration(5.0);
      desired_velocity = CurrStatus.speed;
    } else {
      setDeceleration(5.0);

      // if we are closer than 2m to a object in front always send target_velocity 0.0
      if (-2.0 < beh.followDistance && beh.followDistance < 2.0) {
        desired_velocity = 0.0;
      } else {
        desired_velocity = beh.egoFollowingVelocity;
      }
    }
    return clipTargetVelocityAndWriteToPath(desired_velocity, max_velocity);

  } else if (beh.state == FORWARD_STATE || beh.state == OBSTACLE_AVOIDANCE_STATE) {
    setDeceleration(1.0);
    desired_velocity = max_velocity;
    return clipTargetVelocityAndWriteToPath(desired_velocity, max_velocity);

  } else if (beh.state == STOP_SIGN_WAIT_STATE || beh.state == TRAFFIC_LIGHT_WAIT_STATE) {
    desired_velocity = 0.;
    return clipTargetVelocityAndWriteToPath(desired_velocity, max_velocity);

  } else if (beh.state == FINISH_STATE) {
    desired_velocity = 0.;
    return clipTargetVelocityAndWriteToPath(desired_velocity, max_velocity);

  } else {
    desired_velocity = 0.;
    return clipTargetVelocityAndWriteToPath(desired_velocity, max_velocity);
  }
}

double DecisionMaker::clipTargetVelocityAndWriteToPath(double target_velocity, double max_velocity)
{
  // clip target_velocity between 0 and max_velocity
  target_velocity = std::min(std::max(target_velocity, 0.0), max_velocity);

  // adjust small speeds, don't output smaller than m_params.low_speed_upper_lim=2.0
  if (target_velocity <= m_params.low_speed_lower_lim) {
    target_velocity = 0;
  } else if (m_params.low_speed_lower_lim < target_velocity && target_velocity < m_params.low_speed_upper_lim) {
    target_velocity = m_params.low_speed_upper_lim;
  }

  // write speeds to path
  for (WayPoint& wayPoint : m_Path) {
    wayPoint.v = target_velocity;
  }

  return target_velocity;
}

void DecisionMaker::setAcceleration(double acceleration_limit)
{
  if (acceleration_limit != m_acceleration_limit) {
    m_acceleration_limit = acceleration_limit;
    nh.setParam("/ssc_interface/acceleration_limit", acceleration_limit);
  }
}

void DecisionMaker::setDeceleration(double deceleration_limit)
{
  if (deceleration_limit != m_deceleration_limit) {
    m_deceleration_limit = deceleration_limit;
    nh.setParam("/ssc_interface/deceleration_limit", deceleration_limit);
  }
}

/* namespace PlannerHNS */

PlannerHNS::BehaviorState DecisionMaker::GenerateBehaviorState(const PlannerHNS::VehicleState& vehicleState)
{
  PlannerHNS::PreCalculatedConditions* preCalcPrams = m_pCurrentBehaviorState->GetCalcParams();

  m_pCurrentBehaviorState = m_pCurrentBehaviorState->GetNextState();
  if (m_pCurrentBehaviorState == nullptr) {
    m_pCurrentBehaviorState = m_pInitState;
  }

  PlannerHNS::BehaviorState currentBehavior;

  currentBehavior.state = m_pCurrentBehaviorState->m_Behavior;
  currentBehavior.followDistance = preCalcPrams->distanceToNext;

  currentBehavior.minVelocity = 0;
  currentBehavior.stopDistance = preCalcPrams->distanceToStop();
  currentBehavior.followVelocity = preCalcPrams->velocityOfNext;
  currentBehavior.egoStoppingVelocity = preCalcPrams->egoStoppingVelocity;
  currentBehavior.egoFollowingVelocity = preCalcPrams->egoFollowingVelocity;

  if (preCalcPrams->iPrevSafeTrajectory < 0 ||
      preCalcPrams->iPrevSafeTrajectory >= m_LanesRollOuts.at(m_iCurrentTotalPathId).size()) {
    currentBehavior.iTrajectory = preCalcPrams->iCurrSafeTrajectory;
  } else {
    currentBehavior.iTrajectory = preCalcPrams->iPrevSafeTrajectory;
  }

  currentBehavior.iLane = m_iCurrentTotalPathId;

  double indication_distance = preCalcPrams->minStoppingDistance;
  if (indication_distance < m_params.minIndicationDistance) {
    indication_distance = m_params.minIndicationDistance;
  }

  currentBehavior.indicator = PlanningHelpers::GetIndicatorsFromPath(m_Path, state, indication_distance);
  if (currentBehavior.state == GOAL_STATE || currentBehavior.state == FINISH_STATE || m_params.maxSpeed == 0) {
    currentBehavior.indicator = INDICATOR_BOTH;
  }

  return currentBehavior;
}

PlannerHNS::BehaviorState DecisionMaker::DoOneStep(const double& dt, const PlannerHNS::WayPoint currPose,
                                                   const PlannerHNS::VehicleState& vehicleState,
                                                   const std::vector<TrafficLight>& trafficLight,
                                                   const TrajectoryCost& tc, const bool& bEmergencyStop)
{
  PlannerHNS::BehaviorState beh;
  state = currPose;
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
        m_TotalOriginalPaths.at(i), state, m_params.horizonDistance, m_params.pathDensity, t_centerTrajectorySmoothed,
        m_prev_index.at(i));

    if (m_prev_index.at(i) > 0) {
      m_prev_index.at(i) = m_prev_index.at(i) - 1;
    }

    m_TotalPaths.push_back(t_centerTrajectorySmoothed);
  }

  if (m_TotalPaths.empty()) {
    return beh;
  }

  // UpdateCurrentLane(m_params.maxLaneSearchDistance);

  CalculateImportantParameterForDecisionMaking(vehicleState, bEmergencyStop, trafficLight, tc);

  beh = GenerateBehaviorState(vehicleState);

  beh.bNewPlan = SelectSafeTrajectory();

  if (m_bUseInternalACC) {
    beh.maxVelocity = UpdateVelocityDirectlyToTrajectorySmooth(beh, vehicleState, dt);
  } else {
    beh.maxVelocity = UpdateVelocityDirectlyToTrajectory(beh, vehicleState, dt);
  }

  return beh;
}

}  // namespace PlannerHNS
