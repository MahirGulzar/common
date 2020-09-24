
/// \file DecisionMaker.cpp
/// \brief Initialize behaviors state machine, and calculate required parameters for the state machine transition conditions
/// \author Hatem Darweesh
/// \date Dec 14, 2016

#include "op_planner/DecisionMaker.h"
#include "op_utility/UtilityH.h"
#include "op_planner/PlanningHelpers.h"
#include "op_planner/MappingHelpers.h"
#include "op_planner/MatrixOperations.h"


namespace PlannerHNS
{

DecisionMaker::DecisionMaker()
{
	m_iCurrentTotalPathId = 0;
	pLane = 0;
	m_pCurrentBehaviorState = 0;
	m_pGoToGoalState = 0;
	m_pStopState= 0;
	m_pWaitState= 0;
	m_pMissionCompleteState= 0;
	m_pAvoidObstacleState = 0;
	m_pTrafficLightStopState = 0;
	m_pTrafficLightWaitState = 0;
	m_pStopSignStopState = 0;
	m_pStopSignWaitState = 0;
	m_pFollowState = 0;
	m_MaxLaneSearchDistance = 3.0;
	m_pStopState = 0;
	m_pMissionCompleteState = 0;
	m_pGoalState = 0;
	m_pGoToGoalState = 0;
	m_pWaitState = 0;
	m_pInitState = 0;
	m_pFollowState = 0;
	m_pAvoidObstacleState = 0;
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

void DecisionMaker::Init(const ControllerParams& ctrlParams, const PlannerHNS::PlanningParams& params,const CAR_BASIC_INFO& carInfo)
 	{
 		m_CarInfo = carInfo;
 		m_ControlParams = ctrlParams;
 		m_params = params;
        m_dSpeedDistanceRatio = 0.4;

        nh.getParam("/ssc_interface/acceleration_limit", m_acceleration_limit);
        nh.getParam("/ssc_interface/deceleration_limit", m_deceleration_limit);


        setAcceleration(1.0);
        setDeceleration(1.0);

        // m_pidVelocity.Init(0.01, 0.004, 0.01);
 		m_pidVelocity.Init(0.4, 0.0002, 0.02);
		m_pidVelocity.Setlimit(m_params.maxSpeed, 0);

        //m_pidStopping.Init(0.05, 0.05, 0.1);
		m_pidStopping.Init(m_dSpeedDistanceRatio, 0, 0);
		// m_pidStopping.Setlimit(m_params.horizonDistance, 0);

		m_pidFollowing.Init(0.05, 0.05, 0.01);
		m_pidFollowing.Setlimit(m_params.minFollowingDistance, 0);

		InitBehaviorStates();

		if(m_pCurrentBehaviorState)
			m_pCurrentBehaviorState->SetBehaviorsParams(&m_params);
 	}

void DecisionMaker::InitBehaviorStates()
{

	m_pStopState 				= new StopState(&m_params, 0, 0);
	m_pMissionCompleteState 	= new MissionAccomplishedStateII(m_pStopState->m_pParams, m_pStopState->GetCalcParams(), 0);
	m_pGoalState				= new GoalStateII(m_pStopState->m_pParams, m_pStopState->GetCalcParams(), m_pMissionCompleteState);
	m_pGoToGoalState 			= new ForwardStateII(m_pStopState->m_pParams, m_pStopState->GetCalcParams(), m_pGoalState);
	m_pInitState 				= new InitStateII(m_pStopState->m_pParams, m_pStopState->GetCalcParams(), m_pGoToGoalState);

	m_pFollowState				= new FollowStateII(m_pStopState->m_pParams, m_pStopState->GetCalcParams(), m_pGoToGoalState);
	m_pAvoidObstacleState		= new SwerveStateII(m_pStopState->m_pParams, m_pStopState->GetCalcParams(), m_pGoToGoalState);
	m_pStopSignWaitState		= new StopSignWaitStateII(m_pStopState->m_pParams, m_pStopState->GetCalcParams(), m_pGoToGoalState);
	m_pStopSignStopState		= new StopSignStopStateII(m_pStopState->m_pParams, m_pStopState->GetCalcParams(), m_pStopSignWaitState);

	m_pTrafficLightWaitState	= new TrafficLightWaitStateII(m_pStopState->m_pParams, m_pStopState->GetCalcParams(), m_pGoToGoalState);
	m_pTrafficLightStopState	= new TrafficLightStopStateII(m_pStopState->m_pParams, m_pStopState->GetCalcParams(), m_pGoToGoalState);

	m_pGoToGoalState->InsertNextState(m_pAvoidObstacleState);
	m_pGoToGoalState->InsertNextState(m_pStopSignStopState);
	m_pGoToGoalState->InsertNextState(m_pTrafficLightStopState);
	m_pGoToGoalState->InsertNextState(m_pFollowState);
	m_pGoToGoalState->decisionMakingCount = 0;//m_params.nReliableCount;

	m_pGoalState->InsertNextState(m_pGoToGoalState);

	m_pStopSignWaitState->decisionMakingTime = m_params.stopSignStopTime;
	m_pStopSignWaitState->InsertNextState(m_pStopSignStopState);
	m_pStopSignWaitState->InsertNextState(m_pGoalState);


	m_pTrafficLightStopState->InsertNextState(m_pTrafficLightWaitState);

	m_pTrafficLightWaitState->InsertNextState(m_pTrafficLightStopState);
	m_pTrafficLightWaitState->InsertNextState(m_pGoalState);

	m_pFollowState->InsertNextState(m_pAvoidObstacleState);
	m_pFollowState->InsertNextState(m_pStopSignStopState);
	m_pFollowState->InsertNextState(m_pTrafficLightStopState);
	m_pFollowState->InsertNextState(m_pGoalState);
	m_pFollowState->decisionMakingCount = 0;//m_params.nReliableCount;

	m_pInitState->decisionMakingCount = 0;//m_params.nReliableCount;

	m_pCurrentBehaviorState = m_pInitState;

    m_bWindowReached = false;
}

 bool DecisionMaker::GetNextTrafficLight(const int& prevTrafficLightId, const std::vector<PlannerHNS::TrafficLight>& trafficLights, PlannerHNS::TrafficLight& trafficL)
 {
	 for(unsigned int i = 0; i < trafficLights.size(); i++)
	 {
		 double d = hypot(trafficLights.at(i).pos.y - state.pos.y, trafficLights.at(i).pos.x - state.pos.x);
		 if(d <= trafficLights.at(i).stoppingDistance)
		 {
			 double a_diff = UtilityHNS::UtilityH::AngleBetweenTwoAnglesPositive(UtilityHNS::UtilityH::FixNegativeAngle(trafficLights.at(i).pos.a) , UtilityHNS::UtilityH::FixNegativeAngle(state.pos.a));

			 if(a_diff < M_PI_2 && trafficLights.at(i).id != prevTrafficLightId)
			 {
				 //std::cout << "Detected Light, ID = " << trafficLights.at(i).id << ", Distance = " << d << ", Angle = " << trafficLights.at(i).pos.a*RAD2DEG << ", Car Heading = " << state.pos.a*RAD2DEG << ", Diff = " << a_diff*RAD2DEG << std::endl;
				 trafficL = trafficLights.at(i);
				 return true;
			 }
		 }
	 }

	 return false;
 }

 void DecisionMaker::CalculateImportantParameterForDecisionMaking(const PlannerHNS::VehicleState& car_state,
		 const int& goalID, const bool& bEmergencyStop, const std::vector<TrafficLight>& detectedLights,
		 const TrajectoryCost& bestTrajectory)
 {
	 if(m_TotalPath.size() == 0) return;

 	PreCalculatedConditions* pValues = m_pCurrentBehaviorState->GetCalcParams();

 	if(m_CarInfo.max_deceleration != 0)
 		pValues->minStoppingDistance = -pow(car_state.speed, 2)/(m_CarInfo.max_deceleration * 2.0);

 	pValues->iCentralTrajectory		= m_pCurrentBehaviorState->m_pParams->rollOutNumber/2;

	if(pValues->iPrevSafeTrajectory < 0)
		pValues->iPrevSafeTrajectory = pValues->iCentralTrajectory;

 	pValues->stoppingDistances.clear();
 	pValues->currentVelocity 		= car_state.speed;
 	pValues->bTrafficIsRed 			= false;
 	pValues->currentTrafficLightID 	= -1;
 	pValues->bFullyBlock 			= false;

 	pValues->distanceToNext = bestTrajectory.closest_obj_distance;
 	pValues->velocityOfNext = bestTrajectory.closest_obj_velocity;

 	if(bestTrajectory.index >=0 &&  bestTrajectory.index < (int)m_RollOuts.size())
 		pValues->iCurrSafeTrajectory = bestTrajectory.index;
 	else
 		pValues->iCurrSafeTrajectory = pValues->iCentralTrajectory;

	pValues->bFullyBlock = bestTrajectory.bBlocked;

 	if(bestTrajectory.lane_index >=0)
 		pValues->iCurrSafeLane = bestTrajectory.lane_index;
 	else
 	{
 		PlannerHNS::RelativeInfo info;
 		PlannerHNS::PlanningHelpers::GetRelativeInfoRange(m_TotalPath, state, m_params.rollOutDensity*m_params.rollOutNumber/2.0 + 0.1, info);
 		pValues->iCurrSafeLane = info.iGlobalPath;
 	}

 	double critical_long_front_distance =  m_CarInfo.wheel_base/2.0 + m_CarInfo.length/2.0 + m_params.verticalSafetyDistance;

	if(ReachEndOfGlobalPath(pValues->minStoppingDistance + critical_long_front_distance, pValues->iCurrSafeLane))
		pValues->currentGoalID = -1;
	else
		pValues->currentGoalID = goalID;

 	m_iCurrentTotalPathId = pValues->iCurrSafeLane;

 	int stopLineID = -1;
 	int stopSignID = -1;
 	int trafficLightID = -1;
 	double distanceToClosestStopLine = 0;
 	bool bGreenTrafficLight = true;

  	distanceToClosestStopLine = PlanningHelpers::GetDistanceToClosestStopLineAndCheck(m_TotalPath.at(pValues->iCurrSafeLane), state, m_params.giveUpDistance, stopLineID, stopSignID, trafficLightID) - critical_long_front_distance;

  	// std::cout << "S StopLineID: " << stopLineID << ", StopSignID: " << stopSignID << ", TrafficLightID: " << trafficLightID << ", Distance: " << distanceToClosestStopLine << ", MinStopDistance: " << pValues->minStoppingDistance << std::endl;
  	// distanceToClosestStopLine << ", " << m_params.giveUpDistance << ", " << pValues->minStoppingDistance + 1.0 << ", " << m_CarInfo.max_deceleration << std::endl;
  	// changed 1.0 to 5.0
  	// std::cout <<  distanceToClosestStopLine << ", " << (pValues->minStoppingDistance + 1.0) << std::endl;

    // calculate distanceWindow to enter stopping state
    double distanceWindow = pValues->currentVelocity * m_params.d_stop;

    // std::cout << car_state.speed << ", " << m_CarInfo.max_deceleration << ", dW: " << distanceWindow << ", " << (distanceToClosestStopLine <= distanceWindow + bufferLength) << std::endl;
    if(distanceToClosestStopLine > m_params.giveUpDistance && distanceToClosestStopLine <= distanceWindow)
    {
        m_bWindowReached = true;
	// std::cout << "in window" << std::endl;
    }

    if(distanceToClosestStopLine <= m_params.giveUpDistance) {
        m_bWindowReached = false;
    }

 	if(m_bWindowReached)
 	{
 		if(m_pCurrentBehaviorState->m_pParams->enableTrafficLightBehavior)
 		{
 			pValues->currentTrafficLightID = trafficLightID;
 			//std::cout << "Detected Traffic Light: " << trafficLightID << std::endl;
 			for(unsigned int i=0; i< detectedLights.size(); i++)
 			{
 				if(detectedLights.at(i).id == trafficLightID)
 					bGreenTrafficLight = (detectedLights.at(i).lightState == GREEN_LIGHT);
 			}
 		}

 		if(m_pCurrentBehaviorState->m_pParams->enableStopSignBehavior)
 			pValues->currentStopSignID = stopSignID;

		pValues->stoppingDistances.push_back(distanceToClosestStopLine);
		//std::cout << "LP => D: " << pValues->distanceToStop() << ", PrevSignID: " << pValues->prevTrafficLightID << ", CurrSignID: " << pValues->currentTrafficLightID << ", Green: " << bGreenTrafficLight << endl;
 	}


 	//std::cout << "Distance To Closest: " << distanceToClosestStopLine << ", Stop LineID: " << stopLineID << ", Stop SignID: " << stopSignID << ", TFID: " << trafficLightID << std::endl;

 	pValues->bTrafficIsRed = !bGreenTrafficLight;

 	if(bEmergencyStop)
	{
		pValues->bFullyBlock = true;
		pValues->distanceToNext = 1;
		pValues->velocityOfNext = 0;
	}
 	//cout << "Distances: " << pValues->stoppingDistances.size() << ", Distance To Stop : " << pValues->distanceToStop << endl;
 }

 void DecisionMaker::UpdateCurrentLane(const double& search_distance)
 {
	 PlannerHNS::Lane* pMapLane = 0;
	PlannerHNS::Lane* pPathLane = 0;
	pPathLane = MappingHelpers::GetLaneFromPath(state, m_TotalPath.at(m_iCurrentTotalPathId));
	if(!pPathLane)
	{
		std::cout << "Performance Alert: Can't Find Lane Information in Global Path, Searching the Map :( " << std::endl;
		pMapLane  = MappingHelpers::GetClosestLaneFromMap(state, m_Map, search_distance);
	}

	if(pPathLane)
		pLane = pPathLane;
	else if(pMapLane)
		pLane = pMapLane;
	else
		pLane = 0;
 }

 bool DecisionMaker::ReachEndOfGlobalPath(const double& min_distance, const int& iGlobalPathIndex)
 {
	 if(m_TotalPath.size()==0) return false;

	 PlannerHNS::RelativeInfo info;
	 PlanningHelpers::GetRelativeInfo(m_TotalPath.at(iGlobalPathIndex), state, info);

	 double d = 0;
	 for(unsigned int i = info.iFront; i < m_TotalPath.at(iGlobalPathIndex).size()-1; i++)
	 {
		 d+= hypot(m_TotalPath.at(iGlobalPathIndex).at(i+1).pos.y - m_TotalPath.at(iGlobalPathIndex).at(i).pos.y, m_TotalPath.at(iGlobalPathIndex).at(i+1).pos.x - m_TotalPath.at(iGlobalPathIndex).at(i).pos.x);
		 if(d > min_distance)
			 return false;
	 }

	 return true;
 }

 void DecisionMaker::SetNewGlobalPath(const std::vector<std::vector<WayPoint> >& globalPath)
 {
	 if(m_pCurrentBehaviorState)
	 {
		 m_pCurrentBehaviorState->GetCalcParams()->bNewGlobalPath = true;
		 m_TotalOriginalPath = globalPath;
	 }
 }

 bool DecisionMaker::SelectSafeTrajectory()
 {
	 bool bNewTrajectory = false;
	 PlannerHNS::PreCalculatedConditions *preCalcPrams = m_pCurrentBehaviorState->GetCalcParams();

	 if(!preCalcPrams || m_RollOuts.size() == 0) return bNewTrajectory;

	int currIndex = PlannerHNS::PlanningHelpers::GetClosestNextPointIndexFast(m_Path, state);
	int index_limit = 0;
	if(index_limit<=0)
		index_limit =  m_Path.size()/2.0;
	if(currIndex > index_limit
			|| preCalcPrams->bRePlan
			|| preCalcPrams->bNewGlobalPath)
	{
		std::cout << "New Local Plan !! " << currIndex << ", "<< preCalcPrams->bRePlan << ", " << preCalcPrams->bNewGlobalPath  << ", " <<  m_TotalOriginalPath.at(0).size() << ", PrevLocal: " << m_Path.size();
		m_Path = m_RollOuts.at(preCalcPrams->iCurrSafeTrajectory);
		std::cout << ", NewLocal: " << m_Path.size() << std::endl;

		preCalcPrams->bNewGlobalPath = false;
		preCalcPrams->bRePlan = false;
		bNewTrajectory = true;
	}

	return bNewTrajectory;
 }

 PlannerHNS::BehaviorState DecisionMaker::GenerateBehaviorState(const PlannerHNS::VehicleState& vehicleState)
 {
	PlannerHNS::PreCalculatedConditions *preCalcPrams = m_pCurrentBehaviorState->GetCalcParams();

    if(m_pCurrentBehaviorState != m_pCurrentBehaviorState->GetNextState()){
        m_pidVelocity.ResetI();
        m_pidVelocity.ResetD();
    }

	m_pCurrentBehaviorState = m_pCurrentBehaviorState->GetNextState();
	if(m_pCurrentBehaviorState==0)
		m_pCurrentBehaviorState = m_pInitState;

	PlannerHNS::BehaviorState currentBehavior;

	currentBehavior.state = m_pCurrentBehaviorState->m_Behavior;
	currentBehavior.followDistance = preCalcPrams->distanceToNext;

	currentBehavior.minVelocity		= 0;
	currentBehavior.stopDistance 	= preCalcPrams->distanceToStop();
	currentBehavior.followVelocity 	= preCalcPrams->velocityOfNext;
	if(preCalcPrams->iPrevSafeTrajectory<0 || preCalcPrams->iPrevSafeTrajectory >= m_RollOuts.size())
		currentBehavior.iTrajectory		= preCalcPrams->iCurrSafeTrajectory;
	else
		currentBehavior.iTrajectory		= preCalcPrams->iPrevSafeTrajectory;

	double average_braking_distance = -pow(vehicleState.speed, 2)/(m_CarInfo.max_deceleration) + m_params.additionalBrakingDistance;

	if(average_braking_distance  < m_params.minIndicationDistance)
		average_braking_distance = m_params.minIndicationDistance;

	currentBehavior.indicator = PlanningHelpers::GetIndicatorsFromPath(m_Path, state, average_braking_distance );
	return currentBehavior;
 }

 double DecisionMaker::UpdateVelocityDirectlyToTrajectory(const BehaviorState& beh, const VehicleState& CurrStatus, const double& dt)
 {
	if(m_TotalOriginalPath.size() ==0 ) return 0;

	RelativeInfo info, total_info;
	PlanningHelpers::GetRelativeInfo(m_TotalOriginalPath.at(m_iCurrentTotalPathId), state, total_info);
	PlanningHelpers::GetRelativeInfo(m_Path, state, info);


	// changed distance calculation for the speed change
    double speed_change_distance = CurrStatus.speed * m_params.d_forward;
    double max_velocity	= PlannerHNS::PlanningHelpers::GetVelocityAhead(m_TotalOriginalPath.at(m_iCurrentTotalPathId), total_info, total_info.iBack, speed_change_distance);

    unsigned int point_index = 0;
	double critical_long_front_distance = m_CarInfo.length/2.0;

	if(beh.state == TRAFFIC_LIGHT_STOP_STATE || beh.state == STOP_SIGN_STOP_STATE)
	{
		double desiredVelocity = 0.0;
		double min_dist = pow(CurrStatus.speed, 2)/(4.0 * 2);
		std::cout << "STOP_";
		if (beh.stopDistance <= min_dist && m_params.enableQuickStop)
		{
			setDeceleration(0.0);
			desiredVelocity = 0.0;
			std::cout << "EXT - ";
		}
		else 
		{
			setDeceleration(5.0);

			desiredVelocity = beh.stopDistance * m_params.k_stop;

            if(desiredVelocity <= m_params.low_speed_lower_lim)
                desiredVelocity = 0;
            else if(m_params.low_speed_lower_lim < desiredVelocity && desiredVelocity < m_params.low_speed_upper_lim)
                desiredVelocity = m_params.low_speed_upper_lim;

            desiredVelocity = std::min(std::max(desiredVelocity, 0.0), max_velocity);
			std::cout << "NRM - ";
		}


		for(unsigned int i =  0; i < m_Path.size(); i++)
			m_Path.at(i).v = desiredVelocity;

		std::cout << "stpDist: " << beh.stopDistance << ", spd: " << CurrStatus.speed << ", dV: " << desiredVelocity  << std::endl;
		return desiredVelocity;

	}
	else if(beh.state == FOLLOW_STATE)
	{
		double desiredVelocity = 0.0;
		double min_dist = pow(CurrStatus.speed - beh.followVelocity, 2)/(4.0 * 2);
		std::cout << "FOLLOW_";
		if (beh.followDistance <= min_dist && m_params.enableQuickStop)
		{
			setDeceleration(0.0);
			desiredVelocity = 0.0;
			std::cout << "EXT - ";
		}
		// HACK
		// First cycles (4-7) when object is detected its velocity is 0.0. It means a standing obstacle and will result
		// in quite hard braking. Actual standing obstacle has very small speeds, so will not fall here.
		// So this else if is to bypass sudden braking when entering follow mode.
		else if (beh.followVelocity == 0.0)
        {
		    desiredVelocity = CurrStatus.speed;
            std::cout << "HCK - ";
        }
		else
		{
			setDeceleration(5.0);

            // clip small speeds of object in front
            double objectVelocity = 0.0;
            if(beh.followVelocity >= m_params.low_speed_upper_lim)
                objectVelocity = beh.followVelocity;

            double normalDistance = beh.followDistance - m_params.d_follow * objectVelocity - m_params.additionalBrakingDistance;
            desiredVelocity = normalDistance * m_params.k_follow + objectVelocity;

            if(desiredVelocity <= m_params.low_speed_lower_lim)
                desiredVelocity = 0;
            else if(m_params.low_speed_lower_lim < desiredVelocity && desiredVelocity < m_params.low_speed_upper_lim)
                desiredVelocity = m_params.low_speed_upper_lim;

            // if we are closer than 2m always send desiredVelocity 0.0
            // TODO - lower limit? car behind in safety box example?
            if(-2.0 < beh.followDistance && beh.followDistance < 2.0)
                desiredVelocity = 0.0;
            
			desiredVelocity = std::min(std::max(desiredVelocity, 0.0), max_velocity);
			std::cout << "NRM - " << "norm_d: " << normal_dist;
		}

		std::cout << ", fD: " << beh.followDistance << ", fV: " << beh.followVelocity << ", spd: " << CurrStatus.speed << ", dV: " << desiredVelocity << ", min_d: " << min_dist <<  std::endl;

		for(unsigned int i = 0; i < m_Path.size(); i++)
			m_Path.at(i).v = desiredVelocity;

		return desiredVelocity;

	}
	else if(beh.state == FORWARD_STATE || beh.state == OBSTACLE_AVOIDANCE_STATE )
	{
        setDeceleration(1.0);

		double desiredVelocity = max_velocity;

		if(desiredVelocity < m_params.minSpeed)
		    desiredVelocity = 0;

		for(unsigned int i = 0; i < m_Path.size(); i++)
		    m_Path.at(i).v = desiredVelocity;

		// for debugging or tuning
		//std::cout << "Target Velocity: " << desiredVelocity << ", Change Slowdown: " << bSlowBecauseChange  << std::endl;
		std::cout << "FORWARD - spd: " << CurrStatus.speed << ", dV: " << desiredVelocity << ", spd_ch_d: " << speed_change_distance << std::endl;

		return desiredVelocity;

	}
	else if(beh.state == STOP_SIGN_WAIT_STATE || beh.state == TRAFFIC_LIGHT_WAIT_STATE)
	{
		double target_velocity = 0;
		for(unsigned int i = 0; i < m_Path.size(); i++)
			m_Path.at(i).v = target_velocity;

		return target_velocity;
	}
	else
	{
		double target_velocity = 0;
		for(unsigned int i = 0; i < m_Path.size(); i++)
			m_Path.at(i).v = target_velocity;

		return target_velocity;
	}

	return max_velocity;
 }

 PlannerHNS::BehaviorState DecisionMaker::DoOneStep(
		const double& dt,
		const PlannerHNS::WayPoint currPose,
		const PlannerHNS::VehicleState& vehicleState,
		const int& goalID,
		const std::vector<TrafficLight>& trafficLight,
		const TrajectoryCost& tc,
		const bool& bEmergencyStop)
{
	PlannerHNS::BehaviorState beh;
	state = currPose;
	m_TotalPath.clear();
	for(unsigned int i = 0; i < m_TotalOriginalPath.size(); i++)
	{
		t_centerTrajectorySmoothed.clear();
		PlannerHNS::PlanningHelpers::ExtractPartFromPointToDistanceDirectionFast(m_TotalOriginalPath.at(i), state, m_params.horizonDistance ,	m_params.pathDensity , t_centerTrajectorySmoothed);
		m_TotalPath.push_back(t_centerTrajectorySmoothed);
	}

	if(m_TotalPath.size()==0) {std::cout << "NOT USING DECISION MAKER\n"; return beh;}

	UpdateCurrentLane(m_MaxLaneSearchDistance);

	CalculateImportantParameterForDecisionMaking(vehicleState, goalID, bEmergencyStop, trafficLight, tc);

	beh = GenerateBehaviorState(vehicleState);

	beh.bNewPlan = SelectSafeTrajectory();

	beh.maxVelocity = UpdateVelocityDirectlyToTrajectory(beh, vehicleState, dt);

	//std::cout << "Eval_i: " << tc.index << ", Curr_i: " <<  m_pCurrentBehaviorState->GetCalcParams()->iCurrSafeTrajectory << ", Prev_i: " << m_pCurrentBehaviorState->GetCalcParams()->iPrevSafeTrajectory << std::endl;

	return beh;
 }

 void DecisionMaker::setAcceleration(double acceleration_limit) {
     if (acceleration_limit != m_acceleration_limit) {
         m_acceleration_limit = acceleration_limit;
         nh.setParam("/ssc_interface/acceleration_limit", acceleration_limit);
     }
 }

 void DecisionMaker::setDeceleration(double deceleration_limit) {
     if (deceleration_limit != m_deceleration_limit) {
         m_deceleration_limit = deceleration_limit;
         nh.setParam("/ssc_interface/deceleration_limit", deceleration_limit);
     }
 }

} /* namespace PlannerHNS */
