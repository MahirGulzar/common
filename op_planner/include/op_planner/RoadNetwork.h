
/// \file RoadNetwork.h
/// \brief Definition of OpenPlanner's data types
/// \author Hatem Darweesh
/// \date May 19, 2016


#ifndef ROADNETWORK_H_
#define ROADNETWORK_H_

#include <string>
#include <vector>
#include <sstream>
#include "op_utility/UtilityH.h"
#include "float.h"

#define OPENPLANNER_ENABLE_LOGS

namespace PlannerHNS
{

enum MAP_TARGET_PROJECTION
{
	NO_PROJ,
	UTM_PROJ,
	MGRS_PROJ
};

enum MAP_SOURCE_TYPE
{
	MAP_AUTOWARE,
	MAP_FOLDER,
	MAP_KML_FILE,
	MAP_ONE_CSV_FILE,
	MAP_LANES_CSV_FILES,
	MAP_NEW,
	MAP_OPEN_DRIVE_FILE,
	MAP_FOLDER_V2,
	MAP_LANELET_2,
	MAP_KML_FILE_NAME
};

enum MAP_SOURCE_FORMAT
{
	MAP_VECTOR,
	MAP_KML,
	MAP_OPEN_DRIVE,
	MAP_LANELET2
};

enum DIRECTION_TYPE {	FORWARD_DIR, FORWARD_LEFT_DIR, FORWARD_RIGHT_DIR,
	BACKWARD_DIR, BACKWARD_LEFT_DIR, BACKWARD_RIGHT_DIR, STANDSTILL_DIR};

enum OBSTACLE_TYPE {SIDEWALK, TREE, CAR, TRUCK, HOUSE, PEDESTRIAN, CYCLIST, GENERAL_OBSTACLE};

enum DRIVABLE_TYPE {DIRT, TARMAC, PARKINGAREA, INDOOR, GENERAL_AREA};

enum GLOBAL_STATE_TYPE {G_WAITING_STATE, G_PLANING_STATE, G_FORWARD_STATE, G_BRANCHING_STATE, G_FINISH_STATE};

enum STATE_TYPE {INITIAL_STATE = 0,
	WAITING_STATE = 1,
	FORWARD_STATE = 2,
	STOPPING_STATE = 3,
	EMERGENCY_STATE = 4,
	TRAFFIC_LIGHT_STOP_STATE = 5,
	TRAFFIC_LIGHT_WAIT_STATE = 6,
	STOP_SIGN_STOP_STATE = 7,
	STOP_SIGN_WAIT_STATE = 8,
	FOLLOW_STATE = 9,
	LANE_CHANGE_STATE = 10,
	OBSTACLE_AVOIDANCE_STATE = 11,
	GOAL_STATE = 12,
	FINISH_STATE = 13,
	YIELDING_STATE = 14,
	BRANCH_LEFT_STATE = 15,
	BRANCH_RIGHT_STATE = 16};

enum LIGHT_INDICATOR {INDICATOR_LEFT, INDICATOR_RIGHT, INDICATOR_BOTH , INDICATOR_NONE};

enum SHIFT_POS {SHIFT_POS_PP = 0x60, SHIFT_POS_RR = 0x40, SHIFT_POS_NN = 0x20,
	SHIFT_POS_DD = 0x10, SHIFT_POS_BB = 0xA0, SHIFT_POS_SS = 0x0f, SHIFT_POS_UU = 0xff };

enum ACTION_TYPE {FORWARD_ACTION, BACKWARD_ACTION, STOP_ACTION, LEFT_TURN_ACTION,
	RIGHT_TURN_ACTION, U_TURN_ACTION, SWERVE_ACTION, OVERTACK_ACTION, START_ACTION, SLOWDOWN_ACTION, CHANGE_DESTINATION, WAITING_ACTION, DESTINATION_REACHED,  UNKOWN_ACTION};

enum BEH_STATE_TYPE {BEH_FORWARD_STATE=0,BEH_STOPPING_STATE=1, BEH_BRANCH_LEFT_STATE=2, BEH_BRANCH_RIGHT_STATE=3, BEH_YIELDING_STATE=4, BEH_ACCELERATING_STATE=5, BEH_PARKING_STATE=6, BEH_UNKNOWN_STATE = 7};

enum SEGMENT_TYPE {NORMAL_ROAD_SEG, INTERSECTION_ROAD_SEG, UTURN_ROAD_SEG, EXIT_ROAD_SEG, MERGE_ROAD_SEG, HIGHWAY_ROAD_SEG};

enum RoadSegmentType {NORMAL_ROAD, INTERSECTION_ROAD, UTURN_ROAD, EXIT_ROAD, MERGE_ROAD, HIGHWAY_ROAD};

enum BOUNDARY_TYPE {NORMAL_ROAD_BOUNDARY, INTERSECTION_BOUNDARY, CROSSING_BOUNDARY, UTURN__BOUNDARY,
	EXIT_ROAD_BOUNDARY, MERGE_ROAD_BOUNDARY, HIGHWAY_BOUNDARY, PARKING_BOUNDARY, FREE_SPACE_BOUNDARY,
	VEGETATION_BOUNDARY, KEEP_OUT_BOUNDARY, BUILDING_BOUNDARY, TRAFFIC_ISLAN_BOUNDARY, WALK_WAY_BOUNDARY,
	SHARED_WALK_WAY_BOUNDARY, EXIT_BOUNDARY};

enum MARKING_TYPE {UNKNOWN_MARK, TEXT_MARK, AF_MARK, AL_MARK, AR_MARK, AFL_MARK, AFR_MARK, ALR_MARK, UTURN_MARK, NOUTURN_MARK, POLYGON_MARK};

enum LaneType{NORMAL_LANE, MERGE_LANE, EXIT_LANE, BUS_LANE, BUS_STOP_LANE, EMERGENCY_LANE};

enum CustomBehaviorType{CUSTOM_AVOIDANCE_DISABLED = 0, CUSTOM_AVOIDANCE_ENABLED = 1};

enum MARKING_COLOR{MARK_COLOR_WHITE, MARK_COLOR_YELLOW, MARK_COLOR_RED, MARK_COLOR_ORANG, MARK_COLOR_BLUE};

enum LINE_TYPE{DEFAULT_WHITE_LINE, CONTINUOUS_LINE, SEPARATION_LINE, SUPPORT_LINE, GENERAL_LINE};

enum TRAFFIC_SIGN_TYPE {UNKNOWN_SIGN, STOP_SIGN, MAX_SPEED_SIGN, MIN_SPEED_SIGN, NO_PARKING_SIGN, SCHOOL_CROSSING_SIGN};

enum TRAFFIC_LIGHT_TYPE {UNKNOWN_LIGHT=0, RED_LIGHT=1, GREEN_LIGHT=2, YELLOW_LIGHT=3, CROSS_GREEN=4, CROSS_RED=5, LEFT_GREEN=6, FORWARD_GREEN=7, RIGHT_GREEN=8, FLASH_YELLOW=9, FLASH_RED=10};

class Lane;
class TrafficLight;
class RoadSegment;
class Boundary;


class ObjTimeStamp
{
public:
	timespec tStamp;

	ObjTimeStamp()
	{
		tStamp.tv_nsec = 0;
		tStamp.tv_sec = 0;
	}
};

class GPSPoint
{
public:
	double x, y, z;
	double lat, lon, alt;
	double dir, a;

	GPSPoint()
	{
		x = y = z = 0;
		lat = lon = alt = 0;
		dir = a = 0;
	}

	GPSPoint(const double& x, const double& y, const double& z, const double& a)
	{
		this->x = x;
		this->y = y;
		this->z = z;
		this->a = a;

		lat = 0;
		lon = 0;
		alt = 0;
		dir = 0;
	}

	std::string ToString()
	{
		std::stringstream str;
		str.precision(12);
		str << "X:" << x << ", Y:" << y << ", Z:" << z << ", A:" << a << std::endl;
		str << "Lon:" << lon << ", Lat:" << lat << ", Alt:" << alt << ", Dir:" << dir << std::endl;
		return str.str();
	}
};

class RECTANGLE

{
public:
  GPSPoint bottom_left;
  GPSPoint top_right;
  double width;
  double length;
  bool bObstacle;


  inline bool PointInRect(GPSPoint p)
  {
    return p.x >= bottom_left.x && p.x <= top_right.x && p.y >= bottom_left.y && p.y <= top_right.y;
  }

  inline bool PointInsideRect(GPSPoint p)
  {
    return p.x > bottom_left.x && p.x < top_right.x && p.y > bottom_left.y && p.y < top_right.y;
  }

  inline bool HitTest(GPSPoint p)
  {
    return PointInRect(p) && bObstacle;
  }

  RECTANGLE()
  {
	  width=0;
	  length = 0;
    bObstacle = true;
  }

  virtual ~RECTANGLE(){}
};

class PolygonShape
{
public:
	std::vector<GPSPoint> points;

	inline int PointInsidePolygon(const PolygonShape& polygon,const GPSPoint& p)
	{
		int counter = 0;
		  int i;
		  double xinters;
		  GPSPoint p1,p2;
		  int N = polygon.points.size();
		  if(N <=0 ) return -1;

		  p1 = polygon.points.at(0);
		  for (i=1;i<=N;i++)
		  {
		    p2 = polygon.points.at(i % N);

		    if (p.y > MIN(p1.y,p2.y))
		    {
		      if (p.y <= MAX(p1.y,p2.y))
		      {
		        if (p.x <= MAX(p1.x,p2.x))
		        {
		          if (p1.y != p2.y)
		          {
		            xinters = (p.y-p1.y)*(p2.x-p1.x)/(p2.y-p1.y)+p1.x;
		            if (p1.x == p2.x || p.x <= xinters)
		              counter++;
		          }
		        }
		      }
		    }
		    p1 = p2;
		  }

		  if (counter % 2 == 0)
		    return 0;
		  else
		    return 1;
	}
};

class MapItem
{
public:
  int id;
  GPSPoint sp; //start point
  GPSPoint ep; // end point
  GPSPoint center;
  double c; //curvature
  double w; //width
  double l; //length
  std::string fileName; //
  std::vector<GPSPoint> polygon;


  MapItem(int ID, GPSPoint start, GPSPoint end, double curvature, double width, double length, std::string objName)
  {
    id = ID;
    sp = start;
    ep = end;
    c = curvature;
    w = width;
    l = length;
    fileName = objName;

  }

  MapItem()
  {
    id = 0; c = 0; w = 0; l = 0;
  }

  virtual ~MapItem(){}

  MapItem(const MapItem & cmi)
  {
        id = cmi.id;
        sp = cmi.sp;
        ep = cmi.ep;
        c = cmi.c;
        w = cmi.w;
        l = cmi.l;
        fileName = cmi.fileName;
  }
  MapItem &operator=(const MapItem &cmi)
  {
    this->id = cmi.id;
      this->sp = cmi.sp;
      this->ep = cmi.ep;
      this->c = cmi.c;
      this->w = cmi.w;
      this->l = cmi.l;
      this->fileName = cmi.fileName;
      return *this;
  }

  virtual int operator==(const MapItem &mi) const
    {
      return this->id == mi.id;
    }
};

class Obstacle : public MapItem
{
  public:
    OBSTACLE_TYPE t;

    Obstacle(int ID, GPSPoint start, GPSPoint end, double curvature, double width, double length,OBSTACLE_TYPE type, std::string fileName ) : MapItem(ID, start, end, curvature, width, length, fileName)
  {
      t = type;
  }
    virtual ~Obstacle()
    {
    }

    Obstacle() : MapItem()
       {
      t = SIDEWALK;
       }

    Obstacle(const Obstacle& ob) : MapItem(ob)
      {
        t = ob.t;
      }

    Obstacle& operator=(const Obstacle& ob)
      {
      this->id = ob.id;
      this->sp = ob.sp;
      this->ep = ob.ep;
      this->c = ob.c;
      this->w = ob.w;
      this->l = ob.l;
      this->t = ob.t;
      this->fileName = ob.fileName;
      return *this;
      }

      virtual int operator==(const Obstacle &ob) const
          {
            return this->id == ob.id && this->t == ob.t;
          }
};

class DrivableArea : public MapItem
{
public:
  DRIVABLE_TYPE t; // drivable area type

  DrivableArea(int ID, GPSPoint start, GPSPoint end, double curvature, double width, double length,DRIVABLE_TYPE type, std::string fileName ) : MapItem( ID, start, end, curvature, width, length, fileName)
  {
    t = type;
  }

  virtual ~DrivableArea()
  {

  }

  DrivableArea() : MapItem()
    {
      t = PARKINGAREA;
    }

  DrivableArea(const DrivableArea& da) : MapItem(da)
  {
    t = da.t;
  }

  DrivableArea& operator=(const DrivableArea& da)
  {
    this->id = da.id;
    this->sp = da.sp;
    this->ep = da.ep;
    this->c = da.c;
    this->w = da.w;
    this->l = da.l;
    this->t = da.t;
    this->fileName = da.fileName;
    return *this;
  }

  virtual int operator==(const DrivableArea &da) const
      {
        return this->id == da.id && this->t == da.t;
      }

};

class Rotation
{
public:
	double x;
	double y;
	double z;
	double w;

	Rotation()
	{
		x = 0;
		y = 0;
		z = 0;
		w = 0;
	}
};

class WayPoint
{
public:
	GPSPoint	pos;
	Rotation 	rot;
	double  	v;
	double  	cost;
	double  	timeCost;
	double  	totalReward;
	double  	collisionCost;
	double 		laneChangeCost;
	double      width;
	int 		laneId;
	int 		id;
	int 		LeftPointId;
	int 		RightPointId;
	int 		LeftLnId;
	int 		RightLnId;
	int 		stopLineID;
	CustomBehaviorType custom_type;
	DIRECTION_TYPE bDir;
	STATE_TYPE	state;
	BEH_STATE_TYPE beh_state;
	int 		iOriginalIndex;
	int boundaryId;

	Lane* pLane;
	Boundary* pBoundary;
	WayPoint* pLeft;
	WayPoint* pRight;
	std::vector<int> 	toIds;
	std::vector<int> 	fromIds;
	std::vector<WayPoint*> pFronts;
	std::vector<WayPoint*> pBacks;
	std::vector<std::pair<ACTION_TYPE, double> > actionCost;

	int			originalMapID;
	int			gid;

	WayPoint()
	{
		id = 0;
		v = 0;
		cost = 0;
		laneId = -1;
		pLane  = nullptr;
		pLeft = nullptr;
		pRight = nullptr;
		pBoundary = nullptr;
		bDir = FORWARD_DIR;
		LeftPointId = 0;
		RightPointId = 0;
		LeftLnId = 0;
		RightLnId = 0;
		timeCost = 0;
		totalReward = 0;
		collisionCost = 0;
		laneChangeCost = 0;
		stopLineID = -1;
		custom_type = CUSTOM_AVOIDANCE_ENABLED;
		state = INITIAL_STATE;
		beh_state = BEH_STOPPING_STATE;
		iOriginalIndex = 0;

		gid = 0;
		originalMapID = -1;
		boundaryId = -1;
		width = 0;
	}

	WayPoint(const double& x, const double& y, const double& z, const double& a)
	{
		pos.x = x;
		pos.y = y;
		pos.z = z;
		pos.a = a;

		id = 0;
		v = 0;
		cost = 0;
		laneId = -1;
		pLane  = nullptr;
		pLeft = nullptr;
		pRight = nullptr;
		pBoundary = nullptr;
		custom_type = CUSTOM_AVOIDANCE_ENABLED;
		bDir = FORWARD_DIR;
		LeftPointId = 0;
		RightPointId = 0;
		LeftLnId = 0;
		RightLnId = 0;
		timeCost = 0;
		totalReward = 0;
		collisionCost = 0;
		laneChangeCost = 0;
		stopLineID = -1;
		iOriginalIndex = 0;
		state = INITIAL_STATE;
		beh_state = BEH_STOPPING_STATE;

		gid = 0;
		originalMapID = -1;
		boundaryId = -1;
		width = 0;
	}
};

class RelativeInfo
{
public:
	double perp_distance;
	double to_front_distance; //negative
	double from_back_distance;
	int iFront;
	int iBack;
	int iGlobalPath;
	WayPoint perp_point;
	double angle_diff; // degrees
	bool bBefore;
	bool bAfter;
	double after_angle;

	RelativeInfo()
	{
		after_angle = 0;
		bBefore = false;
		bAfter = false;
		perp_distance = 0;
		to_front_distance = 0;
		from_back_distance = 0;
		iFront = 0;
		iBack = 0;
		iGlobalPath = 0;
		angle_diff = 0;
	}
};

class Boundary //represent wayarea in vector map
{
public:
	int id;
	int roadId;
	BOUNDARY_TYPE type;
	std::vector<WayPoint> points;
	WayPoint center;
	RoadSegment* pSegment;

	Boundary()
	{
		type = NORMAL_ROAD_BOUNDARY;
		id    = 0;
		roadId =0;
		pSegment = nullptr;
	}
};

class Curb
{
public:
	int id;
	int laneId;
	int roadId;
	double height;
	double width;
	std::vector<WayPoint> points;
	Lane* pLane;

	Curb()
	{
		width = 0;
		height = 0;
		id    = 0;
		laneId =0;
		roadId =0;
		pLane = 0;
	}
};

class Crossing
{
public:
	int id;
	int roadId;
	int boundaryId;
	std::vector<WayPoint> points; // the crossing boundary
	std::vector<int> markings;
	RoadSegment* pSegment;

	Crossing()
	{
		boundaryId = 0;
		id    = 0;
		roadId =0;
		pSegment = nullptr;
	}
};

class StopLine
{
public:
	int id;
	int laneId;
	std::vector<int> laneIds;
	int roadId;
	std::vector<int> lightIds;
	int stopSignID;
	std::vector<WayPoint> points;
	Lane* pLane;
	int linkID;

	StopLine()
	{
		id    = 0;
		laneId =0;
		roadId =0;
		pLane = 0;
		stopSignID = -1;
		linkID = 0;
	}
};

class WaitingLine
{
public:
	int id;
	int laneId;
	int roadId;
	std::vector<WayPoint> points;
	Lane* pLane;

	WaitingLine()
	{
		id    = 0;
		laneId =0;
		roadId =0;
		pLane = 0;
	}
};

class TrafficSign
{
public:
	int id;
	std::vector<int> laneIds;
	int roadId;
	int groupID;
	int stopLineID;
	double horizontal_angle;
	double vertical_angle;
	WayPoint pose;
	TRAFFIC_SIGN_TYPE signType;
	double value;
	double fromValue;
	double toValue;
	std::string strValue;
	timespec timeValue;
	timespec fromTimeValue;
	timespec toTimeValue;

	std::vector<Lane*> pLanes;
	Lane* pLane;

	std::vector<PlannerHNS::WayPoint> points; // sign shape, for future use 

	TrafficSign()
	{
		id    		= 0;
		roadId		= 0;
		groupID     = 0;
		stopLineID  = 0;
		signType  	= UNKNOWN_SIGN;
		value		= 0;
		fromValue	= 0;
		toValue		= 0;
		pLane 		= 0;
		horizontal_angle = 0;
		vertical_angle = 0;
	}
};

class TrafficLight
{
public:
	int id;
	WayPoint pose;
	TRAFFIC_LIGHT_TYPE lightType;
	double stoppingDistance;
	std::vector<int> laneIds;
	std::vector<Lane*> pLanes;
	int linkID;
	int stopLineID; // for lanelet2 matching
	int groupID;
	double horizontal_angle;
	double vertical_angle;

	TrafficLight()
	{
		groupID = 0;
		horizontal_angle = 0;
		vertical_angle = 0;
		stoppingDistance = 2;
		id 			= 0;
		lightType	= GREEN_LIGHT;
		linkID 		= 0;
		stopLineID  = 0;
	}

	bool CheckLane(const int& laneId)
	{
		for(unsigned int i=0; i < laneIds.size(); i++)
		{
			if(laneId == laneIds.at(i))
				return true;
		}
		return false;
	}
};

class Marking
{
public:
	int id;
	int laneId;
	int roadId;
	MARKING_TYPE  mark_type;
	MARKING_COLOR mark_color;
	WayPoint center;
	std::vector<WayPoint> points;
	Lane* pLane;

	Marking()
	{
		id = 0;
		laneId = 0;
		roadId = 0;
		mark_type = UNKNOWN_MARK;
		mark_color = MARK_COLOR_WHITE;
		pLane = nullptr;
	}
};

class RoadSegment
{
public:
	int id;

	SEGMENT_TYPE 	roadType;
	Boundary	boundary;
	Crossing	start_crossing;
	Crossing	finish_crossing;
	double 		avgWidth;
	std::vector<int> fromIds;
	std::vector<int> toIds;
	std::vector<Lane> Lanes;


	std::vector<RoadSegment*> fromLanes;
	std::vector<RoadSegment*> toLanes;

	RoadSegment()
	{
		id = 0;
		avgWidth = 0;
		roadType = NORMAL_ROAD_SEG;
	}


};

class Lane
{
public:
	int id;
	int roadId;
	int areaId;
	int fromAreaId;
	int toAreaId;
	std::vector<int> fromIds;
	std::vector<int> toIds;
	int num; //lane number in the road segment from left to right
	double speed;
	double length;
	double dir;
	LaneType type;
	double width;
	int lane_change;
	std::vector<WayPoint> points;
	std::vector<TrafficLight> trafficlights;
	std::vector<StopLine> stopLines;
	WaitingLine waitingLine;

	std::vector<Lane*> fromLanes;
	std::vector<Lane*> toLanes;
	Lane* pLeftLane;
	Lane* pRightLane;

	RoadSegment * pRoad;

	Lane()
	{
		lane_change = 0;
		id 		= 0;
		num		= 0;
		speed 	= 0;
		length 	= 0;
		dir		= 0;
		type 	= NORMAL_LANE;
		width 	= 0;
		pLeftLane = 0;
		pRightLane = 0;
		pRoad	= 0;
		roadId = 0;
		areaId = 0;
		fromAreaId = 0;
		toAreaId = 0;
	}

};

class Line
{
public:
	int id;
	double width;
	MARKING_COLOR color;
	LINE_TYPE type;
	int original_type;
	int roadID;
	std::vector<WayPoint> points;
	std::vector<int> left_lane_ids;
	std::vector<int> right_lane_ids;

	std::vector<Lane*> left_lanes;
	std::vector<Lane*> right_lanes;

	Line()
	{
		id = 0;
		width = 0;
		roadID = 0;
		color = MARK_COLOR_WHITE;
		type = GENERAL_LINE;
		original_type = 0;
	}
};

class RoadNetwork
{
public:
	std::vector<RoadSegment> roadSegments;
	std::vector<TrafficLight> trafficLights;
	std::vector<StopLine> stopLines;
	std::vector<Curb> curbs;
	std::vector<Boundary> boundaries;
	std::vector<Crossing> crossings;
	std::vector<Marking> markings;
	std::vector<TrafficSign> signs;
	std::vector<Line> lines;

	std::string str_proj;
	WayPoint origin;
	MAP_TARGET_PROJECTION proj;

	RoadNetwork()
	{
		proj = UTM_PROJ;
	}

	void Clear()
	{
		roadSegments.clear();
		trafficLights.clear();
		stopLines.clear();
		curbs.clear();
		boundaries.clear();
		crossings.clear();
		markings.clear();
		signs.clear();
		lines.clear();
		RoadNetwork::g_max_point_id = 0;
		RoadNetwork::g_max_lane_id = 0;
		RoadNetwork::g_max_line_id = 0;
		RoadNetwork::g_max_stop_line_id = 0;
		RoadNetwork::g_max_traffic_light_id = 0;
		RoadNetwork::g_max_traffic_sign_id = 0;
		RoadNetwork::g_max_boundary_area_id = 0;
		RoadNetwork::g_max_marking_id = 0;
		RoadNetwork::g_max_curb_id = 0;
		RoadNetwork::g_max_crossing_id = 0;
	}

	static int g_max_point_id;
	static int g_max_lane_id;
	static int g_max_line_id;
	static int g_max_stop_line_id;
	static int g_max_traffic_light_id;
	static int g_max_traffic_sign_id;
	static int g_max_boundary_area_id;
	static int g_max_marking_id;
	static int g_max_curb_id;
	static int g_max_crossing_id;
};

class VehicleState : public ObjTimeStamp
{
public:
	double speed = 0;
	double steer = 0;
	SHIFT_POS shift = SHIFT_POS_NN;
};

class BehaviorState
{
public:
	STATE_TYPE state;
	double maxVelocity;
	double minVelocity;
	double stopDistance;
	double followVelocity;
	double followDistance;
  double egoStoppingVelocity;
  double egoFollowingVelocity;
  LIGHT_INDICATOR indicator;
	bool bNewPlan;
	int iTrajectory;
	int iLane;


	BehaviorState()
	{
		state = INITIAL_STATE;
		maxVelocity = 0;
		minVelocity = 0;
		stopDistance = 0;
		followVelocity = 0;
		followDistance = 0;
    egoStoppingVelocity = 0.0;
    egoFollowingVelocity = 0.0;
    indicator  = INDICATOR_NONE;
		bNewPlan = false;
		iTrajectory = -1;
		iLane = -1;
	}

};

class DetectedObject
{
public:
	int id;
	std::string label;
	OBSTACLE_TYPE t;
	WayPoint center;
	WayPoint predicted_center;
	WayPoint noisy_center;
	STATE_TYPE predicted_behavior;
	std::vector<WayPoint> centers_list;
	std::vector<GPSPoint> contour;
	std::vector<std::vector<WayPoint> > predTrajectories;
	std::vector<WayPoint*> pClosestWaypoints;
	double w;
	double l;
	double h;
	double distance_to_center;

	double actual_speed;
	double actual_yaw;

	bool bDirection;
	bool bVelocity;
	int acceleration;

	int acceleration_desc;
	double acceleration_raw;

	LIGHT_INDICATOR indicator_state;

	int originalID;
	BEH_STATE_TYPE behavior_state;

	DetectedObject()
	{
		bDirection = false;
		bVelocity = false;
		acceleration = 0;
		acceleration_desc = 0;
		acceleration_raw = 0.0;
		id = 0;
		w = 0;
		l = 0;
		h = 0;
		t = GENERAL_OBSTACLE;
		distance_to_center = 0;
		predicted_behavior = INITIAL_STATE;
		actual_speed = 0;
		actual_yaw = 0;

		acceleration_desc = 0;
		acceleration_raw = 0.0;
		indicator_state = INDICATOR_NONE;

		originalID = -1;
		behavior_state = BEH_STOPPING_STATE;
	}

};

class PlanningParams
{
public:
	double 	maxSpeed;
	double 	minSpeed;
	double 	planningDistance;
	double 	microPlanDistance;
	double 	carTipMargin;
	double 	rollInMargin;
	double 	rollInSpeedFactor;
	double 	pathDensity;
	double 	rollOutDensity;
	int 	rollOutNumber;
	double 	horizonDistance;
	double 	minFollowingDistance; //should be bigger than Distance to follow
	double 	minDistanceToAvoid; // should be smaller than minFollowingDistance and larger than maxDistanceToAvoid
	double	maxDistanceToAvoid; // should be smaller than minDistanceToAvoid
	double 	speedProfileFactor;
	double 	smoothingDataWeight;
	double 	smoothingSmoothWeight;
	double 	smoothingToleranceError;

	double stopSignStopTime;

	double additionalBrakingDistance;
	double verticalSafetyDistance;
	double horizontalSafetyDistancel;

	double giveUpDistance;

	int nReliableCount;

	bool 	enableLaneChange;
	bool 	enableSwerving;
	bool 	enableFollowing;
	bool 	enableHeadingSmoothing;
	bool 	enableTrafficLightBehavior;
	bool 	enableStopSignBehavior;
	bool 	enableQuickStop;
	bool 	enableTimeOutAvoidance;
	double 	avoidanceTimeOut;

	bool 	enabTrajectoryVelocities;
	double minIndicationDistance;

	double maxLaneSearchDistance;
	double goalDiscoveryDistance;

	bool    enableCost;
    double  k_stop;
    double  follow_reaction_time;
	double  follow_deceleration;
	double  stopping_deceleration;
	double  d_forward;
	double  speed_deceleration;
	double  low_speed_upper_lim;
	double  low_speed_lower_lim;


	PlanningParams()
	{
		maxSpeed = 3;
		minSpeed = 0;
		planningDistance = 10000;
		microPlanDistance = 30;
		carTipMargin = 4.0;
		rollInMargin = 12.0;
		rollInSpeedFactor = 0.25;
		pathDensity = 0.25;
		rollOutDensity = 0.5;
		rollOutNumber = 4;
		horizonDistance = 120;
		minFollowingDistance = 35;
		minDistanceToAvoid = 15;
		maxDistanceToAvoid = 5;
		speedProfileFactor = 1.0;
		smoothingDataWeight = 0.47;
		smoothingSmoothWeight = 0.2;
		smoothingToleranceError = 0.05;

		stopSignStopTime = 2.0;

		additionalBrakingDistance = 10.0;
		verticalSafetyDistance = 0.0;
		horizontalSafetyDistancel = 0.0;

		giveUpDistance = -4;
		nReliableCount = 2;

        enableCost                      = true;
		k_stop                          = 0.32;
        follow_reaction_time            = 1.5;
        follow_deceleration             = -1.0;
        stopping_deceleration           = -1.0;
        speed_deceleration              = -1.0;
        d_forward                       = 3.0;
		low_speed_upper_lim             = 2.0;
		low_speed_lower_lim             = 1.0;

		enableHeadingSmoothing = false;
		enableSwerving = false;
		enableFollowing = false;
		enableTrafficLightBehavior = false;
		enableLaneChange = false;
		enableStopSignBehavior = false;
		enableQuickStop = false;
		enabTrajectoryVelocities = false;
		minIndicationDistance = 15;

		enableTimeOutAvoidance = false;
		avoidanceTimeOut = 250; //seconds
		maxLaneSearchDistance = 3.0;
		goalDiscoveryDistance = 2.5;
	}
};

class PreCalculatedConditions
{
public:
	//-------------------------------------------//
	//Global Goals
	int 				currentGoalID;
	int 				prevGoalID;
	//-------------------------------------------//
	//Following
	double 				distanceToNext;
	double				velocityOfNext;
    double              egoFollowingVelocity;
  //-------------------------------------------//
	//For Lane Change
	int 				iPrevSafeLane;
	int 				iCurrSafeLane;
	double				distanceToGoBack;
	double 				timeToGoBack;
	double 				distanceToChangeLane;
	double				timeToChangeLane;
	int 				currentLaneID;
	int 				originalLaneID;
	int 				targetLaneID;
	bool 				bUpcomingLeft;
	bool 				bUpcomingRight;
	bool				bCanChangeLane;
	bool				bTargetLaneSafe;
	//-------------------------------------------//
	//Traffic Lights & Stop Sign
  double      egoStoppingVelocity;
	int 				currentStopSignID;
	int 				prevStopSignID;
	int 				currentTrafficLightID;
	int 				prevTrafficLightID;
	bool 				bTrafficIsRed; //On , off status
	//-------------------------------------------//
	//Swerving
	int 				iPrevSafeTrajectory;
	int 				iCurrSafeTrajectory;
	int 				iCentralTrajectory;
	bool				bFullyBlock;
	LIGHT_INDICATOR 	indicator;

	//-------------------------------------------//
	//General
	bool 				bFinalLocalTrajectory; //the local trajectory that reaches the goal
	bool 				bNewGlobalPath;
	bool 				bRePlan;
	double 				currentVelocity;
	double				minStoppingDistance; //comfortably
	int 				bOutsideControl; // 0 waiting, 1 start, 2 Green Traffic Light, 3 Red Traffic Light, 5 Emergency Stop
	bool				bGreenOutsideControl;
	std::vector<double> stoppingDistances;

	double 				distanceToGoal;


	double distanceToStop()
	{
		if(stoppingDistances.size()==0) return 0;
		double minS = stoppingDistances.at(0);
		for(unsigned int i=0; i< stoppingDistances.size(); i++)
		{
			if(stoppingDistances.at(i) < minS)
			{
				minS = stoppingDistances.at(i);
			}
		}
		return minS;
	}

	PreCalculatedConditions()
	{
		currentGoalID = 0;
		prevGoalID = -1;
		currentVelocity	= 0;
		minStoppingDistance = 1;
		bOutsideControl	= 0;
		bGreenOutsideControl = false;
		//distance to stop
		distanceToNext = -1;
		velocityOfNext = 0;
        egoFollowingVelocity = 0.0;
        egoStoppingVelocity = 0.0;
        currentStopSignID = -1;
		prevStopSignID = -1;
		currentTrafficLightID = -1;
		prevTrafficLightID = -1;
		bTrafficIsRed = false;
		iCurrSafeTrajectory = -1;
		bFullyBlock = false;

		iPrevSafeTrajectory = -1;
		iCentralTrajectory = -1;
		bRePlan	= false;
		bNewGlobalPath = false;

		bCanChangeLane = false;
		distanceToGoBack = 0;
		timeToGoBack = 0;
		distanceToChangeLane = 0;
		timeToChangeLane = 0;
		bTargetLaneSafe = true;
		bUpcomingLeft = false;
		bUpcomingRight = false;
		targetLaneID = -1;
		currentLaneID = -1;
		originalLaneID = -1;
		iCurrSafeLane = -1;
		iPrevSafeLane = 0;

		indicator = INDICATOR_NONE;

		distanceToGoal = DBL_MAX;
		bFinalLocalTrajectory = false;
	}

	virtual ~PreCalculatedConditions(){}

	std::string ToStringHeader()
	{
		return "Time:General>>:currentVelocity:distanceToStop:minStoppingDistance:bStartBehaviorGenerator:bGoalReached:"
				"Following>>:velocityOfNext:distanceToNext:"
				"TrafficLight>>:currentTrafficLightID:bTrafficIsRed:"
				"Swerving>>:iSafeTrajectory:bFullyBlock:";
	}

	std::string ToString(STATE_TYPE beh)
	{
		std::string str = "Unknown";
		switch(beh)
		{
		case PlannerHNS::INITIAL_STATE:
			str = "Init";
			break;
		case PlannerHNS::WAITING_STATE:
			str = "Waiting";
			break;
		case PlannerHNS::FORWARD_STATE:
			str = "Forward";
			break;
		case PlannerHNS::STOPPING_STATE:
			str = "Stop";
			break;
		case PlannerHNS::FINISH_STATE:
			str = "End";
			break;
		case PlannerHNS::FOLLOW_STATE:
			str = "Follow";
			break;
		case PlannerHNS::OBSTACLE_AVOIDANCE_STATE:
			str = "Swerving";
			break;
		case PlannerHNS::TRAFFIC_LIGHT_STOP_STATE:
			str = "Light Stop";
			break;
		case PlannerHNS::TRAFFIC_LIGHT_WAIT_STATE:
			str = "Light Wait";
			break;
		case PlannerHNS::STOP_SIGN_STOP_STATE:
			str = "Sign Stop";
			break;
		case PlannerHNS::STOP_SIGN_WAIT_STATE:
			str = "Sign Wait";
			break;
		default:
			str = "Unknown";
			break;
		}

		return str;
	}
};

class TrajectoryCost
{
public:
	int index;
	int relative_index;
	double closest_obj_velocity;
	double distance_from_center;
	double priority_cost; //0 to 1
	double transition_cost; // 0 to 1
	double closest_obj_cost; // 0 to 1
	double cost;
	double closest_obj_distance;

	int lane_index;
	double lane_change_cost;
	double lateral_cost;
	double longitudinal_cost;
	bool bBlocked;
	std::vector<std::pair<int, double> > lateral_costs;


	TrajectoryCost()
	{
		lane_index = -1;
		index = -1;
		relative_index = -100;
		closest_obj_velocity = 0;
		priority_cost = 0;
		transition_cost = 0;
		closest_obj_cost = 0;
		distance_from_center = 0;
		cost = 0;
		closest_obj_distance = -1;
		lane_change_cost = 0;
		lateral_cost = 0;
		longitudinal_cost = 0;
		bBlocked = false;
	}

	std::string ToString()
	{
		std::ostringstream str;
		str.precision(4);
		str << "LI   : " << lane_index;
		str << ", In : " << relative_index;
		str << ", Co : " << cost;
		str << ", Pr : " << priority_cost;
		str << ", Tr : " << transition_cost;
		str << ", La : " << lateral_cost;
		str << ", Lo : " << longitudinal_cost;
		str << ", Ln : " << lane_change_cost;
		str << ", Bl : " << bBlocked;
		str << ", D : " << closest_obj_distance;
		str << "\n";
		for (unsigned int i=0; i<lateral_costs.size(); i++ )
		{
			str << " - (" << lateral_costs.at(i).first << ", " << lateral_costs.at(i).second << ")";
		}

		return str.str();

	}
};

class OccupancyToGridMap
{
public:
	int width;
	int length;
	double res;
	WayPoint center;

	OccupancyToGridMap(const int& _width, const int& _length, const double& _res, const WayPoint& _center)
	{
		width = _width;
		length = _length;
		res = _res;
		center = _center;
	}

	OccupancyToGridMap()
	{
		width = 0;
		length  = 0;
		res = 0.0;
	}

	bool GetCellIndexFromPoint(const GPSPoint& p, const std::vector<int>& data, int& _cell)
	{
		int col = floor(p.x / res);
		int row = floor(p.y / res);

		int index = -1;
		if(row >= 0 && row < length && col >=0 && col < width)
		{
			index = get2dIndex(row,col);

			if(index >= 0 && index < (int)data.size())
			{
				_cell = data.at((unsigned int)index);
				//printf("Cell Info: P(%f,%f) , D(%f,%f), G(%d,%d), index = %d \n", p.x, p.y, p.x-center.pos.x, p.y-center.pos.y, col, row , index);
				return true;
			}
		}

		if(row+1 >= 0 && row+1 < length && col >=0 && col < width)
		{
			index = get2dIndex(row+1,col);
			if(index >= 0 && index < (int)data.size())
			{
				_cell = data.at((unsigned int)index);
				return true;
			}
		}

		if(row >= 0 && row < length && col+1 >=0 && col+1 < width)
		{
			index = get2dIndex(row,col+1);
			if(index >= 0 && index < (int)data.size())
			{
				_cell = data.at((unsigned int)index);
				return true;
			}
		}

		if(row-1 >= 0 && row-1 < length && col >=0 && col < width)
		{
			index = get2dIndex(row-1,col);
			if(index >= 0 && index < (int)data.size())
			{
				_cell = data.at((unsigned int)index);
				return true;
			}
		}

		if(row >= 0 && row < length && col-1 >=0 && col-1 < width)
		{
			index = get2dIndex(row,col-1);
			if(index >= 0 && index < (int)data.size())
			{
				_cell = data.at((unsigned int)index);
				return true;
			}
		}

		if(row+1 >= 0 && row+1 < length && col+1 >=0 && col+1 < width)
		{
			index = get2dIndex(row+1,col+1);
			if(index >= 0 && index < (int)data.size())
			{
				_cell = data.at((unsigned int)index);
				return true;
			}
		}

		if(row-1 >= 0 && row-1 < length && col-1 >=0 && col-1 < width)
		{
			index = get2dIndex(row-1,col-1);
			if(index >= 0 && index < (int)data.size())
			{
				_cell = data.at((unsigned int)index);
				return true;
			}
		}

		if(row-1 >= 0 && row-1 < length && col+1 >=0 && col+1 < width)
		{
			index = get2dIndex(row-1,col+1);
			if(index >= 0 && index < (int)data.size())
			{
				_cell = data.at((unsigned int)index);
				return true;
			}
		}

		if(row+1 >= 0 && row+1 < length && col-1 >=0 && col-1 < width)
		{
			index = get2dIndex(row+1,col-1);
			if(index >= 0 && index < (int)data.size())
			{
				_cell = data.at((unsigned int)index);
				return true;
			}
		}

		//printf("Error Getting Cell with Info: P(%f,%f) , C(%d,%d), index = %d \n", p.x, p.y, row, col, index);
		return false;
	}
private:

	int get2dIndex(const int& r,const int& c)
	{
		return ((r*width) + c);
	}

};

class ParticleInfo
{
public:
	double vel;
	int acl; //slow down -1 braking , 0 cruising , 1 accelerating
	PlannerHNS::LIGHT_INDICATOR indicator;
	PlannerHNS::STATE_TYPE state;
	RelativeInfo info_to_path;

	ParticleInfo()
	{
		vel = 0;
		acl = 0;
		indicator = PlannerHNS::INDICATOR_NONE;
		state = PlannerHNS::FORWARD_STATE;
	}
};

template<class T>
class EnumString
{
private:
	std::vector<std::pair<T, std::string> > _enum_str_list;
	T _default;
public:
	EnumString(const T& _default, const std::vector<std::pair<T, std::string> >& list) :
		_default(_default), _enum_str_list(list){}

	int GetIndex(const T& _enum)
	{
		for(unsigned int i = 0; i < _enum_str_list.size(); i++)
		{
			if(_enum_str_list.at(i).first == _enum)
			{
				return i;
			}
		}
		return -1;
	}

	int GetIndex(const std::string& _enum_str)
	{
		for(unsigned int i = 0; i < _enum_str_list.size(); i++)
		{
			if(_enum_str_list.at(i).second.compare(_enum_str) == 0)
			{
				return i;
			}
		}
		return -1;
	}

	std::string GetString(const T& _enum)
	{
		for(unsigned int i = 0; i < _enum_str_list.size(); i++)
		{
			if(_enum_str_list.at(i).first == _enum)
			{
				return _enum_str_list.at(i).second;
			}
		}
		return "";
	}

	std::string GetString(const int& _enum_index)
	{
		if(_enum_index >= 0 && _enum_index < _enum_str_list.size())
		{
			return _enum_str_list.at(_enum_index).second;
		}

		return "";
	}

	T GetEnum(const int& _enum_index)
	{
		if(_enum_index >= 0 && _enum_index < _enum_str_list.size())
		{
			return _enum_str_list.at(_enum_index).first;
		}

		return _default;
	}

	T GetEnum(const std::string& _enum_str)
	{
		for(unsigned int i = 0; i < _enum_str_list.size(); i++)
		{
			if(_enum_str_list.at(i).second.compare(_enum_str) == 0)
			{
				return _enum_str_list.at(i).first;
			}
		}

		return _default;
	}

};


}


#endif /* ROADNETWORK_H_ */

