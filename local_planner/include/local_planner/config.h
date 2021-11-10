/*************************************************************************************************************************************
File Name: Config
Developed by: Prajwal Brijesh Ainapur
Description: Contains all the macros required for this node

Dependacies: N/A
Initial Version: 10/09/2019
Last Modification on: 11/11/2019
*************************************************************************************************************************************/

//Macros used in activate_local_planner.cpp

#define TRAJECTORY_GAP 1.0
#define ROAD_WIDTH 9.0
#define LANE_WIDTH 3.5 //3.5
#define LANE_OFFSET_MAINTENANCE 1.0
#define OFF_REF_TRAJECTORY 1.0
#define VALIDATE_SHIFT_TUNER 10
#define VALIDATE_SHIFT_THRESHOLD 0.2
#define WAYPOINT_COUNT 40
#define FILE_PATH "/home/eleanor/eleanor/src/path_planning/FullSmooth_Oct28_1.csv" //rtk_wp5//rtk_wp6Smooth
#define FILE_PATH2 "/home/eleanor/eleanor/src/path_planning/FullSmooth_Oct28_2.csv" 
///I have modified this file to make sure it builds
#define ENDX  364.229928239942
#define ENDY  145.802323307493
#define ENDpsi  0

// Auxilary functions
#define ABSOLUTE_ZERO 0
#define	ABSOLUTE_TWO 2
#define	ABSOLUTE_ONE 1
#define	PSUEDO_INF	9999


#define VEHICLE_BREADTH 1.5
#define VEHICLE_LENGTH 1.0

//main.cpp and activate_local_planner.cpp
#define KEEP_LANE  1  // 1
#define PLCL 2
#define PLCR 3
#define STOP_EMERGENCY 4
#define TAKE_LEFT 5
#define TAKE_RIGHT 6
#define VERIFY_LANE_CHANGE 7

//EXCEPTION
#define LOCALIZATION_EXCEPTION 'x'
#define MINIMUM_CHECK_LOCALIZATION 0


#define KL_IND  0
#define PLCL_IND 0
#define PLCR_IND 0
#define STOP_IND 0
#define VERIFY_LANE_CHANGE_IND 0
#define TAKE_LEFT_IND -1
#define TAKE_RIGHT_IND 1

//LANE SWITCHING PARAMS
#define IN_MIN_SPEED 10
#define IN_MAX_SPEED 60
#define OUT_MIN_INDEX 5
#define OUT_MIN_TRADIUS 10
#define OUT_MAX_TRADIUS 45
#define OUT_MAX_INDEX WAYPOINT_COUNT
#define TURNING_RADIUS 15
#define DUBINS_RESOLUTION 1




