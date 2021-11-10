/*************************************************************************************************************************************
File Name: Activate Local Planner
Developed by: Vimal Raj Ayyappan
Description: Contains the class declarations required for the vehicle database and the local planner

Dependacies: Eigen/Dense, Config file, Transformation Points
Initial Version: 10/09/2019
Last Modification on: 10/09/2019
*************************************************************************************************************************************/

#ifndef LOCAL_PATH_PLANNER
#define LOCAL_PATH_PLANNER

#include <iostream>
#include <exception>
#include <list>
#include "kdtree.h"
#include "Eigen/Dense"
#include "transformation_points.h"
#include "config.h"
#include <map>
#include <ros/ros.h>
#include <ros/console.h>
// #include <local_planner/DateTime.hpp>
#include "date_time.h"


using namespace std;

namespace FluxLocalPlanner
{


    class LaneSwitchTrajectory
    {
        public:
            TransformationPoints switchCurve = TransformationPoints(Eigen::Vector3d::Zero(),Eigen::Matrix3d::Zero());
            int direction;

    };


    class VehicleDataBase
    {
        public:
            
            bool isVehicleStarted;
            bool state_activated;
            Eigen::MatrixXd local_map;
			float local_map_resolution;
			int local_map_width;
            int present_lane_id; 
            int sub_path_id;
            int state_to_be_taken;
            bool stateUpdated = false;

            Eigen::MatrixXd waypoints;
            KDT::KDTree<KDT::MyPoint> *searchTree;

            //change
            Eigen::MatrixXd waypoints2;
            KDT::KDTree<KDT::MyPoint> *searchTree2;
            //change

			TransformationPoints presentTrajectory = TransformationPoints(Eigen::Vector3d::Zero(),Eigen::Matrix3d::Zero());
            TransformationPoints vehicleModel = TransformationPoints(Eigen::Vector3d::Zero(),Eigen::Matrix3d::Zero());
            TransformationPoints ref_trajectory = TransformationPoints(Eigen::Vector3d::Zero(),Eigen::Matrix3d::Zero());
            list<TransformationPoints> generatedPathList;
            std::vector<LaneSwitchTrajectory> laneSwitchCurves;
            double currentSpeed;
            

    };

   


    class LocalPlanner
    {

        public:
            double RadianToDegree(double radians);
            double DegreeToRadian(double degrees);
            double EuclideanDistance(Eigen::VectorXd pointA,Eigen::VectorXd pointB);
            void CalcTrajectoryYawFromXY(Eigen::MatrixXd complete_transformed_mat,Eigen::VectorXd &angles);
            TransformationPoints CreateTrajectoryMatrix(int present_lane_id);
            TransformationPoints TransformCoordinates(TransformationPoints pts_to_be_transformed, Eigen::VectorXd next_state);
            list<TransformationPoints> GenerateAugmentedTrajectories(float map_resolution,
                                                                     float local_map_width,
                                                                     int present_lane_id,
                                                                     Eigen::VectorXd localization,
                                                                     TransformationPoints ref_trajectory);

            Eigen::MatrixXd TrajectoryGenerator(TransformationPoints ref_trajectory,TransformationPoints pathAugmenter);

            bool ValidateShift(int currentID, int number_traj, Eigen::MatrixXd local_map, int map_width, float grid_resolution);

            int ValidateTrajectory(list<TransformationPoints> trajectory_list,
                                                    TransformationPoints vehicle_model,
                                                    Eigen::MatrixXd& local_map, //CHANGEE
                                                    int sub_path_id,
                                                    int stateToBeTaken, 
                                                    bool toBeValidated, 
                                                    int map_width,
                                                    float grid_resolution,Eigen::VectorXd localization);

            bool AccomplishBehavior(Eigen::VectorXd localization, VehicleDataBase &vehicleDB);
            TransformationPoints GetLaneSwitchCurve(Eigen::VectorXd localization,int laneID, VehicleDataBase vehicleDB);
            map<int,int> BuildLaneSelectionMap();
            

            

    };


    
      

}
#endif