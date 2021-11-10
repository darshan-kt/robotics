/*************************************************************************************************************************************
File Name: main
Developed by: Vimal Raj Ayyappan; Prajwal Brijesh Ainapur
Description: Main function to run the node

Dependacies: Activate Local Planner
Initial Version: 10/09/2019
Last Modification on: 11/11/2019
*************************************************************************************************************************************/

#include <local_planner/activate_local_planner.h>
#include <string>
#include <fstream>
#include <typeinfo>
#include <map>
#include <ros/ros.h>
#include <nav_msgs/OccupancyGrid.h>
#include <nav_msgs/Path.h>
#include <std_msgs/Int16.h>
#include <std_msgs/Float64.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Point.h>
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <tf/transform_datatypes.h>
#include <visualization_msgs/Marker.h>
#include <nav_msgs/Odometry.h>
#include <local_planner/date_time.h>
#include <local_planner/curves.h>

using namespace FluxLocalPlanner;

ros::Publisher path_publisher;
ros::Publisher path_publisher_sel;
ros::Publisher path_publisher_global;
ros::Publisher curv_pub;
ros::Publisher marker_pub;
VehicleDataBase vehicleDB;
LocalPlanner loc_plan_obj;


geometry_msgs::PoseStamped global_localization;
nav_msgs::OccupancyGrid global_localMap;
double global_speed;
bool receivedSpeed = false;
bool receivedLoc=false;
bool receivedLocMap=false;

//change
Eigen::Vector3d end_Wp_global;
int first_path_length = 0;
//change

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void VisualizationPublish(int index_path){
    
    visualization_msgs::Marker marker;  
    list <TransformationPoints> :: iterator it;
    int iterator_cnt=0;
    for(it = vehicleDB.generatedPathList.begin(); it != vehicleDB.generatedPathList.end(); ++it)  //iterate number of paths and diplays it on rviz
    
    { 
        if(iterator_cnt != index_path)
        {
                // Set the frame ID and timestamp.  See the TF tutorials for information on these.
                marker.header.frame_id = "local_map";
                marker.header.stamp = ros::Time::now();
                // Set the namespace and id for this marker.  This serves to create a unique ID
                // Any marker sent with the same namespace and id will overwrite the old one
                marker.ns = "basic_shapes";
                marker.id = iterator_cnt;
                // Set the marker type.  Initially this is CUBE, and cycles between that and SPHERE, ARROW, and CYLINDER
                marker.type = visualization_msgs::Marker::POINTS;
                // Set the marker action.  Options are ADD, DELETE, and new in ROS Indigo: 3 (DELETEALL)
                marker.action = visualization_msgs::Marker::ADD;
                // Set the pose of the marker.  This is a full 6DOF pose relative to the frame/time specified in the header
                marker.pose.position.x = 0;
                marker.pose.position.y = 0;
                marker.pose.position.z = 0;
                marker.pose.orientation.x = 0.0;
                marker.pose.orientation.y = 0.0;
                marker.pose.orientation.z = 0.0;
                marker.pose.orientation.w = 1.0;
                
                Eigen::MatrixXd path_waypts=(*it).tf_points;
                for(int row=0;row<path_waypts.rows(); row++)
                {
                    Eigen::VectorXd wpt_vector=path_waypts.row(row);
                    geometry_msgs::Point p;
                    p.x = wpt_vector(0)/(1/vehicleDB.local_map_resolution);
                    p.y = wpt_vector(1)/(1/vehicleDB.local_map_resolution);
                    p.z = 0;
                    marker.points.push_back(p);
                    
                }
                // Set the scale of the marker -- 1x1x1 here means 1m on a side
                marker.scale.x = 0.1;
                marker.scale.y = 0.1;
                marker.scale.z = 0.1;
                marker.color.r = 1.0f;
                marker.color.g = 0.0f;
                marker.color.b = 0.0f;
                marker.color.a = 1.0;
                marker.lifetime = ros::Duration();
                marker_pub.publish(marker);                
        
        }
        iterator_cnt++;
      }    
}





template <typename T> string tostr(const T& t) { 
   ostringstream os; 
   os<<t; 
   return os.str(); 
} 

void write_csv(Eigen::MatrixXd tf_points,char* input1)
{
    std::ofstream myfile;
    myfile.open(input1);
    for(int i=0;i<tf_points.rows();i++)
    {
        myfile << tostr(tf_points.row(i)[0]) + "," + tostr(tf_points.row(i)[1]) + "," + tostr(tf_points.row(i)[2]) +"\n";
        
    }
     myfile.close();
}
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////


/**********************************************************************************************************************************

    FUNCTION        : CreateModel()
    DESCRIPTION     : This function is called when the model of vehicle is to be created based on grid resolution
    INPUT PARAMS    : ARGUMENTS
                        ~ pagridResolution    -- Local Map grid resolution
                     GLOBAL PARAMS
                        ~ NA
    OUTPUT PARAMS   : TransformationPoints -- A matrix structure of Vehcile Model and their corresponding center.

**********************************************************************************************************************************/
TransformationPoints CreateModel(float gridResolution)
{
    // cout << gridResolution << endl;
	// cout<<"\nGenerating Vehicle Model.. \n\n";

	Eigen::VectorXd center(3);
	center<<0,0,0;


	float y_range = (VEHICLE_LENGTH / gridResolution);
	float x_range = (VEHICLE_BREADTH / gridResolution);

	int noPoints = (x_range + 1) * (y_range + 1);
	Eigen::MatrixXd points(noPoints, 3);



	int temp = 0;


	for (int y = (x_range /2); y >= (-x_range /2); y--)
	{
		for (int x = (y_range /2); x >= (-y_range / 2); x--)
		{
			points(temp, 0) = x; 
			points(temp, 1) = y;
			points(temp, 2) = 0;
			temp++;
		}	
	}

	// cout<<"Vehicle model has been generated.. \n\n";
    TransformationPoints vehicle(center, points);
    // ROS_INFO("[%s] [VEHICLE MODEL CREATED]",CurrentDateTime().c_str());
	return vehicle;
}


/**********************************************************************************************************************************

    FUNCTION        : LoadWayPoints()
    DESCRIPTION     : This function is called when the pre-built waypoint csv file is to be loaded.
    INPUT PARAMS    : ARGUMENTS
                        ~ path             -- File path of the waypoints.csv file
                     GLOBAL PARAMS
                        ~ NA
    OUTPUT PARAMS   : MatTemp -- A matrix structure of wayoints.

**********************************************************************************************************************************/
template<typename MatTemp>
MatTemp LoadWayPoints (const std::string & path) {
    std::ifstream indata;
    indata.open(path);
    std::string line;
    std::vector<double> values;
    uint rows = 0;
    while (std::getline(indata, line)) {
        std::stringstream lineStream(line);
        std::string cell;
        while (std::getline(lineStream, cell, ',')) {
            values.push_back(std::stod(cell));
        }
        ++rows;
    }

    // ROS_INFO("Waypoints are loaded from " + tostr(FILE_PATH));

    return Eigen::Map<const Eigen::Matrix<typename MatTemp::Scalar, MatTemp::RowsAtCompileTime, MatTemp::ColsAtCompileTime, Eigen::RowMajor>>(values.data(), rows, values.size()/rows);
}



/**********************************************************************************************************************************

    FUNCTION        : StateCallBack()
    DESCRIPTION     : This function is triggered whenever the current state data is obtained.
    INPUT PARAMS    : ARGUMENTS
                        ~ msg             -- Input state data.
                     GLOBAL PARAMS
                        ~ vehicleDB
    OUTPUT PARAMS   : NA

**********************************************************************************************************************************/
void StateCallBack(const std_msgs::Int16::ConstPtr& msg)
{
  
//   ROS_INFO("StateCallBack : " + tostr(msg->data));  
//   ROS_INFO("[%s] [STATE CALLBACK DONE. CURRENT STATE] [%d]",CurrentDateTime().c_str(),msg->data);
  vehicleDB.stateUpdated = vehicleDB.state_to_be_taken != msg->data ? true : false;
  vehicleDB.state_to_be_taken=msg->data;
  vehicleDB.state_activated=true;
  
}


/**********************************************************************************************************************************

    FUNCTION        : SyncCallBack()
    DESCRIPTION     : This function is triggered whenever the localization pose data is obtained.
    INPUT PARAMS    : ARGUMENTS
                        ~ localizationMsg -- Input pose.
                        ~ mapMsg          -- Input LocalMap.
                     GLOBAL PARAMS
                        ~ vehicleDB,loc_plan_obj
    OUTPUT PARAMS   : NA

**********************************************************************************************************************************/
void SyncCallBack( geometry_msgs::PoseStamped localizationMsg, nav_msgs::OccupancyGrid mapMsg)
{
    // ROS_INFO("[%s] [SYNC CALLBACK STARTED]",CurrentDateTime().c_str());    
    if(vehicleDB.state_activated)
    {
        vector<int8_t> inp_local_map = mapMsg.data;          //accesing local map using vector variable (int8_t is data type which is in taking data type)  it actually recieves the binary data for 1D vector
        vector<double> conv_local_map(inp_local_map.begin(),inp_local_map.end());     //converting int8_t data format to double data format for further processing purpose
        double* ptr=&conv_local_map[0];    //because of data stored in only 1 row vector just copy 0 index
        Eigen::Map<Eigen::MatrixXd,0,Eigen::Stride<0, 0>> local_map(ptr,mapMsg.info.height,mapMsg.info.width);    //This is will convert the binary map data to height*width(ex:200*200) matrix format
        vehicleDB.currentSpeed = global_speed;     //assigning the all data to VehicleDataBase class of active_local_planner.h file (if have lot of variable in that time just use this techinique ie create a class and use this class varible on any files just calling class )
        vehicleDB.local_map=local_map;
        vehicleDB.local_map_resolution=mapMsg.info.resolution;
        vehicleDB.local_map_width=mapMsg.info.width;

        // std::cout << mapMsg.info.width <<std::endl;
        
        tf::Quaternion quaternion(localizationMsg.pose.orientation.x, 
                                localizationMsg.pose.orientation.y,
                                localizationMsg.pose.orientation.z,
                                localizationMsg.pose.orientation.w);
        tf::Matrix3x3 m(quaternion);
        double roll, pitch, yaw;
        m.getRPY(roll, pitch, yaw);            //Converting Quaternion to rpy
        // yaw=yaw-0.244346; 
        Eigen::Vector3d localization(localizationMsg.pose.position.x,
                                    localizationMsg.pose.position.y,
                                    yaw);     //Arranging the localisation input to proper x,y,theta format


        KDT::MyPoint query(localization(0),localization(1));

		//change
        // std::cout<<"distance "<<loc_plan_obj.EuclideanDistance(localization,end_Wp_global)<<std::endl;
        if((*(vehicleDB.searchTree)).nnSearch(query) > (first_path_length - 10))
        {
            vehicleDB.waypoints= vehicleDB.waypoints2;
            vehicleDB.searchTree = vehicleDB.searchTree2; 
        }

        //change
        int closeIdx = (*(vehicleDB.searchTree)).nnSearch(query);    //Till now we have array of trajectory waypoint, in this step using KT tree concept finds least possible distanced waypoint for my current location input. output we'll get least trajectory point index value stored in array vector

        //DOT PRODUCT TO FIND THE POINT LIES BEFORE OR AFTER
        if(closeIdx > 0)
        {
            Eigen::Vector2d closeVect(vehicleDB.waypoints.row(closeIdx).head(2));
            Eigen::Vector2d prevVect(vehicleDB.waypoints.row(closeIdx-1).head(2));
            Eigen::Vector2d poseVect(localization.head(2));
            double dotValue= (closeVect-prevVect).dot((poseVect-closeVect));
            if(dotValue >= 0)
            {
                closeIdx= (closeIdx+1) > (vehicleDB.waypoints.rows()-1) ? closeIdx : closeIdx+1;       //here will get the exact waypoint index on the array of matrix
            }
           
        }
                  
        int adaptive_waypt_count =vehicleDB.waypoints.rows() > (closeIdx+WAYPOINT_COUNT) ? WAYPOINT_COUNT : vehicleDB.waypoints.rows()-closeIdx;  //condition: number of waypoints in matrix(ex:total 90points) > currentindex(ex: 4) + WaypointCount need to consider in front for drawing path(ex:40) is true, draw path by considering WaypointCount values(40), if not draw path for remaining waypoints in matrix array w.r.t current index. (ex: total number of rows is 60 and my current point index of row in that matrix is 45, so now 60-45=15 now i can only able draw path for 15 point avaible in front)
        Eigen::MatrixXd tf_points=vehicleDB.waypoints.block(closeIdx,0,adaptive_waypt_count,3);   //This is the inbuilt function in the matrix, this will blocks the set of rows inside the matrix and form seperate matrix, here we will blocks the rows from closeindex to adoptive_waypoint_count value index and block this as seperate matrix and given this as ref trajectory points for local planner to draw trajectory path. 
        TransformationPoints ref_trajectory(tf_points.row(0),tf_points);
        vehicleDB.ref_trajectory=ref_trajectory;      //Now we have a trajectory points which will use for drawing local trajectory path
        // std::cout << "BASIC: " << float(t2-t1)/CLOCKS_PER_SEC<<std::endl;

        if(vehicleDB.isVehicleStarted)
        {
            if(vehicleDB.stateUpdated)
            {
                map<int,int> laneSelectionMap=loc_plan_obj.BuildLaneSelectionMap();
                vehicleDB.present_lane_id+=laneSelectionMap[vehicleDB.state_to_be_taken];
            }
            
            if(loc_plan_obj.AccomplishBehavior(localization,vehicleDB))
            {

                t3 = clock();
                std::cout << "AccomplishBehavior: " << float(t3-t2)/CLOCKS_PER_SEC<<std::endl;

                // ROS_INFO("[%s] [ACCOMPLISH BEHAVIOR SUCCESS]",CurrentDateTime().c_str());
                // list <TransformationPoints> :: iterator it;
                // int iterator_cnt=0;
                // for(it = vehicleDB.generatedPathList.begin(); it != vehicleDB.generatedPathList.end(); ++it) 
                // {
                //     if(iterator_cnt == vehicleDB.sub_path_id)
                //     {
                //         iterator_cnt++;
                //         continue;
                //     }
                //     nav_msgs::Path path;
                //     path.header.frame_id="local_map";
                //     Eigen::MatrixXd path_waypts=(*it).tf_points;
                //     for(int row=0;row<path_waypts.rows(); row++)
                //     {
                //         geometry_msgs::PoseStamped pose;
                        
                //         Eigen::VectorXd wpt_vector=path_waypts.row(row);
                //         pose.pose.position.x = wpt_vector(0)/(1/vehicleDB.local_map_resolution);
                //         pose.pose.position.y = wpt_vector(1)/(1/vehicleDB.local_map_resolution);
                //         pose.pose.position.z = 0;

                //         tf::Quaternion quaternion;
                //         quaternion.setRPY(0,0,wpt_vector(2));

                //         pose.pose.orientation.x=quaternion[0];
                //         pose.pose.orientation.y=quaternion[1];
                //         pose.pose.orientation.z=quaternion[2];
                //         pose.pose.orientation.w=quaternion[3];

                //         path.poses.push_back(pose);
                //     }
                    
                //     // path_publisher.publish(path);
                //     // ROS_INFO("PATH PUBLISHED: " +  tostr(iterator_cnt));
                //     // cout <<"CHECK OUT IN /localTrajectory topic" <<endl;
                //     iterator_cnt++;

                // }



                    //#################################################################################
                    Eigen::MatrixXd path_waypts;
                    // ROS_INFO("[%s] [CHECKIN READY CURVE %d]",CurrentDateTime().c_str(),vehicleDB.laneSwitchCurves.size());
                    if(vehicleDB.laneSwitchCurves.size() > 0)
                    {
                        local_planner::curves curvy;
                        for(int indx=0;indx < vehicleDB.laneSwitchCurves.size(); indx++)
                        {
                            nav_msgs::Path path;
                            path.header.frame_id="local_map";
                            std:cout << vehicleDB.laneSwitchCurves[indx].switchCurve.tf_points << std::endl;
                            path_waypts = vehicleDB.laneSwitchCurves[indx].switchCurve.tf_points;
                            // ROS_INFO("[%s] [STARTING PUBLISHING CURVE]",CurrentDateTime().c_str());
                            for(int row=0;row<path_waypts.rows(); row++)
                            {
                                geometry_msgs::PoseStamped pose;
                                
                                Eigen::VectorXd wpt_vector=path_waypts.row(row);
                                pose.pose.position.x = wpt_vector(0)/(1/vehicleDB.local_map_resolution);
                                pose.pose.position.y = wpt_vector(1)/(1/vehicleDB.local_map_resolution);
                                pose.pose.position.z = 0;

                                tf::Quaternion quaternion;
                                quaternion.setRPY(0,0,wpt_vector(2));

                                pose.pose.orientation.x=quaternion[0];
                                pose.pose.orientation.y=quaternion[1];
                                pose.pose.orientation.z=quaternion[2];
                                pose.pose.orientation.w=quaternion[3];

                                path.poses.push_back(pose);
                            }
                            if(vehicleDB.laneSwitchCurves[indx].direction == 1)
                            {
                                curvy.pathRight = path;
                            }
                            else
                            {
                                curvy.pathLeft = path;
                            }
                        }
                        curv_pub.publish(curvy);
                        // path_publisher_sel.publish(path);
                    }
                    path_waypts=vehicleDB.presentTrajectory.tf_points;
                    VisualizationPublish(vehicleDB.sub_path_id);

                    // ROS_INFO("[%s] [SELECTED PATH ID] [%d]",CurrentDateTime().c_str(),vehicleDB.sub_path_id);
                    //################################################################################# ----- LOCAL TRAJECTORY VISUALIZATION

                    nav_msgs::Path path;
                    path.header.frame_id="local_map";
                    // Eigen::MatrixXd path_waypts=vehicleDB.presentTrajectory.tf_points;
                    for(int row=0;row<path_waypts.rows(); row++)
                    {
                        geometry_msgs::PoseStamped pose;
                        
                        Eigen::VectorXd wpt_vector=path_waypts.row(row);
                        pose.pose.position.x = wpt_vector(0)/(1/vehicleDB.local_map_resolution); //converting the resolution grid format to normal grid format (resolution = 1)
                        pose.pose.position.y = wpt_vector(1)/(1/vehicleDB.local_map_resolution);
                        pose.pose.position.z = 0;

                        tf::Quaternion quaternion;
                        quaternion.setRPY(0,0,wpt_vector(2));
                        pose.pose.orientation.x = quaternion[0];
                        pose.pose.orientation.y = quaternion[1];
                        pose.pose.orientation.z = quaternion[2];
                        pose.pose.orientation.w = quaternion[3];

                        path.poses.push_back(pose);
                    }
                    path_publisher_sel.publish(path);
                    t4 = clock();
                    // std::cout << "PUBLISH: " << float(t4-t3)/CLOCKS_PER_SEC<<std::endl;
                    // ROS_INFO("[%s] [SELECTED PATH ID] [%d]",CurrentDateTime().c_str(),vehicleDB.sub_path_id);
                    // cout <<"CHECK OUT IN /localTrajectorySelected topic" <<endl;
                    // #################################################################################
 
                    // ################################################################################# -- GLOBAL TRAJECTORY VISUALIZATION
                    path_waypts.col(0)/= (1/vehicleDB.local_map_resolution);
                    path_waypts.col(1)/= (1/vehicleDB.local_map_resolution);
                    Eigen::Vector3d loc_center = Eigen::Vector3d(int(vehicleDB.local_map_resolution * vehicleDB.local_map_width/2),int(vehicleDB.local_map_resolution * vehicleDB.local_map_width/2),0);
                    TransformationPoints selectedLocTrajectory(loc_center, path_waypts );
                    TransformationPoints selectedGlobalTrajectory = loc_plan_obj.TransformCoordinates(selectedLocTrajectory,localization);   //Convert selected local path from local co-ordinate frame to world co-ordinate frame for MPC controlling purpose

                    nav_msgs::Path globalPath;
                    globalPath.header.frame_id="world";       // converting into world co-ordinate frame
                    path_waypts=selectedGlobalTrajectory.tf_points;
                    for(int row=0;row<path_waypts.rows(); row++)
                    {
                        geometry_msgs::PoseStamped pose;
                        pose.header.frame_id="world";
                        Eigen::VectorXd wpt_vector=path_waypts.row(row);
                        pose.pose.position.x = wpt_vector(0);
                        pose.pose.position.y = wpt_vector(1);
                        // pose.pose.position.z = 0;

                        tf::Quaternion quaternion;
                        quaternion.setRPY(0,0,wpt_vector(2));

                        // pose.pose.orientation.x=quaternion[0];
                        // pose.pose.orientation.y=quaternion[1];
                        // pose.pose.orientation.z=quaternion[2];
                        // pose.pose.orientation.w=quaternion[3];

                        globalPath.poses.push_back(pose);
                    }
                    path_publisher_global.publish(globalPath);    
                    // // #################################################################################
                

            }
            else
            {
                // ROS_INFO("[%s] [##################################################################]", CurrentDateTime().c_str());
                // ROS_INFO("[%s] [LOCALIZATION LOST!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!]", CurrentDateTime().c_str());
                // ROS_INFO("[%s] [##################################################################]", CurrentDateTime().c_str());
                exit(0);
            }
            
            
        }
        else
        {
            // auto 
            vehicleDB.vehicleModel = CreateModel(vehicleDB.local_map_resolution);   //This will helps to create my vehicle polygon on local map w.r.t map resolution
            double distance_from_ref_path = loc_plan_obj.EuclideanDistance(localization,ref_trajectory.center);    // calculate the distance between localization point and actual reference trajectory center
            int present_lane_id = ((distance_from_ref_path + OFF_REF_TRAJECTORY) / (ROAD_WIDTH/LANE_WIDTH) ) +1;   //helps to calculate which lane vehicle lies on map
            vehicleDB.present_lane_id=1;          // present_lane_id;
            vehicleDB.sub_path_id=0;
            vehicleDB.state_to_be_taken=KEEP_LANE;
            vehicleDB.isVehicleStarted=loc_plan_obj.AccomplishBehavior(localization,vehicleDB);
                
        }
    
    }
    else
    {
        ROS_INFO("[%s] [STATE MACHINE NOT ACTIVATED YET]", CurrentDateTime().c_str());
    }
    
}


/**********************************************************************************************************************************

    FUNCTION        : PoseCallBack()
    DESCRIPTION     : This function is triggered whenever the localization pose data is obtained.
    INPUT PARAMS    : ARGUMENTS
                        ~ localizationMsg -- Input pose.
                     GLOBAL PARAMS
                        ~ recievedLoc,global_localization
    OUTPUT PARAMS   : NA

**********************************************************************************************************************************/
void PoseCallBack(const geometry_msgs::PoseStamped::ConstPtr& localizationMsg)   //This localization coming w.r.t to global frame
{
    // ROS_INFO("[%s] LOCALIZATION RECEIVED",CurrentDateTime().c_str());    
    global_localization = *localizationMsg;      //Using derefence operator to direct value inside the localizationMsg variable  
    receivedLoc = true;
    // std::cout<<"localization"<<std::endl;
    // std::cout<<receivedLocMap<<", "<<std::endl;
    if(receivedLoc && receivedLocMap && receivedSpeed)
    {
        SyncCallBack(global_localization,global_localMap);     //This is main function perform localization inside local map
    }
}

/**********************************************************************************************************************************

    FUNCTION        : LocMapCallBack()
    DESCRIPTION     : This function is triggered whenever the LocMap data is obtained.
    INPUT PARAMS    : ARGUMENTS
                        ~ mapMsg -- Input local map.
                     GLOBAL PARAMS
                        ~ receivedLocMap,global_localMap
    OUTPUT PARAMS   : NA

**********************************************************************************************************************************/
void LocMapCallBack(const nav_msgs::OccupancyGrid::ConstPtr& mapMsg)
{
    // ROS_INFO("[%s] LOCAL MAP RECEIVED",CurrentDateTime().c_str());    
    global_localMap = *mapMsg;
    receivedLocMap = true;
}

void SpeedCallBack(const std_msgs::Float64::ConstPtr& speedMsg)
{
    // ROS_INFO("[%s] SPEED RECEIVED",CurrentDateTime().c_str());    
    global_speed = speedMsg-> data;
    receivedSpeed = true;
}




/**********************************************************************************************************************************

    FUNCTION        : main()
    DESCRIPTION     : This function is triggered whenever the program is kicked off.
    INPUT PARAMS    :
            ARGUMENTS       - argc ,argv - These are runtime parameters which is not used.
            GLOBAL PARAMS   - path_publisher,path_publisher_sel,marker_pub
    OUTPUT PARAMS   : NA

**********************************************************************************************************************************/
int main(int argc, char **argv)
{

    ros::init(argc, argv, "local_planner");
    ros::NodeHandle n;


    //change
    end_Wp_global << ENDX, ENDY, ENDpsi;
    //change
    
    // ROS_INFO("[%s] LOCAL PLANNER TRIGGERED",CurrentDateTime().c_str());    

    //LOADING KD TREE
    Eigen::MatrixXd waypoints = LoadWayPoints<Eigen::MatrixXd>(FILE_PATH);
    vector<KDT::MyPoint> treePoints(waypoints.rows());

    for(int row=0;row<waypoints.rows();row++)
    {
        Eigen::VectorXd subVector= waypoints.row(row);
        treePoints[row] = KDT::MyPoint(subVector(0), subVector(1));
    }
    KDT::KDTree<KDT::MyPoint> kdtree(treePoints);
    vehicleDB.waypoints=waypoints;
    vehicleDB.searchTree=&kdtree;

	//change
	first_path_length = waypoints.rows();
    Eigen::MatrixXd waypoints2 = LoadWayPoints<Eigen::MatrixXd>(FILE_PATH2);
    vector<KDT::MyPoint> treePoints2(waypoints2.rows());
    for(int row=0;row<waypoints2.rows();row++)
    {
        Eigen::VectorXd subVector= waypoints2.row(row);
        treePoints2[row] = KDT::MyPoint(subVector(0), subVector(1));
    }
    KDT::KDTree<KDT::MyPoint> kdtree2(treePoints2);
    vehicleDB.waypoints2=waypoints2;
    vehicleDB.searchTree2 = &kdtree2;
    //change





    vehicleDB.isVehicleStarted=false;
    vehicleDB.state_activated=false;

    //PUBLISHERS & SUBSCRIBERS
    path_publisher = n.advertise<nav_msgs::Path>("/localTrajectory",10);            
    path_publisher_sel = n.advertise<nav_msgs::Path>("/localTrajectorySelected",10);         //Trajectory selected w.r.t local frame(odom)
    path_publisher_global = n.advertise<nav_msgs::Path>("/planning/localPlanner/globalTrajectory",10);    //Trajectory w.r.t global frame(map/world)
    curv_pub = n.advertise<local_planner::curves>("/curves",10);                               //left or right curves
    marker_pub = n.advertise<visualization_msgs::Marker>("visualization_marker", 1);

    ros::Subscriber state_sub = n.subscribe("/currentState", 1, StateCallBack);   // Coming from Behavior planner to indicate vehicle located in online, left lane or right lane
    ros::Subscriber loc_sub = n.subscribe("/ndt_pose", 1, PoseCallBack);          // This is the main localisation input
    ros::Subscriber locMap_sub = n.subscribe("/local_map", 1, LocMapCallBack);    // Binary map coming from mapping team (they built using lidar pointcloud)
    ros::Subscriber speed_sub = n.subscribe("/speed", 1, SpeedCallBack);          //Odometry callback for speed

    ros::spin();
    return 0;
}


// //COMMENTS
// I had made Validated index 0 - Alter it


// clock_t t1,t2,t3;

// t1 = clock();
// t2 = clock();
// std::cout<<float(t2-t1)/CLOCKS_PER_SEC<<std::endl;

