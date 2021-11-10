/*************************************************************************************************************************************
File Name: Activate Local Planner
Developed by: Vimal Raj Ayyappan
Description: Contains the function defintions require in the local planner

Dependacies: Auxilary Functions
Initial Version: 10/09/2019
Last Modification on: 11/11/2019
*************************************************************************************************************************************/

#include <local_planner/activate_local_planner.h>
#include <local_planner/auxilary_functions.hpp>
#include <local_planner/dubins.h>
#include <math.h>

using namespace FluxLocalPlanner;


/**********************************************************************************************************************************
    
    FUNCTION        : GenerateAugmentedTrajectories()
    DESCRIPTION     : This function make use of TrajectoryMatrix created and split the trajectories accordingly based on their width
                      from the reference line.
    INPUT PARAMS    : ARGUMENTS
                        ~ local_map_resolution  - resolution of input local_map
                        ~ local_map_width       - width of input local_map
                        ~ present_lane_id       - current lane id
                        ~ localization          - localization inputted
                        ~ ref_trajectory        - the reference line
                     GLOBAL PARAMS
                        ~ NA
    OUTPUT PARAMS   : list - list of sub-trajectories as Transformation Points
**********************************************************************************************************************************/
list <TransformationPoints> LocalPlanner::GenerateAugmentedTrajectories(float local_map_resolution,
                                                                        float local_map_width,
                                                                        int present_lane_id,
                                                                        Eigen::VectorXd localization, 
                                                                        TransformationPoints ref_trajectory)
{
    list <TransformationPoints> path_list; 
    TransformationPoints pathTransformer =CreateTrajectoryMatrix(present_lane_id);
    int number_of_paths=pathTransformer.tf_points.rows();      //converting co-ordiante gap points into tf transform points
    int number_of_waypoints=ref_trajectory.tf_points.rows();   //converting certain number of forward points to tf points
    Eigen::MatrixXd global_path_database = TrajectoryGenerator(ref_trajectory,pathTransformer);   //creates trajectory points into transformed trajectory w.r.t reftrajectory
    TransformationPoints global_pathDB_tf(localization,global_path_database); //USED FOR TRANSFORMING IN MOST WAYS --UC
    Eigen::MatrixXd temp_pathDB = global_path_database;
    temp_pathDB.col(0)*= (1/local_map_resolution);
    temp_pathDB.col(1)*= (1/local_map_resolution);   
    Eigen::Vector3d temp_localization = Eigen::Vector3d(localization[0]*(1/local_map_resolution),localization[1]*(1/local_map_resolution),localization[2]);
    TransformationPoints tempTransformer = TransformationPoints(temp_localization,temp_pathDB);
    Eigen::Vector3d localCenter = Eigen::Vector3d(int(local_map_width/2),int(local_map_width/2),0);
    TransformationPoints local_pathDB_tf = TransformCoordinates(tempTransformer,localCenter);
    
    for(int i=0; i <number_of_paths;i++ )
    {
            path_list.push_back(TransformationPoints(localCenter,local_pathDB_tf.tf_points.block(i*number_of_waypoints,0,number_of_waypoints,3)));
    }

    // ROS_INFO("[%s] GENERATE AUGMENTED TRAJECTORIES COMPLETED",CurrentDateTime().c_str());
    

    return path_list;
}


/**********************************************************************************************************************************
    
    FUNCTION        : TrajectoryGenerator()
    DESCRIPTION     : This function augments and create parallel trajectories for the reference line
    INPUT PARAMS    : ARGUMENTS
                        ~ ref_trajectory - the reference line
                        ~ pathAugmenter  - PathAugmenter from trajectory matrix
                      GLOBAL PARAMS
                        ~ NA
    OUTPUT PARAMS   : MatrixXd - sub trajectory matrix as a database
**********************************************************************************************************************************/

Eigen::MatrixXd LocalPlanner::TrajectoryGenerator(TransformationPoints ref_trajectory,TransformationPoints pathAugmenter)
{
    list <TransformationPoints> path_list;    
    int number_of_paths=pathAugmenter.tf_points.rows();
    int number_of_waypoints=ref_trajectory.tf_points.rows();

    Eigen::MatrixXd path_database(number_of_paths*number_of_waypoints,3);
    for(int main_indx=0; main_indx<number_of_waypoints; main_indx++ )
    {
        TransformationPoints augmentedPathSlice= TransformCoordinates(pathAugmenter,ref_trajectory.tf_points.row(main_indx));
        for(int loc_ix=0;loc_ix<number_of_paths;loc_ix++) 
        {
            path_database.row((loc_ix*(number_of_waypoints))+main_indx)=augmentedPathSlice.tf_points.row(loc_ix);

        }
               
    } 
    // ROS_INFO("[%s] TRAJECTORY GENERATOR COMPLETED",CurrentDateTime().c_str());

    return path_database;
}


/**********************************************************************************************************************************
    
    FUNCTION        : CreateTrajectoryMatrix()
    DESCRIPTION     : This function creates trajectory matrix from actual reference line
    INPUT PARAMS    : ARGUMENTS
                        ~ present_lane_id - lane id which you are currently in
                     GLOBAL PARAMS
                        ~ NA
    OUTPUT PARAMS   : TransformationPoints - Trajectoy Augmenter matrix
**********************************************************************************************************************************/

TransformationPoints LocalPlanner::CreateTrajectoryMatrix(int present_lane_id)
{

    float offset_val = present_lane_id == 1 ? OFF_REF_TRAJECTORY : LANE_OFFSET_MAINTENANCE;
    Eigen::VectorXd virtual_center(3);    //creates virtual center and will consider this as current Laneid point based on this point create multiple transformed points towards right and left side
    virtual_center << 0,0,0;
    Eigen::Vector3d sub_vector;
    list <float> trajList;    //created a list to store generated transformed points w.r.t virtual_center

    for (float start_param=0; start_param <= ((LANE_WIDTH-LANE_OFFSET_MAINTENANCE)-offset_val);start_param += TRAJECTORY_GAP)
    {
        // cout << start_param << endl;
        trajList.push_back(start_param);   //This loop will adds left and right points created w.r.t trajectory gap mentioned and inside the lane width boundary
    }

    Eigen::MatrixXd trajectory_matrix(trajList.size(),3);      //storing the transformed list points in matrix
    int count=0;
    list <float> :: iterator it; 
    for(it = trajList.begin(); it != trajList.end(); ++it) 
    {
        sub_vector<< 0,(((*it)*(-1)) - (present_lane_id-1)*LANE_WIDTH) ,0;
        trajectory_matrix.row(count)=sub_vector;
        count++;
    }    //added all list of tranformed points into row matrix format

    TransformationPoints pathTransformer(virtual_center,trajectory_matrix);   //tranforms the virtual center to actual co-ordinate center on map and tranforms all points on trajectory list to map co-ordinates
    // cout << pathTransformer.tf_points <<endl;
    
    // ROS_INFO("[%s] CREATE TRAJECTORY MATRIX COMPLETD",CurrentDateTime().c_str());

    
    return pathTransformer;     //Tranformed points w.r.t map co-ordiante system
}

/**********************************************************************************************************************************
    
    FUNCTION        : RadianToDegree()
    DESCRIPTION     : This function converts  radian to degree
    INPUT PARAMS    : ARGUMENTS
                        ~ radian
                     GLOBAL PARAMS
                        ~ NA
    OUTPUT PARAMS   : double - degrees
**********************************************************************************************************************************/
double LocalPlanner::RadianToDegree(double radians)
{
    return (radians*(180/M_PI));
}

/**********************************************************************************************************************************
    
    FUNCTION        : DegreeToRadian()
    DESCRIPTION     : This function converts degree to radian
    INPUT PARAMS    : ARGUMENTS
                        ~ degrees
                     GLOBAL PARAMS
                        ~ NA
    OUTPUT PARAMS   : double - radians
**********************************************************************************************************************************/
double LocalPlanner::DegreeToRadian(double degrees)
{
    return (degrees*(M_PI/180));
}

/**********************************************************************************************************************************
    
    FUNCTION        : EuclideanDistance()
    DESCRIPTION     : This function determines the eulidean distance between two points
    INPUT PARAMS    : ARGUMENTS
                        ~ pointA      : input point A
                        ~ pointB      : input point B
                     GLOBAL PARAMS
                        ~ NA
    OUTPUT PARAMS   : double - distance
**********************************************************************************************************************************/
double LocalPlanner::EuclideanDistance(Eigen::VectorXd pointA,Eigen::VectorXd pointB)
{
    return sqrt(pow((pointB[0]-pointA[0]),2) + pow((pointB[1]-pointA[1]),2));
}



/**********************************************************************************************************************************
    
    FUNCTION        : TransformCoordinates()
    DESCRIPTION     : This function transforms the given set of co-ordinates with respect to the next state
    INPUT PARAMS    : ARGUMENTS
                        ~ pts_to_be_transformed      : input points to be transformed and their base center
                        ~ next_state                 : state to which all points has to be transformed
                     GLOBAL PARAMS
                        ~ NA
    OUTPUT PARAMS   : TransformationPoints - Transformed co-ordinates
**********************************************************************************************************************************/

TransformationPoints LocalPlanner::TransformCoordinates(TransformationPoints pts_to_be_transformed, Eigen::VectorXd next_state)

{
    
    float rotation_angle= next_state[2]-pts_to_be_transformed.center(2);
    int rowCount=pts_to_be_transformed.tf_points.rows();
    int colCount= pts_to_be_transformed.tf_points.cols();

    Eigen::VectorXd displaced_vec = next_state.head(2)-pts_to_be_transformed.center.head(2);
    Eigen::MatrixXd segmented_points(rowCount,2);
    segmented_points=pts_to_be_transformed.tf_points.block(0,0,rowCount,2);
    Eigen::MatrixXd linear_transform_pts=segmented_points.rowwise()+ displaced_vec.transpose();
    TransformationPoints linearly_transformed(next_state,linear_transform_pts);

    //ROTATION
    Eigen::MatrixXd rotation_matrix(2,2);
    rotation_matrix << cos(rotation_angle),-sin(rotation_angle),
                        sin(rotation_angle),cos(rotation_angle);
    
    Eigen::MatrixXd sub_mat= linearly_transformed.tf_points.rowwise() - linearly_transformed.center.head(2).transpose();
    Eigen::MatrixXd mul_mat= rotation_matrix * sub_mat.transpose() ;
    Eigen::MatrixXd complete_transformed_mat=mul_mat.transpose().rowwise() + linearly_transformed.center.head(2).transpose(); 

    Eigen::VectorXd angles=pts_to_be_transformed.tf_points.col(colCount-1);
    // std::cout << angles*(180/M_PI) << std::endl;
    // std::cout << "############"<< rotation_angle << "###########" << std::endl;
    // angles=angles.array() - rotation_angle;
    CalcTrajectoryYawFromXY(complete_transformed_mat, angles);
    // std::cout << angles*(180/M_PI) << std::endl;
    complete_transformed_mat.conservativeResize(complete_transformed_mat.rows(), complete_transformed_mat.cols()+1);
    complete_transformed_mat.col(complete_transformed_mat.cols()-1) = angles;

    //SETTING THE VALUES
    TransformationPoints transformed_points(linearly_transformed.center,complete_transformed_mat);
    return transformed_points;

    
}


/**********************************************************************************************************************************

    FUNCTION        : AccomplishBehavior()
    DESCRIPTION     : This function churns out local trajectories completing all callbacks in order.
    INPUT PARAMS    : ARGUMENTS
                        ~ localization      : input localization
                        ~ vehicleDB         : a class object that stores all the required fields
                     GLOBAL PARAMS
                        ~ NA
    OUTPUT PARAMS   : bool - True or False whether the trajectory generation has been accomplished or not
**********************************************************************************************************************************/

bool LocalPlanner::AccomplishBehavior(Eigen::VectorXd localization, VehicleDataBase &vehicleDB)
{

        clock_t t1,t2,t3,t4;
        t1=clock();
        bool isSuccess = false;
        bool toBeValidated = true;
        bool considerOwnLane = true;
        vehicleDB.laneSwitchCurves.clear();
        LaneSwitchTrajectory laneSwitchObj;
        int stateToBeTaken=vehicleDB.state_to_be_taken;
        try
        {
            // ROS_INFO("[%s] [STATETOBETAKEN == %d]",CurrentDateTime().c_str(),stateToBeTaken);
            
            switch(stateToBeTaken)
            {
                case VERIFY_LANE_CHANGE:

                    // ROS_INFO("[%s] [VERIFY_LANE_CHANGE SELECTED]",CurrentDateTime().c_str());
                    if(vehicleDB.present_lane_id != 1)
                    {
                        // ROS_INFO("[%s] [GENERATING CURVE FOR LEFT LANE]",CurrentDateTime().c_str());
                        int leftLaneId = vehicleDB.present_lane_id - 1;
                        TransformationPoints leftTrajectory = GetLaneSwitchCurve(localization,leftLaneId,vehicleDB);
                        laneSwitchObj.switchCurve = leftTrajectory;
                        laneSwitchObj.direction = -1;
                        vehicleDB.laneSwitchCurves.push_back(laneSwitchObj);

                    }
                    if(vehicleDB.present_lane_id != int(ROAD_WIDTH/LANE_WIDTH))
                    {
                        // ROS_INFO("[%s] [GENERATING CURVE FOR RIGHT LANE]",CurrentDateTime().c_str());
                        int rightLaneId = vehicleDB.present_lane_id + 1;
                        TransformationPoints rightTrajectory = GetLaneSwitchCurve(localization,rightLaneId,vehicleDB);
                        laneSwitchObj.switchCurve = rightTrajectory;
                        laneSwitchObj.direction = 1;
                        vehicleDB.laneSwitchCurves.push_back(laneSwitchObj);
                    }
                    break;
                    

                case PLCL:

                    ROS_INFO("[%s] [PLCL SELECTED]",CurrentDateTime().c_str());
                    if(vehicleDB.present_lane_id != 1)
                    {
                        ROS_INFO("[%s] [GENERATING CURVE FOR LEFT LANE]",CurrentDateTime().c_str());
                        int leftLaneId = vehicleDB.present_lane_id - 1;
                        TransformationPoints leftTrajectory = GetLaneSwitchCurve(localization,leftLaneId,vehicleDB);
                        laneSwitchObj.switchCurve = leftTrajectory;
                        laneSwitchObj.direction = -1;
                        vehicleDB.laneSwitchCurves.push_back(laneSwitchObj);
                    }
                    break;

                case PLCR:

                    ROS_INFO("[%s] [PLCR SELECTED]",CurrentDateTime().c_str());
                    if(vehicleDB.present_lane_id != int(ROAD_WIDTH/LANE_WIDTH))
                    {
                        ROS_INFO("[%s] [GENERATING CURVE FOR RIGHT LANE]",CurrentDateTime().c_str());
                        int rightLaneId = vehicleDB.present_lane_id + 1;
                        TransformationPoints rightTrajectory = GetLaneSwitchCurve(localization,rightLaneId,vehicleDB);
                        laneSwitchObj.switchCurve = rightTrajectory;
                        laneSwitchObj.direction = 1;
                        vehicleDB.laneSwitchCurves.push_back(laneSwitchObj);
                    }
                    break;

                case TAKE_LEFT:

                    ROS_INFO("[%s] [TAKE_LEFT SELECTED]",CurrentDateTime().c_str());
                    if(vehicleDB.present_lane_id != 1)
                    {
                        ROS_INFO("[%s] [GENERATING CURVE FOR LEFT LANE]",CurrentDateTime().c_str());
                        int leftLaneId = vehicleDB.present_lane_id;
                        TransformationPoints leftTrajectory = GetLaneSwitchCurve(localization,leftLaneId,vehicleDB);
                        vehicleDB.presentTrajectory=leftTrajectory;
                        bool considerOwnLane = false;
                        bool toBeValidated = false;
                        //CHECK WHTHER LEFT IS ACHIEVED OR NOT                        
                    }
                    break;

                case TAKE_RIGHT:
        
                    ROS_INFO("[%s] [TAKE_RIGHT SELECTED]",CurrentDateTime().c_str());
                    if(vehicleDB.present_lane_id != int(ROAD_WIDTH/LANE_WIDTH))
                    {
                        ROS_INFO("[%s] [GENERATING CURVE FOR RIGHT LANE]",CurrentDateTime().c_str());
                        int rightLaneId = vehicleDB.present_lane_id ;
                        TransformationPoints rightTrajectory = GetLaneSwitchCurve(localization,rightLaneId,vehicleDB);
                        vehicleDB.presentTrajectory = rightTrajectory;
                        bool considerOwnLane = false;
                        bool toBeValidated = false;
                    }
                    break;
                
                default:
                    break;

                
            }
            t2 = clock();
            // std::cout << "SWITCH: " << float(t2-t1)/CLOCKS_PER_SEC<<std::endl;
            if(considerOwnLane)              //If your not concentrating Highway lane change related planner then u can directly focus on this alone
            {

                // ROS_INFO("[%s] [CONSIDERING HOME LANE]",CurrentDateTime().c_str());                
                list<TransformationPoints> trajectoryList =	GenerateAugmentedTrajectories(vehicleDB.local_map_resolution,
                                                                            vehicleDB.local_map_width,
                                                                            vehicleDB.present_lane_id,
                                                                            localization,
                                                                            vehicleDB.ref_trajectory);    //This is the main function to generate Augmented trajectory based on reference trajectory point and map information on top of map

                
                t3 = clock();
                // std::cout << "GenerateAugmentedTrajectories: " << float(t3-t2)/CLOCKS_PER_SEC<<std::endl;

                vehicleDB.generatedPathList=trajectoryList;

                //Jammy
                // int validatedIndx;
                //Jammy
                // int validatedIndx = ValidateTrajectory(trajectoryList,
                //                                         vehicleDB.vehicleModel, 
                //                                         vehicleDB.local_map,
                //                                         vehicleDB.sub_path_id,
                //                                         stateToBeTaken,
                //                                         toBeValidated,
                //                                         vehicleDB.local_map_width,
                //                                         vehicleDB.local_map_resolution,localization);

                int validatedIndx = 0;

                t4 = clock();
                std::cout << "ValidateTrajectory: " << validatedIndx<<std::endl;
                std::cout << "ValidateTrajectory: " << trajectoryList.size()<<std::endl;

                // ROS_INFO("[%s] [VALIDATION COMPLETE]",CurrentDateTime().c_str());                
                //Jammy
                // validatedIndx = 0;
                ////Jammy
                list <TransformationPoints> :: iterator it = trajectoryList.begin();
                advance(it,validatedIndx);

            
                TransformationPoints validatedTrajectory = (*it);

                // trajectoryPair=std::make_pair(0,&validatedTrajectory);
                // laneSwitchCurve.push_back(trajectoryPair);

                vehicleDB.sub_path_id = validatedIndx;
                vehicleDB.presentTrajectory=validatedTrajectory;
                
            }
            // ROS_INFO("[%s] ACCOMPLISH BEHAVIOR COMPLETED",CurrentDateTime().c_str());
            // ROS_INFO("[%s] SUB PATH ID SELECTED--> %d",CurrentDateTime().c_str(),vehicleDB.sub_path_id);
            isSuccess = true;
        }
        
        catch(char except)
        {
            // std::cout << "YES the error"
            ROS_INFO("[%s] EXCEPTION FROM ACCOMPLISH BEHAVIOR",CurrentDateTime().c_str());
            // cerr << e.what() << '\n';
            
        }
        
        return isSuccess;
    
}



/**********************************************************************************************************************************

    FUNCTION        : ValidateTrajectory()
    DESCRIPTION     : This function is used to validate the trajectory shifts considering objects to the sides.
    INPUT PARAMS    : ARGUMENTS
                        ~ currentID         : cuurent path id
                        ~ number_traj       : Total number of sub-trajectories
                        ~ local_map         : Local Map
                        ~ map_width         : Width of local map
                        ~ grid_resolution   : resolution of local map
                     GLOBAL PARAMS
                        ~ NA
    OUTPUT PARAMS   : bool - True or False whether we can shift or not.
**********************************************************************************************************************************/
int LocalPlanner::ValidateTrajectory(list<TransformationPoints> trajectory_list,
                                                    TransformationPoints vehicle_model,
                                                    Eigen::MatrixXd& local_map, //CHANGEE
                                                    int sub_path_id,
                                                    int stateToBeTaken,
                                                    bool toBeValidated, 
                                                    int map_width,
                                                    float grid_resolution,Eigen::VectorXd localization)
{
    // ROS_INFO("[%s] [VALIDATE TRAJECTORY ACCESSED]",CurrentDateTime().c_str());
    //Jammy  
    int best_indx = sub_path_id;
    //Jammy
    try
    {
                    
            list <TransformationPoints> :: iterator it;
            Eigen::MatrixXd collisionInfo(trajectory_list.size(),2);
            collisionInfo.col(1).fill(trajectory_list.front().tf_points.rows());
            int count=0; 
            for(it = trajectory_list.begin(); it != trajectory_list.end(); ++it) 
            {
                Eigen::MatrixXd trajectory=(*it).tf_points;
                collisionInfo(count,0)=count;
                for(int indx=0; indx < trajectory.rows(); indx++)
                {
                    TransformationPoints transVehicleModel = TransformCoordinates(vehicle_model,trajectory.row(indx));
                    // ROS_INFO("[%s] [CHECKCOLLISION ACCESSED]",CurrentDateTime().c_str());    
                    bool isCollision = CheckCollision(transVehicleModel,local_map);
                    // ROS_INFO("[%s] [CHECKCOLLISION]",CurrentDateTime().c_str());    

                    if(isCollision)
                    {
                        collisionInfo(count,1)=indx+1;
                        break;
                    }
                
                }
                count++;
            }
            
            std::cout << collisionInfo << std::endl;
            best_indx = CalculateCost(collisionInfo, trajectory_list.front().tf_points.rows(), stateToBeTaken,localization);
            
            // if( toBeValidated && best_indx != sub_path_id)
            // { 
            //     if(ValidateShift(sub_path_id,trajectory_list.size(),local_map, map_width,grid_resolution))
            //     {

            //         return best_indx;
            //     }
            // }
            
            // ROS_INFO("[%s] TRAJECTORY VALIDATION COMPLETE",CurrentDateTime().c_str());    
     }
    catch(char except)
    {
        // ROS_INFO("[%s] [EXCEPTION FROM VALIDATE TEAJECTORY]",CurrentDateTime().c_str());    
        throw LOCALIZATION_EXCEPTION;
    }

     //Jammy
    //  sub_path_id=0;
     //Jammy
     return best_indx; 
}

/**********************************************************************************************************************************

    FUNCTION        : ValidateShift()
    DESCRIPTION     : This function is used to validate the trajectory shifts considering objects to the sides.
    INPUT PARAMS    : ARGUMENTS
                        ~ currentID         : cuurent path id
                        ~ number_traj       : Total number of sub-trajectories
                        ~ local_map         : Local Map
                        ~ map_width         : Width of local map
                        ~ grid_resolution   : resolution of local map
                     GLOBAL PARAMS
                        ~ NA
    OUTPUT PARAMS   : bool- True or False whether we can shift or not.
**********************************************************************************************************************************/
bool LocalPlanner::ValidateShift(int currentID, int number_traj, Eigen::MatrixXd local_map,int map_width, float grid_resolution )
{
    float left_end = ((TRAJECTORY_GAP * currentID) + (VEHICLE_BREADTH / 2)) / grid_resolution;
    float right_end = ((TRAJECTORY_GAP * (number_traj - currentID - 1)) + (VEHICLE_BREADTH / 2)) / grid_resolution;

    Eigen::MatrixXd validate_map = local_map.block((int(map_width / 2) - left_end), (map_width / 2),int(VALIDATE_SHIFT_TUNER / grid_resolution),
                                    int(LANE_WIDTH / grid_resolution));

    float mean = validate_map.mean();

    if(mean < VALIDATE_SHIFT_THRESHOLD)
    {
        // ROS_INFO("[%s] VALIDATE SHIFT ALLOWED",CurrentDateTime().c_str());
        return true;
    }   
    else
    {
        // ROS_INFO("[%s] VALIDATE SHIFT REJECTED",CurrentDateTime().c_str());  
        return false;
    }
        
}


TransformationPoints LocalPlanner::GetLaneSwitchCurve(Eigen::VectorXd localization,int laneID, VehicleDataBase vehicleDB)
{

    // int leftLaneId = vehicleDB.present_lane_id - 1;
    list<TransformationPoints> trajectoryList =	GenerateAugmentedTrajectories(vehicleDB.local_map_resolution,
                                                                            vehicleDB.local_map_width,
                                                                            laneID,
                                                                            localization,
                                                                            vehicleDB.ref_trajectory);
    list <TransformationPoints> :: iterator it = trajectoryList.begin();
    // ROS_INFO("[%s] [trajectoryList] [%d]",CurrentDateTime().c_str(),trajectoryList.size());  
    int requiredIndex = (trajectoryList.size()/2)-1;
    // ROS_INFO("[%s] [REQUIRED INDEX] [%d]",CurrentDateTime().c_str(),requiredIndex);  
    int finalIndex = requiredIndex >= 0 ? requiredIndex : 0;
    // ROS_INFO("[%s] [finalIndex INDEX] [%d]",CurrentDateTime().c_str(),finalIndex);  
    advance(it,finalIndex);
    TransformationPoints outTrajectory = (*it);
    int out_index = outTrajectory.tf_points.rows()-1;
    if(OUT_MAX_INDEX < out_index && OUT_MIN_INDEX < out_index)
    {
        out_index = (( vehicleDB.currentSpeed - IN_MIN_SPEED) * (OUT_MAX_INDEX - OUT_MIN_INDEX) /
                                    (IN_MAX_SPEED - IN_MIN_SPEED)) + OUT_MIN_INDEX ;
    }
    // out_index=15;
    // ROS_INFO("[%s] [out_index INDEX] [%d]",CurrentDateTime().c_str(),out_index);  
    double start[] = {outTrajectory.center(0),outTrajectory.center(1),outTrajectory.center(2)};
    double goal[] = {outTrajectory.tf_points.row(out_index)(0), outTrajectory.tf_points.row(out_index)(1), outTrajectory.tf_points.row(out_index)(2)};
    // std :: cout << "***************************************8" << std::endl; 
    // std :: cout << start[2] << std::endl; 
    // std :: cout << goal[2] << std::endl;  
    // std :: cout << "***************************************8" << std::endl; 


    
    // ROS_INFO("[%s] [GENERATING DUBINS CURVE]",CurrentDateTime().c_str());  
    

    int out_radius = (( vehicleDB.currentSpeed - IN_MIN_SPEED) * (OUT_MAX_TRADIUS - OUT_MIN_TRADIUS) /
                                    (IN_MAX_SPEED - IN_MIN_SPEED)) + OUT_MIN_TRADIUS ;
    
    //In Dubins.h file
    DubinsPath path;
    dubins_shortest_path(&path, start, goal, out_radius);
    Eigen::MatrixXd lanePathMatrix;
    lanePathMatrix = dubins_path_sample_many(&path, DUBINS_RESOLUTION, NULL);
    // ROS_INFO("[%s] [DUBINS GENEATED] [%lf]",CurrentDateTime().c_str(),vehicleDB.currentSpeed);  

    TransformationPoints laneCurve(outTrajectory.center, lanePathMatrix);
    // std :: cout << "CHECK 3" << std::endl;

    return laneCurve;
}




/**********************************************************************************************************************************

    FUNCTION        : BuildLaneSelectionMap()
    DESCRIPTION     : This function is used to map the states w.r.t their corresponding int values.
    INPUT PARAMS    : ARGUMENTS
                        ~ NA
                     GLOBAL PARAMS
                        ~ NA
    OUTPUT PARAMS   : map<int,int>
**********************************************************************************************************************************/
map<int,int> LocalPlanner::BuildLaneSelectionMap()
{
    map<int, int> laneSelectionMap;
    laneSelectionMap[KEEP_LANE]=0;
    laneSelectionMap[PLCL]=0;
    laneSelectionMap[PLCR]=0;
    laneSelectionMap[VERIFY_LANE_CHANGE]=0;
    laneSelectionMap[TAKE_LEFT]=-1;
    laneSelectionMap[TAKE_RIGHT]=1;
    return laneSelectionMap;
            
}

void LocalPlanner::CalcTrajectoryYawFromXY(Eigen::MatrixXd complete_transformed_mat,Eigen::VectorXd &angles)
{
  
  for (unsigned int i = 1; i < complete_transformed_mat.rows() - 1; ++i)
  {
    const double dx = complete_transformed_mat.row(i + 1)(0)-complete_transformed_mat.row(i -1)(0);
    const double dy = complete_transformed_mat.row(i + 1)(1)-complete_transformed_mat.row(i -1)(1);
     angles.row(i)(0) = std::atan2(dy, dx);
  }
  if (complete_transformed_mat.rows() > 1)
  {
    angles.row(0)(0) = angles.row(1)(0);
    angles.row(complete_transformed_mat.rows()-1)(0) = angles.row(complete_transformed_mat.rows()-2)(0);
  }
}