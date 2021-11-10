/*************************************************************************************************************************************
File Name: Auxilary Functions
Developed by: Prajwal Brijesh Ainapur
Description: Contains the functions utilized by the activate local planner such as collision check and cost function.

Dependacies: Eigen/Dense, Config file, Transformation Points
Initial Version: 10/09/2019
Last Modification on: 11/11/2019
*************************************************************************************************************************************/



#include <iostream>
#include <list>
#include <math.h>
#include "./Eigen/Dense"
#include "transformation_points.h"
#include "config.h"
// #include "time.h"

using namespace std;
using namespace FluxLocalPlanner;



bool computeDist(Eigen::VectorXd A, Eigen::VectorXd B)
{
	float eucddist = sqrt(pow((A(0) - B(0)),2) + pow((A(1) - B(1)),2));

	//##Prajwal: Threshold Distance - 2m; If distance is more than that, enable the clear flag
	if (eucddist < 2)
	{
		return false;
	}
	else
	{
		return true;
	}

}
/**********************************************************************************************************************************
    
    FUNCTION        : CalculateCost()
    DESCRIPTION     : This function calculates cost dynamically for each sub path generated
                      from the reference line.
    INPUT PARAMS    : ARGUMENTS
                        ~ collision_matrix : It holds each trajectory with their collision point on them
                        ~ grid_length      : Length of the input local_map
                        ~ state_name       : The state for which the trajectories are expected/generated
                     GLOBAL PARAMS
                        ~ NA
    OUTPUT PARAMS   : int - Sub-Path id which has least cost
**********************************************************************************************************************************/
//temp variable - remove after demo
int prevTraj = ABSOLUTE_ZERO;
Eigen::VectorXd prevLoc=Eigen::Vector3d::Zero();
int oldTraj = 0;
bool noTraj = false;
int counter = 0;


int CalculateCost(Eigen::MatrixXd collision_matrix, int grid_length, int state_name, Eigen::VectorXd localization) //nx2
{

    int count = ABSOLUTE_ZERO;
    std::cout <<"grid_length "<< grid_length << std::endl;

    int number_traj = collision_matrix.rows();
    //int optimal_traj = number_traj / ABSOLUTE_TWO;
    int optimal_traj = ABSOLUTE_ZERO;
    int best = ABSOLUTE_ZERO;
    float best_cost = PSUEDO_INF;
    float dist_left = PSUEDO_INF;
    float coll_ego = PSUEDO_INF;

    //##Prajwal: Modify w1 and w2 based on requirements; 
    /*
		If collision matrix is :(0, 10) and (1, 21) with total grid length(number of waypoints) at 25;
		
		cost of 0 is: 0 + 1.5 = 1.5
		cost of 1 is: 1 + 0.4 = 1.4

		Thus traj 1 is preferred. 
		Note: Based on current waypoints, at least a difference of 11 waypoints i.e. 1 meter is required
				for triggering trajectory change. Reduce w2 to bring that value down even further.
    */
    float w1 = 0.1;
    float w2 = 1;

    bool clearFlag = true;
    bool flagger = false;
    int county = 0;

    for(int i = ABSOLUTE_ZERO; i < number_traj; i++)
    {
        //First Cost
        // float dist_middle = (abs(optimal_traj - collision_matrix(i,ABSOLUTE_ZERO)) / number_traj*ABSOLUTE_TWO);
        dist_left = (abs(ABSOLUTE_ZERO - collision_matrix(i,ABSOLUTE_ZERO))) ;
        // float dist_right = (abs(number_traj - ABSOLUTE_ONE - collision_matrix(i,ABSOLUTE_ZERO)) / number_traj*ABSOLUTE_TWO);

        //Collision Cost
        coll_ego = (abs(grid_length - collision_matrix(i,ABSOLUTE_ONE)) );

        // float cost = coll_ego + ((state_name == KEEP_LANE) ? dist_middle : ABSOLUTE_ZERO)
        //                       + ((state_name == PLCL) ? dist_left : ABSOLUTE_ZERO)
        //                       + ((state_name == PLCR) ? dist_right : ABSOLUTE_ZERO);



        float cost = (w1 * coll_ego)  + (w2 * dist_left);

        if(best_cost >= cost)
        {    
            best_cost = cost;
            best = collision_matrix(i,ABSOLUTE_ZERO);
        }
        std::cout<<"Costs:"<< (w1 * coll_ego)<<"|"<<collision_matrix(i,ABSOLUTE_ONE)
        <<"|"<<collision_matrix(i,ABSOLUTE_ZERO)<<"|"<<(w2 * dist_left)<<" Best: "<<best<<std::endl; 

        if (collision_matrix(i,ABSOLUTE_ONE) < 10)
        {
            county++;
        }

    }
    std::cout << "Best1 " << best << std::endl;
        //##Prajwal: If calculated trajectory is not the preferred one i.e. the left most trajectory
    // We need to keep following it; and once it becomes zero back again, we need to stick around for a bit longer
    if (best == 0)
    {
    	if (prevTraj != best)
    	{
    		clearFlag = computeDist(prevLoc, localization);
    	}
    	
    }
    else
    {
    	prevLoc = localization;
    		// clearFlag = true;
    }

    //##Prajwal: If clear flag is enabled, 
    //'we'll follow the new trajectory and update prevTraj' or else 'we'll keep following the old one.''
    
    if (clearFlag and best == 0)
    {
       	prevTraj = best;	
    }
    else
    {
    	best = prevTraj;
    }
    if (oldTraj != best)
    {
        if (counter > 5) //Rolling Window Size adjust based on preference
        {
            oldTraj = best;
        }

        else
        {
            best = oldTraj;
        }

        counter++;
    }
    else
    {
        counter = 0;
        oldTraj = best;
    }


    //##Prajwal: Flag if there are any obstacles under 1m for all generated trajectories
    if (county >= 3)//No of Traectories being generate; Update this with variable from the config file
    {
        flagger = true;
    }


    // std::cout<<"Costs:"<< (w1 * coll_ego)<<"|"<<collision_matrix(best,ABSOLUTE_ONE)<<"|"<<collision_matrix(best,ABSOLUTE_ZERO)<<"|"<<(w2 * dist_left)<<" Best: "<<best<<"\n"; 
    std::cout << "Best " << best << std::endl;
    return best;
}



/**********************************************************************************************************************************
    
    FUNCTION        : CheckCollision()
    DESCRIPTION     : This function calculates checks for collision on input transormation points laying it over the localMap
    INPUT PARAMS    : ARGUMENTS
                        ~ current_pos      : set of transformation points to be checked for collision
                        ~ local_map        : input local map over which the collision is checked
                     GLOBAL PARAMS
                        ~ NA
    OUTPUT PARAMS   : bool - True/False whether there is a collion or not.
**********************************************************************************************************************************/
bool CheckCollision(TransformationPoints current_pos, Eigen::MatrixXd& local_map) //CHANGEE
{
	int row = current_pos.tf_points.rows();
	Eigen::MatrixXd pos_x = current_pos.tf_points.col(0);
	Eigen::MatrixXd pos_y = current_pos.tf_points.col(1);
    int map_row = local_map.rows();

	for(int i = 0; i <row; i++)
	{	
        unsigned int x_floored = floor(pos_x(i));
        unsigned int y_floored = floor(pos_y(i));

        // cout << "OUT" << endl;
		if( (x_floored < map_row && x_floored >=0) && (y_floored < map_row && y_floored >=0 ))
        {
            // cout << pos_x(i) << " " << map_row << " " << pos_y(i) << endl;
            // cout << "IN" << endl;    
            // cout << x_floored << " " << map_row << " " << y_floored << endl;
            // cout
            if(local_map(x_floored, y_floored) != 0 )
            {
                // cout << "IN" << endl;    
			    return true;
            }
                
        }
        else
        {
            // cout << x_floored << " " << map_row << " " << y_floored << endl;
            // if(i ==2)
            // {
            //     throw LOCALIZATION_EXCEPTION;
            // }
            
            // cout<<pos_x(i)<<" -- "<< pos_y(i) <<endl;
            // cout<<"Trajectory's gone out of the local map\n";
            
            // return false;
        }
        
	}
	return false;	
}




