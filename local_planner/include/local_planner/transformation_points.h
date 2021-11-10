/*************************************************************************************************************************************
File Name: Transformation Points
Developed by: Vimal Raj Ayyappan
Description: Contains the constructor for the transformation points

Dependacies: Eigen/Dense
Initial Version: 10/09/2019
Last Modification on: 10/09/2019
*************************************************************************************************************************************/

#ifndef TRANS_POINTS
#define TRANS_POINTS

#include "Eigen/Dense"

// using namespace Eigen
namespace FluxLocalPlanner
{


    class TransformationPoints
    {
        public:
            TransformationPoints(Eigen::VectorXd center,Eigen::MatrixXd tf_points)
            {
                this->center=center;
                this->tf_points=tf_points;
            }
            // TransformationPoints(){}

            Eigen::VectorXd center;
            Eigen::MatrixXd tf_points;      

    };

}
#endif