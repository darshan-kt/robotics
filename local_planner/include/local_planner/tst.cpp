#include "Eigen/Dense"
#include "transformation_points.h"
#include <iostream>
#include <fstream>


using namespace FluxLocalPlanner;
using namespace std;






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


/**********************************************************************************************************************************
CONVERT RADIAN TO DEGREE
**********************************************************************************************************************************/
double RadianToDegree(double radians)
{
    return (radians*(180/M_PI));
}

/**********************************************************************************************************************************
CONVERT DEGREE TO RADIAN
**********************************************************************************************************************************/
double DegreeToRadian(double degrees)
{
    return (degrees*(M_PI/180));
}


TransformationPoints TransformCoordinates(TransformationPoints pts_to_be_transformed, Eigen::VectorXd next_state)

{
    /////DEBUG
    // for pt in vehicle_point_object.input_co_ordinates:
    // new_x=(pt[0]-vehicle_point_object.center[0])*np.cos(angle_of_rotation) + (pt[1]-vehicle_point_object.center[1])*np.sin(angle_of_rotation)+next_state[0]
    // new_y=-(pt[0]-vehicle_point_object.center[0])*np.sin(angle_of_rotation)+(pt[1]-vehicle_point_object.center[1])*np.cos(angle_of_rotation)+next_state[1]
    // lst.append([new_x,new_y])

    // ///PYTRANS
    // Eigen::MatrixXd transformedPoints(pts_to_be_transformed.tf_points.rows(),3);
    // // double rotation_angle= next_state[2];
    // float rotation_angle= -1 *next_state[2];
    // for(int row=0;row<pts_to_be_transformed.tf_points.rows();row++)
    // {
    //     Eigen::VectorXd rowValue=pts_to_be_transformed.tf_points.row(row);
    //     double x=(rowValue(0)-pts_to_be_transformed.center(0))*cos(rotation_angle)  + 
    //                 (rowValue(1)-pts_to_be_transformed.center(1))*sin(rotation_angle) + next_state(0);
    //     double y= -(rowValue(0)-pts_to_be_transformed.center(0))*sin(rotation_angle) +
    //                 (rowValue(1)-pts_to_be_transformed.center(1))*cos(rotation_angle) + next_state(1);
    //     double theta = pts_to_be_transformed.tf_points.row(row)(2)-rotation_angle;
    //     transformedPoints.row(row) << x,y,theta;

    // }
    // TransformationPoints final_points(next_state,transformedPoints);
    // return final_points;
    /////DEBUG
    ///PYTRANS


    //TRANSFORMATION
    // float rotation_angle= -1 *next_state[2];
    float val=pts_to_be_transformed.center(2);
    float rotation_angle= next_state[2]-val;
    int rowCount=pts_to_be_transformed.tf_points.rows();
    int colCount= pts_to_be_transformed.tf_points.cols();

    Eigen::VectorXd displaced_vec = next_state.head(2)-pts_to_be_transformed.center.head(2);
    Eigen::MatrixXd segmented_points(rowCount,2);
    segmented_points=pts_to_be_transformed.tf_points.block(0,0,rowCount,2);
    Eigen::MatrixXd linear_transform_pts=segmented_points.rowwise()+ displaced_vec.transpose();
    TransformationPoints linearly_transformed(next_state,linear_transform_pts);
    // cout << linearly_transformed.tf_points<<endl; 

    //ROTATION
    Eigen::MatrixXd rotation_matrix(2,2);
    rotation_matrix << cos(rotation_angle),-sin(rotation_angle),
                        sin(rotation_angle),cos(rotation_angle);
    
    Eigen::MatrixXd sub_mat= linearly_transformed.tf_points.rowwise() - linearly_transformed.center.head(2).transpose();
    // cout << "subMat" << sub_mat << endl;

    // cout << "----"<<endl;
    // cout << linearly_transformed.tf_points * rotation_matrix << endl;
    // cout << "----"<<endl;
    // Eigen::MatrixXd mul_mat= sub_mat * rotation_matrix;
    Eigen::MatrixXd mul_mat= rotation_matrix * sub_mat.transpose() ;
    // cout << mul_mat << endl;
    Eigen::MatrixXd complete_transformed_mat=mul_mat.transpose().rowwise() + linearly_transformed.center.head(2).transpose(); 

    Eigen::VectorXd angles=pts_to_be_transformed.tf_points.col(colCount-1);
    // angles=((-1)*angles.array())+rotation_angle;
    angles=((-1)*angles.array())+rotation_angle;
    complete_transformed_mat.conservativeResize(complete_transformed_mat.rows(), complete_transformed_mat.cols()+1);
    complete_transformed_mat.col(complete_transformed_mat.cols()-1) = angles;

    //SETTING THE VALUES
    TransformationPoints transformed_points(linearly_transformed.center,complete_transformed_mat);
    return transformed_points;

    
}

int main()

{

    // vector<int8_t> inp_local_map ={1,2,3,4,5,6,7,8,9};
    // vector<double> conv_local_map(inp_local_map.begin(),inp_local_map.end());
    // double* ptr=&conv_local_map[0];
    // Eigen::Map<Eigen::MatrixXd,0,Eigen::Stride<0, 0>> local_map(ptr,3,3); /////////////////
    // cout << local_map << endl;

    // cout << local_map(1,2) << endl;

    float rad1=DegreeToRadian(90);
    Eigen::MatrixXd mat1(5,3);
    mat1 << 2,2,rad1,
            2,3,rad1,
            2,4,rad1,
            2,5,rad1,
            2,6,rad1;

    char s[] ="p1.csv";
    // char* n = &s;
    write_csv(mat1,s);
    // cout << mat1 << endl;
    // cout << "-------------------------------------------" << endl;

    Eigen::Vector3d vec(2,2,rad1);
    // // cout << "YES" << endl;
    TransformationPoints tf(vec,mat1);
    // // cout << "YES" << endl;
    Eigen::Vector3d nxt_st(2,2,DegreeToRadian(90));
    // // cout << "YES" << endl;
    TransformationPoints ret= TransformCoordinates(tf,nxt_st);
    cout << ret.tf_points << endl;
     char t[] ="p2.csv";
    // char* m = &t;
    write_csv(ret.tf_points,t);


    return 0;
}