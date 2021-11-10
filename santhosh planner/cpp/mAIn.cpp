#include <iostream>
#include <string>
#include <opencv2/opencv.hpp>
#include "Eigen/Dense"
#include "trEE.hpp"
#include "qUEUE.hpp"
#include "dIctIOnAry.hpp"
#include "pUrEpUrsUIt.hpp"
#include "fUnctIOns.hpp"
#include "grAssfIrE.hpp"
#include "brUshfIrE.hpp"
#include "glObAlplAnnEr.hpp"
#include "trAjOpt.hpp"

void main_purePursuit(){
	clock_t t1,t2,t3,t4;

	std::vector<std::pair<float,float>> path_pts;
	cv::Mat img = cv::Mat::zeros(500,500,CV_8UC1);

	for(int i = 100;i<=400;i++){
		path_pts.push_back(std::make_pair(i,100));
		img.at<uchar>(100,i) = 255;
	}

	for(int i = 400,j = 100;i>=100 && j <= 400;i--,j++){
		path_pts.push_back(std::make_pair(i,j));
		img.at<uchar>(j,i) = 255;
	}


	t1 = clock();
	PurePursuit obj_pp(path_pts[0],0,path_pts[path_pts.size()-1],path_pts);
	std::vector<std::pair<float,float>> path_pursuit = obj_pp.genPursuit();
	t2 = clock();


	std::cout<<"Time : "<<float(t2-t1)/CLOCKS_PER_SEC<<std::endl;	

	for(int i = 0;i < path_pursuit.size();i++){
		std::pair<float,float> pt = path_pursuit[i];
		std::pair<int,int> pt_int = inRo(pt);

		img.at<uchar>(pt_int.second,pt_int.first) = 155;
	}

	cv::imshow("image",img);
	cv::waitKey(0);
 
}

void main_grassfire(){

	cv::Mat img = cv::imread("mAInmAp.png",0);
	GrassFire grassObj(img,std::make_pair(470,460));
	Eigen::MatrixXf matCosts = grassObj.genCosts();
	cv::Mat img_disp = grassObj.genPath(std::make_pair(90,465),matCosts);

	dispImg("image",img_disp,0);
}

void main_brushfire(){
	cv::Mat img = cv::imread("mAInmAp.png",0);

	// cv::Mat img = cv::Mat::zeros(100,100,CV_8UC1);
	// cv::Mat roi = img(cv::Rect(25,25,50,50));
	// roi.setTo(255);

	// dispImg("image",img,0);
	// chngPix(img,std::make_pair(3,3),255);
	// chngPix(img,std::make_pair(5,5),255);
	// chngPix(img,std::make_pair(7,7),255);

	BrushFire brush_obj(img);
	Eigen::MatrixXf mat_potential(img.rows,img.cols);
	std::unordered_map<std::pair<float,float>,std::pair<float,float>,BrushFireHashFunction> obs_loc_mat_pot;
	brush_obj.genCosts(mat_potential,obs_loc_mat_pot);
}

void main_astar(){
	cv::Mat img = cv::imread("mAInmAp.png",0);
	std::pair<std::pair<float,float>,std::pair<float,float>> stGo = std::make_pair(std::make_pair(90,465),std::make_pair(470,460));
	float orientation = -PI/2;

	AstarBicycle obj_astar(stGo,orientation,img);
	std::vector<struct_pathPts> way_pts;
	std::vector<std::pair<float,float>> glob_path_pts;
	bool solution = obj_astar.genGlobPath(way_pts,glob_path_pts);

	for(int i = 0;i < glob_path_pts.size();i++){
		chngPix(img,glob_path_pts[i],155);
	}
	dispImg("image",img,0);
}

void main_trajOpt(){
	TrajOptimization optObj;
	optObj.genOptima();
}

int main(){
	clock_t t1,t2;
	t1 = clock();
	// main_purePursuit();
	// main_grassfire();
	// main_brushfire();
	// main_astar();
	main_trajOpt();
	t2 = clock();
	std::cout<<"--------C++--------"<<std::endl;
	std::cout<<"Time : "<<float(t2-t1)/CLOCKS_PER_SEC<<std::endl;
}