#ifndef GRASSFIRE_HPP
#define GRASSFIRE_HPP
#include <iostream>
#include <opencv2/opencv.hpp>
#include "Eigen/Dense"
#include "fUnctIOns.hpp"


class GrassFire{
public:
	cv::Mat img;
	std::pair<float,float> goal;

	GrassFire(cv::Mat img,std::pair<float,float> goal){
		this->img = img;
		this->goal = goal;
	}

	std::vector<std::pair<std::pair<float,float>,float>> genNeigh(std::pair<float,float> pt){
		std::vector<std::pair<std::pair<float,float>,float>> neighGlob;
		int xMin = 0;
		int xMax = this->img.cols;
		int yMin = 0;
		int yMax = this->img.rows;

		for(int i = -1;i <= 1;i++){
			for(int j = -1;j <= 1;j++){
				if(!(i == 0 && j == 0)){
					std::pair<float,float> neighPt = std::make_pair(pt.first+i,pt.second+j);
					if(neighPt.first >= xMin && neighPt.first < xMax && neighPt.second >= yMin && neighPt.second < yMax && pixAt(this->img,neighPt) != 255){
						neighGlob.push_back(std::make_pair(neighPt,distance(pt,neighPt)));
					}
				}
			}
		}

		return neighGlob;
	}

	std::pair<int,int> minInd(Eigen::MatrixXf pts){
		float minVal = std::numeric_limits<float>::infinity();
		std::pair<int,int> ind;

		for(int i = 0;i < pts.cols();i++){
			for(int j = 0;j < pts.rows();j++){
				if(pts(j,i) < minVal){
					minVal = pts(j,i);
					ind = std::make_pair(i,j);
				}
			}
		}

		return ind;
	}

	cv::Mat genPath(std::pair<float,float> start,Eigen::MatrixXf mat_costs){
		std::pair<float,float> curr = start;
		cv::Mat imgCp = this->img;

		while(true){
			if(mat_costs(curr.second,curr.first) == 0){
				std::cout<<"Path Generated"<<std::endl;
				break;
			}

			int x_low = curr.first - 1;
			if(x_low < 0){
				x_low = 0;}

			int x_high = curr.first + 2;
			if(x_high > imgCp.cols){
				x_high = imgCp.cols;}

			int y_low = curr.second - 1;
			if(y_low < 0){
				y_low = 0;}

			int y_high = curr.second + 2;
			if(y_high > imgCp.rows){
				y_high = imgCp.rows;}

			Eigen::MatrixXf mat_window = mat_costs.block(y_low,x_low,y_high-y_low,x_high-x_low);

			std::pair<int,int> ind_lowNeigh = this->minInd(mat_window);
			ind_lowNeigh = std::make_pair(ind_lowNeigh.first+x_low,ind_lowNeigh.second+y_low);
			chngPix(imgCp,ind_lowNeigh,155);
			// dispImg("image",imgCp,1);
			curr = ind_lowNeigh;
		}

		return imgCp;
	}

	Eigen::MatrixXf genCosts(){
		Eigen::MatrixXf mat_costs = std::numeric_limits<float>::infinity()*Eigen::MatrixXf::Ones(this->img.rows,this->img.cols);
		mat_costs(this->goal.second,this->goal.first) = 0;
		std::vector<std::pair<float,float>> list_open;
		list_open.push_back(this->goal);

		while(list_open.size() != 0){
			std::pair<float,float> curr = list_open[0];
			list_open.erase(list_open.begin());
			std::vector<std::pair<std::pair<float,float>,float>> childNodes = this->genNeigh(curr);

			for(int j = 0;j < childNodes.size();j++){
				std::pair<std::pair<float,float>,float> child = childNodes[j];
				std::pair<float,float> child_loc = child.first;
				if(pixAt(this->img,child.first) == 0){
					if(mat_costs(curr.second,curr.first) + child.second < mat_costs(child_loc.second,child_loc.first)){
						mat_costs(child_loc.second,child_loc.first) = mat_costs(curr.second,curr.first) + child.second;
						list_open.push_back(child_loc);
					}
				}			
			}
		}

		return mat_costs;
	}
};


#endif