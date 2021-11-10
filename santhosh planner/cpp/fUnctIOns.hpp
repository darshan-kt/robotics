#ifndef FUNCTIONS_H
#define FUNCTIONS_H
#include <cmath>
#include "Eigen/Dense"
#include <opencv2/opencv.hpp>



int inRo(float val){
	return (int)round(val);}

std::pair<int,int> inRo(std::pair<float,float> pt){
	return std::make_pair(inRo(pt.first),inRo(pt.second));}

float distance(std::pair<float,float> pt1,std::pair<float,float> pt2){
	return sqrt(pow(pt1.first - pt2.first,2) + pow(pt1.second - pt2.second,2));}

bool isIn(cv::Mat img,std::pair<float,float> pt){
	return (pt.first >= 0 && pt.first < img.cols && pt.second >= 0 && pt.second < img.rows) ? true:false;}

unsigned int pixAt(cv::Mat& img,std::pair<float,float> pt){
	std::pair<int,int> pt_int = inRo(pt);

	if(isIn(img,pt_int) == true){
		return img.at<uchar>(pt_int.second,pt_int.first);}

	else{
		throw std::invalid_argument("ERROR, Point outside of the map");}
}

void chngPix(cv::Mat& img,std::pair<float,float> pt,int Color){
	std::pair<int,int> pt_int = inRo(pt);
	if(isIn(img,pt_int)){
		img.at<uchar>(pt_int.second,pt_int.first) = Color;}

	else{
		throw std::invalid_argument("ERROR, Point outside of the map");}
}

void dispImg(std::string img_name,cv::Mat& img,int waitTime){
	cv::imshow(img_name,img);
	cv::waitKey(waitTime);
}

Eigen::MatrixXf matTOeigenOperation(cv::Mat& img,float val){

	Eigen::MatrixXf img_eigen = Eigen::MatrixXf::Zero(img.rows,img.cols);
	std::vector<cv::Point> nonZerPts;
	cv::findNonZero(img,nonZerPts);

	for(int i = 0;i < nonZerPts.size();i++){
		cv::Point pt = nonZerPts[i];
		img_eigen(pt.y,pt.x) = val;
	}

	return img_eigen;
}

#endif