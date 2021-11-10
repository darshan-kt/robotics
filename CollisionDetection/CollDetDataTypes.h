#ifndef COLLDETDATATYPES
#define COLLDETDATATYPES

#include <iostream>
#include <opencv2/opencv.hpp>

#define PI 3.141592653589793238
#define RADIUS_OFFSET 5


void Dilation(cv::Mat& img_map_main, int dilation_size, cv::Mat& img_map_config)
{
	cv::Mat element = cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(2*dilation_size + 1, 2*dilation_size+1), cv::Point(dilation_size, dilation_size));
	dilate(img_map_main, img_map_config, element);
}

std::pair<float,float> Transform(std::pair<float,float> pt_translate, std::pair<std::pair<float,float>,std::pair<float,float>> transMat, std::pair<float,float> pt){
	std::pair<float,float> pt_transf;

	pt_transf.first = -pt_translate.first + transMat.first.first*pt.first + transMat.second.first*pt.second;
	pt_transf.second = -pt_translate.second + transMat.first.second*pt.first + transMat.second.second*pt.second;

	return pt_transf;
}

float Distance(std::pair<float,float> pt1, std::pair<float,float> pt2){
	return sqrt(pow(pt2.first-pt1.first,2) + pow(pt2.second-pt1.second,2));
}

class VehicleParam{
public:
	float length;
	float width;
	float height;

	VehicleParam(){}
	VehicleParam(float leng,float wid) : length(leng), width(wid){}
	VehicleParam(float leng,float wid, float hei) : length(leng), width(wid), height(hei){}
};

class State{
public:
	float x, y, orien;

	State();
	State(float xi, float yi, float orieni) : x(xi),y(yi),orien(orieni){}
};

class Object{
public:
	State state;
	VehicleParam vehicleparam;
	float radiOffset = RADIUS_OFFSET;
	float radiB;
	std::vector<std::pair<float,float>> partCens;

	Object(State st, VehicleParam vehpar) : state(st), vehicleparam(vehpar){
		partPlane();
	}

	void partPlane(){
		partCens.clear();
		float ratio = vehicleparam.length/vehicleparam.width;

		if(ratio >= 1){
			int ratioR = round(ratio);
			float dist_bet_cens = vehicleparam.length/ratioR;
			radiB = sqrt(pow(dist_bet_cens/2,2) + pow(vehicleparam.width/2,2)) + radiOffset;

			for(float i = 0;i < ratioR;i++){
				std::pair<std::pair<float,float>, std::pair<float,float>> transf_mat;

				transf_mat = std::make_pair(std::make_pair(cos(-state.orien),-sin(-state.orien)),std::make_pair(sin(-state.orien),cos(-state.orien)));
				partCens.push_back(Transform(std::make_pair(-state.x,-state.y),transf_mat,std::make_pair(-vehicleparam.length/2 + dist_bet_cens*(i+0.5),0)));	
			}
		}
		else{
			int ratioR = round(1/ratio);
			float dist_bet_cens = vehicleparam.width/ratioR;
			radiB = sqrt(pow(dist_bet_cens/2,2) + pow(vehicleparam.length/2,2)) + radiOffset;

			for(float i = 0;i < ratioR;i++){
				std::pair<std::pair<float,float>, std::pair<float,float>> transf_mat;

				transf_mat = std::make_pair(std::make_pair(cos(-state.orien),-sin(-state.orien)),std::make_pair(sin(-state.orien),cos(-state.orien)));
				partCens.push_back(Transform(std::make_pair(-state.x,-state.y),transf_mat,std::make_pair(0,-vehicleparam.width/2 + dist_bet_cens*(i+0.5))));	
			}
		}

	}

};


void plotPoly(cv::Mat& imgP, VehicleParam vp, State vs){
	std::pair<std::pair<float,float>,std::pair<float,float>> mat = std::make_pair(std::make_pair(cos(-vs.orien),-sin(-vs.orien)),std::make_pair(sin(-vs.orien),cos(-vs.orien)));
	std::pair<float,float> pt_translate = std::make_pair(-vs.x,-vs.y);
	
	std::pair<float,float> pt1, pt2, pt3, pt4;
	pt1 = Transform(pt_translate,mat,std::make_pair(-vp.length/2,vp.width/2));
	pt2 = Transform(pt_translate,mat,std::make_pair(vp.length/2,vp.width/2));
	pt3 = Transform(pt_translate,mat,std::make_pair(vp.length/2,-vp.width/2));
	pt4 = Transform(pt_translate,mat,std::make_pair(-vp.length/2,-vp.width/2));

	cv::Point pt1p = cv::Point(pt1.first,pt1.second); 
	cv::Point pt2p = cv::Point(pt2.first,pt2.second);
	cv::Point pt3p = cv::Point(pt3.first,pt3.second);
	cv::Point pt4p = cv::Point(pt4.first,pt4.second);

	cv::line(imgP,pt1p,pt2p,cv::Vec3b(255,255,255),1);
	cv::line(imgP,pt2p,pt3p,cv::Vec3b(255,255,255),1);
	cv::line(imgP,pt3p,pt4p,cv::Vec3b(255,255,255),1);
	cv::line(imgP,pt4p,pt1p,cv::Vec3b(255,255,255),1);
}



#endif