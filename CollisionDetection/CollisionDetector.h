#ifndef COLLISIONDETECTORH
#define COLLISIONDETECTORH

#include <opencv2/opencv.hpp>
#include "CollDetDataTypes.h"


// length is along the direction of orientation, width is perpendicular to the orientation direction.



class CollisionDetector{
	cv::Mat mapMain, mapConfig;
	VehicleParam vp;
	float radiOffset = RADIUS_OFFSET;
	float radiB;
	std::vector<std::pair<float,float>> partCens;
public:

	CollisionDetector(cv::Mat& img_map, VehicleParam vp){
		this->vp = vp;
		mapMain = img_map.clone();
		cv::line(mapMain,cv::Point(0,0),cv::Point(mapMain.cols-1,0),255,1);
		cv::line(mapMain,cv::Point(0,0),cv::Point(0,mapMain.rows-1),255,1);
		cv::line(mapMain,cv::Point(0,mapMain.rows-1),cv::Point(mapMain.cols-1,mapMain.rows-1),255,1);
		cv::line(mapMain,cv::Point(mapMain.cols-1,0),cv::Point(mapMain.cols-1,mapMain.rows-1),255,1);

		partPlane();
		Dilation(mapMain,round(radiB),mapConfig);
	}

	void partPlane(){
		partCens.clear();

		float ratio = vp.length/vp.width;

		if(ratio >= 1){
			int ratioR = round(ratio);
			float dist_bet_cens = vp.length/ratioR;
			radiB = sqrt(pow(dist_bet_cens/2,2) + pow(vp.width/2,2)) + radiOffset;

			for(float i = 0;i < ratioR;i++)
				partCens.push_back(std::make_pair(-vp.length/2 + dist_bet_cens*(i+0.5),0));
		}
		else{
			ratio = 1/ratio;
			int ratioR = round(ratio);
			float dist_bet_cens = vp.width/ratioR;
			radiB = sqrt(pow(dist_bet_cens/2,2) + pow(vp.length/2,2)) + radiOffset;

			for(float i = 0;i < ratioR;i++)
				partCens.push_back(std::make_pair(0,-vp.width/2 + dist_bet_cens*(i+0.5)));
		}

	}

	bool edgeCheck(std::pair<float,float> pt){
		if(pt.first < 0 || pt.second < 0 || pt.first >= mapConfig.cols || pt.second >= mapConfig.rows)
			return true;

		return false;
	}

	bool collCheckMap(State vs){
		std::pair<std::pair<float,float>, std::pair<float,float>> transf_mat;
		transf_mat = std::make_pair(std::make_pair(cos(-vs.orien),-sin(-vs.orien)),std::make_pair(sin(-vs.orien),cos(-vs.orien)));

		for(int i = 0;i < partCens.size();i++){
			std::pair<float,float> cen_transf = Transform(std::make_pair(-vs.x,-vs.y),transf_mat,partCens[i]);
	
			if(mapConfig.at<uchar>((uint)cen_transf.second,(uint)cen_transf.first) != 0 || edgeCheck(cen_transf))
				return true;
		}

		return false;
	}

	bool collCheck(State vs,std::vector<Object> obs){
		std::pair<std::pair<float,float>, std::pair<float,float>> transf_mat;
		transf_mat = std::make_pair(std::make_pair(cos(-vs.orien),-sin(-vs.orien)),std::make_pair(sin(-vs.orien),cos(-vs.orien)));

		if(collCheckMap(vs)){
			return true;
		}
		else{
			for(int i = 0;i < obs.size();i++){
				for(int j = 0;j < partCens.size();j++){
					for(int k = 0;k < obs[i].partCens.size();k++){				
						std::pair<float,float> partCenTransf = Transform(std::make_pair(-vs.x,-vs.y),transf_mat,partCens[j]);

						if(Distance(obs[i].partCens[k], partCenTransf) <= radiB + obs[i].radiB)
							return true;
						
					}
				}
			}

		}

		return false;
	}

};


class TruckTrailerCollisonCheck{
	VehicleParam vehicleparamTruck;
	VehicleParam vehicleparamTrailer;
	cv::Mat mapMain;

public:
	TruckTrailerCollisonCheck(){}
	TruckTrailerCollisonCheck(cv::Mat& img_map, State stTru, State stTra, VehicleParam vehParTru, VehicleParam vehParTra){
		vehicleparamTruck = vehParTru;
		vehicleparamTrailer = vehParTra;
		mapMain = img_map;
	}

	bool CollCheck(State stateTruck, State stateTrailer,std::vector<Object> obs){
		CollisionDetector tru_colldet(mapMain,vehicleparamTruck);
		CollisionDetector tra_colldet(mapMain,vehicleparamTrailer);

		bool tru_ret = tru_colldet.collCheck(stateTruck,obs);
		bool tra_ret = tra_colldet.collCheck(stateTrailer,obs);

		if(tru_ret || tra_ret)
			return true;
		else
			return false;
	}
};



#endif