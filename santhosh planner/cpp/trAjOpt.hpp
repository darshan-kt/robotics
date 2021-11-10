#include <iostream>
#include <cmath>
#include "pUrEpUrsUIt.hpp"
#include "glObAlplAnnEr.hpp"


class TrajOptimization
{
public:
	float w_obsColl = 0.2;
	float w_smth = 0.4;

	float dmax = 20;
	float leng_veh = 5;

	TrajOptimization(){}

	std::pair<float,float> partDerObsColl(std::pair<float,float> xi,std::pair<float,float> oi){
		std::pair<float,float> deriv;

		if(distance(xi,oi) < this->dmax){
			float deriv_k = 2*this->w_obsColl*(distance(xi,oi) - this->dmax)/distance(xi,oi);
			float deriv_1 = deriv_k*(xi.first - oi.first);
			float deriv_2 = deriv_k*(xi.second - oi.second);
			deriv = std::make_pair(deriv_1,deriv_2);
		}

		else{
			deriv = std::make_pair(0,0);}

		return deriv;
	}

	std::pair<float,float> partDerSmth(int i,std::vector<std::pair<float,float>>& path_pts){
		std::pair<float,float> deriv;

		if(i == 0 || i == path_pts.size() - 1){
			deriv = std::make_pair(0,0);}

		else if(i == 1){
			float deriv_1 = 2*this->w_smth*(path_pts[i+2].first - 4*path_pts[i+1].first + 5*path_pts[i].first - 2*path_pts[i-1].first);
			float deriv_2 = 2*this->w_smth*(path_pts[i+2].second - 4*path_pts[i+1].second + 5*path_pts[i].second - 2*path_pts[i-1].second);}

		else if(i > 1 && i < path_pts.size() - 2){
			float deriv_1 = 2*this->w_smth*(path_pts[i+2].first - 4*path_pts[i+1].first + 6*path_pts[i].first - 4*path_pts[i-1].first + path_pts[i-2].first);
			float deriv_2 = 2*this->w_smth*(path_pts[i+2].second - 4*path_pts[i+1].second + 6*path_pts[i].second - 4*path_pts[i-1].second + path_pts[i-2].second);}

		else if(i == path_pts.size() - 2){
			float deriv_1 = 2*this->w_smth*(-2*path_pts[i+1].first + 5*path_pts[i].first - 4*path_pts[i-1].first + path_pts[i-2].first);
			float deriv_2 = 2*this->w_smth*(-2*path_pts[i+1].second + 5*path_pts[i].second - 4*path_pts[i-1].second + path_pts[i-2].second);}

		return deriv;
	}

	void genOptima(){
		cv::Mat img = cv::imread("mAInmAp.png",0);
		std::pair<std::pair<float,float>,std::pair<float,float>> stGo = std::make_pair(std::make_pair(90,465),std::make_pair(470,460));
		float init_orien = -PI/2;

		AstarBicycle obj_astar(stGo,init_orien,img);
		std::vector<struct_pathPts> way_pts;
		std::vector<std::pair<float,float>> path_pts;
		bool solution = obj_astar.genGlobPath(way_pts,path_pts);
		std::cout<<"A* gEnErAtEd"<<std::endl;
		std::cout<<"###############################"<<std::endl;

		BrushFire brush_obj(img);
		Eigen::MatrixXf mat_dist(img.rows,img.cols);
		std::unordered_map<std::pair<float,float>,std::pair<float,float>,BrushFireHashFunction> mat_ptObs;
		brush_obj.genCosts(mat_dist,mat_ptObs);
		std::cout<<"brUshfIrE gEnErAtEd"<<std::endl;
		std::cout<<"-------------------------------"<<std::endl;

		std::vector<std::pair<float,float>> path_opt = path_pts;
		int itr_max = 50;
		float alpha_lr = 0.1;
		std::vector<std::pair<float,float>> pd_coll_lis;

		for(int itr = 1;itr <= itr_max;itr++){
			std::vector<std::pair<float,float>> path_opt_next;

			if(itr != 0 && itr%10 == 0){
				PurePursuit obj_pp(stGo.first,init_orien,stGo.second,path_opt);
				path_opt = obj_pp.genPursuit();
			}

			for(int i = 0;i < path_opt.size();i++){
				std::pair<float,float> pt = path_opt[i];
				std::pair<int,int> pt_iR = inRo(pt);

				std::pair<float,float> pt_obs = mat_ptObs[pt_iR];
				float pt_obs_dist = mat_dist(pt_iR.second,pt_iR.first);

				std::pair<float,float> pd_coll = partDerObsColl(pt,pt_obs);
				std::pair<float,float> pd_smth = partDerSmth(i,path_opt);

				pd_coll_lis.push_back(pd_coll);
				std::pair<float,float> pt_new = std::make_pair(pt.first - alpha_lr*(pd_coll.first + pd_smth.first),pt.second - alpha_lr*(pd_coll.second + pd_smth.second));

				path_opt_next.push_back(pt_new);
			}
			path_opt = path_opt_next;
			std::cout<<"EpOch : "<<itr<<std::endl;

		}

		// cv::Mat imgD = img.clone();
		// for(int i = 0;i < path_opt.size();i++){
		// 	chngPix(imgD,path_opt[i],155);
		// }
		// dispImg("image",imgD,0);
	}

};
