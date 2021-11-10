#ifndef PUREPURSUIT_HPP
#define PUREPURSUIT_HPP

#include <iostream>
#include <algorithm>
#include <cmath>
#include <opencv2/opencv.hpp>
#include "fUnctIOns.hpp"
#include "cOnstAnts.hpp"


class PurePursuit
{
public:
	std::pair<float,float> bot_loc;
	float bot_orien;
	std::vector<std::pair<float,float>> path_pts;
	float velocity;
	float len_veh;
	float lookAhead;
	float time_step;
	std::pair<float,float> goal;
	float steer_max;

	PurePursuit(std::pair<float,float> bot_loc,float bot_orien,std::pair<float,float> goal,std::vector<std::pair<float,float>> path_pts){
		this->bot_loc = bot_loc;
		this->bot_orien = bot_orien;
		this->path_pts = path_pts;
		this->goal = goal;
		this->velocity = VELOCITY_VEHICLE;
		this->len_veh = LENGTH_VEHICLE;
		this->lookAhead = PUREPURSUIT_LOOKAHEAD;
		this->time_step = PUREPURSUIT_TIMESTEP;
		this->steer_max = PUREPURSUIT_STEER_MAX;
		}

	float evalEta(std::pair<float,float> bot_loc,float bot_orien,std::pair<float,float> loc_LA){

		std::pair<float,float> locLAtrnl = std::make_pair<float,float>(loc_LA.first - bot_loc.first,loc_LA.second - bot_loc.second);
		float ct = cos(bot_orien);
		float st = sin(bot_orien);

		std::pair<float,float> loc_lookAhead_transform = std::make_pair(ct*locLAtrnl.first + st*locLAtrnl.second,-st*locLAtrnl.first + ct*locLAtrnl.second);
		float eta = -atan2(loc_lookAhead_transform.second,loc_lookAhead_transform.first);

		return eta;
	}

	float evalAlpha(std::pair<float,float> bot_loc,float bot_orien,std::pair<float,float> loc_lookAhead){
		float Lfw = distance(bot_loc,loc_lookAhead);
		float eta = this->evalEta(bot_loc,bot_orien,loc_lookAhead);
		float alpha = -atan(2*this->len_veh*sin(eta)/Lfw);

		return alpha;
		}

	std::pair<int,std::pair<float,float>> getNearPt(std::pair<float,float> bot_loc){
		std::vector<float> dists;

		for(int i = 0;i < this->path_pts.size();i++){
			dists.push_back(distance(path_pts[i],bot_loc));
		}
		int ind = std::min_element(dists.begin(),dists.end()) - dists.begin();

		return std::make_pair(ind,this->path_pts[ind]);
	}

	std::pair<int,std::pair<float,float>> getLookaheadInd(int ind_nearPt){
		int ind_lookAhead = ind_nearPt;

		while(ind_lookAhead < this->path_pts.size()){
			if(distance(this->path_pts[ind_nearPt],this->path_pts[ind_lookAhead]) >= this->lookAhead){
				return std::make_pair(ind_lookAhead,this->path_pts[ind_lookAhead]);
			}
			ind_lookAhead = ind_lookAhead + 1;
		}

		return std::make_pair(ind_lookAhead-1,this->path_pts[ind_lookAhead-1]);	
	}

	std::pair<std::pair<float,float>, float> predPt(std::pair<float,float> bot_loc,float bot_orien,float alpha){
		alpha = alpha*180/PI;
		float bot_orien_new;
		std::pair<float,float> bot_loc_new;

		if(round(alpha) != 0){
			bot_orien_new = bot_orien + this->velocity*tan(alpha*PI/180/this->len_veh)*this->time_step;
			bot_loc_new = std::make_pair(bot_loc.first + (this->len_veh/tan(alpha*PI/180))*(sin(bot_orien_new) - sin(bot_orien)), bot_loc.second + (this->len_veh/tan(alpha*PI/180))*(cos(bot_orien) - cos(bot_orien_new)));
		}
		else{
			bot_orien_new = bot_orien;
			bot_loc_new = std::make_pair(bot_loc.first + this->velocity*cos(bot_orien)*this->time_step,bot_loc.second + this->velocity*sin(bot_orien)*this->time_step);
		}

		return std::make_pair(bot_loc_new,bot_orien_new);
	}

	std::vector<std::pair<float,float>> genPursuit(){

		std::pair<float,float> bot_loc,bot_loc_new;
		float bot_orien,bot_orien_new;

		bot_loc = this->bot_loc;
		bot_orien = this->bot_orien;

		std::vector<std::pair<float,float>> path_pursuit;
		path_pursuit.push_back(bot_loc);

		while(true){
			if(distance(bot_loc,this->goal) < PUREPURSUIT_GOAL_THRESHOLD){
				break;}

			std::pair<int,std::pair<float,float>>  NearPt = this->getNearPt(bot_loc);
			int ind_nearPt = NearPt.first;
			std::pair<float,float> nearPt = NearPt.second;

			std::pair<int,std::pair<float,float>> LookAheadPt = this->getLookaheadInd(ind_nearPt);
			int ind_lookAhead = LookAheadPt.first;
			std::pair<float,float> pt_lookAhead = LookAheadPt.second;
			float alpha = this->evalAlpha(bot_loc,bot_orien,pt_lookAhead);

	 		if(std::abs(alpha) > this->steer_max){
				alpha = (std::abs(alpha)/alpha)*this->steer_max;
			}

			std::pair<std::pair<float,float>, float> newPt = this->predPt(bot_loc,bot_orien,alpha);
			bot_loc_new = newPt.first;
			bot_orien_new = newPt.second;

			path_pursuit.push_back(bot_loc);
			
			bot_loc = bot_loc_new;
			bot_orien = bot_orien_new;
		}

		return path_pursuit;
	}
};

#endif