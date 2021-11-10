#include <iostream>
#include <cmath>
#include <opencv2/opencv.hpp>
#include "trEE.hpp"
#include "qUEUE.hpp"
#include "dIctIOnAry.hpp"
#include "cOnstAnts.hpp"
#include "fUnctIOns.hpp"
#include "Eigen/Dense"
#include "grAssfIrE.hpp"
#include "brUshfIrE.hpp"


struct struct_pathPts{
	std::pair<float,float> cur_location,prev_location,centre;
	float orientation,alpha;
};

struct struct_action{
	std::pair<float,float> location,centre;
	float orientation,alpha,cost;
};


class AstarBicycle{
public:
	std::pair<std::pair<float,float>,std::pair<float,float>> stGo;
	float orientation;
	cv::Mat img;
	int img_h = img.rows;
	int img_w = img.cols;
	float len_veh = 5;
	float velocity = 5;
	float time_step = 3;
	Eigen::MatrixXf costs_heuristic;
	std::vector<Tree> data_tree;

	AstarBicycle(std::pair<std::pair<float,float>,std::pair<float,float>> stGo,float orientation,cv::Mat img){
		this->stGo = stGo;
		this->orientation = orientation;
		this->img = img;

		clock_t t1,t2;
		t1 = clock();
		GrassFire grassObj(img,stGo.second);
		this->costs_heuristic = grassObj.genCosts();
		std::cout<<"grAssfIrE gEnErAtEd"<<std::endl;
		t2 = clock();
		// std::cout<<float(t2-t1)/CLOCKS_PER_SEC<<std::endl;
		std::cout<<"##################################"<<std::endl;
	}

	float Heuristic(std::pair<float,float> pt){
		return distance(pt,inRo(pt)) + this->costs_heuristic(inRo(pt.second),inRo(pt.first));}

	std::vector<struct_action> createActionSet(std::pair<float,float> node,float orientation){
		std::vector<struct_action> actSet;
		float radi,orientation_new,theta_trans;
		std::pair<float,float> node_new,centre_botFram,loc_trans,centre_global;

		int alpha = -30;
		while(alpha <= 30){
			if(round(alpha) != 0){
				radi = this->len_veh/tan(alpha*PI/180);
				
				orientation_new = orientation + (this->velocity*tan(alpha*PI/180)/this->len_veh)*this->time_step;
				node_new = std::make_pair(node.first + (len_veh/tan(alpha*PI/180))*(sin(orientation_new) - sin(orientation)),node.second + (len_veh/tan(alpha*PI/180))*(cos(orientation) - cos(orientation_new)));

				centre_botFram = std::make_pair(0,radi);
				theta_trans = -orientation;
				loc_trans = std::make_pair(-node.first,-node.second);
				centre_global = std::make_pair(radi*sin(theta_trans)-loc_trans.first,radi*cos(theta_trans)-loc_trans.second);

				struct_action act;
				act.location = node_new;
				act.orientation = orientation_new;
				act.alpha = alpha*PI/180;
				act.centre = centre_global;
				act.cost = 1.03*time_step;

				actSet.push_back(act);
			}

			else{
				node_new = std::make_pair(node.first + this->velocity*cos(orientation)*time_step,node.second + this->velocity*sin(orientation)*time_step);
				orientation_new = orientation;

				centre_global = std::make_pair(INF,INF);

				struct_action act;
				act.location = node_new;
				act.orientation = orientation_new;
				act.alpha = alpha*PI/180;
				act.centre = centre_global;
				act.cost = time_step;

				actSet.push_back(act);
			}

			alpha = alpha + 5;

		}

		return actSet;
	}

	void gen_action_path(std::pair<float,float> node1,float theta1,float alpha,std::vector<std::pair<float,float>>& path_pts){

		int n = 30;
		float  time_jump = time_step/n;
		float theta2;
		std::pair<float,float> node2;

		for(int i = 0;i < n;i++){
			theta2 = theta1 + (this->velocity*tan(alpha)/this->len_veh)*time_jump;
			if(alpha != 0){
				node2 = std::make_pair(node1.first + (this->len_veh/tan(alpha))*(sin(theta2)-sin(theta1)),node1.second + (this->len_veh/tan(alpha))*(cos(theta1)-cos(theta2)));}
			else{
				node2 = std::make_pair(node1.first + this->velocity*cos(theta1)*time_jump,node1.second + this->velocity*sin(theta1)*time_jump);}

			if(isIn(this->img,node2)){
				path_pts.push_back(node2);
				chngPix(img,node2,125);
			}

			node1 = node2;
			theta1 = theta2;
		}
	}

	void disp_action(cv::Mat& img,std::pair<float,float> loc1,std::pair<float,float> loc2,float alpha,std::pair<float,float> centre){
		float radi;
		float time_rev;
		if(round(alpha*180/PI) != 0){
			radi = this->len_veh/tan(alpha);
			time_rev = 2*PI*std::abs(radi)/this->velocity;

			if(time_step < time_rev){
				float ang1 = atan2(loc1.second-centre.second,loc1.first-centre.first);
				float ang2 = atan2(loc2.second-centre.second,loc2.first-centre.first);

				if(tan(alpha) > 0){
					if(ang2 < ang1){
						ang2 = ang2 + 2*PI;}
				}
				else{
					if(ang2 > ang1){
						ang1 = ang1 + 2*PI;}
				}

				// cv::ellipse(img, cv::Point(inRo(centre.first),inRo(centre.second)), cv::Size(inRo(radi),inRo(radi)),0,ang1*180/PI,ang2*180/PI,175,1);
			}

			else{
				;// cv::circle(img,cv::Point(inRo(centre.first),inRo(centre.second)),inRo(radi),175,1);
			}
		}

		else{
			;// cv::line(img,cv::Point(inRo(loc1.first),inRo(loc1.second)),cv::Point(inRo(loc2.first),inRo(loc2.second)),175,1);
		}
	}

	bool obstacle_check(std::pair<float,float> node1,float theta1,float alpha){
		int n = 30;
		float time_jump = this->time_step/n;
		float theta2;
		std::pair<float,float> node2;

		for(int i = 0;i <= n;i++){
			theta2 = theta1 + (this->velocity*tan(alpha)/this->len_veh)*time_jump;
			if(alpha != 0){
				node2 = std::make_pair(node1.first + (this->len_veh/tan(alpha))*(sin(theta2) - sin(theta1)),node1.second + (this->len_veh/tan(alpha))*(cos(theta1) - cos(theta2)));}
			else{
				node2 = std::make_pair(node1.first + this->velocity*cos(theta1)*time_jump,node1.second + this->velocity*sin(theta1)*time_jump);}

			if(isIn(this->img,node2)){
				if(pixAt(this->img,node2) == 255){
					return true;}
			}
			else{
				return true;}

			node1 = node2;
			theta1 = theta2;
		}

		return false;
	}

	void getPathPts(bool solution,Tree end_node,std::vector<struct_pathPts>& dts_path,std::vector<std::pair<float,float>>& glob_path_pts){
		std::pair<float,float> start = this->stGo.first;
		Tree prev = end_node;

		unsigned int cur_ind;
		Tree cur;

		if(solution){
			while(true){
				cur_ind = prev.parent;
				cur = this->data_tree[cur_ind];

				struct_pathPts path_pt;
				path_pt.cur_location = cur.location;
				path_pt.orientation = cur.orientation;
				path_pt.prev_location = prev.location;
				path_pt.alpha = prev.alpha;
				path_pt.centre = prev.centre;
				dts_path.insert(dts_path.begin(),path_pt);

				if(distance(cur.location,start) == 0){
					break;}

				prev = cur;
			}

			glob_path_pts.push_back(start);
			for(int i = 0;i < dts_path.size();i++){
				struct_pathPts path_pt = dts_path[i];
				std::vector<std::pair<float,float>> action_path_pts;

				this->gen_action_path(path_pt.cur_location,path_pt.orientation,path_pt.alpha,action_path_pts);
				glob_path_pts.insert(glob_path_pts.end(),action_path_pts.begin(),action_path_pts.end());
			}

		}
		else{
			throw std::invalid_argument("ERROR, solution = false :=> no Path found");
		}
	}

	bool genOptima(std::vector<struct_pathPts>& way_pts,std::vector<std::pair<float,float>>& glob_path_pts){
		clock_t t1,t2;
		t1 = clock();
		img = this->img.clone();
		std::pair<float,float> start = this->stGo.first;
		std::pair<float,float> goal = this->stGo.second;

		Queue que_open;
		Dictionary dict_open;
		Dictionary dict_closed;

		bool solution = false;

		Tree start_node(start,this->orientation,0,std::make_pair(INF,INF),0,this->Heuristic(start)/this->velocity);
		start_node.ind = this->data_tree.size();

		que_open.push(start_node);
		dict_open.append(start_node);
		this->data_tree.push_back(start_node);
		Tree node_current;

		while(que_open.length() > 0){			
			node_current = que_open.pop();
			dict_open.del(node_current);
			std::pair<float,float> loc = node_current.location;

			if(distance(node_current.location,goal) < 20){
				solution = true;
				break;
			}

			dict_closed.append(node_current);
			std::vector<struct_action> action_set = this->createActionSet(node_current.location,node_current.orientation);

			for(int i = 0;i < action_set.size();i++){
				struct_action action = action_set[i];

				std::pair<float,float> node_successor_loc = action.location;
				float node_successor_orien = action.orientation;
				float node_successor_g = node_current.g + action.cost;
				float node_successor_f = node_successor_g + (this->Heuristic(node_successor_loc)/this->velocity);

				Tree node_successor(node_successor_loc,node_successor_orien,action.alpha,action.centre,node_successor_g,node_successor_f);
				node_successor.parent = node_current.ind;
				node_successor.ind = this->data_tree.size();
				this->data_tree.push_back(node_successor);

				bool val_open = dict_open.isAvailable(node_successor);
				bool val_closed = dict_closed.isAvailable(node_successor);

				bool valObs = this->obstacle_check(node_current.location,node_current.orientation,node_successor.alpha);

				if(!valObs){
					if(!val_closed){
						if(!val_open){
							que_open.push(node_successor);
							dict_open.append(node_successor);}

						else{
							Tree open_successor = dict_open.get(node_successor);
							if(node_successor.g < open_successor.g){
								open_successor = node_successor;}
						}
					}
				}
			}
		}
		t2 = clock();
		std::cout<<node_current.g<<std::endl;
		// std::cout<<float(t2-t1)/CLOCKS_PER_SEC<<std::endl;
		this->getPathPts(solution,node_current,way_pts,glob_path_pts);
		return solution;
	}
};