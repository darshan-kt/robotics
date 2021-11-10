#ifndef BRUSHFIRE_HPP
#define BRUSHFIRE_HPP
#include <iostream>
#include <opencv2/opencv.hpp>
#include "cOnstAnts.hpp"
#include "fUnctIOns.hpp"
#include "Eigen/Dense"


class BrushFireHashFunction{
public:
	size_t operator()(const std::pair<float,float> pt) const{
		std::string locX = std::to_string(inRo(pt.first));
		std::string locY = std::to_string(inRo(pt.second));

		std::string str_arg = locX + locY;
		std::hash<std::string> Hash;

		return Hash(str_arg);
	}
};


class BrushFire{
public:
	cv::Mat img;
	std::vector<float> t45;
	std::vector<float> t67;
	std::vector<float> t78;
	std::vector<float> t89;
	std::vector<float> t1011;
	std::vector<float> t012;
	std::vector<float> t112;

	clock_t ct0,ct1,ct2,ct3,ct4,ct5,ct6,ct7,ct8,ct9,ct10,ct11,ct12;


	BrushFire(cv::Mat img){
		this->img = img;}

	std::vector<std::pair<int,int>> getBoundPts(){
		std::vector<std::pair<int,int>> bound_pts;
		cv::Mat img_dilate;

		cv::dilate(this->img.clone(),img_dilate,cv::Mat(),cv::Point(-1,-1),1,1,1);
		cv::Mat img_bound = img_dilate - this->img;
		std::vector<cv::Point> bound_pts_cv;
		cv::findNonZero(img_bound,bound_pts_cv);

		for(int i = 0;i < bound_pts_cv.size();i++){
			std::pair<int,int> pt = std::make_pair(bound_pts_cv[i].x,bound_pts_cv[i].y);
			bound_pts.push_back(pt);
		}

		return bound_pts;
	}

	void getNeighPts(std::pair<float,float> pt,Eigen::MatrixXf& potentials_map,std::vector<std::pair<int,int>>& list_pts_bound,std::vector<float>& list_vals_bound,std::vector<std::pair<int,int>>& list_pts_empty){
		ct0 = clock();

		list_pts_empty.clear();
		list_vals_bound.clear();
		list_pts_bound.clear();
		std::pair<int,int> ind;
		float pot_val_ind;

		std::vector<std::pair<int,int>> neighs_perp;
		neighs_perp.push_back(std::make_pair(0,1));
		neighs_perp.push_back(std::make_pair(0,-1));
		neighs_perp.push_back(std::make_pair(1,0));
		neighs_perp.push_back(std::make_pair(-1,0));

		int i,j;

		ct1 = clock();

		for(int k = 0;k < neighs_perp.size();k++)
		{
			i = neighs_perp[k].first;
			j = neighs_perp[k].second;

			ct4 = clock();		
			ind = std::make_pair(pt.first+i,pt.second+j);
			ct5 = clock();		
			this->t45.push_back(float(ct5-ct4)/CLOCKS_PER_SEC);
			if(ind.first >= 0 && ind.second >= 0 && ind.first < potentials_map.cols() && ind.second < potentials_map.rows())
			{
				ct6 = clock();		
				pot_val_ind = potentials_map(ind.second,ind.first);
				ct7 = clock();		
				this->t67.push_back(float(ct7-ct6)/CLOCKS_PER_SEC);
				if(pot_val_ind > 0)
				{
					ct8 = clock();		
					list_pts_bound.push_back(ind);
					list_vals_bound.push_back(potentials_map(ind.second,ind.first) + 1);
					ct9 = clock();		
					this->t89.push_back(float(ct9-ct8)/CLOCKS_PER_SEC);
				}

				else if(inRo(pot_val_ind) == 0)
				{
					ct10 = clock();		
					list_pts_empty.push_back(ind);
					ct11 = clock();		
					this->t1011.push_back(float(ct11-ct10)/CLOCKS_PER_SEC);
				}
			}

		}
	
	ct12 = clock();
	// t112.push_back(float(t12-t1)/CLOCKS_PER_SEC);
	t012.push_back(float(ct12-ct0)/CLOCKS_PER_SEC);
}

std::vector<std::pair<float,float>> getBoundObs(std::vector<std::pair<int,int>> bound_pts){
	std::vector<std::pair<float,float>> bound_pts_obs;

	std::vector<std::pair<float,float>> neighs;
	neighs.push_back(std::make_pair(-1,0));
	neighs.push_back(std::make_pair(0,1));
	neighs.push_back(std::make_pair(1,0));
	neighs.push_back(std::make_pair(0,-1));
	neighs.push_back(std::make_pair(-1,-1));
	neighs.push_back(std::make_pair(-1,1));
	neighs.push_back(std::make_pair(1,1));
	neighs.push_back(std::make_pair(1,-1));

	for(int i = 0;i < bound_pts.size();i++){

		std::pair<float,float> pt = bound_pts[i];
		for(int j = 0;j < neighs.size();j++){

			std::pair<float,float> pt_neigh = std::make_pair(pt.first + neighs[j].first,pt.second + neighs[j].second);
			if(pt_neigh.first >= 0 && pt_neigh.first < this->img.cols && pt_neigh.second >= 0 && pt_neigh.second < this->img.rows){
				if(pixAt(this->img,pt_neigh) == 255){
					bound_pts_obs.push_back(pt_neigh);
					break;
				}
			}
		}
	}

	if(bound_pts_obs.size() != bound_pts.size()){

		throw std::invalid_argument("ERROR, glitch in finding the closest Obstacles for Boundary Points.");
	}

	return bound_pts_obs;
	}

	void genCosts(Eigen::MatrixXf& mat_potential,std::unordered_map<std::pair<float,float>,std::pair<float,float>,BrushFireHashFunction>& obs_loc_mat_pot){

		clock_t t1,t2,t3,t4,t5,t6,t7,t8,t9,t10,t11;
		t1 = clock();
		std::vector<std::pair<int,int>> brush_que = this->getBoundPts();
		t2 = clock();


		t3 = clock();
		mat_potential = matTOeigenOperation(this->img,-255);
		t4 = clock();

		std::vector<std::pair<float,float>> bound_obs_loc = this->getBoundObs(brush_que);
		t5 = clock();

		std::unordered_map<std::pair<float,float>,std::pair<float,float>,BrushFireHashFunction> brush_que_dict;

		cv::Mat img_disp = this->img.clone();
		t6 = clock();
		std::pair<float,float> pt;
		std::pair<float,float> pt_obs;
		for(int i = 0;i < brush_que.size();i++){
			pt = brush_que[i];
			pt_obs = bound_obs_loc[i];

			mat_potential(pt.second,pt.first) = 1;
			obs_loc_mat_pot[pt] = pt_obs;
			brush_que_dict[pt] = pt;
		}

		std::vector<float> vec78;
		std::vector<float> vec89;
		std::vector<float> vec910;
		std::vector<float> vec1011;

		std::pair<float,float> pt_ex;
		std::pair<int,int> near_neigh;
		std::pair<float,float> pt_emp;
		std::vector<std::pair<int,int>> neighs_empty;
		std::vector<std::pair<int,int>> neighs_pts;
		std::vector<float> neighs_val;

		int itr = 0;
		while(true){
			t7 = clock();
			if(brush_que.size() == 0){
				// std::cout<<"mAp gEnErAtEd"<<stcoutd::endl;
				break;
			}
			pt_ex = brush_que[0];

			brush_que.erase(brush_que.begin());
			brush_que_dict.erase(pt_ex);
			t8 = clock();

			vec78.push_back(float(t8-t7)/CLOCKS_PER_SEC);

			this->getNeighPts(pt_ex,mat_potential,neighs_pts,neighs_val,neighs_empty);

			t9 = clock();
			vec89.push_back(float(t9-t8)/CLOCKS_PER_SEC);

			if(mat_potential(pt_ex.second,pt_ex.first) == 0){
				near_neigh = neighs_pts[std::min_element(neighs_val.begin(),neighs_val.end()) - neighs_val.begin()];
				mat_potential(pt_ex.second,pt_ex.first) = mat_potential(near_neigh.second,near_neigh.first) + 1;
				obs_loc_mat_pot[pt_ex] = obs_loc_mat_pot[near_neigh];
			}

			t10 = clock();
			vec910.push_back(float(t10-t9)/CLOCKS_PER_SEC);

			for(int i = 0;i < neighs_empty.size();i++){
				pt_emp = neighs_empty[i];
				if(brush_que_dict.count(pt_emp) == 0){
					brush_que.push_back(pt_emp);
					brush_que_dict[pt_emp] = pt_emp;
				}
			}

			t11 = clock();
			vec1011.push_back(float(t11-t10)/CLOCKS_PER_SEC);

			chngPix(img_disp,pt_ex,75 + 3*mat_potential(pt_ex.second,pt_ex.first));
			itr = itr + 1;
		}


		// std::cout<<"78 : "<<std::accumulate(vec78.begin(), vec78.end(), 0.0)<<std::endl;
		// std::cout<<"89 : "<<std::accumulate(vec89.begin(), vec89.end(), 0.0)<<std::endl;
		// std::cout<<"910 : "<<std::accumulate(vec910.begin(), vec910.end(), 0.0)<<std::endl;
		// std::cout<<"1011 : "<<std::accumulate(vec1011.begin(), vec1011.end(), 0.0)<<std::endl;
		// std::cout<<"---------------------------------------"<<std::endl;
		// std::cout<<"i45 : "<<std::accumulate(this->t45.begin(), this->t45.end(), 0.0)<<std::endl;
		// std::cout<<"i67 : "<<std::accumulate(this->t67.begin(), this->t67.end(), 0.0)<<std::endl;
		// std::cout<<"i89 : "<<std::accumulate(this->t89.begin(), this->t89.end(), 0.0)<<std::endl;
		// std::cout<<"i1011 : "<<std::accumulate(this->t1011.begin(), this->t1011.end(), 0.0)<<std::endl;
		// std::cout<<"i112 : "<<std::accumulate(this->t112.begin(), this->t112.end(), 0.0)<<std::endl;
		// std::cout<<"i012 : "<<std::accumulate(this->t012.begin(), this->t012.end(), 0.0)<<std::endl;

		// dispImg("image",img_disp,0);
		// return std::make_pair(mat_potential,obs_loc_mat_pot);
	}
};



		// std::cout<<"12-1 : "<<float(t12-t1)/CLOCKS_PER_SEC<<std::endl;
		// std::cout<<"======================================="<<std::endl;


			// std::cout<<"5-4 : "<<float(t5-t4)/CLOCKS_PER_SEC<<std::endl;
				// std::cout<<"7-6 : "<<float(t7-t6)/CLOCKS_PER_SEC<<std::endl;
					// std::cout<<"9-8 : "<<float(t9-t8)/CLOCKS_PER_SEC<<std::endl;
					// std::cout<<"11-10 : "<<float(t11-t10)/CLOCKS_PER_SEC<<std::endl;

		// std::cout<<"---------------------------------------------"<<std::endl;

		// std::cout<<"1-2 : "<<float(t2-t1)/CLOCKS_PER_SEC<<std::endl;
		// std::cout<<"2-3 : "<<float(t3-t2)/CLOCKS_PER_SEC<<std::endl;
		// std::cout<<"3-4 : "<<float(t4-t3)/CLOCKS_PER_SEC<<std::endl;
		// std::cout<<"4-5 : "<<float(t5-t4)/CLOCKS_PER_SEC<<std::endl;
		// std::cout<<"5-6 : "<<float(t6-t5)/CLOCKS_PER_SEC<<std::endl;
		// std::cout<<"6-7 : "<<float(t7-t6)/CLOCKS_PER_SEC<<std::endl;

		// for(int i = 0;i < vec78.size();i++){
		// 	t78 = t78 + vec78[i];
		// 	t89 = t89 + vec89[i];
		// 	t910 = t910 + vec910[i];
		// 	t1011 = t1011 + vec1011[i];
		// }
			// std::cout<<"10-11 : "<<float(t11-t10)/CLOCKS_PER_SEC<<std::endl;
				// std::cout<<pt_emp.first<<","<<pt_emp.second<<std::endl;				
			// std::cout<<"k"<<std::endl;
			// std::cout<<"##########################"<<std::endl;
			// std::cout<<pt_ex.first<<","<<pt_ex.second<<std::endl;
			// std::cout<<"7-8 : "<<float(t8-t7)/CLOCKS_PER_SEC<<std::endl;
			// std::cout<<"8-9 : "<<float(t9-t8)/CLOCKS_PER_SEC<<std::endl;
			// std::cout<<"9-10 : "<<float(t10-t9)/CLOCKS_PER_SEC<<std::endl;
		// std::cout<<itr<<std::endl;
		// dispImg("image2",img_disp,0);


#endif
