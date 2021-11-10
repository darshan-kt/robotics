#include <iostream>
#include "CollisionDetector.h"


int main(){
	cv::Mat img_map = cv::imread("map.png");

	VehicleParam vp(30,60);
	State vs(245,220,PI/6);

	Object rob(vs, vp);
	Object obj1(State(330,210, -PI/12), VehicleParam(80,25));
	Object obj2(State(150,210, PI/12), VehicleParam(80,25));

	std::vector<Object> obs;
	obs.push_back(obj1);
	obs.push_back(obj2);

	cv::Mat img_map_ogrid;
	cv::cvtColor(img_map,img_map_ogrid,cv::COLOR_BGR2GRAY);

	CollisionDetector collchck(img_map_ogrid,rob.vehicleparam);
	bool ret = collchck.collCheck(rob.state,obs);

	plotPoly(img_map, rob.vehicleparam, rob.state);
	plotPoly(img_map, obj1.vehicleparam, obj1.state);
	plotPoly(img_map, obj2.vehicleparam, obj2.state);

	for(int i = 0;i < rob.partCens.size();i++)
		cv::circle(img_map,cv::Point(rob.partCens[i].first,rob.partCens[i].second),rob.radiB,cv::Vec3b(0,255,0),1);

	for(int i = 0;i < obj1.partCens.size();i++)
		cv::circle(img_map,cv::Point(obj1.partCens[i].first,obj1.partCens[i].second),obj1.radiB,cv::Vec3b(0,0,255),1);

	for(int i = 0;i < obj2.partCens.size();i++)
		cv::circle(img_map,cv::Point(obj2.partCens[i].first,obj2.partCens[i].second),obj2.radiB,cv::Vec3b(0,0,255),1);



	if(ret)
		cv::putText(img_map, "Collision", cvPoint(30,30), cv::FONT_HERSHEY_COMPLEX_SMALL, 0.8, cvScalar(200,200,250), 1, CV_AA);
	else
		cv::putText(img_map, "No Collision", cvPoint(30,30), cv::FONT_HERSHEY_COMPLEX_SMALL, 0.8, cvScalar(200,200,250), 1, CV_AA);

	cv::imshow("image",img_map);
	cv::waitKey(0);
	std::cout<<"Collision : ";
	ret ? std::cout<<"True" : std::cout<<"False";
	std::cout<<std::endl;
	// cv::imwrite("CollCheckImgs/collCheck7.png",img_map);
}



// truck
// length, width

// trailer
// length, width
// max(truck width, trailer width)
// fix length as max of measurements and width as least.
