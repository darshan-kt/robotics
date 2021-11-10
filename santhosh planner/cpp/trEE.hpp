#ifndef TREE_HPP
#define TREE_HPP
#include <iostream>
#include "cOnstAnts.hpp"
#include <cmath>
#include "fUnctIOns.hpp"


// struct Tree{
// 	std::pair<float,float> location,centre;
// 	float orientation,alpha,g,f;
// 	Tree* parent;

// 	Tree(){}
// 	Tree(std::pair<float,float> location, float orientation, float alpha,std::pair<float, float> centre, float g, float f){
// 		this->location = location;
// 		this->orientation = orientation;
// 		this->alpha = alpha;
// 		this->centre = centre;
// 		this->g = g;
// 		this->f = f;
// 	}
// };

struct Tree{
	std::pair<float,float> location,centre;
	float orientation,alpha,g,f;
	unsigned int parent,ind;

	Tree(){}
	Tree(std::pair<float,float> location, float orientation, float alpha,std::pair<float, float> centre, float g, float f){
		this->location = location;
		this->orientation = orientation;
		this->alpha = alpha;
		this->centre = centre;
		this->g = g;
		this->f = f;
	}
};


bool operator <=(const Tree& node1, const Tree& node2)
	{return node1.f <= node2.f;}

bool operator <(const Tree& node1, const Tree& node2)
	{return node1.f < node2.f;}

bool operator ==(const Tree& node1, const Tree& node2)
	{return (inRo(node1.location.first) == inRo(node2.location.first)) && (inRo(node1.location.second) == inRo(node2.location.second)) && (inRo(node1.orientation*180/PI) == inRo(node2.orientation*180/PI));}

bool operator >=(const Tree& node1, const Tree& node2)
	{return node1.f >= node2.f;}

bool operator >(const Tree& node1, const Tree& node2)
	{return node1.f > node2.f;}

#endif
