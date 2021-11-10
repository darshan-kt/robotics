#ifndef QUEUE_HPP
#define QUEUE_HPP
#include <iostream>
#include <queue>
#include "trEE.hpp"


class Queue{
public:
	std::priority_queue<Tree,std::vector<Tree>,std::greater<Tree>> que;

	void push(Tree& node){
		this->que.push(node);
	}

	Tree pop(){
		if(this->length() > 0){
			Tree node = que.top();
			this->que.pop();
			return node;			
		}

		else{
			throw std::invalid_argument("ERROR, Queue is empty.");}

	}

	int length(){
		return this->que.size(); 
	}

};


#endif