#ifndef DICTIONARY_H
#define DICTIONARY_H
#include <iostream>
#include <unordered_map>
#include "trEE.hpp"
#include <string>
#include <vector>
#include "cOnstAnts.hpp"
#include "mAthfUnctIOns.hpp"


class HashFunction{
public:
	size_t operator()(const Tree node) const{
		std::string locX = std::to_string(inRo(node.location.first));
		std::string locY = std::to_string(inRo(node.location.second));
		std::string locT = std::to_string(inRo(node.orientation*180/PI));

		std::string str_arg = locX + locY + locT;
		std::hash<std::string> Hash;

		return Hash(str_arg);
	}
};


class Dictionary{
public:
	std::unordered_map<Tree,size_t,HashFunction> dictionary;
	std::vector<Tree> database;

	void append(Tree node){
		this->database.push_back(node);
		this->dictionary[node] = database.size()-1;
	}

	void del(Tree node){
		if(this->isAvailable(node) > 0){
			size_t ind_node = this->dictionary[node];
			this->dictionary.erase(node);
			this->database.erase(database.begin()+ind_node);}

		else{
			throw std::invalid_argument("ERROR, no Key to delete in the Dictionary with the given input.");}
	}

	int length(){
		if(this->dictionary.size() != this->database.size()){
			throw std::invalid_argument("ERROR, a glitch in the Dictionary (sizes of map and vector don't match) :(");}

		else{
			return this->dictionary.size();}
	}

	Tree get(Tree node){
		if(this->isAvailable(node) > 0){
			size_t ind_node = this->dictionary[node];
			std::cout<<ind_node<<std::endl;
			return this->database[ind_node];}

		else{
			throw std::invalid_argument("ERROR, no Key in the Dictionary with given input.");}
	}

	bool isAvailable(Tree node){
		return this->dictionary.count(node) > 0 ? true:false;}
};

#endif

// Need to design the Hash Function and include the check node function.