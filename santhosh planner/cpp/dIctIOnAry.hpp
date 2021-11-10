#ifndef DICTIONARY_H
#define DICTIONARY_H
#include <iostream>
#include <unordered_map>
#include "trEE.hpp"
#include <string>
#include "cOnstAnts.hpp"
#include "fUnctIOns.hpp"


struct HashFunction{
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
	std::unordered_map<Tree,Tree,HashFunction> dictionary;

	void append(Tree& node){
		this->dictionary[node] = node;}

	void del(Tree& node){
		this->dictionary.erase(node);}

	int length(){
		return this->dictionary.size();}

	Tree get(Tree& node){
		if(this->isAvailable(node) > 0){
			return this->dictionary[node];}
		else{
			throw std::invalid_argument("ERROR, no Key in the Dictionary with given input.");}
	}

	bool isAvailable(Tree& node){
		return this->dictionary.count(node) > 0 ? true:false;}
};

#endif

// Need to design the Hash Function and include the check node function.