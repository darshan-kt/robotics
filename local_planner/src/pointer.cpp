#include<iostream>
#include <cstring>

using namespace std;

class Flux
{
public:
	int auto = 32;
	int Age(){
		cout<<"Just 25"<<endl;
		return 0;
	}
	void height(int &a){
		a = 5;
	}
};

int main(){
	int num = 6;
	Flux laptop;
	cout<<"Initial num value: "<<num <<endl;
	laptop.Age();
	//Passing reference
	laptop.height(num);
	cout<<"auto height after MF call: "<< num <<endl;

	return 0;
}
