/**

	Automaatio- ja Systeemitekniikan killan robokäsiprojekti
	Serial.cpp

	@version    0.1
	@author     Veikka Kähkönen
	@date       2015-04-24

**/

#include "Serial.hpp"
#include <cstdint>
#include <cstring>
#include <string>
#include <iostream>
#include <fstream>
#include <cstdlib>

Serial::Serial() :
	input_(),
	output_() {
}

void Serial::open(std::string port) {
	this->input_.open(port, std::ifstream::in);
	this->output_.open(port, std::ofstream::out | std::ofstream::app);
	std::cout << "opened" << std::endl;
	if(!this->output_.is_open())
		perror("Serial port not opened");
}

void Serial::close() {
	this->input_.close();
}

void Serial::pushAngles(const std::vector<float>& angles) {
	std::string data;
	for(auto theta : angles) {
		data.append(std::to_string(theta));
	}
	data.erase(data.size()-1);
	this->output_ << data << std::endl;
}

bool Serial::getStatus() {
	while(!this->input_.gcount()) {
	}
	std::string temp;
	this->input_ >> temp;
	std::cout << temp;
	return false;
}
