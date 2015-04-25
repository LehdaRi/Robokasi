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
	std::cout << "lel" << std::endl;
	popen("stty -F /dev/ttyACM0 cs8 9600 ignbrk -brkint -icrnl -imaxbel -opost -onlcr -isig -icanon -iexten -echo -echoe -echok -echoctl -echoke -noflsh -ixon -crtscts\n", "r");
	std::cout << "lel" << std::endl;
	this->input_.open(port, std::ifstream::in);
	if(!this->input_.is_open())
		perror("Serial port input not opened");
	this->output_.open(port, std::ofstream::out | std::ofstream::app);
	if(!this->output_.is_open())
		perror("Serial port output not opened");
}

void Serial::close() {
	this->input_.close();
}

void Serial::pushAngles(const std::vector<float>& angles) {
	std::string data;
	for(auto theta : angles) {
		data.append(std::to_string(theta));
		data.append(";");
	}
	data.erase(data.size()-1);
	this->output_ << data << std::endl;
    this->output_.flush();
}

