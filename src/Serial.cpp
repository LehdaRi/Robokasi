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
	std::string command;
	// Configure serial port
	command = "stty -F " + port + " cs8 9600 ignbrk -brkint -icrnl -imaxbel -opost -onlcr -isig -icanon -iexten -echo -echoe -echok -echoctl -echoke -noflsh -ixon -crtscts\n";
	popen(command.c_str(), "r");
	// Open input stream
	this->input_.open(port, std::ifstream::in);
	if(!this->input_.is_open())
		perror("Serial port input not opened");
	// Open ouput stream
	this->output_.open(port, std::ofstream::out | std::ofstream::app);
	if(!this->output_.is_open())
		perror("Serial port output not opened");
}

void Serial::close() {
	this->input_.close();
	this->output_.close();
}

void Serial::pushAngles(const std::vector<float>& angles) {
	/*
	Construct a string with semicolon separated values ending in a newline.
	*/
	std::string data;
	for(auto theta : angles) {
		data.append(std::to_string(theta));
		data.append(";");
	}
	data.erase(data.size()-1);
	this->output_ << data << std::endl;
	// Flush the buffer
    this->output_.flush();
}

