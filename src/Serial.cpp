/**

	Automaatio- ja Systeemitekniikan killan robokäsiprojekti
	Serial.cpp

	@version    0.1
	@author     Veikka Kähkönen
	@date       2015-04-24

**/

#include "Serial.hpp"
#include <string>
#include <sstream>
#include <iostream>
#include <fstream>
#include <array>
#include <vector>

Serial::Serial() :
	input_(),
	angles_({0, 0, 0, 0, 0, 0}),
	safemode_(0),
	brake_(0),
	gripper_(0)
{
}

void Serial::open(std::string port) {
	std::string command;
	// Configure serial port
	//command = "stty -F " + port + " cs8 9600 ignbrk -brkint -icrnl -imaxbel -opost -onlcr -isig -icanon -iexten -echo -echoe -echok -echoctl -echoke -noflsh -ixon -crtscts\n";
	popen(command.c_str(), "r");
	// Open input stream
	this->input_.open(port, std::ifstream::in);
	if(!this->input_.is_open())
		perror("Serial port input not opened");
}

void Serial::close() {
	this->input_.close();
}

void Serial::setAngles(const std::vector<float>& angles, int safemode, int brake, int gripper) {
	/*
	Construct a string with semicolon separated values ending in a newline.
	*/
	if(angles.size() != 6) {
		std::cerr << "Serial error: Invalid angle vector: Expected size 6, got " << angles.size() << std::endl;
		return;
	}

	std::stringstream dataStream;
	for(int i = 0; i < 6; ++i) {
		dataStream << "j" << i+1 << ": " << angles[i] << ", ";
	}
	dataStream << "safemode: " << safemode << ", brake: " << brake << ", gripper: " <<  gripper << std::endl;
	std::string data = dataStream.str();
	data.erase(data.size()-1);
	// Echo angles
	std::cout << data << "\r\n";
	// Flush the buffer
    std::cout.flush();
}

void Serial::parseStatus() {
    std::string response = "";
    std::string temp = "";    
    do {
    	response = temp;
    	std::getline(this->input_, temp);
    } while(temp != "");
   	std::vector<float> angles{0, 0, 0, 0, 0, 0};
   	int safemode, brake, gripper;
    int amount = sscanf(response.c_str(), "j1: %f, j2: %f, j3: %f, j4: %f, j5: %f, j6: %f, safemode: %i, brake: %i, gripper: %i",
    	&angles[0], &angles[1], &angles[2], &angles[3], &angles[4], &angles[5],
    	&safemode, &brake, &gripper);
    if(amount == 9) {
    	this->angles_ = angles;
    	this->safemode_ = safemode;
    	this->brake_ = brake;
    	this->gripper_ = gripper;
    } else if(response != "") {
    	std::cerr << "Serial error: Invalid input" << std::endl;
    }
}

std::vector<float>& Serial::getAngles(){
	return this->angles_;
}

int& Serial::getSafemode() {
	return this->safemode_;
}

int& Serial::getBrake() {
	return this->brake_;
}

int& Serial::getGripper() {
	return this->gripper_;
}
