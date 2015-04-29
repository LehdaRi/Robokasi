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
#include <chrono>
#include <thread>

Serial::Serial() :
	input_()
{ }

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


void Serial::moveTrajectory(const Trajectory& trajectory) {
    auto traj = trajectory.getVector();

    for (auto& point : traj) {
        moveToPoint(point);
        std::this_thread::sleep_for(std::chrono::milliseconds(point.params[3]));
    }
}

void Serial::moveToPoint(const TrajectoryPoint& point) {
	/*
	Construct a string with semicolon separated values ending in a newline.
	*/

	std::stringstream dataStream;
	for(int i = 0; i < 6; ++i) {
		dataStream << "j" << i+1 << ": " << point.angles[i] << ", ";
	}
	dataStream << "safemode: " << point.params[0]
               << ", brake: " << point.params[1]
               << ", gripper: " <<  point.params[2] << "\r\n";
	std::string data = dataStream.str();
	data.erase(data.size()-1);
	// Echo angles
	std::cout << data;
	// Flush the buffer
    std::cout.flush();
}

TrajectoryPoint Serial::getStatus(void) {
    std::string response = "";
    std::string temp = "";
    do {
    	response = temp;
    	std::getline(this->input_, temp);
    } while(temp != "");

    TrajectoryPoint point;
    int amount = sscanf(response.c_str(), "j1: %f, j2: %f, j3: %f, j4: %f, j5: %f, j6: %f, safemode: %i, brake: %i, gripper: %i",
    	&point.angles[0], &point.angles[1], &point.angles[2], &point.angles[3], &point.angles[4], &point.angles[5],
    	&point.params[0], &point.params[1], &point.params[2]);

    if(amount != 9 && response != "")
    	std::cerr << "Serial error: Invalid input" << std::endl;

    return point;
}
