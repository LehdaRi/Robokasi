/**

    Automaatio- ja Systeemitekniikan killan robokäsiprojekti
    Trajectory.cpp

    @version    0.1
    @author     Veikka Kähkönen
    @date       2015-04-24

**/


#include "Trajectory.hpp"
#include "LinearAlgebra.hpp"
#include <fstream>
#include <iostream>
#include <string>
#include <sstream>
#include <vector>


namespace {
	std::vector<std::string> split(const char *str, char c) {
	    std::vector<std::string> result;
	    do
	    {
	        const char *begin = str;
	        while(*str != c && *str)
	            str++;
	        result.push_back(std::string(begin, str));
	    } while (0 != *str++);
	    return result;
	}
}


void Trajectory::loadFromFile(const std::string& fileName) {
	std::ifstream f (fileName);
	std::string line;
	if (!f.is_open())
    	perror("Error while opening file");
	while(std::getline(f, line)) {
		auto items = split(line.c_str(), ' ');
		if(items[0] == "MV") {
			// End effector position
			auto x1 = std::stof(items[1]);
			auto y1 = std::stof(items[2]);
			auto z1 = std::stof(items[3]);
			Vector3Glf pos {x1, y1, z1};
			// End effector direction
			auto x2 = std::stof(items[4]);
			auto y2 = std::stof(items[5]);
			auto z2 = std::stof(items[6]);
			Vector3Glf dir {x2, y2, z2};
			this->trajectory_.emplace_back(pos, dir);
		}
	}
	if (f.bad())
    	perror("Error while reading file");
}

std::vector<std::pair<Vector3Glf, Vector3Glf>>& Trajectory::getTrajectory() {
	return this->trajectory_;
}
