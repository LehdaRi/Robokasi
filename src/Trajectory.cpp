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
		/*
		Tokenize C style string.
		*/
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
	/*
	Load a trajectory from file.

	File format:
	MV x1 y1 z1 x2 y2 z2 x3 y3 z3
	Where:
	(x1, y1, z1): End effector position vector
	(x2, y2, z2): End effector forward direction vector
	(x3, y3, z3): End effector upward position vector
	*/

	// Open file
	std::ifstream f (fileName);
	std::string line;
	if (!f.is_open())
    	perror("Error while opening file");
 	// Parse
	while(std::getline(f, line)) {
		// Tokenize
		auto items = split(line.c_str(), ' ');
		// Move command
		if(items.size() == 10 && items[0] == "MV") {
			// End effector position
			auto x1 = std::stof(items[1]);
			auto y1 = std::stof(items[2]);
			auto z1 = std::stof(items[3]);
			Vector3Glf pos(x1, y1, z1);
			// End effector forward direction
			auto x2 = std::stof(items[4]);
			auto y2 = std::stof(items[5]);
			auto z2 = std::stof(items[6]);
			Vector3Glf dirF(x2, y2, z2);
			// End effector upward direction
			auto x3 = std::stof(items[7]);
			auto y3 = std::stof(items[8]);
			auto z3 = std::stof(items[9]);
			Vector3Glf dirU(x3, y3, z3);
			// Insert the vectors into an array
			std::array<Vector3Glf, 3> vects{{pos, dirF, dirU}};
			// Insert the array into a vector
			this->trajectory_.emplace_back(vects);
		} else {
			std::string error = "Invalid trajectory file line: " + line;
			throw(error);
		}
	}
}


std::vector<std::array<Vector3Glf,3>>& Trajectory::getTrajectory() {
	return this->trajectory_;
}
