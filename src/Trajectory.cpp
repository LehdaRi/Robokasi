/**

    Automaatio- ja Systeemitekniikan killan robokäsiprojekti
    Trajectory.cpp

    @version    0.1
    @author     Veikka Kähkönen / Miika Lehtimäki
    @date       2015-04-29

**/


#include "Trajectory.hpp"
#include <fstream>
#include <iostream>
#include <string>
#include <sstream>


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


void Trajectory::addPoint(const TrajectoryPoint& point) {
    trajectory_.push_back(point);
}


void Trajectory::loadFromFile(const std::string& fileName) {
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
		if(items.size() == 10) {
			TrajectoryPoint newPoint = {
                { std::stof(items[0]),
                  std::stof(items[1]),
                  std::stof(items[2]),
                  std::stof(items[3]),
                  std::stof(items[4]),
                  std::stof(items[5]) },
                { std::stoi(items[6]),
                  std::stoi(items[7]),
                  std::stoi(items[8]),
                  std::stoi(items[9]) }
			};

			trajectory_.push_back(newPoint);
		} else {
			std::string error = "Invalid trajectory file line: " + line;
			throw(error);
		}
	}
}

void Trajectory::saveToFile(const std::string& fileName) {
	// Open file
	std::ofstream f (fileName);
	std::string line;
	if (!f.is_open())
    	perror("Error while opening file");

 	for (auto& point : trajectory_) {
 	    for (auto i=0; i<6; ++i)
            f << point.angles[i] << " ";
        f << point.params[0] << " "
          << point.params[1] << " "
          << point.params[2] << " "
          << point.params[3] << std::endl;
	}
}

size_t Trajectory::size(void) const {
    return trajectory_.size();
}

const TrajectoryVector& Trajectory::getVector(void) const {
	return trajectory_;
}

TrajectoryPoint& Trajectory::operator[](int pointId) {
    return trajectory_.at(pointId);
}
