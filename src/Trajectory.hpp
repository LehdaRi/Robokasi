/**

    Automaatio- ja Systeemitekniikan killan robokäsiprojekti
    Trajectory.hpp

    @version    0.1
    @author     Veikka Kähkönen
    @date       2015-04-24

**/


#ifndef ROBOKASI_ROBOSERIAL_HPP
#define ROBOKASI_ROBOSERIAL_HPP

#include "LinearAlgebra.hpp"
#include <vector>
#include <array>

class Trajectory {
public:
	void loadFromFile(const std::string& fileName);
	std::vector<std::array<Vector3Glf,3>>& getTrajectory();

private:
	std::vector<std::array<Vector3Glf,3>> trajectory_;
};

#endif
