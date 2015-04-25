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

class Trajectory {
public:
	void loadFromFile(const std::string& fileName);
	std::vector<std::pair<Vector3Glf, Vector3Glf>>& getTrajectory();

private:
	std::vector<std::pair<Vector3Glf, Vector3Glf>> trajectory_;
};

#endif
