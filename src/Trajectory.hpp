/**

    Automaatio- ja Systeemitekniikan killan robokäsiprojekti
    Trajectory.hpp

    @version    0.1
    @author     Veikka Kähkönen
    @date       2015-04-24

**/


#include "LinearAlgebra.hpp"
#include <vector>

class Trajectory {
public:
	void loadFromFile(const std::string& fileName, const float dt);
	std::vector<Vector3Glf>& getTrajectory();

private:
	std::vector<Vector3Glf> trajectory_;
};
