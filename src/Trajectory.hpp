#include "LinearAlgebra.hpp"
#include <vector>

class Trajectory {
public:
	void loadFromFile(const std::string& fileName, const float dt);
	std::vector<Vector3Glf>& getTrajectory();

private:
	std::vector<std::string> split(const char *str, char c = ' ');
	std::vector<Vector3Glf> trajectory_;
};
