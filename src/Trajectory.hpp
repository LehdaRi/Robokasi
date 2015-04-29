/**

    Automaatio- ja Systeemitekniikan killan robokäsiprojekti
    Trajectory.hpp

    @version    0.1
    @author     Veikka Kähkönen / Miika Lehtimäki
    @date       2015-04-29

**/


#ifndef ROBOKASI_TRAJECTORY_HPP
#define ROBOKASI_TRAJECTORY_HPP


#include <vector>
#include <array>


struct TrajectoryPoint {
    /*
        joint angles
    */
    std::array<float, 6> angles;
    /*
        0:  safeMode (1: on / 0: off)
        1:  brake (1: on / 0: off)
        2:  gripper (1: on / 0: off)
        3:  delay (in milliseconds)
    */
    std::array<int, 4> params;
};


typedef std::vector<TrajectoryPoint> TrajectoryVector;


class Trajectory {
public:
    void addPoint(const TrajectoryPoint& point);

    /*
        Load a trajectory from file.

        File format:
        j1 j2 j3 j4 j5 j6 safeMode brake gripper delay
        float float float float float float int int int int
    */
	void loadFromFile(const std::string& fileName);
	void saveToFile(const std::string& fileName);

	size_t size(void) const;

	const TrajectoryVector& getVector(void) const;

	TrajectoryPoint& operator[](int pointId);

private:
	TrajectoryVector trajectory_;
};


#endif // ROBOKASI_TRAJECTORY_HPP
