/**

    Automaatio- ja Systeemitekniikan killan robokäsiprojekti
    Serial.hpp

    @version    0.1
    @author     Veikka Kähkönen / Miika Lehtimäki
    @date       2015-04-29

**/


#ifndef ROBOKASI_SERIAL_HPP
#define ROBOKASI_SERIAL_HPP


#include "Trajectory.hpp"

#include <vector>
#include <fstream>


class Serial {
public:
	Serial();
	void open(std::string port);
	void close();

    void moveTrajectory(const Trajectory& trajectory);
    void moveToPoint(const TrajectoryPoint& point);
    TrajectoryPoint getStatus(void);

private:
	std::ifstream input_;
};

#endif
