/**

    Automaatio- ja Systeemitekniikan killan robokäsiprojekti
    Trajectory.hpp

    @version    0.1
    @author     Veikka Kähkönen
    @date       2015-04-24

**/


#ifndef ROBOKASI_SERIAL_HPP
#define ROBOKASI_SERIAL_HPP


#include <vector>
#include <fstream>


class Serial {
public:
	Serial();
	void open(std::string port);
	void close();
	void pushAngles(const std::vector<float>& angles);

private:
	std::ifstream input_;
	std::ofstream output_;
};

#endif
