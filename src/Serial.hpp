/**

    Automaatio- ja Systeemitekniikan killan robokäsiprojekti
    Serial.hpp

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
	void setAngles(const std::vector<float>& angles, int safemode, int brake, int gripper);
	void parseStatus();
	std::vector<float>& getAngles();
	int& getSafemode();
	int& getBrake();
	int& getGripper();

private:
	std::ifstream input_;
	std::vector<float> angles_;
	int safemode_;
	int brake_;
	int gripper_;
};

#endif
