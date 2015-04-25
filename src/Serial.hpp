/**

    Automaatio- ja Systeemitekniikan killan robokäsiprojekti
    Trajectory.hpp

    @version    0.1
    @author     Veikka Kähkönen
    @date       2015-04-24

**/


#include <vector>
#include <fstream>

class Serial {
public:
	Serial();
	void open(std::string port);
	void close();
	void pushAngles(const std::vector<float>& angles);
	bool getStatus();

private:
	std::ifstream input_;
	std::ofstream output_;
};
