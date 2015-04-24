/**

    Automaatio- ja Systeemitekniikan killan robokäsiprojekti
    Arm.hpp

    @version    0.1
    @author     Miika 'LehdaRi' Lehtimäki
    @date       2015-04-24

**/


#ifndef ROBOKASI_ARM_HPP
#define ROBOKASI_ARM_HPP


#include "Joint.hpp"

#include <vector>


class Arm {
public:
    Arm(Shader& shader, std::vector<Matrix4Glf> matrices);

    void solve(Vector3Glf goal_point, int life_count);
    Eigen::Matrix<float, 1, 3> compute_jacovian_segment(int seg_num, Vector3Glf goal_point, Vector3f angle);
    Vector3Glf calculate_end_effector(int segment_num = -1);
    float get_max_length(void);

private:
    std::vector<Joint> joints_;
};


#endif // ROBOKASI_ARM_HPP
