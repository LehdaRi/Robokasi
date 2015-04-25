/**

    Automaatio- ja Systeemitekniikan killan robokäsiprojekti
    Arm.hpp

    @version    0.1
    @author     Miika 'LehdaRi' Lehtimäki
    @date       2015-04-25

**/


#ifndef ROBOKASI_ARM_HPP
#define ROBOKASI_ARM_HPP


#include "Joint.hpp"
#include "Mesh.hpp"

#include <vector>
#include <array>
#include <random>


class Arm {
public:
    Arm(Shader& jointShader, Shader& meshShader, std::vector<Matrix4Glf> matrices);

    void setJointMesh(unsigned jointId, Mesh* mesh,
                      const Vector3Glf& color = Vector3Glf(1.0f, 1.0f, 1.0f));
    void setJointConstraints(unsigned jointId, float lowerLimit, float upperLimit);
    void setJointTheta(unsigned jointId, float theta, bool recalculate = true);
    void draw(const Camera& camera) const;

    void solve(const Vector3Glf& goal,
               const Vector3Glf& toolOrientation,
               unsigned nMaxIterations);
    Eigen::Matrix<float, 1, 3> compute_jacovian_segment(int seg_num, Vector3Glf goal_point, Vector3f angle);

    Vector3Glf calculateEndEffector(int jointId = -1) const;
    float getMaxLength(void);

private:
    std::vector<Joint> joints_;
    std::vector<std::array<float, 2>> constraints_;
    std::vector<std::pair<Mesh*, Vector3Glf>> meshes_;

    Shader& meshShader_;

    std::default_random_engine rnd_;

    void recalculateJoints(void);
    float getPositionError(const Vector3Glf& goal) const;
    float getOrientationError(const Vector3Glf& goalOrientation) const;
};


#endif // ROBOKASI_ARM_HPP

