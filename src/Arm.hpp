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
#include <random>


class Arm {
public:
    Arm(Shader& jointShader, Shader& meshShader, std::vector<Matrix4Glf> matrices);

    void setJointTheta(unsigned jointId, float theta);
    void setJointMesh(unsigned jointId, Mesh* mesh,
                      const Vector3Glf& color = Vector3Glf(1.0f, 1.0f, 1.0f));
    void draw(const Camera& camera) const;

    void solve(Vector3Glf goal, unsigned nMaxIterations);
    Eigen::Matrix<float, 1, 3> compute_jacovian_segment(int seg_num, Vector3Glf goal_point, Vector3f angle);

    Vector3Glf calculateEndEffector(int jointId = -1);
    float getMaxLength(void);

private:
    std::vector<Joint> joints_;
    std::vector<std::pair<Mesh*, Vector3Glf>> meshes_;

    Shader& meshShader_;

    std::default_random_engine rnd_;

    void recalculateJoints(void);
};


#endif // ROBOKASI_ARM_HPP

