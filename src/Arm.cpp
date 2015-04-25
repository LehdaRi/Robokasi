/**

    Automaatio- ja Systeemitekniikan killan robokäsiprojekti
    Arm.cpp

    @version    0.1
    @author     Miika 'LehdaRi' Lehtimäki
    @date       2015-04-25

**/


#include "Arm.hpp"

#include <iostream>
#include <ctime>


Arm::Arm(Shader& jointShader, Shader& meshShader, std::vector<Matrix4Glf> matrices) :
    meshShader_(meshShader),
    rnd_(time(NULL))
{
    for (auto& m : matrices)
        joints_.emplace_back(jointShader, m);

    meshes_.resize(joints_.size(), std::make_pair(nullptr, Vector3Glf(0.0f, 0.0f, 0.0f)));
}

void Arm::setJointTheta(unsigned jointId, float theta) {
    if (jointId >= joints_.size())
        return;

    joints_[jointId].setTheta(theta);
    recalculateJoints();
}

void Arm::setJointMesh(unsigned jointId, Mesh* mesh, const Vector3Glf& color) {
    if (jointId >= meshes_.size())
        return;

    meshes_[jointId] = std::make_pair(mesh, color);
}

void Arm::draw(const Camera& camera) const {
    for (auto i=0u; i<joints_.size(); ++i) {
        if (meshes_[i].first)
            meshes_[i].first->render(meshShader_, camera, joints_[i], meshes_[i].second);
    }

    glDisable(GL_DEPTH_TEST);

    for (auto& j : joints_)
        j.draw(camera);

    glEnable(GL_DEPTH_TEST);
}

void Arm::solve(Vector3Glf goal, unsigned nMaxIterations) {
    float error = (goal - calculateEndEffector()).norm();
    float newError = error;

    std::vector<float> lastThetas;
    for (auto& joint : joints_)
        lastThetas.push_back(joint.getTheta());

    for (auto i=0u; i<nMaxIterations; ++i) {
        float errScale = error / getMaxLength();

        for (auto& joint : joints_)
            joint.setTheta(joint.getTheta() - 0.5f*errScale + (rnd_()%10000 * 0.0001f)*errScale);

        recalculateJoints();

        newError = (goal - calculateEndEffector()).norm();
        if (newError > error) {
            //printf("Reverting changes..\n%f\n", error);
            for (auto j=0u; j<joints_.size(); ++j)
                joints_[j].setTheta(lastThetas[j]);
            recalculateJoints();
            error = (goal - calculateEndEffector()).norm();
            //printf("%f\n", error);
        }
        else {
            error = newError;

            for (auto j=0u; j<joints_.size(); ++j)
                lastThetas[j] = joints_[j].getTheta();
        }
    }

    printf("Error: %f\n", error);
}

Eigen::Matrix<float, 1, 3> Arm::compute_jacovian_segment(int seg_num, Vector3Glf goal_point, Vector3f angle) {
    Joint& j = joints_.at(seg_num);
    // mini is the amount of angle you go in the direction for numerical calculation
    float mini = 0.0005;

    Vector3Glf transformed_goal = goal_point;
    for(int i=joints_.size()-1; i>seg_num; i--) {
        // transform the goal point to relevence to this segment
        // by removing all the transformations the segments afterwards
        // apply on the current segment
        transformed_goal -= joints_[i].getEndPoint();
    }

    Vector3Glf my_end_effector = calculateEndEffector(seg_num);

    // transform them both to the origin
    if (seg_num-1 >= 0) {
        my_end_effector -= calculateEndEffector(seg_num-1);
        transformed_goal -= calculateEndEffector(seg_num-1);
    }

    // original end_effector
    Vector3Glf original_ee = calculateEndEffector();

    // angle input is the one you rotate around
    // remove all the rotations from the previous segments by applying them
    Eigen::AngleAxis<GLfloat> t = Eigen::AngleAxis<GLfloat>(mini, angle);

    Matrix4Glf m, mi;

    m << t.matrix(),        Vector3Glf(0.0f, 0.0f, 0.0f),
         0.0f, 0.0f, 0.0f,  1.0f;

    mi <<   t.inverse().matrix(),   Vector3Glf(0.0f, 0.0f, 0.0f),
            0.0f, 0.0f, 0.0f,       1.0f;

    // transform the segment by some delta(theta)
    j.setJointMatrix(m * j.getJointMatrix());
    // new end_effector
    Vector3Glf new_ee = calculateEndEffector();
    // reverse the transformation afterwards
    j.setJointMatrix(mi * j.getJointMatrix());

    // difference between the end_effectors
    // since mini is very small, it's an approximation of
    // the derivative when divided by mini
    Vector3f diff = new_ee - original_ee;

    // return the row of dx/dtheta, dy/dtheta, dz/dtheta
    Eigen::Matrix<float, 1, 3> ret;
    ret << diff[0]/mini, diff[1]/mini, diff[2]/mini;
    return ret;
}

// computes end_effector up to certain number of segments
Vector3Glf Arm::calculateEndEffector(int jointId /* = -1 */) {
    if (jointId == -1)
        jointId = joints_.size()-1;

    return joints_[jointId].getEndPoint();
}

float Arm::getMaxLength(void) {
    float ret = 0;
    for (unsigned int i=0; i<joints_.size(); i++)
        ret += joints_[i].getLength();
    return ret;
}

void Arm::recalculateJoints(void) {
    for (auto i=0u; i<joints_.size()-1; ++i) {
        joints_[i+1].applyJoint(joints_[i]);
    }
}
