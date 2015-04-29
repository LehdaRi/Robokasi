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

    constraints_.resize(joints_.size(), {-2*PI, 2*PI});
    meshes_.resize(joints_.size(), std::make_pair(nullptr, Vector3Glf(0.0f, 0.0f, 0.0f)));
}

void Arm::setJointMesh(unsigned jointId, Mesh* mesh, const Vector3Glf& color) {
    if (jointId >= meshes_.size())
        return;

    meshes_[jointId] = std::make_pair(mesh, color);
}

void Arm::setJointConstraints(unsigned jointId, float lowerLimit, float upperLimit) {
    if (jointId >= constraints_.size())
        return;

    constraints_[jointId] = { lowerLimit, upperLimit };
    setJointTheta(jointId, (lowerLimit + upperLimit)*0.5f);
}

void Arm::setJointTheta(unsigned jointId, float theta, bool recalculate) {
    if (jointId >= joints_.size())
        return;

    if (theta >= constraints_[jointId][0] && theta <= constraints_[jointId][1]) {
        joints_[jointId].setTheta(theta);
        if (recalculate)
            recalculateJoints();
    }
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

void Arm::solve(const Vector3Glf& goal,
                const Vector3Glf& toolOrientation,
                unsigned nMaxIterations) {
    float posError = getPositionError(goal);
    float newPosError = posError;
    float oriError = getOrientationError(toolOrientation);
    float newOriError = oriError;

    std::vector<float> lastThetas;
    for (auto& joint : joints_)
        lastThetas.push_back(joint.getTheta());

    float b = 0.00005f;
    for (auto i=0u; i<nMaxIterations; ++i) {
        float errScale = posError / getMaxLength();
        float a = ((i+1)/(float)nMaxIterations);

        b *= 1.01f;
        //  position
        for (auto j=0u; j<joints_.size(); ++j) {
            float t = joints_[j].getTheta();
            float r = rnd_()%10000 * 0.0001f;

            float minTheta = (constraints_[j][0] - t) * errScale;// * 10.0f;
            float maxTheta = (constraints_[j][1] - t) * errScale;// * 10.0f;
            float newTheta = t + minTheta + r*(maxTheta - minTheta);

            //printf("b: %f theta: %f minTheta: %f maxTheta: %f newTheta: %f\n", b, t, minTheta, maxTheta, newTheta);

            setJointTheta(j, newTheta, false);
        }
        //for (auto j=0u; j<joints_.size(); ++j)
        //    setJointTheta(j, joints_[j].getTheta() - 0.5f*100.0f*errScale + (rnd_()%10000 * 0.0001f)*100.0f*errScale, false);

        recalculateJoints();

        newPosError = getPositionError(goal);

        if (newPosError > posError) {
            for (auto j=0u; j<joints_.size(); ++j)
                joints_[j].setTheta(lastThetas[j]);

            recalculateJoints();
            posError = getPositionError(goal);
        }
        else {
            posError = newPosError;
            for (auto j=0u; j<joints_.size(); ++j)
                lastThetas[j] = joints_[j].getTheta();
        }

        //  orientation
        /*for (auto i=0u; i<joints_.size(); ++i)
            setJointTheta(i, joints_[i].getTheta() - 0.5f*errScale + (rnd_()%10000 * 0.0001f)*errScale, false);

        recalculateJoints();

        newOriError = getOrientationError(toolOrientation);

        if (newOriError > oriError) {
            for (auto j=0u; j<joints_.size(); ++j)
                joints_[j].setTheta(lastThetas[j]);

            recalculateJoints();
            oriError = getOrientationError(toolOrientation);
        }
        else {
            oriError = newOriError;
            for (auto j=0u; j<joints_.size(); ++j)
                lastThetas[j] = joints_[j].getTheta();
        }*/
    }

    printf("joint 0 theta: %f\n", joints_[0].getTheta());
    printf("posError: %f\n", posError);
    printf("oriError: %f\n", oriError);
}
/*
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
}*/

// computes end_effector up to certain number of segments
Vector3Glf Arm::calculateEndEffector(int jointId /* = -1 */) const {
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

float Arm::getPositionError(const Vector3Glf& goal) const {
    return (goal - calculateEndEffector()).norm();
}

float Arm::getOrientationError(const Vector3Glf& goalOrientation) const {
    auto& lastJoint = joints_.back();
    //std::cout << lastJoint.getForward().normalized().transpose() << std::endl;
    return (1.0f - goalOrientation.normalized().dot(lastJoint.getForward().normalized())) * 0.5f;
}
