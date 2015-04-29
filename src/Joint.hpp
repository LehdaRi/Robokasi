/**

    Automaatio- ja Systeemitekniikan killan robokäsiprojekti
    Joint.hpp

    @version    0.1
    @author     Miika 'LehdaRi' Lehtimäki
    @date       2015-04-24

**/


#ifndef ROBOKASI_JOINT_HPP
#define ROBOKASI_JOINT_HPP


#include "LinearAlgebra.hpp"
#include "Camera.hpp"
#include "Shader.hpp"

#include <array>


class Joint {
public:
    Joint(Shader& shader, const Matrix4Glf& jointMatrix);
    ~Joint(void);

    Joint(const Joint& other);
    Joint(Joint&& other);
    Joint& operator=(const Joint& other);
    Joint& operator=(Joint&& other);

    //  Set Denavit-Hartenberg parameters for next joint
    void setDHParameters(float theta, float r, float d, float alpha);

    //  Apply previous joint transformation to this joint
    void applyJoint(const Joint& other);

    Matrix4Glf getOrientation(void) const;
    float getLength(void) const;

    Vector3Glf getRight(void) const;
    Vector3Glf getUp(void) const;
    Vector3Glf getForward(void) const;

    Vector3Glf getEndPoint(void) const;

    void setJointMatrix(const Matrix4Glf& m);
    const Matrix4Glf& getJointMatrix(void) const;
    void setTheta(float theta);
    float getTheta(void) const;

    void draw(const Camera& camera) const;

private:
    static std::array<float, 4> refFrameVertexPosData__[6];
    static std::array<float, 3> refFrameVertexColData__[6];

    Matrix4Glf orientation_;
    Matrix4Glf jointMatrix_;
    float theta_;
    Matrix4Glf rotZ_;

    Shader& shader_;
    GLuint vertexArrayObjectId_;
    GLuint posBuffer_;
    GLuint colBuffer_;
};


#endif // ROBOKASI_JOINT_HPP
