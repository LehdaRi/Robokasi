/**

    Automaatio- ja Systeemitekniikan killan robokäsiprojekti
    Joint.hpp

    @version    0.1
    @author     Miika 'LehdaRi' Lehtimäki
    @date       2015-04-18

**/


#ifndef ROBOKASI_JOINT_HPP
#define ROBOKASI_JOINT_HPP


#include "LinearAlgebra.hpp"
#include "Camera.hpp"
#include "Shader.hpp"

#include <array>


class Joint {
public:
    Joint(Shader& shader);
    ~Joint(void);

    Joint(const Joint& other)             = delete;
    Joint(Joint&& other)                  = delete;
    Joint& operator=(const Joint& other)  = delete;
    Joint& operator=(Joint&& other)       = delete;

    //  Set Denavit-Hartenberg parameters for next joint
    void setDHParameters(float theta, float d, float r, float alpha);

    //  Apply previous joint transformation to this joint
    void applyJoint(const Joint& other);

    Matrix4Glf getOrientation(void) const;

    void setPosition(const Vector3Glf& position);
    void setRotation(const Matrix3Glf& rotation);

    void draw(const Camera& camera) const;

private:
    static std::array<float, 3> refFrameVertexPosData__[6];
    static std::array<float, 3> refFrameVertexColData__[6];

    Vector3Glf position_;
    Matrix3Glf rotation_;

    Matrix4Glf orientation_;
    Matrix4Glf orientationNext_;
    Matrix4Glf orientation2_;

    Shader& shader_;
    GLuint posBuffer_;
    GLuint colBuffer_;
};


#endif // ROBOKASI_JOINT_HPP
