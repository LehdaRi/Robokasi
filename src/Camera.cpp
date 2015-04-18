/**

    Automaatio- ja Systeemitekniikan killan robokäsiprojekti
    Camera.cpp

    @version    0.1
    @author     Miika 'LehdaRi' Lehtimäki
    @date       2015-04-18

**/


#include "Camera.hpp"


Camera::Camera(void) :
    orientation_(Matrix4Glf::Identity()),
    projection_(Matrix4Glf::Identity())
{}

void Camera::lookAt(const Vector3Glf& from, const Vector3Glf& to, const Vector3Glf& up) {
    Vector3Glf xAxis, yAxis, zAxis, t;

    zAxis = (to - from).normalized();
    xAxis = up.cross(zAxis).normalized();
    yAxis = zAxis.cross(xAxis);

    orientation_ << xAxis[0]    , xAxis[1]  , xAxis[2]  , -xAxis.dot(from),
                    yAxis[0]    , yAxis[1]  , yAxis[2]  , -yAxis.dot(from),
                    -zAxis[0]   , -zAxis[1] , -zAxis[2] , zAxis.dot(from) ,
                    0.0f        , 0.0f      , 0.0f      , 1.0f;
}

void Camera::lookAt(Vector3Glf&& from, Vector3Glf&& to, Vector3Glf&& up) {
    lookAt(from, to, up);
}


void Camera::projection(float fov, float aspectRatio, float near, float far) {
    float r = tanf(fov / 2.0f) * near;
    float t = r / aspectRatio;

    projection_ << near/r   , 0.0f      , 0.0f                  , 0.0f                      ,
                   0.0f     , near/t    , 0.0f                  , 0.0f                      ,
                   0.0f     , 0.0f      , -(far+near)/(far-near), -2.0f*far*near/(far-near) ,
                   0.0f     , 0.0f      , -1.0f                 , 0.0f                      ;
}

Matrix4Glf Camera::getVP(void) const {
    return projection_ * orientation_;
}
