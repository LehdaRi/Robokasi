/**

    Automaatio- ja Systeemitekniikan killan robokäsiprojekti
    Camera.hpp

    @version    0.1
    @author     Miika 'LehdaRi' Lehtimäki
    @date       2015-04-18

**/


#ifndef ROBOKASI_CAMERA_HPP
#define ROBOKASI_CAMERA_HPP


#include "LinearAlgebra.hpp"


class Camera {
public:
    Camera(void);

    void lookAt(const Vector3Glf& from, const Vector3Glf& to, const Vector3Glf& up);
    void lookAt(Vector3Glf&& from, Vector3Glf&& to, Vector3Glf&& up);
    void projection(float fov, float aspectRatio, float near, float far);

    Matrix4Glf getVP(void) const;

private:
    Matrix4Glf orientation_;
    Matrix4Glf projection_;
};


#endif // ROBOKASI_CAMERA_HPP
