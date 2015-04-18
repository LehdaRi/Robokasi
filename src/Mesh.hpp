/**

    Automaatio- ja Systeemitekniikan killan robokäsiprojekti
    Mesh.hpp

    @version    0.1
    @author     Miika 'LehdaRi' Lehtimäki
    @date       2015-04-18

**/


#ifndef ROBOKASI_MESH_HPP
#define ROBOKASI_MESH_HPP


#include "Shader.hpp"
#include "Camera.hpp"
#include "Joint.hpp"
#include "LinearAlgebra.hpp"

#include <string>
#include <GL/glew.h>


class Mesh {
public:
    Mesh(void);
    ~Mesh(void);

    Mesh(const Mesh& other)             = delete;
    Mesh(Mesh&& other)                  = delete;
    Mesh& operator=(const Mesh& other)  = delete;
    Mesh& operator=(Mesh&& other)       = delete;

    void loadFromObj(const std::string& fileName);
    void render(const Shader& shader,
                const Camera& camera,
                const Vector3Glf& color = Vector3Glf(1.0f, 1.0f, 1.0f)) const;

    void render(const Shader& shader,
                const Camera& camera,
                const Joint& joint,
                const Vector3Glf& color = Vector3Glf(1.0f, 1.0f, 1.0f)) const;

    void setPosition(const Vector3Glf& position);
    void setRotation(const Matrix3Glf& rotation);

    Matrix4Glf getOrientation(void) const;

private:
    GLuint vertexArrayObjectId_;
    unsigned nIndices_;

    GLuint positionBufferId_;
    GLuint texCoordBufferId_;
    GLuint normalBufferId_;
    GLuint elementBufferId_;

    Vector3Glf position_;
    Matrix3Glf rotation_;
};


#endif // ROBOKASI_MESH_HPP
