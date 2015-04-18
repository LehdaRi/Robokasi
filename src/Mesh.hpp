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
#include "LinearAlgebra.hpp"

#include <string>
#include <Gl/glew.h>


class Mesh {
public:
    Mesh(void);

    void loadFromObj(const std::string& fileName);
    void render(const Shader& shader,
                const Camera& camera,
                const Vector3Glf& color = Vector3Glf(1.0f, 1.0f, 1.0f)) const;

private:
    GLuint vertexArrayObjectId_;
    unsigned nIndices_;

    GLuint positionBufferId_;
    GLuint texCoordBufferId_;
    GLuint normalBufferId_;
    GLuint elementBufferId_;
};


#endif // ROBOKASI_MESH_HPP
