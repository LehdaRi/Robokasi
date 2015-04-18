#ifndef ROBOKASI_SHADER_HPP
#define ROBOKASI_SHADER_HPP


#include "LinearAlgebra.hpp"

#include <string>
#include <GL/glew.h>


class Shader {
public:
    Shader(const std::string& vsFileName, const std::string& fsFileName);

    GLuint getId(void) const;
    void useShader(const Matrix4Glf& mvp, const Vector3Glf& color) const;

private:
    GLuint programId_;
    GLuint uniformPosition_MVP_;
    GLuint uniformPosition_Color_;
};


#endif // ROBOKASI_SHADER_HPP
