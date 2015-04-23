/**

    Automaatio- ja Systeemitekniikan killan robokäsiprojekti
    Joint.cpp

    @version    0.1
    @author     Miika 'LehdaRi' Lehtimäki
    @date       2015-04-18

**/


#include "Joint.hpp"

#include <GL/glew.h>


///  Static members

std::array<float, 3> Joint::refFrameVertexPosData__[] = {
    {0.0f, 0.0f, 0.0f},
    {8.0f, 0.0f, 0.0f},
    {0.0f, 0.0f, 0.0f},
    {0.0f, 8.0f, 0.0f},
    {0.0f, 0.0f, 0.0f},
    {0.0f, 0.0f, 8.0f}
};

std::array<float, 3> Joint::refFrameVertexColData__[] = {
    {1.0f, 0.0f, 0.0f},
    {1.0f, 0.0f, 0.0f},
    {0.0f, 1.0f, 0.0f},
    {0.0f, 1.0f, 0.0f},
    {0.0f, 0.0f, 1.0f},
    {0.0f, 0.0f, 1.0f}
};


///  Member function definitions

Joint::Joint(Shader& shader) :
    position_(0.0f, 0.0f, 0.0f),
    rotation_(Matrix3Glf::Identity()),
    orientation_(Matrix4Glf::Identity()),
    orientationNext_(Matrix4Glf::Identity()),
    shader_(shader),
    posBuffer_(0),
    colBuffer_(0)
{
    glGenBuffers(1, &posBuffer_);
    glBindBuffer(GL_ARRAY_BUFFER, posBuffer_);
    glBufferData(GL_ARRAY_BUFFER, sizeof(refFrameVertexPosData__), refFrameVertexPosData__, GL_STATIC_DRAW);

    glGenBuffers(1, &colBuffer_);
    glBindBuffer(GL_ARRAY_BUFFER, colBuffer_);
    glBufferData(GL_ARRAY_BUFFER, sizeof(refFrameVertexColData__), refFrameVertexColData__, GL_STATIC_DRAW);
}

Joint::~Joint(void) {
    glDeleteBuffers(1, &posBuffer_);
    glDeleteBuffers(1, &colBuffer_);
}

void Joint::setDHParameters(float theta, float d, float r, float alpha) {
    float st = sinf(theta);
    float ct = cosf(theta);
    float sa = sinf(alpha);
    float ca = cosf(alpha);
    orientationNext_ << ct      , -st*ca    , st*sa     , r*ct  ,
                        st      , ct*ca     , -ct*sa    , r*st  ,
                        0.0f    , sa        , ca        , d     ,
                        0.0f    , 0.0f      , 0.0f      , 1.0f  ;

    orientation2_ << ct     , -st   , 0.0f  , 0.0f  ,
                     st     , ct    , 0.0f  , 0.0f  ,
                     0.0f   , 0.0f  , 1.0f  , d     ,
                     0.0f   , 0.0f  , 0.0f  , 1.0f  ;

    std::array<float, 3> pos[] = {
        refFrameVertexPosData__[0],
        refFrameVertexPosData__[1],
        refFrameVertexPosData__[2],
        refFrameVertexPosData__[3],
        refFrameVertexPosData__[4],
        refFrameVertexPosData__[5],
        {0.0f, 0.0f, 0.0f},
        {0.0f, 0.0f, d}
    };

    std::array<float, 3> col[8] = {
        refFrameVertexColData__[0],
        refFrameVertexColData__[1],
        refFrameVertexColData__[2],
        refFrameVertexColData__[3],
        refFrameVertexColData__[4],
        refFrameVertexColData__[5],
        {1.0f, 1.0f, 1.0f},
        {1.0f, 1.0f, 1.0f}
    };

    glBindBuffer(GL_ARRAY_BUFFER, posBuffer_);
    glBufferData(GL_ARRAY_BUFFER, sizeof(pos), pos, GL_STATIC_DRAW);

    glBindBuffer(GL_ARRAY_BUFFER, colBuffer_);
    glBufferData(GL_ARRAY_BUFFER, sizeof(col), col, GL_STATIC_DRAW);
}

void Joint::applyJoint(const Joint& other) {
    orientation_ = other.orientation_ * other.orientationNext_;
}

Matrix4Glf Joint::getOrientation(void) const {
    return orientation_ * orientation2_;
}

void Joint::setPosition(const Vector3Glf& position) {
    position_ = position;
}

void Joint::setRotation(const Matrix3Glf& rotation) {
    rotation_ = rotation;
}

void Joint::draw(const Camera& camera) const {
    shader_.useShader(camera.getVP() * orientation_);

    glBindBuffer(GL_ARRAY_BUFFER, posBuffer_);
    glEnableVertexAttribArray(0);
    glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 0, (GLvoid*)0);

    glBindBuffer(GL_ARRAY_BUFFER, colBuffer_);
    glEnableVertexAttribArray(1);
    glVertexAttribPointer(1, 3, GL_FLOAT, GL_FALSE, 0, (GLvoid*)0);

    glDrawArrays(GL_LINES, 0, 8);

    glDisableVertexAttribArray(0);
}
