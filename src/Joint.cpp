/**

    Automaatio- ja Systeemitekniikan killan robokäsiprojekti
    Joint.cpp

    @version    0.1
    @author     Miika 'LehdaRi' Lehtimäki
    @date       2015-04-29

**/


#include "Joint.hpp"

#include <GL/glew.h>
#include <iostream> // TEMP


///  Static members

std::array<float, 4> Joint::refFrameVertexPosData__[] = {
    {0.0f, 0.0f, 0.0f, 1.0},
    {8.0f, 0.0f, 0.0f, 1.0},
    {0.0f, 0.0f, 0.0f, 1.0},
    {0.0f, 8.0f, 0.0f, 1.0},
    {0.0f, 0.0f, 0.0f, 1.0},
    {0.0f, 0.0f, 8.0f, 1.0}
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

Joint::Joint(Shader& shader, const Matrix4Glf& jointMatrix) :
    orientation_(Matrix4Glf::Identity()),
    jointMatrix_(jointMatrix),
    theta_(0.0f),
    rotZ_(Matrix4Glf::Identity()),
    shader_(shader),
    vertexArrayObjectId_(0),
    posBuffer_(0),
    colBuffer_(0)
{
    glGenVertexArrays(1, &vertexArrayObjectId_);
    glBindVertexArray(vertexArrayObjectId_);

    glGenBuffers(1, &posBuffer_);
    glBindBuffer(GL_ARRAY_BUFFER, posBuffer_);
    glBufferData(GL_ARRAY_BUFFER, sizeof(refFrameVertexPosData__), refFrameVertexPosData__, GL_STATIC_DRAW);
    glEnableVertexAttribArray(0);
    glVertexAttribPointer(0, 4, GL_FLOAT, GL_FALSE, 0, (GLvoid*)0);

    glGenBuffers(1, &colBuffer_);
    glBindBuffer(GL_ARRAY_BUFFER, colBuffer_);
    glBufferData(GL_ARRAY_BUFFER, sizeof(refFrameVertexColData__), refFrameVertexColData__, GL_STATIC_DRAW);
    glEnableVertexAttribArray(1);
    glVertexAttribPointer(1, 3, GL_FLOAT, GL_FALSE, 0, (GLvoid*)0);

    glBindVertexArray(0);
}

Joint::Joint(const Joint& other) :
    orientation_(other.orientation_),
    jointMatrix_(other.jointMatrix_),
    theta_(other.theta_),
    rotZ_(other.rotZ_),
    shader_(other.shader_),
    posBuffer_(0),
    colBuffer_(0)
{
    glGenVertexArrays(1, &vertexArrayObjectId_);
    glBindVertexArray(vertexArrayObjectId_);

    glGenBuffers(1, &posBuffer_);
    glBindBuffer(GL_ARRAY_BUFFER, posBuffer_);
    glBufferData(GL_ARRAY_BUFFER, sizeof(refFrameVertexPosData__), refFrameVertexPosData__, GL_STATIC_DRAW);
    glEnableVertexAttribArray(0);
    glVertexAttribPointer(0, 4, GL_FLOAT, GL_FALSE, 0, (GLvoid*)0);

    glGenBuffers(1, &colBuffer_);
    glBindBuffer(GL_ARRAY_BUFFER, colBuffer_);
    glBufferData(GL_ARRAY_BUFFER, sizeof(refFrameVertexColData__), refFrameVertexColData__, GL_STATIC_DRAW);
    glEnableVertexAttribArray(1);
    glVertexAttribPointer(1, 3, GL_FLOAT, GL_FALSE, 0, (GLvoid*)0);

    glBindVertexArray(0);
}

Joint::Joint(Joint&& other) :
    orientation_(other.orientation_),
    jointMatrix_(other.jointMatrix_),
    theta_(other.theta_),
    rotZ_(other.rotZ_),
    shader_(other.shader_),
    posBuffer_(0),
    colBuffer_(0)
{
    glGenVertexArrays(1, &vertexArrayObjectId_);
    glBindVertexArray(vertexArrayObjectId_);

    glGenBuffers(1, &posBuffer_);
    glBindBuffer(GL_ARRAY_BUFFER, posBuffer_);
    glBufferData(GL_ARRAY_BUFFER, sizeof(refFrameVertexPosData__), refFrameVertexPosData__, GL_STATIC_DRAW);
    glEnableVertexAttribArray(0);
    glVertexAttribPointer(0, 4, GL_FLOAT, GL_FALSE, 0, (GLvoid*)0);

    glGenBuffers(1, &colBuffer_);
    glBindBuffer(GL_ARRAY_BUFFER, colBuffer_);
    glBufferData(GL_ARRAY_BUFFER, sizeof(refFrameVertexColData__), refFrameVertexColData__, GL_STATIC_DRAW);
    glEnableVertexAttribArray(1);
    glVertexAttribPointer(1, 3, GL_FLOAT, GL_FALSE, 0, (GLvoid*)0);

    glBindVertexArray(0);

    glDeleteVertexArrays(1, &other.vertexArrayObjectId_);
    glDeleteBuffers(1, &other.posBuffer_);
    glDeleteBuffers(1, &other.colBuffer_);
}

Joint& Joint::operator=(const Joint& other) {
    orientation_ = other.orientation_;
    jointMatrix_ = other.jointMatrix_;
    theta_ = other.theta_;
    rotZ_ = other.rotZ_;

    return *this;
}

Joint& Joint::operator=(Joint&& other) {
    orientation_ = other.orientation_;
    jointMatrix_ = other.jointMatrix_;
    theta_ = other.theta_;
    rotZ_ = other.rotZ_;

    glDeleteVertexArrays(1, &other.vertexArrayObjectId_);
    glDeleteBuffers(1, &other.posBuffer_);
    glDeleteBuffers(1, &other.colBuffer_);

    return *this;
}

Joint::~Joint(void) {
    glDeleteBuffers(1, &posBuffer_);
    glDeleteBuffers(1, &colBuffer_);
}

/*
void Joint::rotY(float theta){
    Matrix4Glf rotT;
    float st = sinf(other.theta_);
    float ct = cosf(other.theta_);
    rotT   <<   ct  , -st , 0.0f, 0.0f,
                st  , ct  , 0.0f, 0.0f,
                0.0f, 0.0f, 1.0f, 0.0f,
                0.0f, 0.0f, 0.0f, 1.0f;

    orientation_ =  other.orientation_ * rotT * other.jointMatrix_;
}
*/

/*void Joint::setDHParameters(float theta, float r, float d, float alpha) {
    float st = sinf(theta);
    float ct = cosf(theta);
    float sa = sinf(alpha);
    float ca = cosf(alpha);

    jointMatrix_ << ct      , -st*ca    , st*sa     , r*ct  ,
                        st      , ct*ca     , -ct*sa    , r*st  ,
                        0.0f    , sa        , ca        , d     ,
                        0.0f    , 0.0f      , 0.0f      , 1.0f  ;
}*/

void Joint::applyJoint(const Joint& other) {
    orientation_ =  other.orientation_ * other.rotZ_ * other.jointMatrix_;
}

Matrix4Glf Joint::getOrientation(void) const {
    return orientation_ * rotZ_;
}

float Joint::getLength(void) const {
    return jointMatrix_.block<3, 1>(0, 3).norm();
}

Vector3Glf Joint::getRight(void) const {
    return (getOrientation() * Vector4Glf(1.0f, 0.0f, 0.0f, 0.0f)).block<3, 1>(0, 0);
}

Vector3Glf Joint::getUp(void) const {
    return (getOrientation() * Vector4Glf(0.0f, 1.0f, 0.0f, 0.0f)).block<3, 1>(0, 0);
}

Vector3Glf Joint::getForward(void) const {
    return (getOrientation() * Vector4Glf(0.0f, 0.0f, 1.0f, 0.0f)).block<3, 1>(0, 0);
}

Vector3Glf Joint::getEndPoint(void) const {
    return (getOrientation() * jointMatrix_ * Vector4Glf(0.0f, 0.0f, 0.0f, 1.0f)).block<3, 1>(0, 0);
}

void Joint::setJointMatrix(const Matrix4Glf& m) {
    jointMatrix_ = m;
}

const Matrix4Glf& Joint::getJointMatrix(void) const {
    return jointMatrix_;
}

void Joint::setTheta(float theta) {
    theta_ = theta;

    float st = sinf(theta_);
    float ct = cosf(theta_);
    rotZ_  <<   ct  , -st , 0.0f, 0.0f,
                st  , ct  , 0.0f, 0.0f,
                0.0f, 0.0f, 1.0f, 0.0f,
                0.0f, 0.0f, 0.0f, 1.0f;
}

float Joint::getTheta(void) const {
    return theta_;
}

void Joint::draw(const Camera& camera) const {
    shader_.useShader(camera.getVP() * getOrientation());

    glBindVertexArray(vertexArrayObjectId_);

    glDrawArrays(GL_LINES, 0, 6);

    glBindVertexArray(0);
}
