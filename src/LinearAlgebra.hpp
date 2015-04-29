#ifndef CUCCA_CORE_LINEARALGEBRA_HPP
#define CUCCA_CORE_LINEARALGEBRA_HPP


#include <Eigen/Dense>
#include <GL/glew.h>


#define PI 3.14159265358979323846


///  Typedefs

typedef Eigen::Matrix<float, 2, 1, Eigen::DontAlign> Vector2f;
typedef Eigen::Matrix<float, 3, 1, Eigen::DontAlign> Vector3f;
typedef Eigen::Matrix<float, 4, 1, Eigen::DontAlign> Vector4f;

typedef Eigen::Matrix<float, 2, 2, Eigen::DontAlign> Matrix2f;
typedef Eigen::Matrix<float, 3, 3, Eigen::DontAlign> Matrix3f;
typedef Eigen::Matrix<float, 4, 4, Eigen::DontAlign> Matrix4f;

typedef Eigen::Matrix<GLfloat, 2, 1, Eigen::DontAlign> Vector2Glf;
typedef Eigen::Matrix<GLfloat, 3, 1, Eigen::DontAlign> Vector3Glf;
typedef Eigen::Matrix<GLfloat, 4, 1, Eigen::DontAlign> Vector4Glf;

typedef Eigen::Matrix<GLfloat, 2, 2, Eigen::DontAlign> Matrix2Glf;
typedef Eigen::Matrix<GLfloat, 3, 3, Eigen::DontAlign> Matrix3Glf;
typedef Eigen::Matrix<GLfloat, 4, 4, Eigen::DontAlign> Matrix4Glf;

typedef Eigen::Quaternion<float>  Quaternionf;
typedef Eigen::Quaternion<GLfloat>  QuaternionGlf;


///  Functions

template<typename T, int W, int H>
void clamp(Eigen::Matrix<T, W, H, Eigen::DontAlign>& matrix, const T& min, const T& max);


///  Function definitions

//  Clamps all values in given matrix in range specified
template<typename T, int W, int H>
void clamp(Eigen::Matrix<T, W, H, Eigen::DontAlign>& matrix, const T& min, const T& max) {
    for (int x=0; x<W; ++x)
        for (int y=0; y<H; ++y)
            if (matrix(x, y) < min)
                matrix(x, y) = min;
            else if (matrix(x, y) > max)
                matrix(x, y) = max;
}


//  Moore-penrose pseudoinverse
template<typename _Matrix_Type_>
_Matrix_Type_ pseudoInverse(const _Matrix_Type_ &a, double epsilon =
std::numeric_limits<double>::epsilon())
{
        Eigen::JacobiSVD< _Matrix_Type_ > svd(a ,Eigen::ComputeThinU |
    Eigen::ComputeThinV);
        double tolerance = epsilon * std::max(a.cols(), a.rows())
    *svd.singularValues().array().abs()(0);
        return svd.matrixV() *  (svd.singularValues().array().abs() >
    tolerance).select(svd.singularValues().array().inverse(),
    0).matrix().asDiagonal() * svd.matrixU().adjoint();
}


#endif // CUCCA_CORE_LINEARALGEBRA_HPP
