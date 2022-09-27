#ifndef MATRIX_QUATERNION_H
#define MATRIX_QUATERNION_H



#include "matrix_base.h"


//No its the Fast Math Library namespace. Nothing else
namespace FML {


/**
 * Quaternion matrix class for quaternion calculations.
 */
template<typename TYPE>
class Quat: public Matrix<TYPE, 4, 1> {
public:

    Quat();

    Quat(const TYPE& w, const TYPE& x, const TYPE& y, const TYPE& z);

    /**
     * @brief Construct a new Quat object from an axis and an angle. 
     * @note Axis must be normalised!
     * 
     * @param axis 3x1 axis around the rotation is to be.
     * @param angle Angle [Rad] rotation.
     */
    Quat(const Matrix<TYPE, 3, 1>& axis, const TYPE& angle);

    /**
     * @brief Construct a new Quat object from a 4x1 vector representing a quaternion.
     * 
     * @param quaternion 4x1 vector.
     */
    template<typename TYPE2>
    Quat(const Matrix<TYPE2, 4, 1>& quaternion);

    /**
     * @brief Construct a new Quat object from a vector. X,Y,Z are set to vector components and W to 0;
     * 
     * @param rotVector 3x1 Vector to be converted to a quaternion.
     */
    template<typename TYPE2>
    Quat(const FML::Vector<TYPE2, 3>& vector);

    /**
     * @brief Construct a new Quat object from a 3x3 rotation matrix.
     * 
     * @param rotMatrix 3x3 rotation matrix.
     */
    //Quat(const Matrix<TYPE, 3, 3>& rotMatrix);


    /**
     * @brief Sets this quaternion to represent the rotation given by the 3D Rotation vector who's direction is the axis and length is the angle [Rad].
     * 
     * @tparam TYPE2 
     * @param rotVec Direction is axis and length is angle [Rad].
     */
    template<typename TYPE2>
    void fromRotVec(Matrix<TYPE2, 3, 1> rotVec);
    
    /**
     * @brief Converts this rotation into a vector who's axis in the vector direction and angle [Rad] is the vector length.
     * 
     * @return Matrix<TYPE, 3, 1> 
     */
    Matrix<TYPE, 3, 1> toRotVec() const;


    /**
     * @brief Conjugates the quaternion. Index 0 sign left as is and 1-3 signs are flipped. 
     * 
     * @return Quat<TYPE> 
     */
    Quat<TYPE> conjugate() const;

    /**
     * @brief Rotates the given vector using this quaternion.
     * 
     * @tparam TYPE2 
     * @param vector 
     * @return Vector<TYPE2> 
     */
    template<typename TYPE2>
    Vector<TYPE2, 3> rotateVec(const Matrix<TYPE2, 3, 1>& vector) const;

    /**
     * @brief Does quaternion multiplication. Different than matrix or vector multiplication.
     * 
     * @tparam TYPE2 
     * @param quat 
     * @return Quat<TYPE> 
     */
    template<typename TYPE2>
    Quat<TYPE> operator* (const Quat<TYPE2>& quat) const;

    template<typename TYPE2>
    Quat<TYPE> operator* (const Vector<TYPE2, 3>& vector) const;

    template<typename TYPE2>
    Quat<TYPE> operator* (const TYPE2& scaler) const;

    template<typename TYPE2>
    Quat<TYPE> operator/ (const TYPE2& divider) const;

    /**
     * @brief Converts to a 3x3 rotation matrix
     * 
     * @tparam TYPE2 
     * @return FML::Matrix<TYPE2, 3, 3> 
     */
    template<typename TYPE2>
    operator FML::Matrix<TYPE2, 3, 3>() const;

    /**
     * @brief Converts to a vector by setting the x,y,z components and leaving w out.
     * 
     * @tparam TYPE2 
     * @return FML::Matrix<TYPE2, 3, 1> 
     */
    template<typename TYPE2>
    operator FML::Matrix<TYPE2, 3, 1>() const;

    /**
     * @brief Simply converts to a 4x1 matrix containing the w,x,y,z components.
     * 
     * @tparam TYPE2 
     * @return FML::Matrix<TYPE2, 4, 1> 
     */
    template<typename TYPE2>
    operator FML::Matrix<TYPE2, 4, 1>() const;

};


template<typename TYPE>
Quat<TYPE>::Quat() {
    this->r[0][0] = 1.0f;
    this->r[1][0] = 0.0f;
    this->r[2][0] = 0.0f;
    this->r[3][0] = 0.0f;
}

template<typename TYPE>
Quat<TYPE>::Quat(const TYPE& w, const TYPE& x, const TYPE& y, const TYPE& z) {
    this->r[0][0] = w;
    this->r[1][0] = x;
    this->r[2][0] = y;
    this->r[3][0] = z;
}

template<typename TYPE>
Quat<TYPE>::Quat(const Matrix<TYPE, 3, 1>& axis, const TYPE& angle) {

    TYPE sa = sin(angle/2);
                
    this->r[0][0] = cos(angle/2);
    this->r[1][0] = axis.r[0][0]*sa;
    this->r[2][0] = axis.r[1][0]*sa;
    this->r[3][0] = axis.r[2][0]*sa;

}

template<typename TYPE>
template<typename TYPE2>
Quat<TYPE>::Quat(const Matrix<TYPE2, 4, 1>& quaternion) {
                
    this->r[0][0] = quaternion.r[0][0];
    this->r[1][0] = quaternion.r[1][0];
    this->r[2][0] = quaternion.r[2][0];
    this->r[3][0] = quaternion.r[3][0];

}

template<typename TYPE>
template<typename TYPE2>
Quat<TYPE>::Quat(const FML::Vector<TYPE2, 3>& vector) {
                
    this->r[0][0] = 0;
    this->r[1][0] = vector.r[0][0];
    this->r[2][0] = vector.r[1][0];
    this->r[3][0] = vector.r[2][0];

}

/*template<typename TYPE>
Quat<TYPE>::Quat(const Matrix<TYPE, 3, 3>& rotMatrix) {



}*/


template<typename TYPE>
template<typename TYPE2>
void Quat<TYPE>::fromRotVec(FML::Matrix<TYPE2, 3, 1> rotVec) {

    TYPE angle = rotVec.magnitude();
    TYPE factor = sin(angle/2) / angle; //Factor to normalise and scale vector.
                
    this->r[0][0] = cos(angle/2);
    this->r[1][0] = rotVec.r[0][0]*factor;
    this->r[2][0] = rotVec.r[1][0]*factor;
    this->r[3][0] = rotVec.r[2][0]*factor;

}


template<typename TYPE>
FML::Matrix<TYPE, 3, 1> Quat<TYPE>::toRotVec() const {

    FML::Matrix<TYPE, 3, 1> rotVec;
    rotVec.r[0][0] = this->r[1][0];
    rotVec.r[1][0] = this->r[2][0];
    rotVec.r[2][0] = this->r[3][0];

    rotVec = rotVec * (acos(this->r[0][0]) * 2 / rotVec.magnitude());

    return rotVec;

}


template<typename TYPE>
Quat<TYPE> Quat<TYPE>::conjugate() const {

    Quat<TYPE> conj = *this;
    conj.r[1][0] = -conj.r[1][0];
    conj.r[2][0] = -conj.r[2][0];
    conj.r[3][0] = -conj.r[3][0];

    return conj;

}


template<typename TYPE>
template<typename TYPE2>
Vector<TYPE2, 3> Quat<TYPE>::rotateVec(const Matrix<TYPE2, 3, 1>& vector) const {

    return (*this) * Quat<TYPE2>(vector) * this->conjugate();

}


template<typename TYPE>
template<typename TYPE2>
Quat<TYPE> Quat<TYPE>::operator* (const Quat<TYPE2>& quat) const {

    Quat<TYPE> result (
        quat.r[0][0]*this->r[0][0] - quat.r[1][0]*this->r[1][0] - quat.r[2][0]*this->r[2][0] - quat.r[3][0]*this->r[3][0],
        quat.r[0][0]*this->r[1][0] + quat.r[1][0]*this->r[0][0] + quat.r[2][0]*this->r[3][0] - quat.r[3][0]*this->r[2][0],
        quat.r[0][0]*this->r[2][0] - quat.r[1][0]*this->r[3][0] + quat.r[2][0]*this->r[0][0] + quat.r[3][0]*this->r[1][0],
        quat.r[0][0]*this->r[3][0] + quat.r[1][0]*this->r[2][0] - quat.r[2][0]*this->r[1][0] + quat.r[3][0]*this->r[0][0]
    );

    return result;

}


template<typename TYPE>
template<typename TYPE2>
Quat<TYPE> Quat<TYPE>::operator* (const Vector<TYPE2, 3>& vector) const {

    Quat<TYPE> vectorQuat = vector;

    return (*this) * vectorQuat;

}


template<typename TYPE>
template<typename TYPE2>
Quat<TYPE> Quat<TYPE>::operator* (const TYPE2& scaler) const {

    Quat<TYPE> quat(*this);
    quat.r[0][0] = quat.r[0][0] * scaler;
    quat.r[1][0] = quat.r[1][0] * scaler;
    quat.r[2][0] = quat.r[2][0] * scaler;
    quat.r[3][0] = quat.r[3][0] * scaler;

    return quat;

}


template<typename TYPE, typename TYPE2>
Quat<TYPE> operator * (const TYPE2& scaler, const Quat<TYPE>& right) {
    return right * scaler;
}


template<typename TYPE>
template<typename TYPE2>
Quat<TYPE> Quat<TYPE>::operator/ (const TYPE2& divider) const {

    Quat<TYPE> quat(*this);
    quat.r[0][0] = quat.r[0][0] / divider;
    quat.r[1][0] = quat.r[1][0] / divider;
    quat.r[2][0] = quat.r[2][0] / divider;
    quat.r[3][0] = quat.r[3][0] / divider;

    return quat;

}


template<typename TYPE>
template<typename TYPE2>
Quat<TYPE>::operator FML::Matrix<TYPE2, 3, 3>() const {

    Matrix<TYPE2, 3, 3> mat;
    mat.r[0][0] = 1 - 2 * (this->r[2][0] * this->r[2][0] + this->r[3][0] * this->r[3][0]);
	mat.r[0][1] = 2 * (this->r[1][0] * this->r[2][0] - this->r[0][0] * this->r[3][0]);
	mat.r[0][2] = 2 * (this->r[0][0] * this->r[2][0] + this->r[1][0] * this->r[3][0]);
	mat.r[1][0] = 2 * (this->r[1][0] * this->r[2][0] + this->r[0][0] * this->r[3][0]);
	mat.r[1][1] = 1 - 2 * (this->r[1][0] * this->r[1][0] + this->r[3][0] * this->r[3][0]);;
	mat.r[1][2] = 2 * (this->r[2][0] * this->r[3][0] - this->r[0][0] * this->r[1][0]);
	mat.r[2][0] = 2 * (this->r[1][0] * this->r[3][0] - this->r[0][0] * this->r[2][0]);
	mat.r[2][1] = 2 * (this->r[0][0] * this->r[1][0] + this->r[2][0] * this->r[3][0]);
	mat.r[2][2] =  1 - 2 * (this->r[2][0] * this->r[2][0] + this->r[1][0] * this->r[1][0]);;

    return mat;

}

template<typename TYPE>
template<typename TYPE2>
Quat<TYPE>::operator FML::Matrix<TYPE2, 3, 1>() const {

    FML::Matrix<TYPE2, 3, 1> rotVec;

    rotVec.r[0][0] = this->r[1][0];
    rotVec.r[1][0] = this->r[2][0];
    rotVec.r[2][0] = this->r[3][0];

    rotVec = rotVec * (acos(this->r[0][0])*2 / rotVec.magnitude());

    return rotVec;

}

template<typename TYPE>
template<typename TYPE2>
Quat<TYPE>::operator FML::Matrix<TYPE2, 4, 1>() const {
    return *this;
}



using Quat_F = Quat<float>; //For float quaternion types

using Quat_D = Quat<double>; //For double quaternion types



} //Namespace end



#endif