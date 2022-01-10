#ifndef MATRIX_FORMS_H
#define MATRIX_FORMS_H



#include "matrix_base.h"


//No its the Fast Math Library namespace. Nothing else
namespace FML {



/**
 * 3x3 Matrix class with special functions for rotations etc.
 */
template<typename TYPE>
class Matrix33: public Matrix<TYPE, 3, 3> {
public:

    Matrix33() {}

    Matrix33(const TYPE& val): Matrix<TYPE, 3, 3>(val) {}

    Matrix33(const Matrix<TYPE, 3, 3>& matrix): Matrix<TYPE, 3, 3>(matrix) {}
    

};

using Matrix33_F = Matrix33<float>; //For float vector types
using Matrix33_D = Matrix33<double>; //For float vector types



/**
 * 4x4 Matrix class with special functions for transforming coordinate systems.
 */
template<typename TYPE>
class Matrix44: public Matrix<TYPE, 4, 4> {
public:



};

using Matrix44_F = Matrix44<float>; //For float vector types
using Matrix44_D = Matrix44<double>; //For float vector types


} //Namespace end


#endif