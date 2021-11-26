#ifndef MATRIX_FORMS_H
#define MATRIX_FORMS_H



#include "matrix_base.h"



/**
 * 3x3 Matrix class with special functions for rotations etc.
 */
template<typename TYPE>
class Matrix33: public Matrix<TYPE, 3, 3> {
public:



};



/**
 * 3x1 Matrix class with special functions that are similar to Vector calculations
 * Recommended to used specifically Vector class for higher perfomance.
 */
template<typename TYPE>
class Matrix31: public Matrix<TYPE, 3, 1> {
public:



};



/**
 * 4x4 Matrix class with special functions for transforming coordinate systems.
 */
template<typename TYPE>
class Matrix44: public Matrix<TYPE, 4, 4> {
public:



};



#endif