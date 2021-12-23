#ifndef MATRIX_VECTOR_H
#define MATRIX_VECTOR_H



#include "matrix_base.h"


//No its the Fast Math Library namespace. Nothing else
namespace FML {


/**
 * Nx1 Matrix class for vectors.
 */
template<typename TYPE, uint16_t ROWS>
class MVector: public Matrix<TYPE, ROWS, 1> {
public:

    MVector();

    MVector(const TYPE& value);

    /**
     * Copies given vector or Nx1 matrix and allows for copying a given part into a given place.
     * @param startRow What index to start copying from. Defaults to 0 (beginning).
     * @param numberIndex Number of values starting from startIndex to copy. Defaults to ROW or number of rows this vector has.
     * @param offset Where to start placing values into. Defaults to 0 (beginning).
     */
    template<typename TYPE2, uint16_t ROWS2>
    MVector(const Matrix<TYPE2, ROWS2, 1>& vectorMatrix, uint16_t startIndex = 0, uint16_t numberIndex = ROWS, uint16_t offset = 0);

    /**
     * Calculates the length of a vector. Also has the option of calculating from only a given section for example in state vectors.
     * @param startIndex Start of vector.
     * @param endIndex End of vector´.
     */
    TYPE magnitude(uint16_t startIndex = 0, uint16_t endIndex = ROWS-1);

    /**
     * Normalises the vector. Also has the option of normalising only a section for state vectors.
     * E.g. if a state vector is 6D (3 position, 3 velocity) and position must be normalised then the correct parameters would be normalise(0, 2).
     * @param startIndex Start of vector to normalise. 
     * @param endIndex End of vector to normalise.
     */
    MVector<TYPE, ROWS> normalise(uint16_t startIndex = 0, uint16_t endIndex = ROWS-1);


};


template<typename TYPE, uint16_t ROWS>
MVector<TYPE, ROWS>::MVector() {
    for (uint16_t i = 0; i < ROWS; i++) this->r[i][0] = 0.0f;
}


template<typename TYPE, uint16_t ROWS>
MVector<TYPE, ROWS>::MVector(const TYPE& value) {
    for (uint16_t i = 0; i < ROWS; i++) this->r[i][0] = value;
}


template<typename TYPE, uint16_t ROWS>
template<typename TYPE2, uint16_t ROWS2>
MVector<TYPE, ROWS>::MVector(const Matrix<TYPE2, ROWS2, 1>& vectorMatrix, uint16_t startIndex, uint16_t numberIndex, uint16_t offset) {
    for (uint16_t i = offset; i < ROWS && i-offset < numberIndex; i++) this->r[i][0] = vectorMatrix[startIndex + i][0];
}


template<typename TYPE, uint16_t ROWS>
TYPE MVector<TYPE, ROWS>::magnitude(uint16_t startIndex, uint16_t endIndex) {

    TYPE temp = 0;

    for (uint16_t i = startIndex; i <= endIndex; i++) {

        temp = this->r[i][0]*this->r[i][0];

    }

    return sqrt(temp);

}


template<typename TYPE, uint16_t ROWS>
MVector<TYPE, ROWS> MVector<TYPE, ROWS>::normalise(uint16_t startIndex, uint16_t endIndex) {

    TYPE mag = magnitude(startIndex, endIndex);

    MVector<TYPE, ROWS> normalisedVector;

    for (uint16_t i = startIndex; i <= endIndex; i++) {

        this->r[i][0] = this->r[i][0]/mag;

    }

}



using Vector3_F = MVector<float, 3>; //For float vector types

using Vector3_D = MVector<double, 3>; //For double vector types



} //Namespace end



#endif