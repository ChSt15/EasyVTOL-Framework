#ifndef MATRIX_FLOAT_H
#define MATRIX_FLOAT_H



#include "stdint.h"

#include "matrix_base.h"
#include "matrix_forms.h"


/**
 * Creates a matrix with given size and floats as internal type.
 */
template<uint16_t ROWS, uint16_t COLS>
using Matrix_F = Matrix<float, ROWS, COLS>;

/**
 * Creates a matrix with given size and doubles as internal type.
 */
template<uint16_t ROWS, uint16_t COLS>
using Matrix_D = Matrix<double, ROWS, COLS>;



#endif