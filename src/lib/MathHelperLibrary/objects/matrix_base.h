#ifndef MATRIX_BASE_H
#define MATRIX_BASE_H



#include "stdint.h"
#include "math.h"


//No its the Fast Math Library namespace. Nothing else
namespace FML {


/**
 * General Matrix class used for matrix calculations. Can be any type (float, double, int etc.) and any size.
 * Recommended to used the from this derived forms for 3D, 4D etc matrices and specifically Vector and Quaternion for higher performance and more features.
 */
template<typename TYPE, uint16_t ROWS, uint16_t COLS>
class Matrix {
public:

    /**
     * These are the raw internal values.
     * Access via matrix(row, column) is safer but adds slight overhead.
     * Can be accessed alternatively with matrix[row][column] with (presumably) 0 overhead, but no out of bounds checking.
     */
    TYPE r[ROWS][COLS];


    /**
     * Below are constructors.
     */ 

    /**
     * Standard constructor that filles all values with 0.
     */
    Matrix();

    /**
     * Contructor that creates a matrix with the diagonal filled with given values and the rest are 0. 
     * Similar to identity matrix multplied by the given value. 
     * If the entire array should be filled then use the static function Matrix::fill(value);
     * @param value What value to fill diagonal with. 
     */
    Matrix(TYPE value);

    /**
     * An array can be given as so: 
     * Matrix a({
     *  2,2,
     *  2,2,
     *  2,2
     * })
     * And the matrix will be filled with the given values.
     * Warning: array must have correct size or undetermined values will be written into matrix.
     * @param array Array containing values.
     */
    //Matrix(const TYPE (&array)[]);

    /**
     * An array can be given as so: 
     * float array[] = {
     *  2,2,
     *  2,2,
     *  2,2
     * };
     * Matrix a(array);
     * And the matrix will be filled with the given values.
     * Warning: array must have correct size or undetermined values will be written into matrix.
     * @param array Array containing values.
     */
    Matrix(const TYPE* array);


    /**
     * Conversion constructor. Converts given matrix into type that this matrix is (e.g float -> int)
     * @param matrix Which matrix to copy.
     */
    template<typename TYPE2>
    Matrix(const Matrix<TYPE2, ROWS, COLS>& matrix);


    /**
     * Conversion constructor. Copies given matrix but only a part of it or all of it depending on size.
     * @param matrix Which matrix to copy.
     * @param startingRow At which row to start copying from/to. Defaults to 0.
     * @param startingColumn At which column to start copying from/to. Defaults to 0.
     * @param mode If true then the starting row/column is the matrix getting written to (this). Defaults to false.
     */
    template<typename TYPE2, uint16_t ROWS2, uint16_t COLS2>
    Matrix(const Matrix<TYPE2, ROWS2, COLS2>& matrix, uint16_t startingRow, uint16_t startingColumn = 0, bool mode = false);


    /**
     * Below are generators for often needed matrices.
     */

    /**
     * Generates a matrix where all diagonal values are set to given value and the rest are set to 0. 
     * @param eyeVal Value the diagonal values should have.
     * @returns generated matrix.
     */
    static Matrix<TYPE, ROWS, COLS> eye(TYPE eyeVal = 1);

    /**
     * Generates a matrix that is filled with the given value.
     * @param fillVal Value to fill array with.
     * @returns generated matrix.
     */
    static Matrix<TYPE, ROWS, COLS> fill(TYPE fillVal);

    /**
     * Below are generial functions to get the size of array or copy it.
     */

    /**
     * @returns total number of values the matrix has.
     */
    uint16_t getNumRows() const;

    /**
     * @returns number of rows a matrix has.
     */
    uint16_t getNumColumns() const;

    /**
     * @returns number of columns a matrix has.
     */
    uint32_t getNumValues() const;

    /**
     * @returns a copy of the matrix.
     */
    Matrix<TYPE, ROWS, COLS> copy() const;

    /**
     * Special functions to calculate transpose, determinant, inverse etc.
     */

    /**
     * @returns transposed matrix.
     */
    Matrix<TYPE, COLS, ROWS> transpose();

    /**
     * @returns matrix determinant.
     */
    TYPE determinant();

    /**
     * Used Gauß-Jordan method from https://www.geeksforgeeks.org/finding-inverse-of-a-matrix-using-gauss-jordan-method/
     * @returns inverse of matrix.
     */
    Matrix<TYPE, ROWS, COLS> inverse();

    /**
     * Below are access operators to access matrix values.
     */

    /**
     * Another way to access the internal data.
     * Low overhead and direct access to internal data, but no protection against going out of bounds.
     */
    TYPE* operator [] (uint16_t row);

    /**
     * Another way to access the internal data. Return value is const.
     * Low overhead and direct access to internal data, but no protection against going out of bounds.
     */
    const TYPE* operator [] (uint16_t row) const;

    /**
     * This operator accesses the first row until the end then goes to the beginning of the next row.
     */
    TYPE& operator () (uint32_t index);

    /**
     * Return value is constant.
     * This operator accesses the first row until the end then goes to the beginning of the next row.
     */
    const TYPE& operator () (uint32_t index) const;
    
    /**
     * This operator is to access the matrix values while keeping you inside the bounds of data.
     */
    TYPE& operator () (uint16_t row, uint16_t column);

    /**
     * Return value is constant.
     * This operator is to access the matrix values while keeping you inside the bounds of data.
     */
    const TYPE& operator () (uint16_t row, uint16_t column) const;

    /**
     * Below are math operations.
     */

    /**
     * Adds 2 matrices together.
     */ 
    Matrix<TYPE, ROWS, COLS> operator + (const Matrix<TYPE, ROWS, COLS>& right);

    /**
     * Subtracts right matrix from left.
     */
    Matrix<TYPE, ROWS, COLS> operator - (const Matrix<TYPE, ROWS, COLS>& right);

    /**
     * Generates an eye matrix from given value on left und then subtracts right matrix from generated value.
     */
    //friend Matrix<TYPE, ROWS, COLS> operator - (const TYPE& val, const Matrix<TYPE, ROWS, COLS>& right);

    /**
     * Multiplies a scaler with the matrix.
     */
    Matrix<TYPE, ROWS, COLS> operator * (const TYPE& scaler);

    /**
     * Divides a scaler with the matrix.
     */
    Matrix<TYPE, ROWS, COLS> operator / (const TYPE& scaler);

    /**
     * Multiplies a scaler with the matrix just like matrix*float but allows to be swaped to float*matrix.
     */
    //friend Matrix<TYPE, ROWS, COLS> operator * (const TYPE& scaler, const Matrix<TYPE, ROWS, COLS>& right);

    /**
     * Does matrix multiplication with given matrices. Recommended to use auto so save from having to determine matrix size und writing out.
     * E.g: auto C = A*B;
     * This is as normal matrix multiplication not commutative meaning: A*B will result in something different than B*A.
     */
    template<uint16_t RIGHTCOLS>
    Matrix<TYPE, ROWS, RIGHTCOLS> operator * (const Matrix<TYPE, COLS, RIGHTCOLS>& right);

    /**
     * Copies value from one matrix into the other even with differing internal value types.
     */ 
    template<typename TYPE2>
    Matrix<TYPE, ROWS, COLS> operator = (const Matrix<TYPE2, ROWS, COLS>& right);
    

};




/**
 * Below are the implementations of all above functions.
 */



template<typename TYPE, uint16_t ROWS, uint16_t COLS>
Matrix<TYPE, ROWS, COLS>::Matrix() {

    static_assert((ROWS > 0 && COLS > 0), "Matrix must be at least 1X1. Number of Rows or Columns or both are 0.");

    for (uint16_t row = 0; row < ROWS; row++) {
        for (uint16_t col = 0; col < COLS; col++) {

            r[row][col] = 0;

        }
    }  

}


template<typename TYPE, uint16_t ROWS, uint16_t COLS>
Matrix<TYPE, ROWS, COLS>::Matrix(TYPE val) {

    static_assert((ROWS > 0 && COLS > 0), "Matrix must be at least 1X1. Number of Rows or Columns or both are 0.");

    for (uint16_t row = 0; row < ROWS; row++) {
        for (uint16_t col = 0; col < COLS; col++) {

            if (row == col) r[row][col] = val;
            else r[row][col] = 0;

        }
    }  

}


/*template<typename TYPE, uint16_t ROWS, uint16_t COLS>
Matrix<TYPE, ROWS, COLS>::Matrix(const TYPE (&array)[]) {

    static_assert((ROWS > 0 && COLS > 0), "Matrix must be at least 1X1. Number of Rows or Columns or both are 0.");

    for (uint16_t i = 0; i < getNumValues(); i++) (*this)(i) = array[i];

}*/


template<typename TYPE, uint16_t ROWS, uint16_t COLS>
Matrix<TYPE, ROWS, COLS>::Matrix(const TYPE* array) {

    static_assert((ROWS > 0 && COLS > 0), "Matrix must be at least 1X1. Number of Rows or Columns or both are 0.");

    for (uint16_t i = 0; i < getNumValues(); i++) (*this)(i) = array[i];

}


template<typename TYPE, uint16_t ROWS, uint16_t COLS>
template<typename TYPE2>
Matrix<TYPE, ROWS, COLS>::Matrix(const Matrix<TYPE2, ROWS, COLS>& matrix) {

    for (uint16_t row = 0; row < ROWS; row++) {
        for (uint16_t col = 0; col < COLS; col++) {

            r[row][col] = matrix.r[row][col];

        }
    } 

}


template<typename TYPE, uint16_t ROWS, uint16_t COLS>
template<typename TYPE2, uint16_t ROWS2, uint16_t COLS2>
Matrix<TYPE, ROWS, COLS>::Matrix(const Matrix<TYPE2, ROWS2, COLS2>& matrix, uint16_t startingRow, uint16_t startingColumn, bool mode) {


    if (mode) {

        for (uint16_t row = 0; row < startingRow + ROWS; row++) {
            for (uint16_t col = 0; col < startingColumn + COLS; col++) {
                
                if (row > startingRow && col > startingColumn) (*this)(row, col) = matrix(row + startingRow, col + startingColumn);
                else (*this)(row, col) = 0;

            }
        } 

    } else {

        for (uint16_t row = 0; row < ROWS; row++) {
            for (uint16_t col = 0; col < COLS; col++) {

                if (row + startingRow > ROWS2-1 || col + startingColumn > COLS2-1) (*this)(row, col) = 0;
                else (*this)(row, col) = matrix(row + startingRow, col + startingColumn);

            }
        } 

    }

}


template<typename TYPE, uint16_t ROWS, uint16_t COLS>
Matrix<TYPE, ROWS, COLS> Matrix<TYPE, ROWS, COLS>::eye(TYPE eyeVal) {

    Matrix<TYPE, ROWS, COLS> m;

    for (uint16_t i = 0; i < ROWS && i < COLS; i++) m.r[i][i] = eyeVal;

    return m;

}


template<typename TYPE, uint16_t ROWS, uint16_t COLS>
Matrix<TYPE, ROWS, COLS> Matrix<TYPE, ROWS, COLS>::fill(TYPE fillVal) {

    Matrix<TYPE, ROWS, COLS> m;

    for (uint16_t row = 0; row < ROWS; row++) {
        for (uint16_t col = 0; col < COLS; col++) {

            m.r[row][col] = fillVal;

        }
    }

    return m;

}


template<typename TYPE, uint16_t ROWS, uint16_t COLS>
uint16_t Matrix<TYPE, ROWS, COLS>::getNumRows() const {
    return ROWS;
}


template<typename TYPE, uint16_t ROWS, uint16_t COLS>
uint16_t Matrix<TYPE, ROWS, COLS>::getNumColumns() const {
    return COLS;
}


template<typename TYPE, uint16_t ROWS, uint16_t COLS>
uint32_t Matrix<TYPE, ROWS, COLS>::getNumValues() const {
    return (uint32_t)ROWS*COLS;
}


template<typename TYPE, uint16_t ROWS, uint16_t COLS>
Matrix<TYPE, ROWS, COLS> Matrix<TYPE, ROWS, COLS>::copy() const {
    return *this;
}


template<typename TYPE, uint16_t ROWS, uint16_t COLS>
Matrix<TYPE, COLS, ROWS> Matrix<TYPE, ROWS, COLS>::transpose() {

    Matrix<TYPE, COLS, ROWS> m;

    for (uint16_t row = 0; row < ROWS; row++) {
        for (uint16_t col = 0; col < COLS; col++) {

            m.r[col][row] = r[row][col];

        }
    }   

    return m;

}


template<typename TYPE, uint16_t ROWS, uint16_t COLS>
Matrix<TYPE, ROWS, COLS> Matrix<TYPE, ROWS, COLS>::inverse() {

    static_assert((ROWS == COLS), "Matrix must be square (NxN) in order to get inverse. Pseudoinverse might solve the issue.");

    TYPE temp;

    Matrix<TYPE, ROWS, COLS*2> matrix(*this);

    // Create the augmented matrix
    // Add the identity matrix
    // of order at the end of original matrix.
    for (int i = 0; i < ROWS; i++) {
 
        for (int j = 0; j < 2 * ROWS; j++) {
 
            // Add '1' at the diagonal places of
            // the matrix to create a identity matrix
            if (j == (i + ROWS))
                matrix[i][j] = 1;
        }
    }
 
    // Interchange the row of matrix,
    // interchanging of row will start from the last row
    for (int i = ROWS - 1; i > 0; i--) {
 
        // Swapping each and every element of the two rows
        if (matrix[i - 1][0] < matrix[i][0]) {
            for (int j = 0; j < 2 * ROWS; j++) {
                temp = matrix[i][j];
                matrix[i][j] = matrix[i - 1][j];
                matrix[i - 1][j] = temp;
            }
        }
 
        // Directly swapping the rows using pointers saves
        // time
 
        /*if (matrix[i - 1][0] < matrix[i][0]) {
            TYPE* temp = matrix[i];
            matrix[i] = matrix[i - 1];
            matrix[i - 1] = temp;
        }*/
    }
 
    // Replace a row by sum of itself and a
    // constant multiple of another row of the matrix
    for (int i = 0; i < ROWS; i++) {
 
        for (int j = 0; j < ROWS; j++) {
 
            if (j != i) {
 
                temp = matrix[j][i] / matrix[i][i];
                for (int k = 0; k < 2 * ROWS; k++) {
 
                    matrix[j][k] -= matrix[i][k] * temp;
                }
            }
        }
    }
 
    // Multiply each row by a nonzero integer.
    // Divide row element by the diagonal element
    for (int i = 0; i < ROWS; i++) {
 
        temp = matrix[i][i];
        for (int j = 0; j < 2 * ROWS; j++) {
 
            matrix[i][j] = matrix[i][j] / temp;
        }
    }

    return Matrix<TYPE, ROWS, COLS>(matrix, 0, COLS);

}


/*template<>
float Matrix<float, 1, 1>::determinant() {
    return r[0][0];
}


template<>
double Matrix<double, 1, 1>::determinant() {
    return r[0][0];
}


template<>
int32_t Matrix<int32_t, 1, 1>::determinant() {
    return r[0][0];
}*/


template<typename TYPE, uint16_t ROWS, uint16_t COLS>
TYPE Matrix<TYPE, ROWS, COLS>::determinant() {

    static_assert((COLS == ROWS), "Matrix must be square (NxN) to have a determinant");

    if (COLS == 1 && ROWS == 1) return r[0][0];

    TYPE determinant = 0;
    Matrix<TYPE, ROWS - 1, COLS - 1> temp;

    for (unsigned int j1 = 0; j1 < ROWS; ++j1) {
        for (unsigned int i = 1; i < ROWS; ++i) {
            unsigned int j2 = 0;
            for (unsigned int j = 0; j < ROWS; ++j) {
                if (j == j1)
                    continue;
                temp.r[i - 1][j2] = r[i][j];
                j2++;
            }
        }
        determinant += pow(-1.0, j1 + 2.0) * r[0][j1] * temp.determinant();
    }

    return determinant;

}


template<typename TYPE, uint16_t ROWS, uint16_t COLS>
TYPE* Matrix<TYPE, ROWS, COLS>::operator [] (uint16_t row) {
    return r[row];
}


template<typename TYPE, uint16_t ROWS, uint16_t COLS>
const TYPE* Matrix<TYPE, ROWS, COLS>::operator [] (uint16_t row) const {
    return r[row];
}


template<typename TYPE, uint16_t ROWS, uint16_t COLS>
TYPE& Matrix<TYPE, ROWS, COLS>::operator () (uint32_t index) {
    return r[(index/COLS)%ROWS][index%COLS];
}


template<typename TYPE, uint16_t ROWS, uint16_t COLS>
const TYPE& Matrix<TYPE, ROWS, COLS>::operator () (uint32_t index) const {
    return r[(index/COLS)%ROWS][index%COLS];
}


template<typename TYPE, uint16_t ROWS, uint16_t COLS>
TYPE& Matrix<TYPE, ROWS, COLS>::operator () (uint16_t row, uint16_t column) {
    return r[row%ROWS][column%COLS];
} 


template<typename TYPE, uint16_t ROWS, uint16_t COLS>
const TYPE& Matrix<TYPE, ROWS, COLS>::operator () (uint16_t row, uint16_t column) const {
    return r[row%ROWS][column%COLS];
} 


template<typename TYPE, uint16_t ROWS, uint16_t COLS>
Matrix<TYPE, ROWS, COLS> Matrix<TYPE, ROWS, COLS>::operator + (const Matrix<TYPE, ROWS, COLS>& right) {

    Matrix<TYPE, ROWS, COLS> m;

    for (uint16_t row = 0; row < ROWS; row++) {
        for (uint16_t col = 0; col < COLS; col++) {

            m.r[row][col] =  r[row][col] + right.r[row][col];

        }
    }  

    return m;

}


template<typename TYPE, uint16_t ROWS, uint16_t COLS>
Matrix<TYPE, ROWS, COLS> Matrix<TYPE, ROWS, COLS>::operator - (const Matrix<TYPE, ROWS, COLS>& right) {

    Matrix<TYPE, ROWS, COLS> m;

    for (uint16_t row = 0; row < ROWS; row++) {
        for (uint16_t col = 0; col < COLS; col++) {

            m.r[row][col] = r[row][col] - right.r[row][col];

        }
    }  

    return m;

}


template<typename TYPE, uint16_t ROWS, uint16_t COLS>
Matrix<TYPE, ROWS, COLS> operator - (const TYPE& val, const Matrix<TYPE, ROWS, COLS>& right) {

    Matrix<TYPE, ROWS, COLS> m;

    for (uint16_t row = 0; row < ROWS; row++) {
        for (uint16_t col = 0; col < COLS; col++) {

            if (row == col) m.r[row][col] = val - right.r[row][col];
            else m.r[row][col] = -right.r[row][col];

        }
    }  

    return m;

}


template<typename TYPE, uint16_t ROWS, uint16_t COLS>
Matrix<TYPE, ROWS, COLS> Matrix<TYPE, ROWS, COLS>::operator * (const TYPE& scaler) {

    Matrix<TYPE, ROWS, COLS> m;

    for (uint16_t row = 0; row < ROWS; row++) {
        for (uint16_t col = 0; col < COLS; col++) {

            m.r[row][col] = this->r[row][col]*scaler;

        }
    }  

    return m;

}


template<typename TYPE, uint16_t ROWS, uint16_t COLS>
Matrix<TYPE, ROWS, COLS> Matrix<TYPE, ROWS, COLS>::operator / (const TYPE& scaler) {

    Matrix<TYPE, ROWS, COLS> m;

    for (uint16_t row = 0; row < ROWS; row++) {
        for (uint16_t col = 0; col < COLS; col++) {

            m.r[row][col] = this->r[row][col]/scaler;

        }
    }  

    return m;

}


template<typename TYPE, uint16_t ROWS, uint16_t COLS>
Matrix<TYPE, ROWS, COLS> operator * (const TYPE& scaler, const Matrix<TYPE, ROWS, COLS>& right) {

    Matrix<TYPE, ROWS, COLS> m;

    for (uint16_t row = 0; row < ROWS; row++) {
        for (uint16_t col = 0; col < COLS; col++) {

            m.r[row][col] =  scaler*right.r[row][col];

        }
    }  

    return m;

}


template<typename TYPE, uint16_t ROWS, uint16_t COLS>
template<uint16_t RIGHTCOLS>
Matrix<TYPE, ROWS, RIGHTCOLS> Matrix<TYPE, ROWS, COLS>::operator * (const Matrix<TYPE, COLS, RIGHTCOLS>& right) {

    Matrix<TYPE, ROWS, RIGHTCOLS> m;

    for (uint16_t lr = 0; lr < ROWS; lr++) {
        for (uint16_t rc = 0; rc < RIGHTCOLS; rc++) {

            TYPE val = 0;

            for (uint16_t i = 0; i < COLS; i++) {

                val += this->r[lr][i] * right.r[i][rc];

            }

            m.r[lr][rc] = val;

        }
    }  

    return m;

}


template<typename TYPE, uint16_t ROWS, uint16_t COLS>
template<typename TYPE2>
Matrix<TYPE, ROWS, COLS> Matrix<TYPE, ROWS, COLS>::operator = (const Matrix<TYPE2, ROWS, COLS>& right) {

    for (uint16_t r = 0; r < ROWS; r++) {
        for (uint16_t c = 0; c < COLS; c++) {

            this->r[r][c] = right.r[r][c];

        }
    } 

    return *this;

}



} //Namespace end


#endif