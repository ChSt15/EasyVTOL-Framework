#ifndef MATRIX_INTERFACE_H
#define MATRIX_INTERFACE_H



#include "stdint.h"


//No its the Fast Math Library namespace. Nothing else
namespace FML {


/**
 * Matrix giving the interface needed for matrix pointers that can be usefull.
 */
template<typename TYPE>
class Matrix_Interface {
public:

    ///Needed as this is expected to be inhereted from.
    virtual ~Matrix_Interface() {}

    /**
     * @returns total number of values the matrix has.
     */
    virtual uint32_t getNumValues() const = 0;

    /**
     * @returns number of rows a matrix has.
     */
    virtual uint16_t getNumRows() const = 0;

    /**
     * @returns number of columns a matrix has.
     */
    virtual uint16_t getNumColumns() const = 0;

    /**
     * Provides direct access to array rows. No protection against going out of bounds.
     * Use as: matrix[row][column] for value or matrix[row] for a pointer to the entire row.
     * @param row Which row to return.
     * @returns pointer to row.
     */
    virtual TYPE* operator [] (uint16_t row) = 0;

    /**
     * Accesses the matrix like it were a 1D array with the rows place behind each other.
     * @param index What value to return. Goes down the row untill the end where it enters beginning of next row.
     * @returns reference to array value.
     */
    virtual TYPE& operator () (uint32_t index) = 0;

    /**
     * Accesses the matrix. But should protect against going out of bounds, with the downside of additional overhead.
     * Use as: matrix(row, column) for value access.
     * @param row Which row to return.
     * @param column Which column to return.
     * @returns reference to array value.
     */
    virtual TYPE& operator () (uint16_t row, uint16_t column) = 0;

    /**
     * Provides direct access to array rows. No protection against going out of bounds.
     * Use as: matrix[row][column] for value or matrix[row] for a pointer to the entire row.
     * @param row Which row to return.
     * @returns pointer to row.
     */
    virtual const TYPE* operator [] (uint16_t row) const = 0;

    /**
     * Accesses the matrix like it were a 1D array with the rows place behind each other.
     * @param index What value to return. Goes down the row untill the end where it enters beginning of next row.
     * @returns reference to array value.
     */
    virtual const TYPE& operator () (uint32_t index) const = 0;

    /**
     * Accesses the matrix. But should protect against going out of bounds, with the downside of additional overhead.
     * Use as: matrix(row, column) for value access.
     * @param row Which row to return.
     * @param column Which column to return.
     * @returns reference to array value.
     */
    virtual const TYPE& operator () (uint16_t row, uint16_t column) const = 0;

};


} //Namespace end


#endif