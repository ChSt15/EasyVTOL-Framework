#ifndef QUATERNION_H
#define QUATERNION_H


#include "math.h"

#ifdef Arduino_h
#include "WString.h"
#endif


#include "../vector_math.h"



#ifndef PI
#define PI 3.1415926541f
#endif

#ifndef DEGREES
#define DEGREES 180.0f/PI
#endif


//No its the Fast Math Library namespace. Nothing else
namespace FML {


template <typename T = float>
class Quaternion {
    public:
    
        T w;
        T x;
        T y;
        T z;
        
        Quaternion<T>() {
            w = 0.0f;
            x = 0.0f;
            y = 0.0f;
            z = 0.0f;
        }

        Quaternion<T>(const T &value) {
            w = value;
            x = value;
            y = value;
            z = value;
        }

        /**
         * Creates a Quaternion<T> where the rotational axis
         * is around the given VectorOLD<T> (Does not need to be a unit vector)
         * and the rotation of "angle" [Rad].
         *
         * @param values axis[Vector] and angle[Rad]
         * @return none.
         */
        Quaternion<T>(const VectorOLD<T>& axis, const T& angle) {

            if (axis.isZeroVector()) {

                *this = Quaternion<T>(1,0,0,0);

            } else {

                VectorOLD<T> axisNorm = axis.copy().normalize();

                T sa = sinf(angle/2);
                
                w = cosf(angle/2);
                x = axisNorm.x*sa;
                y = axisNorm.y*sa;
                z = axisNorm.z*sa;

            }

        }


        Quaternion<T>(const VectorOLD<T> &vector) {
    
            w = 0;
            x = vector.x;
            y = vector.y;
            z = vector.z;

        }
        

        inline VectorOLD<T> toVector() const {
            return VectorOLD<T>(x,y,z);
        }

        
        Quaternion<T>(const T &nw, const T &nx, const T &ny, const T &nz) {
            this->w = nw;
            this->x = nx;
            this->y = ny;
            this->z = nz;
        }

        /**
         * Simpler version of Quaternion<T>(T nw, T nx, T ny, T nz)
         * but w is set to 0.
         *
         * @param values x, y and z
         * @return none.
         */

        Quaternion<T>(const T &nx, const T &ny, const T &nz) {
            this->w = 0.0f;
            this->x = nx;
            this->y = ny;
            this->z = nz;
        }

        /**
         * Conjugates the Quaternion.
         * Will change values of quaternion.
         * USe .copy() to keep values.
         * 
         * @param values none
         * @return copy of conjugated Quaternion.
         */
        inline Quaternion<T>& conjugate() {
            x = -x;
            y = -y;
            z = -z;
            return *this;
        }

        /**
         * Computes the Magnitude (Length) of
         * the Quaternion
         *
         * @param values none
         * @return Magnitude.
         */
        inline T getMagnitude() const {

            return sqrt(w*w + x*x + y*y + z*z);

        }

        /**
         * Computes the axis[Unit Vector] of rotation and angle[Rad].
         *
         * @param values pointer to axis VectorOLD<T> and angle T
         * @return none.
         */
        inline void getAxisAngle(VectorOLD<T> &axis, float &angle) const {

            axis.x = this->x;
            axis.y = this->y;
            axis.z = this->z;

            axis.normalize();

            angle = acosf(w)*2;

        }

        /**
         * Normalises the Quaternion<T> (Gives length of 1).
         * If sign is true then w will be positive
         * (Makes calculations sometimes more reliable)
         *
         * @param values sign
         * @return normalized Quaternion.
         */
        inline Quaternion& normalize(const bool &sign = false) {

            T m = getMagnitude();

            if (true) {

                if (sign && w < 0.0f) m = -m;
                
                w /= m;
                x /= m;
                y /= m;
                z /= m;

            }

            if (w==w && x==x && y==y && z==z) return *this; //Check for NAN event

            //valid = false;

            return *this;

        }
        
        /**
         * Returns a copy of the Quaternion
         *
         * @param values none
         * @return copy of Quaternion.
         */
        inline Quaternion<T> copy() const {
            return Quaternion<T>(w,x,y,z);
        }

        /**
         * Returns a copy of the Quaternion
         *
         * @param values none
         * @return copy of Quaternion.
         */
        inline VectorOLD<T> rotateVector(const VectorOLD<T> &vectorToRotate) const {
            return (*this*vectorToRotate*(copy().conjugate())).toVector();
        }

        #ifdef Arduino_h
        /**
         * Returns a String containing components.
         * Form:
         * w: ..., x: ..., y: ..., z:...
         * Where ... is the value.
         * Default digits is 2.
         * 
         *
         * @param values digits.
         * @return String.
         */
        inline String toString(const uint8_t &digits = 2) const {
            return "w:" + String(w, digits) + " x:" + String(x, digits) + " y:" + String(y, digits) + " z:" + String(z, digits);  
        }
        #endif

        inline Quaternion<T> operator + (const Quaternion<T> &b) const {
            return Quaternion<T>(w + b.w, x + b.x, y + b.y, z + b.z);
        }

        inline Quaternion<T> operator - (const Quaternion<T> &b) const {
            return Quaternion<T>(w - b.w, x - b.x, y - b.y, z - b.z);
        }

        inline Quaternion<T> operator - (void) const {
            return Quaternion<T>(-w, -x, -y, -z);
        }

        inline Quaternion<T> operator * (const T &c) const {
            return Quaternion<T>(w*c, x*c, y*c, z*c);
        }

        inline Quaternion<T> operator / (const T &c) const {
            return Quaternion<T>(w/c, x/c, y/c, z/c);
        }

        inline void operator *= (const float &c) {
            w *= c;
            x *= c;
            y *= c;
            z *= c;
        }

        inline void operator += (const Quaternion<T> &b) {
            w += b.w;
            x += b.x;
            y += b.y;
            z += b.z;
        }

        inline Quaternion<T> operator * (const Quaternion<T> &q) const {
            return Quaternion<T>(
                w*q.w - x*q.x - y*q.y - z*q.z,  // new w
                w*q.x + x*q.w + y*q.z - z*q.y,  // new x
                w*q.y - x*q.z + y*q.w + z*q.x,  // new y
                w*q.z + x*q.y - y*q.x + z*q.w); // new z
        }

        inline Quaternion<T> operator ^ (const Quaternion<T> &q) const {
            return Quaternion<T>(
                w*q.w - x*q.x - y*q.y - z*q.z,  // new w
                w*q.x + x*q.w + y*q.z - z*q.y,  // new x
                w*q.y - x*q.z + y*q.w + z*q.x,  // new y
                w*q.z + x*q.y - y*q.x + z*q.w); // new z
        }

};

template <typename T = float>
inline extern Quaternion<T> sqrt(const Quaternion<T> &a);



} //Namespace end


#endif