#ifndef _VECTOR_MATH_H_
#define _VECTOR_MATH_H_


#include "math.h"


#ifdef Arduino_h
#include "WString.h"
#endif


#ifndef PI
#define PI 3.1415926541
#endif

#ifndef DEGREES
#define DEGREES (PI/180.0)
#endif


template <typename T = float>
class VectorOLD {
    public:
    
        T x;
        T y;
        T z;

        VectorOLD<T>() {
            x = 0.0f;
            y = 0.0f;
            z = 0.0f;
        }

        /**
         * This constructor sets all components to n.
         */
        VectorOLD<T>(T n) {
            x = n;
            y = n;
            z = n;
        }

        VectorOLD<T>(T nx, T ny, T nz) {
            x = nx;
            y = ny;
            z = nz;
        }

        //VectorOLD<T>(const Quaternion &quaternion);

        /**
         * Calculates the magnitude of the vector.
         *
         * @return magnitude of the vector.
         */
        inline T magnitude() const {

            T m = x*x + y*y + z*z;

            m = sqrt(m);
            
            if (m == m) { // NAN check
                return m;
            }
            return 0.0f;
        }

        /**
         * Copies the vector.
         *
         * @return copy of the vector.
         */
        inline VectorOLD<T> copy() const {
            return VectorOLD<T>(x, y, z);
        }

        /**
         * Normalizes the Vector.
         *
         * @return copy of the normalized vector.
         */
        inline VectorOLD<T>& normalize() {

            T mag = magnitude();

            if (mag != 0.0f) {
                x /= mag;
                y /= mag;
                z /= mag;
            } else {
                x = 0.0f;
                y = 0.0f;
                z = 0.0f;
            }

            return *this;

        }

        /**
         * Checks if VectorOLD<T> is zero vector.
         *
         * @return true if vecter is zero vector.
         */
        inline bool isZeroVector() const {

            return magnitude() < 0.0001f;

        }

        /**
         * Multiplies the vectors components to this scheme:
         * VectorOLD<T>(x*x, y*y, z*z)
         *
         * @param vec VectorOLD<T> to multiply with
         * @return vector.
         */
        inline VectorOLD<T> compWiseMulti(const VectorOLD<T> &vec) const {
            return VectorOLD<T>(x*vec.x, y*vec.y, z*vec.z);
        }

        /**
         * Adds the components of the vectors.
         *
         * @param values none.
         * @return copy of the vector.
         */
        inline VectorOLD<T> operator + (const VectorOLD<T> &b) const {
            return VectorOLD<T>(x + b.x, y + b.y, z + b.z);
        }

        /**
         * Subtracts the components of the vectors.
         *
         * @param values none.
         * @return copy of the vector.
         */
        inline VectorOLD<T> operator - (const VectorOLD<T> &b) const {
            return VectorOLD<T>(x - b.x, y - b.y, z - b.z);
        }

        /**
         * Negates the components of the vectors.
         *
         * @param values none.
         * @return copy of the vector.
         */
        inline VectorOLD<T> operator - (void) const {
            return VectorOLD<T>(-x, -y, -z);
        }

        /**
         * Scales the vector.
         *
         * @param c Value to scale with.
         * @return copy of scaled vector.
         */
        inline VectorOLD<T> operator * (const float &c) const {
            return VectorOLD<T>(x*c, y*c, z*c);
        }

        /**
         * Divides the components of the vectors.
         *
         * @param values none.
         * @return copy of the vector.
         */
        inline VectorOLD<T> operator / (const float &c) const {
            return VectorOLD<T>(x/c, y/c, z/c);
        }

        /**
         * Compound scaling.
         *
         * @param values none.
         * @return nothing.
         */
        inline void operator *= (const float &c) {
            x *= c;
            y *= c;
            z *= c;
        }

        /**
         * Compound addition.
         *
         * @param values none.
         * @return nothing.
         */
        inline void operator += (const VectorOLD<T> &b) {
            x += b.x;
            y += b.y;
            z += b.z;
        }

        /**
         * Compound addition.
         *
         * @param values none.
         * @return nothing.
         */
        inline void operator -= (const VectorOLD<T> &b) {
            x -= b.x;
            y -= b.y;
            z -= b.z;
        }

        /**
         * VectorOLD<T> component wise multiplication.
         *
         * @param values none.
         * @return multiplication result in T.
         */
        inline VectorOLD<T> operator * (const VectorOLD<T> &b) const {
            return VectorOLD<T>(x*b.x, y*b.y, z*b.z);
        }

        /**
         * VectorOLD<T> component wise multiplication.
         *
         * @param values none.
         * @return multiplication result in T.
         */
        inline VectorOLD<T> operator / (const VectorOLD<T> &b) const {
            return VectorOLD<T>(x/b.x, y/b.y, z/b.z);
        }

        /**
         * VectorOLD<T> multiplication.
         * 
         * x1*x2 + y1*y2 + z1*z2
         *
         * @param values none.
         * @return multiplication result in T.
         */
        inline T operator ^ (const VectorOLD<T> &b) const {
            return x*b.x + y*b.y + z*b.z;
        }

        /**
         * VectorOLD<T> equals.
         * True is all components are equal
         *
         * @param values none.
         * @return bool.
         */
        inline bool operator == (const VectorOLD<T> &b) const {
            return x==b.x && y==b.y && z==b.z;
        }

        /**
         * VectorOLD<T> not equals.
         * true if any component not equal
         *
         * @param values none.
         * @return bool.
         */
        inline bool operator != (const VectorOLD<T> &b) const {
            return !(x==b.x && y==b.y && z==b.z);
        }

        /**
         * @return bool.
         */
        inline bool operator < (const VectorOLD<T> &b) const {
            return this->magnitude() < b.magnitude();
        }

        /**
         * @return bool.
         */
        inline bool operator > (const VectorOLD<T> &b) const {
            return this->magnitude() > b.magnitude();
        }

        /**
         * Squares the components the the vector
         * @return Vector.
         */
        inline VectorOLD<T> square() const {
            return VectorOLD<T>(
                x*x,
                y*y,
                z*z
            );
        } 

        /**
         * Computes cross multiplication of 2 vectors.
         * 
         *
         * @param values none.
         * @return Vector.
         */
        inline VectorOLD<T> cross(const VectorOLD<T> &b) const {
            return VectorOLD<T>(
                y*b.z-z*b.y,
                z*b.x-x*b.z,
                x*b.y-y*b.x
            );
        } 

        /**
         * Computes angle between 2 vectors.
         * 
         *
         * @param values none.
         * @return angle(T) in radians.
         */
        inline T getAngleTo(const VectorOLD<T> &b) const {
            
            T ca = (*this)^b/(magnitude()*b.magnitude());

            return acos(ca);

        }


        /**
         * Computes the projection onto another vector.
         * 
         *
         * @param values none.
         * @return Vector.
         */
        inline VectorOLD<T> getProjectionOn(VectorOLD<T> b) const {

            b.normalize();

            return b*(*this*b);

        }

        /**
         * Computes the projection onto a plane.
         * 
         * @param planeNormal Planes normal vector. Gives plane its angles.
         * @return Vector.
         */
        inline VectorOLD<T> getProjectionOnPlane(const VectorOLD<T> &planeNormal) const {

            VectorOLD<T> normalNorm = planeNormal.copy().normalize();

            return *this-(*this^planeNormal)*normalNorm;
        }

        #ifdef Arduino_h
        /**
         * Returns a String containing components.
         * Form:
         * x: ..., y: ..., z:...
         * Where ... is the value.
         * Default digits is 2.
         * 
         *
         * @param values digits.
         * @return String.
         */
        inline String toString(const uint8_t &digits = 2) const {
            return "x:" + String(x, digits) + " y:" + String(y, digits) + " z:" + String(z, digits);  
        }
        #endif


};


#define GRAVITY_MAGNITUDE 9.8066
//Is a VectorOLD<T> with a z component of -9.81.
#define GRAVITY_VECTOR VectorOLD<>(0, 0, -GRAVITY_MAGNITUDE)

template <typename T = float>
inline VectorOLD<T> sqrt(VectorOLD<T> a)  {
    return VectorOLD<T>(sqrt(a.x), sqrt(a.y), sqrt(a.z));
}

template <typename T = float>
inline VectorOLD<T> operator / (const float &a, const VectorOLD<T> &b)  {
    return VectorOLD<T>(a/b.x, a/b.y, a/b.z);
}


#endif