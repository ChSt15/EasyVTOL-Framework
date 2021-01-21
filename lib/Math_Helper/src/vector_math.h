#ifndef _VECTOR_MATH_H_
#define _VECTOR_MATH_H_


//#include <Arduino.h>


#ifndef PI
#define PI 3.1415926541f
#endif

#ifndef DEGREES
#define DEGREES 180.0f/PI
#endif


class Vector {
    public:
    
        float x;
        float y;
        float z;

        bool valid = true;


        Vector() {
            x = 0.0f;
            y = 0.0f;
            z = 0.0f;
        }
        

        Vector(float nx, float ny = 0, float nz = 0) {
            x = nx;
            y = ny;
            z = nz;
        }

        /**
         * Calculates the magnitude of the vector.
         *
         * @param values none.
         * @return magnitude of the vector.
         */
        float magnitude() {

            float m = x*x + y*y + z*z;

            m = sqrtf(m);
            
            if (m == m) { // NAN check
                return m;
            }
            valid = false;
            return 0.0f;
        }

        /**
         * Copies the vector.
         *
         * @param values none.
         * @return copy of the vector.
         */
        Vector copy() {
            return Vector(x, y, z);
        }

        /**
         * Normalizes the Vector.
         *
         * @param values none.
         * @return copy of the normalized vector.
         */
        Vector normalize() {

            float mag = magnitude();

            x /= mag;
            y /= mag;
            z /= mag;

            return *this;

        }

        /**
         * Adds the components of the vectors.
         *
         * @param values none.
         * @return copy of the vector.
         */
        Vector operator + (Vector b) {
            return Vector(x + b.x, y + b.y, z + b.z);
        }

        /**
         * Subtracts the components of the vectors.
         *
         * @param values none.
         * @return copy of the vector.
         */
        Vector operator - (Vector b) {
            return Vector(x - b.x, y - b.y, z - b.z);
        }

        /**
         * Negates the components of the vectors.
         *
         * @param values none.
         * @return copy of the vector.
         */
        Vector operator - (void) {
            return Vector(-x, -y, -z);
        }

        /**
         * Scales the vector.
         *
         * @param values none.
         * @return copy of the vector.
         */
        Vector operator * (float c) {
            return Vector(x*c, y*c, z*c);
        }

        /**
         * Divides the components of the vectors.
         *
         * @param values none.
         * @return copy of the vector.
         */
        Vector operator / (float c) {
            return Vector(x/c, y/c, z/c);
        }

        /**
         * Compound scaling.
         *
         * @param values none.
         * @return nothing.
         */
        void operator *= (float c) {
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
        void operator += (Vector b) {
            x += b.x;
            y += b.y;
            z += b.z;
        }

        /**
         * Vector multiplication.
         * 
         * x1*x2 + y1*y2 + z1*z2
         *
         * @param values none.
         * @return multiplication result in float.
         */
        float operator * (Vector b) {
            return x*b.x + y*b.y + z*b.z;
        }

        /**
         * Vector equals.
         * True is all components are equal
         *
         * @param values none.
         * @return bool.
         */
        bool operator == (Vector b) {
            return x==b.x && y==b.y && z==b.z;
        }

        /**
         * Vector not equals.
         * true if any component not equal
         *
         * @param values none.
         * @return bool.
         */
        bool operator != (Vector b) {
            return !(x==b.x && y==b.y && z==b.z);
        }

        /**
         * Computes cross multiplication of 2 vectors.
         * 
         *
         * @param values none.
         * @return Vector.
         */
        Vector cross(Vector b) {
            return Vector(
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
         * @return angle(float) in radians.
         */
        float getAngleTo(Vector b) {
            
            float ca = *this*b/this->magnitude()/b.magnitude();

            return acos(ca);

        }


        /**
         * Computes the projection onto another vector.
         * 
         *
         * @param values none.
         * @return Vector.
         */
        Vector getProjectionOn(Vector b) {

            b.normalize();

            return b*(*this*b);


        }


};


#endif /* _HELPER_3DMATH_H_ */