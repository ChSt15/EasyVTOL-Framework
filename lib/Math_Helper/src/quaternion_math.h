#ifndef _QUATERNION_MATH_H_
#define _QUATERNION_MATH_H_



#include "vector_math.h"


#ifndef PI
#define PI 3.1415926541f
#endif

#ifndef DEGREES
#define DEGREES 180.0f/PI
#endif




class Quaternion {
    public:
    
        float w;
        float x;
        float y;
        float z;

        bool valid = true;
        
        Quaternion() {
            w = 0.0f;
            x = 0.0f;
            y = 0.0f;
            z = 0.0f;
        }

        /**
         * Creates a Quaternion where the rotational axis
         * is around the given Vector (Does not need to be a unit vector)
         * and the rotation of "angle" [Rad].
         *
         * @param values axis[Vector] and angle[Rad]
         * @return none.
         */

        Quaternion(Vector axis, float angle) {

            axis.normalize();

            angle /= 2.0f;
            float sa = sinf(angle);
            
            w = cosf(angle);
            x = axis.x*sa;
            y = axis.y*sa;
            z = axis.z*sa;

        }
        
        Quaternion(float w, float x, float y, float z) {
            this->w = w;
            this->x = x;
            this->y = y;
            this->z = z;
        }

        /**
         * Simpler version of Quaternion(float nw, float nx, float ny, float nz)
         * but w is set to 0.
         *
         * @param values x, y and z
         * @return none.
         */

        Quaternion(float x, float y, float z) {
            this->w = 0.0f;
            this->x = x;
            this->y = y;
            this->z = z;
        }

        /**
         * Conjugates the Quaternion 
         *
         * @param values none
         * @return copy of conjugated Quaternion.
         */
        Quaternion conjugate() {
            x = -x;
            y = -y;
            y = -z;
            return *this;
        }

        /**
         * Computes the Magnitude (Length) of
         * the Quaternion
         *
         * @param values none
         * @return Magnitude.
         */
        float getMagnitude() {

            float m = w*w + x*x + y*y + z*z;

            m = sqrtf(m);
            
            if (m == m) { // Check for NAN event
                
            } else valid = false;

            return m;

        }

        /**
         * Computes the axis[Unit Vector] of rotation and angle[Rad].
         *
         * @param values pointer to axis Vector and angle float
         * @return none.
         */
        void getAxisAngle(Vector *axis, float *angle) {

            axis->x = this->x;
            axis->y = this->y;
            axis->z = this->z;

            axis->normalize();

            *angle = acos(w*2);

        }

        /**
         * Normalises the Quaternion (Gives length of 1).
         * If sign is true then w will be positive
         * (Makes calculations sometimes more reliable)
         *
         * @param values sign
         * @return normalized Quaternion.
         */
        Quaternion normalize(bool sign = false) {

            float m = getMagnitude();

            if (valid) {

                if (sign && w < 0) m = -m;
                
                w /= m;
                x /= m;
                y /= m;
                z /= m;

            }

            if (w==w && x==x && y==y && z==z) return *this; //Check for NAN event

            valid = false;

            return *this;

        }
        
        /**
         * Returns a copy of the Quaternion
         *
         * @param values none
         * @return copy of Quaternion.
         */
        Quaternion copy() {
            return *this;
        }

        Quaternion operator + (Quaternion b) {
            return Quaternion(w + b.w, x + b.x, y + b.y, z + b.z);
        }

        Quaternion operator - (Quaternion b) {
            return Quaternion(w - b.w, x - b.x, y - b.y, z - b.z);
        }

        Quaternion operator - (void) {
            return Quaternion(-w, -x, -y, -z);
        }

        Quaternion operator * (float c) {
            return Quaternion(w*c, x*c, y*c, z*c);
        }

        Quaternion operator / (float c) {
            return Quaternion(w/c, x/c, y/c, z/c);
        }

        void operator *= (float c) {
            w *= c;
            x *= c;
            y *= c;
            z *= c;
        }

        void operator += (Quaternion b) {
            w += b.w;
            x += b.x;
            y += b.y;
            z += b.z;
        }

        Quaternion operator * (Quaternion q) {
            return Quaternion(
                w*q.w - x*q.x - y*q.y - z*q.z,  // new w
                w*q.x + x*q.w + y*q.z - z*q.y,  // new x
                w*q.y - x*q.z + y*q.w + z*q.x,  // new y
                w*q.z + x*q.y - y*q.x + z*q.w); // new z
        }

};


#endif /* _HELPER_3DMATH_H_ */