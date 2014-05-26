// author: manuel scholz

#ifndef VEC3_H
#define VEC3_H

//*** INCLUDE *************************************************************************************

#include "fastmath.h"
#include "limits.h"
#include "assert.h"

//*** CLASS ***************************************************************************************

template<class T> class vec3
{
    public:
        typedef T Scalar;

        void set(T x, T y, T z) {
            vec3::x = x;
            vec3::y = y;
            vec3::z = z;
        }

        T squaredLength() {
            return x*x + y*y + z*z;
        }

        T length() {
            // TODO: eventually use template spezialization for float to ensure that single precision square root is used
            return sqrt(x*x + y*y + z*z);
        }

        void normalize() {
            T invl = ((T)1.0)/length();
            x *= invl;
            y *= invl;
            z *= invl;
        }

        vec3 normalized() {
            T invl = ((T)1.0)/length();
            return vec3(x*invl, y*invl, z*invl);
        }

        void scale(vec3 s) {
            x *= s.x;
            y *= s.y;
            z *= s.z;
        }

        // component wise minimum
        vec3 minimum(vec3 v) {
            return vec3(x < v.x ? x : v.x, y < v.y ? y : v.y, z < v.z ? z : v.z);
        }

        // component wise minimmaximum
        vec3 maximum(vec3 v) {
            return vec3(x > v.x ? x : v.x, y > v.y ? y : v.y, z > v.z ? z : v.z);
        }

        bool equals(vec3 v) {
            return (v.x == x) && (v.y == y) && (v.z == z);
        }

        // performs rounding to nearest integer
        vec3<int> roundToInt() {
            assert((x >= LONG_MIN-0.5) && (y >= LONG_MIN-0.5) && (z >= LONG_MIN-0.5));
            assert((x <= LONG_MAX+0.5) && (y <= LONG_MAX+0.5) && (z <= LONG_MAX+0.5));
            return vec3<int>((x>0.0)?(x+0.5):(x-0.5),
                             (y>0.0)?(y+0.5):(y-0.5),
                             (z>0.0)?(z+0.5):(z-0.5)); 
        }

        //--- OPERATORS -------------------------------

        // assign
        vec3& operator=(const vec3& v) { 
            x = v.x; 
            y = v.y; 
            z = v.z; 

            return (*this);
        }

        // equal
        bool operator==(const vec3 v) const {
            return (v.x == x) && (v.y == y) && (v.z == z);
        }

        // not equal
        bool operator!=(const vec3 v) const {
            return (v.x != x) || (v.y != y) || (v.z != z);
        }

        // less or equal
        bool operator<=(const vec3 v) const {
            return (x <= v.x) && (y <= v.y) && (z <= v.z);
        }

        // greater or equal
        bool operator>=(const vec3 v) const {
            return (x >= v.x) && (y >= v.y) && (z >= v.z);
        }

        // less
        bool operator<(const vec3 v) const {
            return (x < v.x) && (y < v.y) && (z < v.z);
        }

        // greater
        bool operator>(const vec3 v) const {
            return (x > v.x) && (y > v.y) && (z > v.z);
        }

        // add
        vec3 operator+(const vec3 v) const {
            return vec3(x+v.x, y+v.y, z+v.z);
        }

        // sub
        vec3 operator-(const vec3 v1) const {
            return vec3(x-v1.x, y-v1.y, z-v1.z);
        }

        // dot product
        T operator*(const vec3 v) const {
            return x*v.x + y*v.y + z*v.z;
        }

        // component wise division
        vec3 operator/(const vec3 v) const {
            return vec3(x/v.x, y/v.y, z/v.z);
        }

        // add assign
        vec3 operator+=(const vec3 v) {
            x += v.x;
            y += v.y;
            z += v.z;
            return *this;
        }

        // sub assign
        vec3 operator-=(const vec3 v) {
            x -= v.x;
            y -= v.y;
            z -= v.z;
            return *this;
        }

        // cross product
        vec3 operator%(const vec3 v) const {
            return vec3(y*v.z - z*v.y, z*v.x - x*v.z, x*v.y - y*v.x);
        }

        // component wise multiplication
        vec3 operator&(const vec3 v1) const {
            return vec3(x*v1.x, y*v1.y, z*v1.z);
        }

        // multiply with scalar
        vec3 operator*(const T f) const {
            return vec3(x*f, y*f, z*f);
        }

        // division by scalar
        vec3 operator/(const T f) const {
            return vec3(x/f, y/f, z/f);
        }

        // multiply assign with scalar
        vec3 operator*=(const T f) {
            x *= f;
            y *= f;
            z *= f;
            return *this;
        }

        // division assign by scalar
        vec3 operator/=(const T f) {
            x /= f;
            y /= f;
            z /= f;
            return *this;
        }

        // negate
        vec3 operator-(void) const {
            return vec3(-x, -y, -z);
        }

        // component selector
        T& operator[](const unsigned int i) {
            assert((i >= 0) && (i < 3));
            return v[i]; 
        }

        //--- constructor -----------------------------

        vec3() {
        }

        vec3(const vec3& v)
        {
            x = v.x;
            y = v.y;
            z = v.z;
        }

        template<class S> vec3(const vec3<S>& v)
        {
            x = (T)v.x;
            y = (T)v.y;
            z = (T)v.z;
        }

        vec3(T x, T y, T z)
        {
            vec3::x = x;
            vec3::y = y;
            vec3::z = z;
        }
     
        //--- variable -----------------------------------------------------------------------------------

        union {
            struct {
                T x;
                T y;
                T z;
            };

            T v[3];
        };
};

//*** TYPEDEFS ************************************************************************************

typedef vec3<unsigned char> uchar3;
typedef vec3<int>			int3;
typedef vec3<unsigned int>	uint3;
typedef vec3<float>			float3;
typedef vec3<double>		double3;

//*************************************************************************************************

#endif
