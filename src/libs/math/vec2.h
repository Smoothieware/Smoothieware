// author: manuel scholz

#ifndef VEC2_H
#define VEC2_H

//*** INCLUDE *************************************************************************************

#include "fastmath.h"
#include "limits.h"
#include "assert.h"

//*** CLASS ***************************************************************************************

template<class T> class vec2
{
    public:
        typedef T Scalar;

        void set(T x, T y) {
            vec2::x = x;
            vec2::y = y;
        }

        T squaredLength() {
            return x*x + y*y;
        }

        T length() {
            // TODO: eventually use template spezialization for float to ensure that single precision square root is used
            return sqrt(x*x + y*y);
        }

        void normalize() {
            T invl = ((T)1.0)/length();
            x *= invl;
            y *= invl;
        }

        vec2 normalized() {
            T invl = ((T)1.0)/length();
            return vec2(x*invl, y*invl);
        }

        // component wise scale
        void scale(vec2 s) {
            x *= s.x;
            y *= s.y;
        }

        // component wise minimum
        vec2 minimum(vec2 v) {
            return vec2(x < v.x ? x : v.x, 
                        y < v.y ? y : v.y);
        }

        // component wise maximum
        vec2 maximum(vec2 v) {
            return vec2(x > v.x ? x : v.x, 
                        y > v.y ? y : v.y);
        }

        // returns 1/vector
        vec2 invert() {
            return vec2(Scalar(1.0)/x, Scalar(1.0)/y);
        }
    
        //be aware of numerical issues wehn comparing floating point vectors
        bool equals(vec2 v) {
            return (v.x == x) && (v.y == y);
        }

        // performs rounding to nearest integer
        vec2<int> roundToInt() {
            assert((x >= LONG_MIN-0.5) && (y >= LONG_MIN-0.5));
            assert((x <= LONG_MAX+0.5) && (y <= LONG_MAX+0.5));
            return vec2<int>((x>0.0)?(x+0.5):(x-0.5),
                             (y>0.0)?(y+0.5):(y-0.5)); 
        }

        //--- OPERATORS -------------------------------

        // assign
        vec2& operator=(const vec2& v) { 
            x = v.x; 
            y = v.y; 
            return (*this);
        }

        // equal
        bool operator==(const vec2 v) const {
            return (v.x == x) && (v.y == y);
        }

        // not equal
        bool operator!=(const vec2 v) const {
            return (v.x != x) || (v.y != y);
        }

        // less or equal
        bool operator<=(const vec2 v) const {
            return (x <= v.x) && (y <= v.y);
        }

        // greater or equal
        bool operator>=(const vec2 v) const {
            return (x >= v.x) && (y >= v.y);
        }

        // less
        bool operator<(const vec2 v) const {
            return (x < v.x) && (y < v.y);
        }

        // greater
        bool operator>(const vec2 v) const {
            return (x > v.x) && (y > v.y);
        }

        // add
        vec2 operator+(const vec2 v) const {
            return vec2(x+v.x, y+v.y);
        }

        // sub
        vec2 operator-(const vec2 v1) const {
            return vec2(x-v1.x, y-v1.y);
        }

        // dot product
        T operator*(const vec2 v) const {
            return x*v.x + y*v.y;
        }

        // component wise division
        vec2 operator/(const vec2 v) const {
            return vec2(x/v.x, y/v.y);
        }

        // add assign
        vec2 operator+=(const vec2 v) {
            x += v.x;
            y += v.y;
            return *this;
        }

        // sub assign
        vec2 operator-=(const vec2 v) {
            x -= v.x;
            y -= v.y;
            return *this;
        }

        // component wise multiplication
        vec2 operator&(const vec2 v1) const {
            return vec2(x*v1.x, y*v1.y);
        }

        // multiply with scalar
        vec2 operator*(const T f) const {
            return vec2(x*f, y*f);
        }

        // division by scalar
        vec2 operator/(const T f) const {
            return vec2(x/f, y/f);
        }

        // multiply assign with scalar
        vec2 operator*=(const T f) {
            x *= f;
            y *= f;
            return *this;
        }

        // division assign by scalar
        vec2 operator/=(const T f) {
            x /= f;
            y /= f;
            return *this;
        }

        // negate
        vec2 operator-(void) const {
            return vec2(-x, -y);
        }

        // component selector
        T& operator[](const T i) {
            assert((i >= 0) && (i < 3));
            return v[i]; 
        }

        //--- constructor --------------------------------------------------------------------------------

        vec2() {
        }

        vec2(const vec2& v) {
            x = v.x;
            y = v.y;
        }

        template<class S> vec2(const vec2<S>& v) {
            x = (T)v.x;
            y = (T)v.y;
        }

        vec2(T x, T y) {
            vec2::x = x;
            vec2::y = y;
        }
     
        //--- variable -----------------------------------------------------------------------------------

        union {
            struct {
                T x;
                T y;
            };

            T v[2];
        };
};

//*** TYPEDEFS ************************************************************************************

typedef vec2<int>			int2;
typedef vec2<unsigned int>	uint2;
typedef vec2<float>			float2;
typedef vec2<double>		double2;

//*************************************************************************************************

#endif
