// author: manuel scholz

#ifndef VEC4_H
#define VEC4_H

//*** INCLUDE *************************************************************************************

#include "fastmath.h"
#include "limits.h"
#include "assert.h"

//*** CLASS ***************************************************************************************

template<class T> class vec4 {
	public:

		//--- constructor -------------------------------------------------------------------------

		vec4() {
		}

		vec4(const vec4& v)
		{
			x = v.x;
			y = v.y;
			z = v.z;
			w = v.w;
		}

		vec4(T x, T y, T z, T w)
		{
			vec4::x = x;
			vec4::y = y;
			vec4::z = z;
			vec4::w = w;
		}
	 
		//--- methods -----------------------------------------------------------------------------

		vec3<T> xyz() {
			return vec3<T>(x,y,z);
		}

		void vec4::set(T x, T y, T z, T w) {
			vec4::x = x;
			vec4::y = y;
			vec4::z = z;
			vec4::w = w;
		}

		void vec4::scale(const vec4& v) {
			x *= v.x;
			y *= v.y;
			z *= v.z;
			w *= v.w;
		}
	
		void vec4::scaleUniform(T s) {
			x *= s;
			y *= s;
			z *= s;
			w *= s;
		}

		vec4 vec4::mul(const T s) {
			return vec4(x*s, y*s, z*s, w*s);
		}

		T dot(const vec4& v) {
			return x*v.x + y*v.y + z*v.z + w*v.w;
		}

		vec4 vec4::normalized() {
			T t = 1.0f / length();
			return vec4(x*t, y*t, z*t, w*t);
		}

		void vec4::normalize() {
			T t = 1.0f / length();
			x *= t;
			y *= t;
			z *= t;
			w *= t;
		}

		T vec4::length() {
			return sqrt(x*x + y*y + z*z + w*w);
		}

		T vec4::squaredLength() {
			return x*x + y*y + z*z + w*w;
		}

		vec4 vec4::minimum(vec4 v1) {
			return vec4(x < v1.x ? x : v1.x, 
				y < v1.y ? y : v1.y, 
				z < v1.z ? z : v1.z, 
				w < v1.w ? w : v1.w);
		}

		vec4 vec4::maximum(vec4 v1) {
			return vec4(x > v1.x ? x : v1.x, 
				y > v1.y ? y : v1.y, 
				z > v1.z ? z : v1.z, 
				w > v1.w ? w : v1.w);
		}

		bool vec4::equals(vec4 v1, T Epsilon) {
			return (abs(v1.x-x)<=Epsilon) && (abs(v1.y-y)<=Epsilon) && (abs(v1.z-z)<=Epsilon) && (abs(v1.w-w)<=Epsilon);
		}

		// performs rounding to nearest integer
		vec4<int> roundToInt() {
			assert((x >= LONG_MIN-0.5) && (y >= LONG_MIN-0.5) && (z >= LONG_MIN-0.5) && (w >= LONG_MIN-0.5));
			assert((x <= LONG_MAX+0.5) && (y <= LONG_MAX+0.5) && (z <= LONG_MAX+0.5) && (w <= LONG_MAX+0.5));
			return vec4<int>((x>0.0)?(x+0.5):(x-0.5),
							 (y>0.0)?(y+0.5):(y-0.5),
							 (z>0.0)?(z+0.5):(z-0.5),
							 (w>0.0)?(w+0.5):(w-0.5)); 
		}

		//--- operators ---------------------------------------------------------------------------

		// assign
		vec4& vec4::operator=(const vec4 v1) { 
			x = v1.x; 
			y = v1.y; 
			z = v1.z; 
			w = v1.w; 

			return (*this);
		}

		// component selector
		T& vec4::operator[](const unsigned int i) {
			assert((i >= 0) && (i < 4));
			return v[i]; 
		}

		vec4 vec4::operator+=(const vec4& v) {
			x += v.x;
			y += v.y;
			z += v.z;
			w += v.w;
			return *this;
		}

		vec4 vec4::operator*(T v) {
			return vec4(x*v, y*v, z*v, w*v);
		}

		vec4 vec4::operator+(const vec4& v) {
			return vec4(x+v.x, y+v.y, z+v.z, w+v.w);
		}

		vec4 vec4::operator-(const vec4& v) {
			return vec4(x-v.x, y-v.y, z-v.z, w-v.w);
		}

		//--- variable -----------------------------------------------------------------------------------

		union {
			struct {
				T x;
				T y;
				T z;
				T w;
			};

			T v[4];
		};
};

//*** TYPEDEFS ************************************************************************************

typedef vec4<unsigned char> uchar4;
typedef vec4<int>			      int4;
typedef vec4<unsigned int>	uint4;
typedef vec4<float>			    float4;
typedef vec4<double>		    double4;

//*************************************************************************************************

#endif
