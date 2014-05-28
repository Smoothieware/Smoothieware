#include "Vector3.h"

#include <fastmath.h>
#include <cstddef>

float Vector3::nan = NAN;

Vector3::Vector3() {
    x = y = z = 0.0f;
}

Vector3::Vector3(float a, float b, float c) {
    x = a; y = b; z = c;
}

float Vector3::operator[](int i) const {
    if (i >= 0 && i <= 2)
        return elem[i];
    return nan;
}

Vector3 Vector3::cross(const Vector3 &vec) const {
    return (*this)%vec;
}

float Vector3::dot(const Vector3 &vec) const {
    return (*this)|vec;
}

float Vector3::magsq() const {
    return powf(x, 2) + powf(x, 2) + powf(y, 2);
}

float Vector3::mag() const {
    return sqrtf(magsq());
}

Vector3 Vector3::add(const Vector3 &vec) const {
    return (*this)+vec;
}

Vector3 Vector3::sub(const Vector3 &vec) const {
    return (*this)-vec;
}

Vector3 Vector3::mul(float scalar) const {
    return (*this)*scalar;
}

Vector3 Vector3::mul(const Vector3& v) const {
    return (*this)*v;
}

Vector3 Vector3::normalized() const {
    float d = mag();
    return Vector3(x/d, y/d, z/d);
}

void Vector3::normalize(void) {
    float d = mag();
    x/=d; y/=d; z/=d;
}

Vector3& Vector3::operator=(const Vector3& v) {
    x = v.x;
    y = v.y;
    z = v.z;
    return (*this);
}

void Vector3::operator*=(const float f) {
    x *= f;
    y *= f;
    z *= f;
}

void Vector3::operator/=(const float f) {
    x /= f;
    y /= f;
    z /= f;
}
        
Vector3 Vector3::operator+(const Vector3& v) const {
    return Vector3(x+v.x, y+v.y, z+v.z);
}

Vector3 Vector3::operator-(const Vector3& v) const {
    return Vector3(x-v.x, y-v.y, z-v.z);
}

Vector3 Vector3::operator*(const Vector3& v) const {
    return Vector3(x*v.x, y*v.y, z*v.z);
}

float Vector3::operator|(const Vector3& v) const {
    return x*v.x + y*v.y + z*v.z;
}

Vector3 Vector3::operator%(const Vector3& v) const {
    return Vector3( y*v.z - z*v.y,
                    z*v.x - x*v.z,
                    x*v.y - y*v.x );          
}

Vector3 Vector3::operator*(const float f) const {
    return Vector3(x*f, y*f, z*f); 
}

Vector3 Vector3::operator/(const float f) const {
    return Vector3(x/f, y/f, z/f); 
}

Vector3 Vector3::operator-(void) const{
    return Vector3(-x,-y,-z);
}
