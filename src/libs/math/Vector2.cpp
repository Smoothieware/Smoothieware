#include "Vector2.h"

#include <fastmath.h>
#include <cstddef>

float Vector2::nan = NAN;

Vector2::Vector2() {
    x = y = 0.0f;
}

Vector2::Vector2(float a, float b) {
    x = a; y = b;
}

float Vector2::operator[](int i) const {
    if (i >= 0 && i <= 1)
        return elem[i];
    return nan;
}

float Vector2::dot(const Vector2 &vec) const {
    return (*this)|vec;
}

float Vector2::magsq() const {
    return powf(elem[0], 2) + powf(elem[1], 2);
}

float Vector2::mag() const {
    return sqrtf(magsq());
}

Vector2 Vector2::add(const Vector2 &vec) const {
    return (*this)+vec;
}

Vector2 Vector2::sub(const Vector2 &vec) const {
    return (*this)-vec;
}

Vector2 Vector2::mul(float scalar) const {
    return (*this)*scalar;
}

Vector2 Vector2::mul(const Vector2& v) const {
    return (*this)*v;
}

Vector2 Vector2::normalized() const {
    float d = mag();
    return Vector2(x/d, y/d);
}
       
void Vector2::normalize(void) {
    float d = mag();
    x/=d; y/=d;
}

Vector2& Vector2::operator=(const Vector2& v) {
    x = v.x;
    y = v.y;
    return (*this);
}

void Vector2::operator*=(const float f) {
    x *= f;
    y *= f;
}

void Vector2::operator/=(const float f) {
    x /= f;
    y /= f;
}

void Vector2::operator+=(const Vector2& v) {
    x += v.x;
    y += v.y;
}

void Vector2::operator-=(const Vector2& v) {
    x -= v.x;
    y -= v.y;
}      
   
Vector2 Vector2::operator+(const Vector2& v) const {
    return Vector2(x+v.x, y+v.y);
}

Vector2 Vector2::operator-(const Vector2& v) const {
    return Vector2(x-v.x, y-v.y);
}

Vector2 Vector2::operator*(const Vector2& v) const {
    return Vector2(x*v.x, y*v.y);
}

float Vector2::operator|(const Vector2& v) const {
    return x*v.x + y*v.y;
}

Vector2 Vector2::operator*(const float f) const {
    return Vector2(x*f, y*f); 
}

Vector2 Vector2::operator/(const float f) const {
    return Vector2(x/f, y/f); 
}

Vector2 Vector2::operator-(void) const{
    return Vector2(-x,-y);
}
