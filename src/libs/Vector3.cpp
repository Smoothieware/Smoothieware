#include "Vector3.h"

#include <fastmath.h>
#include <cstddef>

float Vector3::nan = NAN;

Vector3::Vector3()
{
    elem[0] = elem[1] = elem[2] = 0.0F;
}

Vector3::Vector3(float a, float b, float c)
{
    elem[0] = a;
    elem[1] = b;
    elem[2] = c;
}

float Vector3::operator[](int i) const
{
    if (i >= 0 && i <= 2)
        return elem[i];
    return nan;
}

Vector3 Vector3::cross(const Vector3 &vec) const
{
    Vector3 out;

    out.elem[0] = elem[1] * vec.elem[2] - elem[2] * vec.elem[1];
    out.elem[1] = elem[2] * vec.elem[0] - elem[0] * vec.elem[2];
    out.elem[2] = elem[0] * vec.elem[1] - elem[1] * vec.elem[0];

    return out;
}

float Vector3::dot(const Vector3 &vec) const
{
    return  elem[0] * vec.elem[0] +
            elem[1] * vec.elem[1] +
            elem[2] * vec.elem[2];
}

float Vector3::magsq() const
{
    return  powf(elem[0], 2) +
            powf(elem[1], 2) +
            powf(elem[2], 2);
}

float Vector3::mag() const
{
    return sqrtf(magsq());
}

Vector3 Vector3::add(const Vector3 &vec) const
{
    Vector3 out;

    out.elem[0] = elem[0] + vec.elem[0];
    out.elem[1] = elem[1] + vec.elem[1];
    out.elem[2] = elem[2] + vec.elem[2];

    return out;
}

Vector3 Vector3::sub(const Vector3 &vec) const
{
    Vector3 out;

    out.elem[0] = elem[0] - vec.elem[0];
    out.elem[1] = elem[1] - vec.elem[1];
    out.elem[2] = elem[2] - vec.elem[2];

    return out;
}

Vector3 Vector3::mul(float scalar) const
{
    Vector3 out;

    out.elem[0] = elem[0] * scalar;
    out.elem[1] = elem[1] * scalar;
    out.elem[2] = elem[2] * scalar;

    return out;
}

Vector3 Vector3::mul(const Vector3& v) const
{
    Vector3 out;

    out.elem[0] = elem[0] * v[0];
    out.elem[1] = elem[1] * v[1];
    out.elem[2] = elem[2] * v[2];

    return out;
}

Vector3 Vector3::unit() const
{
    Vector3 out;

    float denom = mag();

    out.elem[0] = elem[0] / denom;
    out.elem[1] = elem[1] / denom;
    out.elem[2] = elem[2] / denom;

    return out;
}
